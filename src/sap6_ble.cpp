/**
 * SAP6 / CaveBLE GATT server for TopoDroid & SexyTopo.
 * Protocol: https://github.com/furbrain/CircuitPython_CaveBLE
 */

#include "sap6_ble.h"

#ifdef ARDUINO_ARCH_ESP32

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <esp32-hal-bt.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_gap_ble_api.h>
#include <cstring>

#ifndef SAP6_BLE_DEVICE_NAME
#define SAP6_BLE_DEVICE_NAME "SAP6_0001"
#endif

static const char *kSvcUuid     = "137c4435-8a64-4bcb-93f1-3792c6bdc965";
static const char *kNameUuid    = "137c4435-8a64-4bcb-93f1-3792c6bdc966";
static const char *kCmdUuid     = "137c4435-8a64-4bcb-93f1-3792c6bdc967";
static const char *kLegUuid     = "137c4435-8a64-4bcb-93f1-3792c6bdc968";

static constexpr uint8_t kAck0 = 0x55;
static constexpr uint8_t kAck1 = 0x56;

static constexpr size_t kQueueMax = 20;
static constexpr uint32_t kResendMs = 5000UL;

struct Sap6QueuedLeg {
    float az, inc, roll, dist;
};

static BLEServer *g_server = nullptr;
static BLECharacteristic *g_leg_char = nullptr;
static bool g_stack_ready = false;
static bool g_connected = false;

static Sap6QueuedLeg g_queue[kQueueMax];
static size_t g_q_head = 0;
static size_t g_q_tail = 0;
static size_t g_q_count = 0;

static uint8_t g_seq_bit = 0;
static bool g_waiting_ack = false;
static uint32_t g_last_send_ms = 0;
static uint8_t g_last_leg_pkt[17];

static uint32_t g_legs_sent = 0;
static uint32_t g_acks_ok = 0;
static uint32_t g_acks_wrong = 0;
static uint32_t g_resends = 0;
static uint32_t g_adv_kick_ms = 0;
static char g_adv_name[21] = SAP6_BLE_DEVICE_NAME;

extern void sap6_on_command(uint8_t cmd);

static void sap6_ble_configure_advertising(const char *device_name);
static void sap6_ble_radio_quiet(void);

static void queue_push(float az, float inc, float roll, float dist)
{
    if (g_q_count >= kQueueMax)
        return;
    g_queue[g_q_tail] = { az, inc, roll, dist };
    g_q_tail = (g_q_tail + 1) % kQueueMax;
    g_q_count++;
}

static bool queue_pop(Sap6QueuedLeg &out)
{
    if (g_q_count == 0)
        return false;
    out = g_queue[g_q_head];
    g_q_head = (g_q_head + 1) % kQueueMax;
    g_q_count--;
    return true;
}

static void build_leg_packet(uint8_t seq, float az, float inc, float roll, float dist)
{
    memcpy(g_last_leg_pkt, &seq, 1);
    memcpy(g_last_leg_pkt + 1, &az, 4);
    memcpy(g_last_leg_pkt + 5, &inc, 4);
    memcpy(g_last_leg_pkt + 9, &roll, 4);
    memcpy(g_last_leg_pkt + 13, &dist, 4);
}

static void notify_leg_packet(void)
{
    if (!g_leg_char || !g_connected)
        return;
    g_leg_char->setValue(g_last_leg_pkt, sizeof(g_last_leg_pkt));
    g_leg_char->notify();
    g_last_send_ms = millis();
    g_legs_sent++;
}

static void send_next_leg_from_queue(void)
{
    Sap6QueuedLeg leg;
    if (!queue_pop(leg))
        return;
    build_leg_packet(g_seq_bit, leg.az, leg.inc, leg.roll, leg.dist);
    g_seq_bit ^= 1;
    notify_leg_packet();
    g_waiting_ack = true;
}

static void handle_command_byte(uint8_t cmd)
{
    if (cmd == 0)
        return;

    if (g_waiting_ack) {
        const uint8_t expected = (g_seq_bit == 0) ? kAck1 : kAck0;
        const uint8_t wrong    = (g_seq_bit == 0) ? kAck0 : kAck1;
        if (cmd == expected) {
            g_waiting_ack = false;
            g_acks_ok++;
            send_next_leg_from_queue();
            return;
        }
        if (cmd == wrong) {
            g_acks_wrong++;
            return;
        }
    }

    sap6_on_command(cmd);
}

class Sap6ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *) override
    {
        g_connected = true;
    }
    void onDisconnect(BLEServer *srv) override
    {
        g_connected = false;
        g_waiting_ack = false;
        sap6_ble_configure_advertising(g_adv_name);
        (void)srv;
    }
};

class Sap6CmdCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *ch) override
    {
        std::string v = ch->getValue();
        if (!v.empty())
            handle_command_byte((uint8_t)v[0]);
    }
};

static void sap6_ble_radio_quiet(void)
{
    WiFi.mode(WIFI_OFF);
    esp_wifi_stop();
}

static void sap6_ble_configure_advertising(const char *device_name)
{
    if (!device_name || !device_name[0])
        device_name = SAP6_BLE_DEVICE_NAME;
    strncpy(g_adv_name, device_name, sizeof(g_adv_name) - 1);
    g_adv_name[sizeof(g_adv_name) - 1] = '\0';

    if (!btStarted())
        return;

    esp_ble_gap_set_device_name(g_adv_name);

    /*
     * Padrao ESP32 BLE server (visivel no nRF): UUID no ADV, nome no scan response.
     * A biblioteca trata os eventos GAP; nao chamar esp_ble_gap_config_adv_data() a mao.
     */
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(kSvcUuid);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);
    adv->setMaxPreferred(0x12);
    adv->setMinInterval(0x20);
    adv->setMaxInterval(0x40);
    BLEDevice::startAdvertising();
    g_adv_kick_ms = millis();
}

static bool sap6_ble_create_gatt_server(void)
{
    if (g_server)
        return true;

    g_server = BLEDevice::createServer();
    if (!g_server)
        return false;
    g_server->setCallbacks(new Sap6ServerCallbacks());

    BLEService *svc = g_server->createService(kSvcUuid);
    if (!svc)
        return false;

    BLECharacteristic *name_ch = svc->createCharacteristic(
        kNameUuid, BLECharacteristic::PROPERTY_READ);
    name_ch->setValue("SAP6");

    BLECharacteristic *cmd_ch = svc->createCharacteristic(
        kCmdUuid, BLECharacteristic::PROPERTY_WRITE);
    cmd_ch->setCallbacks(new Sap6CmdCallbacks());

    g_leg_char = svc->createCharacteristic(
        kLegUuid,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    g_leg_char->addDescriptor(new BLE2902());

    svc->start();
    return true;
}

void sap6_ble_begin(const char *device_name)
{
    if (!device_name || !device_name[0])
        device_name = SAP6_BLE_DEVICE_NAME;

    if (g_stack_ready && g_server && btStarted()) {
        sap6_ble_configure_advertising(device_name);
        return;
    }

    g_stack_ready = false;
    g_connected = false;
    g_waiting_ack = false;

    sap6_ble_radio_quiet();

    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    }

    if (!btStarted() && !btStart()) {
        return;
    }

    if (!BLEDevice::getInitialized()) {
        BLEDevice::init(device_name);
    } else {
        esp_ble_gap_set_device_name(device_name);
    }

    if (!btStarted()) {
        return;
    }

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);

    if (!sap6_ble_create_gatt_server()) {
        return;
    }

    sap6_ble_configure_advertising(device_name);
    g_stack_ready = btStarted();
}

void sap6_ble_restart(const char *device_name)
{
    g_connected = false;
    g_waiting_ack = false;
    g_stack_ready = false;

    sap6_ble_radio_quiet();

    if (BLEDevice::getInitialized()) {
        BLEDevice::stopAdvertising();
        BLEDevice::deinit(true);
    } else if (btStarted()) {
        btStop();
    }

    g_server = nullptr;
    g_leg_char = nullptr;

    vTaskDelay(pdMS_TO_TICKS(200));
    sap6_ble_begin(device_name);
}

void sap6_ble_poll(void)
{
    if (!g_stack_ready)
        return;

    if (!g_connected)
        return;

    if (g_waiting_ack && (millis() - g_last_send_ms) >= kResendMs) {
        notify_leg_packet();
        g_resends++;
    } else if (!g_waiting_ack && g_q_count > 0) {
        send_next_leg_from_queue();
    }
}

bool sap6_ble_stack_ready(void) { return g_stack_ready && btStarted(); }
bool sap6_ble_connected(void) { return g_connected; }

void sap6_ble_get_mac_str(char *buf, size_t len)
{
    if (!buf || len < 18) return;
    uint64_t mac = ESP.getEfuseMac();
    snprintf(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X",
             (unsigned)((mac >> 40) & 0xff), (unsigned)((mac >> 32) & 0xff),
             (unsigned)((mac >> 24) & 0xff), (unsigned)((mac >> 16) & 0xff),
             (unsigned)((mac >> 8) & 0xff), (unsigned)(mac & 0xff));
}

void sap6_ble_format_status(char *buf, size_t len)
{
    if (!buf || len < 8)
        return;
    snprintf(buf, len, "BT ctrl %d  stack %s  heap %u",
             (int)esp_bt_controller_get_status(),
             (g_stack_ready && btStarted()) ? "OK" : "OFF",
             (unsigned)ESP.getFreeHeap());
}

void sap6_ble_send_leg(float azimuth_deg, float inclination_deg, float roll_deg,
                       float distance_m)
{
    queue_push(azimuth_deg, inclination_deg, roll_deg, distance_m);
    if (!g_waiting_ack && g_connected)
        send_next_leg_from_queue();
}

int sap6_ble_stream_points(int count, const void *pts, size_t pt_stride,
                           float (*get_az)(const void *), float (*get_inc)(const void *),
                           float (*get_roll)(const void *), float (*get_dist)(const void *))
{
    int n = 0;
    for (int i = 0; i < count; i++) {
        const uint8_t *p = (const uint8_t *)pts + (size_t)i * pt_stride;
        sap6_ble_send_leg(get_az(p), get_inc(p), get_roll(p), get_dist(p));
        n++;
    }
    return n;
}

void sap6_ble_clear_bonds(void)
{
    int dev_num = esp_ble_get_bond_device_num();
    if (dev_num <= 0)
        return;
    esp_ble_bond_dev_t list[8];
    if (dev_num > 8)
        dev_num = 8;
    if (esp_ble_get_bond_device_list(&dev_num, list) == ESP_OK) {
        for (int i = 0; i < dev_num; i++)
            esp_ble_remove_bond_device(list[i].bd_addr);
    }
}

uint32_t sap6_ble_legs_sent(void) { return g_legs_sent; }
uint32_t sap6_ble_acks_ok(void) { return g_acks_ok; }
uint32_t sap6_ble_acks_wrong(void) { return g_acks_wrong; }
uint32_t sap6_ble_resends(void) { return g_resends; }
uint32_t sap6_ble_queue_depth(void) { return (uint32_t)g_q_count + (g_waiting_ack ? 1u : 0u); }

#endif
