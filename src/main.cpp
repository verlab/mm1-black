/**
 * BRICS-5 MM1  –  Smart Tape / UART laser range + IMU
 * ESP32 CYD  |  ST7796 480×320  |  LVGL 8.3
 *
 * Flat point list; BRIC5-style table. Botão físico: mantém pressionado para pedir medições laser;
 * ao soltar grava o ponto na tabela (curto = shot, ≥650 ms = nav). Debounce 50 ms.
 * Two point types:
 *   S (Sample)    – measurement samples
 *   N (Navigation) – reference points for transforms
 *
 * Tabs: POINTS | SENSOR | FILES | SETUP
 *
 * BT (SPP, nome BLE típico BRICS5-MM1): MEAS | CLEAR | LIST | EXPORT | FILES | FILE_SEND,<path>
 * Export CSV alinhado BRIC5/TopoDroid. Ref. outro dispositivo (Mr_Zappy): alguns enviam
 * linhas texto COMPASS/CLINO/DIST — aqui o foco é CSV + comandos acima.
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_BNO08x.h>
#include <lvgl.h>
#include <SD.h>
#include <BluetoothSerial.h>
#ifdef ARDUINO_ARCH_ESP32
#include <esp_log.h>
#include "esp32-hal-ledc.h"
#endif

/* Bitmap RGB565 gerado em mira_splash_img.c (480×320); splash só TFT, sem LVGL. */
extern const uint8_t mira_splash_map[];

// ── Pins ─────────────────────────────────────────────────────────────────────
#define SCREEN_W  480
#define SCREEN_H  320
#ifndef SPLASH_MS
#define SPLASH_MS 3000UL
#endif
#define SD_CS 5
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define I2C_SDA 32
#define I2C_SCL 25
/* GPIO17: botão físico (INPUT_PULLUP, ativo em LOW). IMU não usa reset por hardware aqui. */
#define USER_BUTTON_PIN 17
#define IMU_RST_PIN (-1)
#define IMU_INT 16
/* E32R40T: amplificador de áudio — LOW = ligado (demo Waveshare). Buzzer via PWM no pino do DAC. */
#define AUDIO_EN_PIN    4
#define SPEAKER_PWM_PIN 26
#define BUZZER_LEDC_CH  7
#define IMU_ADDR 0x4B
#define BAT_ADC_PIN 34

/* Eixo do laser no referencial do sensor (corpo), normalizado em runtime.
 * Padrão +X na placa; se o laser apontar para +Y ou +Z, ajuste IMU_LASER_AXIS_{BX,BY,BZ}. */
#ifndef IMU_LASER_AXIS_BX
#define IMU_LASER_AXIS_BX (1.0f)
#endif
#ifndef IMU_LASER_AXIS_BY
#define IMU_LASER_AXIS_BY (0.0f)
#endif
#ifndef IMU_LASER_AXIS_BZ
#define IMU_LASER_AXIS_BZ (0.0f)
#endif

// Laser M01-style UART (9600). Wiring: módulo TX → RX do ESP; módulo RX → TX do ESP.
// Níveis: use só 3V3 no VCC e nas linhas I/O do laser; GPIO do ESP32 é 3,3 V TTL (não 5 V no RX).
//
// LZR_SHARE_USB_UART=1: UART0 = RXD0(IO3) + TXD0(IO1) na placa ESP32-32E (E32R40T manual).
//   Serial.begin(9600, SERIAL_8N1, 3, 1)  ==  RX=GPIO3, TX=GPIO1
// Sem tráfego de texto no UART0 (partilhado com laser). Só comandos laser em lzr_port.
// Com cabo USB ligado, o CH340 e o laser disputam a mesma linha RX — em campo alimente por bateria
// ou desconecte o USB ao depurar o laser neste par.
//
// LZR_SHARE_USB_UART=0: UART extra (LZR_UART_NUM) em LZR_PIN_RX / LZR_PIN_TX.
#ifndef LZR_SHARE_USB_UART
#define LZR_SHARE_USB_UART 0
#endif
#if LZR_SHARE_USB_UART
#define DBG_PRINT(...) ((void)0)
#else
#define DBG_PRINT(...) do { Serial.printf(__VA_ARGS__); } while (0)
#endif
#ifndef LZR_UART_NUM
#define LZR_UART_NUM 1
#endif
// Manual ESP32-32E: RXD0 = GPIO3, TXD0 = GPIO1. lzr_port.begin(baud, config, RX, TX).
// Se não houver bytes na RX, experimente em platformio: -DLZR_SWAP_RXTX=1 (troca só na API begin).
#ifndef LZR_SWAP_RXTX
#define LZR_SWAP_RXTX 0
#endif
#if LZR_SHARE_USB_UART
#if LZR_SWAP_RXTX
#define LZR_PIN_RX 1
#define LZR_PIN_TX 3
#else
#define LZR_PIN_RX 3
#define LZR_PIN_TX 1
#endif
#else
#ifndef LZR_PIN_RX
#define LZR_PIN_RX 22
#endif
#ifndef LZR_PIN_TX
#define LZR_PIN_TX 21
#endif
#endif
#ifndef LZR_ENA_PIN
#define LZR_ENA_PIN (-1)
#endif
#ifndef LZR_BAUD
#define LZR_BAUD 9600
#endif
#ifndef STALE_MS
#define STALE_MS 4000UL
#endif
#ifndef RECOVER_MIN_MS
#define RECOVER_MIN_MS 6000UL
#endif
#ifndef RX_BURST_MAX
#define RX_BURST_MAX 256
#endif
#ifndef PARSE_IDLE_MS
#define PARSE_IDLE_MS 250UL
#endif
// Taxa base (aba POINTS). Aba SENSOR: 1 Hz (SENSOR_TAB_POLL_MS).
#ifndef POLL_INTERVAL_MS
#define POLL_INTERVAL_MS 350UL
#endif
#ifndef SENSOR_TAB_POLL_MS
#define SENSOR_TAB_POLL_MS 1000UL
#endif
#ifndef FILES_TAB_POLL_MS
#define FILES_TAB_POLL_MS 2500UL
#endif
#ifndef POLL_TIMEOUT_MS
#define POLL_TIMEOUT_MS 3200UL
#endif
#ifndef UART_HARD_REINIT
#define UART_HARD_REINIT 0
#endif
// 1 = laser em medição contínua (útil no UART compartilhado / alguns M01). 0 = só pedido único.
#ifndef LZR_CONTINUOUS
#define LZR_CONTINUOUS 0
#endif
#ifndef LZR_KEEPALIVE_MS
#define LZR_KEEPALIVE_MS 45000UL
#endif
/* BRIC5 CSV “Dip” column ≈ inclinação magnética local; MM1 não tem magnetómetro — valor nominal para compatibilidade TopoDroid/BRIC5. */
#ifndef EXPORT_BRIC_DIP_DEG
#define EXPORT_BRIC_DIP_DEG (29.88f)
#endif
#ifndef POSIX_FALLBACK_ANCHOR_SEC
#define POSIX_FALLBACK_ANCHOR_SEC (1767225600UL)
#endif

static const uint16_t TOUCH_CAL[5] = { 254, 3643, 176, 3693, 7 };

// ── Colours ──────────────────────────────────────────────────────────────────
#define C_BG        0xF0F4F8u
#define C_TOPBAR    0xFFFFFFu
#define C_TBL_HDR   0x1E3A5Fu
#define C_ROW_ODD   0xFFFFFFu
#define C_ROW_EVEN  0xEEF2F7u
#define C_ROW_SEL   0xBBDEFBu
#define C_ROW_NAV   0xE0F2F1u   // light teal tint for NAV rows
#define C_TEXT      0x1A2027u
#define C_HDR_LINE  0x1565C0u
#define C_BORDER    0xCFD8DCu
#define C_WHITE     0xFFFFFFu
#define C_GREY      0x546E7Au
#define C_BT_ON     0x1565C0u
#define C_BT_OFF    0x90A4AEu
#define C_SD_ON     0x2E7D32u
#define C_SD_OFF    0xD32F2Fu
#define C_BTN_SAMP  0x00897Bu   // teal  – add sample
#define C_BTN_NAV   0x4A148Cu   // purple – add nav
#define C_BTN_DEL   0xC62828u
#define C_BTN_GPS   0x1565C0u
#define C_BTN_SAVE  0x1B5E20u
#define C_BTN_NEW   0x00897Bu
#define C_BTN_USE   0x4A148Cu
#define C_FILE_ACT  0xC8E6C9u
#define C_REF_S     0x1565C0u
#define C_TYPE_S    0x00695Cu   // sample text
#define C_TYPE_N    0x6A1B9Au   // nav text
#define C_BAT_OK    0x2E7D32u
#define C_BAT_LOW   0xD32F2Fu

// ── Data / CSV ───────────────────────────────────────────────────────────────
// Exportação compatível com BRIC5 / TopoDroid (cf. bric5/data/*.csv). Dois espaços antes de “Measurement Type”.
#define BRIC_CSV_HEADER \
    "Time-Stamp, POSIX Time, Index, Distance (meters), Azimuth (deg), Inclination (deg), Dip (deg), Roll (deg), Temperature (Celsius),  Measurement Type, Error Log"
#define MAX_PTS  50
#define MAX_FILES 16

enum PtType : uint8_t { PT_SAMPLE = 0, PT_NAV = 1 };

struct MeasPoint {
    uint32_t id;
    PtType   type;
    float    dist, roll, pitch, yaw;
    bool     laser_ok;
    uint32_t posix_sec;
    float    dip_deg;
    float    temp_c;
    char     error_log[48];
};

// ── Globals ──────────────────────────────────────────────────────────────────
static TFT_eSPI          tft;
#ifdef ARDUINO_ARCH_ESP32
/** SD no HSPI: nunca usar `SPI.begin(...)` no VSPI global — TFT_eSPI (display + XPT2046) usa VSPI em 12/13/14. */
static SPIClass sd_spi(HSPI);
#endif
static BluetoothSerial   SerialBT;
static Adafruit_BNO08x   bno08x(-1);
static sh2_SensorValue_t sensorValue;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t        *lvgl_buf = nullptr;

static MeasPoint pts[MAX_PTS];
static int       pt_count = 0;
static int       sel_row  = -1;
static uint32_t  next_id  = 1;

// Sensors
static uint32_t tof_dist_mm = 0;
/** Azimute 0–360° e inclinação (° acima/baixo da horizontal) ao longo do eixo do laser. */
static float    imu_azimuth_deg = 0, imu_inclination_deg = 0;
static float    imu_roll = 0, imu_pitch = 0, imu_yaw = 0;
static bool     imu_ok = false, tof_ok = false;
static volatile bool imu_irq = false;

// Battery (placeholder – reads ADC or shows 100%)
static int bat_pct = 100;

// SD / files
static bool sd_ready = false, bt_conn = false;
static char active_csv[32] = "/brics5_mm1.csv";
static char file_names[MAX_FILES][32];
static int  file_count = 0, file_sel = -1;

// UI handles
static lv_obj_t *ui_tbl_pts      = nullptr;
static lv_obj_t *ui_lbl_bt       = nullptr;
static lv_obj_t *ui_lbl_sd       = nullptr;
static lv_obj_t *ui_lbl_bat      = nullptr;
static lv_obj_t *ui_lbl_time     = nullptr;
static lv_obj_t *ui_lbl_count    = nullptr;
static lv_obj_t *ui_lbl_tof_val  = nullptr;
static lv_obj_t *ui_lbl_imu_val  = nullptr;
static lv_obj_t *ui_lbl_sens_stat= nullptr;
static lv_obj_t *ui_tbl_files    = nullptr;
static lv_obj_t *ui_lbl_active   = nullptr;
static lv_obj_t *ui_lbl_fstatus  = nullptr;
static lv_obj_t *ui_lbl_setup_acc = nullptr;

/** Precisão estimada do vetor de rotação SH-2 (rad → graus no ecrã SETUP). */
static float       imu_rv_accuracy_deg = NAN;

// Tab order: 0=POINTS, 1=SENSOR, 2=FILES, 3=SETUP (must match lv_tabview_add_tab order).
static uint8_t     ui_active_tab    = 0;
static uint32_t    lzr_poll_gap_ms  = POLL_INTERVAL_MS;

// Forward declarations
static void sd_init();
static void sensor_init();
static void refresh_table();
static void refresh_sensor_display();
static void refresh_setup_display();
static void set_fstatus(const char *msg);
static void audio_init_hw();
static void play_boot_chime();
static void play_button_ack();
static void add_point(PtType type, bool sync_laser_before);

void IRAM_ATTR imuISR() { imu_irq = true; }

#ifdef ARDUINO_ARCH_ESP32
static void audio_init_hw()
{
    pinMode(AUDIO_EN_PIN, OUTPUT);
    digitalWrite(AUDIO_EN_PIN, LOW);
    ledcSetup(BUZZER_LEDC_CH, 1000, 10);
    ledcAttachPin(SPEAKER_PWM_PIN, BUZZER_LEDC_CH);
    ledcWrite(BUZZER_LEDC_CH, 0);
}

static void buzzer_note(unsigned freq_hz, unsigned dur_ms)
{
    if (freq_hz == 0) {
        delay(dur_ms);
        return;
    }
    ledcWriteTone(BUZZER_LEDC_CH, freq_hz);
    delay(dur_ms);
    ledcWriteTone(BUZZER_LEDC_CH, 0);
}

static void play_boot_chime()
{
    const uint16_t seq[] = { 523, 659, 784, 1047, 784, 1047, 1318 };
    const uint8_t  ms[]  = { 110, 110, 130, 150,  90, 100, 200 };
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++) {
        buzzer_note(seq[i], ms[i]);
        delay(28);
    }
}

static void play_button_ack()
{
    buzzer_note(1174, 42);
    delay(28);
    buzzer_note(1568, 62);
}
#else
static void audio_init_hw() {}
static void play_boot_chime() {}
static void play_button_ack() {}
#endif

// ── Display / touch ──────────────────────────────────────────────────────────
static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *c)
{
    uint32_t w = area->x2 - area->x1 + 1, h = area->y2 - area->y1 + 1;
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors(reinterpret_cast<uint16_t *>(c), w * h, true);
    tft.endWrite();
    lv_disp_flush_ready(drv);
}

static void touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static int16_t lx = 0, ly = 0;
    uint16_t tx = 0, ty = 0;
    /* Threshold 350 (< default 600) — XPT2046 / pressão Z. */
    if (tft.getTouch(&tx, &ty, 350)) {
        lx = (int16_t)tx;
        ly = (int16_t)ty;
        data->point.x = lx;
        data->point.y = ly;
        data->state   = LV_INDEV_STATE_PR;
    } else {
        data->point.x = lx;
        data->point.y = ly;
        data->state   = LV_INDEV_STATE_REL;
    }
    data->continue_reading = false;
}

// ── Sensor functions ─────────────────────────────────────────────────────────
#if LZR_SHARE_USB_UART
static HardwareSerial &lzr_port = Serial;
#else
static HardwareSerial   lzr_hw(LZR_UART_NUM);
static HardwareSerial  &lzr_port = lzr_hw;
#endif

static const uint8_t CMD_LASER_ON[] = {0xAA, 0x00, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x01, 0xC1};
static const uint8_t CMD_SINGLE[]   = {0xAA, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x21};
static const uint8_t CMD_CONTINUOUS[] = {0xAA, 0x00, 0x00, 0x21, 0x00, 0x01, 0x00, 0x00, 0x22};
static const uint8_t CMD_QUICK[]    = {0xAA, 0x00, 0x00, 0x22, 0x00, 0x01, 0x00, 0x00, 0x23};
static const uint8_t CMD_READ_RES[] = {0xAA, 0x80, 0x00, 0x22, 0xA2};

static float      lzr_last_m = NAN;
static uint8_t    lzr_parse_buf[16];
static int        lzr_parse_pos = 0;
static unsigned long lzr_last_byte_ms = 0, lzr_last_valid_ms = 0, lzr_last_recover_ms = 0;
static uint32_t   lzr_decode_tick = 0;
static unsigned long lzr_recover_count = 0;
static uint8_t    lzr_poll_state = 0;
static unsigned long lzr_poll_sent_ms = 0, lzr_next_poll_ms = 0;
static uint32_t   lzr_decode_at_send = 0;
static uint32_t   lzr_rx_bytes_total = 0;
/** After sensor_init(): periodic laser polls only on SENSOR tab (1); POINTS/FILES use one-shot on capture. */
static bool       lzr_post_init      = false;
static bool       lzr_one_shot_armed = false;
#if LZR_CONTINUOUS
static unsigned long lzr_keepalive_ms = 0;
#endif

static void lzr_sync_poll_gap_for_tab(uint8_t tab)
{
    if (tab == 1)
        lzr_poll_gap_ms = SENSOR_TAB_POLL_MS;
    else if (tab == 2)
        lzr_poll_gap_ms = FILES_TAB_POLL_MS;
    else
        lzr_poll_gap_ms = POLL_INTERVAL_MS;
}

static inline void lzr_ena_high()
{
    if (LZR_ENA_PIN >= 0) {
        pinMode(LZR_ENA_PIN, OUTPUT);
        digitalWrite(LZR_ENA_PIN, HIGH);
    }
}

static inline void lzr_ena_low()
{
    if (LZR_ENA_PIN >= 0) {
        pinMode(LZR_ENA_PIN, OUTPUT);
        digitalWrite(LZR_ENA_PIN, LOW);
    }
}

static void lzr_uart_drain()
{
    int n = 0;
    while (lzr_port.available() && n++ < 512) {
        (void)lzr_port.read();
        if ((n & 31) == 0) yield();
    }
}

static inline bool lzr_csum_ok(const uint8_t *f, int n)
{
    if (n < 3) return false;
    uint32_t s = 0;
    for (int i = 1; i < n - 1; i++) s += f[i];
    return ((uint8_t)s) == f[n - 1];
}

static inline uint32_t lzr_bcd32(const uint8_t *b)
{
    uint32_t v = 0;
    for (int i = 0; i < 4; i++)
        v = v * 100 + ((b[i] >> 4) & 0x0F) * 10 + (b[i] & 0x0F);
    return v;
}

static bool lzr_decode_frame13(const uint8_t *f)
{
    if (f[0] != 0xAA || !lzr_csum_ok(f, 13)) return false;
    uint8_t func = f[3];
    if (func != 0x20 && func != 0x21 && func != 0x22) return false;
    if (f[4] == 0x00 && f[5] == 0x04) {
        lzr_last_m = lzr_bcd32(&f[6]) / 1000.0f;
        return isfinite(lzr_last_m);
    }
    return false;
}

static bool lzr_try_decode13(const uint8_t *f)
{
    if (lzr_decode_frame13(f)) return true;
    if (f[0] == 0xAA && lzr_csum_ok(f, 13)) {
        uint8_t func = f[3];
        if (func == 0x20 || func == 0x21 || func == 0x22) {
            lzr_last_m = lzr_bcd32(&f[6]) / 1000.0f;
            return isfinite(lzr_last_m);
        }
    }
    return false;
}

static void lzr_parser_resync_shift()
{
    int syncIdx = -1;
    for (int i = 1; i < 13; i++) {
        if (lzr_parse_buf[i] == 0xAA) { syncIdx = i; break; }
    }
    if (syncIdx > 0) {
        int keep = 13 - syncIdx;
        memmove(lzr_parse_buf, lzr_parse_buf + syncIdx, (size_t)keep);
        lzr_parse_pos = keep;
    } else {
        lzr_parse_pos = 0;
    }
}

static void lzr_feed_byte(uint8_t x)
{
    /* Fluxo M01 polido (ESP32-C3 ref): só aceita 0xAA como primeiro byte do frame. */
    if (lzr_parse_pos == 0 && x != 0xAA)
        return;
    if (lzr_parse_pos >= (int)sizeof(lzr_parse_buf)) {
        lzr_parse_pos = 0;
        return;
    }
    lzr_parse_buf[lzr_parse_pos++] = x;
    if (lzr_parse_pos < 13) return;

    if (lzr_try_decode13(lzr_parse_buf)) {
        lzr_last_valid_ms = millis();
        lzr_decode_tick++;
        lzr_parse_pos = 0;
        return;
    }
    lzr_parser_resync_shift();
}

static void lzr_drain_stale_parser()
{
    if (lzr_parse_pos > 0 && (millis() - lzr_last_byte_ms) > PARSE_IDLE_MS)
        lzr_parse_pos = 0;
}

static void lzr_process_incoming()
{
    int n = 0;
    while (lzr_port.available() && n++ < RX_BURST_MAX) {
        uint8_t b = (uint8_t)lzr_port.read();
        lzr_rx_bytes_total++;
        lzr_last_byte_ms = millis();
        lzr_feed_byte(b);
    }
    lzr_drain_stale_parser();
}

static void lzr_send_continuous()
{
    lzr_port.write(CMD_CONTINUOUS, sizeof(CMD_CONTINUOUS));
    lzr_port.flush();
}

static void lzr_on()
{
    lzr_port.write(CMD_LASER_ON, sizeof(CMD_LASER_ON));
    lzr_port.flush();
}

static void lzr_uart_begin()
{
#if LZR_SHARE_USB_UART
    Serial.flush();
    Serial.end();
    delay(50);
#endif
    lzr_port.setRxBufferSize(1024);
    lzr_port.begin(LZR_BAUD, SERIAL_8N1, LZR_PIN_RX, LZR_PIN_TX);
#if LZR_SHARE_USB_UART
    Serial.setDebugOutput(false);
#endif
}

#if UART_HARD_REINIT
static void lzr_uart_reinit()
{
    lzr_port.end();
    delay(30);
    lzr_uart_begin();
}
#endif

static void lzr_recover(bool power_cycle_ena)
{
    if (power_cycle_ena && LZR_ENA_PIN >= 0) {
        lzr_ena_low();
        delay(120);
        lzr_ena_high();
        delay(200);
    }
    lzr_uart_drain();
#if UART_HARD_REINIT
    lzr_uart_reinit();
#endif
    lzr_parse_pos = 0;
    lzr_last_m = NAN;
    lzr_on();
    delay(80);
#if LZR_CONTINUOUS
    lzr_send_continuous();
    lzr_keepalive_ms = millis();
#else
    lzr_next_poll_ms = millis();
    lzr_poll_state = 0;
#endif
    unsigned long t = millis();
    lzr_last_recover_ms = t;
    lzr_last_valid_ms = t;
}

static void lzr_poll_send_measure()
{
    lzr_uart_drain();
    lzr_parse_pos = 0;
    lzr_decode_at_send = lzr_decode_tick;
    lzr_on();
    delay(25);
    lzr_port.write(CMD_SINGLE, sizeof(CMD_SINGLE));
    lzr_port.flush();
    lzr_poll_sent_ms = millis();
    lzr_poll_state = 1;
}

static void lzr_poll_fallback()
{
    lzr_port.write(CMD_QUICK, sizeof(CMD_QUICK));
    lzr_port.flush();
    delay(5);
    lzr_port.write(CMD_READ_RES, sizeof(CMD_READ_RES));
    lzr_port.flush();
}

#if !LZR_CONTINUOUS
static bool lzr_periodic_poll_allowed()
{
    if (!lzr_post_init) return true;
    return ui_active_tab == 1;
}
#endif

static void lzr_poll_tick(unsigned long now)
{
#if LZR_CONTINUOUS
    (void)now;
    return;
#else
    if (lzr_poll_state == 0) {
        if ((long)(now - lzr_next_poll_ms) < 0) return;
        if (!lzr_periodic_poll_allowed() && !lzr_one_shot_armed)
            return;
        if (lzr_one_shot_armed)
            lzr_one_shot_armed = false;
        lzr_poll_send_measure();
        return;
    }
    if (lzr_decode_tick != lzr_decode_at_send) {
        lzr_poll_state = 0;
        if (lzr_periodic_poll_allowed())
            lzr_next_poll_ms = now + lzr_poll_gap_ms;
        else
            lzr_next_poll_ms = now;
        return;
    }
    if ((now - lzr_poll_sent_ms) > POLL_TIMEOUT_MS) {
        lzr_poll_fallback();
        if ((now - lzr_last_recover_ms) >= RECOVER_MIN_MS) {
            lzr_recover_count++;
            lzr_recover((LZR_ENA_PIN >= 0) && ((lzr_recover_count % 4UL) == 0UL));
        } else {
            lzr_uart_drain();
            lzr_parse_pos = 0;
            lzr_on();
            delay(25);
            lzr_port.write(CMD_QUICK, sizeof(CMD_QUICK));
            lzr_port.flush();
        }
        lzr_poll_state = 0;
        lzr_next_poll_ms = now + 250UL;
    }
#endif
}

static void lzr_apply_to_globals(unsigned long now)
{
    if ((now - lzr_last_valid_ms) <= STALE_MS && isfinite(lzr_last_m)) {
        tof_dist_mm = (uint32_t)lrintf(fmaxf(0.f, lzr_last_m) * 1000.0f);
        tof_ok = true;
    } else {
        tof_ok = false;
    }
}

/** Stale-UART recover: only while SENSOR tab polls, or during boot (before lzr_post_init). */
#if !LZR_CONTINUOUS
static bool lzr_stale_recover_allowed(unsigned long now)
{
    (void)now;
    if (!lzr_post_init) return true;
    return ui_active_tab == 1;
}
#endif

static void lzr_loop_tick(unsigned long now)
{
    lzr_process_incoming();
#if LZR_CONTINUOUS
    if ((now - lzr_keepalive_ms) >= LZR_KEEPALIVE_MS) {
        lzr_keepalive_ms = now;
        lzr_on();
        delay(25);
        lzr_send_continuous();
    }
#else
    lzr_poll_tick(now);
#endif
#if LZR_CONTINUOUS
    if ((now - lzr_last_valid_ms) > STALE_MS && (now - lzr_last_recover_ms) >= RECOVER_MIN_MS) {
        lzr_recover_count++;
        bool pwr = (LZR_ENA_PIN >= 0) && ((lzr_recover_count % 3UL) == 0UL);
        lzr_recover(pwr);
    }
#else
    if (lzr_stale_recover_allowed(now) && (now - lzr_last_valid_ms) > STALE_MS
        && (now - lzr_last_recover_ms) >= RECOVER_MIN_MS) {
        lzr_recover_count++;
        bool pwr = (LZR_ENA_PIN >= 0) && ((lzr_recover_count % 3UL) == 0UL);
        lzr_recover(pwr);
    }
#endif
    lzr_apply_to_globals(now);
}

static void lzr_init()
{
    lzr_ena_high();
    lzr_uart_begin();
    lzr_recover(false);
#if !LZR_SHARE_USB_UART
    DBG_PRINT("[LASER UART] UART%d RX=GPIO%d TX=GPIO%d %lu baud\n",
              LZR_UART_NUM, LZR_PIN_RX, LZR_PIN_TX, (unsigned long)LZR_BAUD);
#endif
    unsigned long t0 = millis();
    while ((millis() - t0) < 4000U && !tof_ok) {
        lzr_loop_tick(millis());
        delay(5);
    }
}

static float norm_deg360(float d)
{
    float x = fmodf(d + 360.f, 360.f);
    return (x >= 0.f) ? x : x + 360.f;
}

static void imu_vec_body_to_world(float qw, float qx, float qy, float qz,
                                  float bx, float by, float bz,
                                  float *vx, float *vy, float *vz)
{
    float xx = qx * qx, yy = qy * qy, zz = qz * qz;
    float xy = qx * qy, xz = qx * qz, yz = qy * qz;
    float wxq = qw * qx, wyq = qw * qy, wzq = qw * qz;
    *vx = (1.f - 2.f * (yy + zz)) * bx + 2.f * (xy - wzq) * by + 2.f * (xz + wyq) * bz;
    *vy = 2.f * (xy + wzq) * bx + (1.f - 2.f * (xx + zz)) * by + 2.f * (yz - wxq) * bz;
    *vz = 2.f * (xz - wyq) * bx + 2.f * (yz + wxq) * by + (1.f - 2.f * (xx + yy)) * bz;
}

/** Atualiza euler (R/P/Y internos) + azimute/inclinação do feixe laser no frame da fusion IMU. */
static void imu_update_angles_from_quat(float qw, float qx, float qy, float qz)
{
    const float r2d = 180.f / (float)M_PI;
    imu_yaw   = atan2f(2.f * (qw * qz + qx * qy), 1.f - 2.f * (qy * qy + qz * qz)) * r2d;
    imu_pitch = asinf(fmaxf(-1.f, fminf(1.f, 2.f * (qw * qy - qz * qx)))) * r2d;
    imu_roll  = atan2f(2.f * (qw * qx + qy * qz), 1.f - 2.f * (qx * qx + qy * qy)) * r2d;

    float bx = IMU_LASER_AXIS_BX, by = IMU_LASER_AXIS_BY, bz = IMU_LASER_AXIS_BZ;
    float n = sqrtf(bx * bx + by * by + bz * bz);
    if (n > 1e-6f) {
        bx /= n;
        by /= n;
        bz /= n;
    }

    float wx, wy, wz;
    imu_vec_body_to_world(qw, qx, qy, qz, bx, by, bz, &wx, &wy, &wz);
    float rh = sqrtf(wx * wx + wy * wy);
    imu_inclination_deg = atan2f(wz, rh) * r2d;
    imu_azimuth_deg = norm_deg360(atan2f(wy, wx) * r2d);
}

static void poll_imu()
{
    if (!imu_ok) return;
    if (bno08x.wasReset()) {
        bno08x.enableReport(SH2_ROTATION_VECTOR, 20000);
        bno08x.enableReport(SH2_ACCELEROMETER,   50000);
        delay(100);
    }
    while (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            float qw = sensorValue.un.rotationVector.real;
            float qx = sensorValue.un.rotationVector.i;
            float qy = sensorValue.un.rotationVector.j;
            float qz = sensorValue.un.rotationVector.k;
            float acc_r = sensorValue.un.rotationVector.accuracy;
            if (isfinite(acc_r) && acc_r >= 0.f)
                imu_rv_accuracy_deg = acc_r * (180.f / (float)M_PI);
            else
                imu_rv_accuracy_deg = NAN;
            imu_update_angles_from_quat(qw, qx, qy, qz);
        }
    }
}

/** BLOCKING: request one laser reading before storing a point (POINTS/FILES); SENSOR tab uses continuous polling. */
static void lzr_sync_for_capture()
{
#if LZR_CONTINUOUS
    return;
#else
    if (!lzr_post_init) return;
    if (ui_active_tab == 1) return;

    const unsigned long timeout_ms = POLL_TIMEOUT_MS + 800UL;
    lzr_one_shot_armed = true;
    lzr_next_poll_ms = millis();

    unsigned long t0 = millis();
    uint32_t decode0 = lzr_decode_tick;
    while ((millis() - t0) < timeout_ms) {
        lzr_loop_tick(millis());
        poll_imu();
        if (lzr_decode_tick != decode0 && lzr_poll_state == 0)
            break;
        delay(2);
    }
#endif
}

/** Enquanto o botão está pressionado (POINTS/FILES): solicita nova medição laser sem gravar ponto. */
static void lzr_service_hold_capture(bool btn_held)
{
#if LZR_CONTINUOUS
    (void)btn_held;
#else
    if (!btn_held || !lzr_post_init || ui_active_tab == 1)
        return;
    if (lzr_poll_state != 0)
        return;
    lzr_one_shot_armed = true;
    lzr_next_poll_ms = millis();
#endif
}

static void read_battery()
{
    int raw = analogRead(BAT_ADC_PIN);
    // 12-bit ADC, 3.3V ref, typical voltage divider: adjust as needed
    float v = raw * 3.3f / 4095.0f * 2.0f;   // ×2 for divider
    bat_pct = constrain((int)((v - 3.0f) / (4.2f - 3.0f) * 100.0f), 0, 100);
}

// ── Table draw / click ───────────────────────────────────────────────────────
static void pts_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (dsc->part != LV_PART_ITEMS) return;
    uint32_t row = dsc->id / lv_table_get_col_cnt(obj);
    uint32_t col = dsc->id % lv_table_get_col_cnt(obj);

    if (row == 0) {
        dsc->rect_dsc->bg_color = lv_color_hex(C_TBL_HDR);
        dsc->rect_dsc->bg_opa   = LV_OPA_COVER;
        dsc->label_dsc->color   = lv_color_hex(C_WHITE);
        dsc->label_dsc->align   = LV_TEXT_ALIGN_CENTER;
        dsc->label_dsc->font    = &lv_font_montserrat_14;
    } else {
        /* Linha 1 = medição mais recente (pts[pt_count-1]). */
        int idx = pt_count - (int)row;
        bool selected = (idx == sel_row);
        bool is_nav   = (idx >= 0 && idx < pt_count && pts[idx].type == PT_NAV);
        if (selected)
            dsc->rect_dsc->bg_color = lv_color_hex(C_ROW_SEL);
        else if (is_nav)
            dsc->rect_dsc->bg_color = lv_color_hex(C_ROW_NAV);
        else
            dsc->rect_dsc->bg_color = (row%2==0) ? lv_color_hex(C_ROW_EVEN)
                                                  : lv_color_hex(C_ROW_ODD);
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;
        if (col == 2 && idx >= 0 && idx < pt_count && !pts[idx].laser_ok) {
            dsc->label_dsc->color = lv_color_hex(C_BAT_LOW);
            dsc->label_dsc->align = LV_TEXT_ALIGN_CENTER;
        } else if (col == 0) {
            dsc->label_dsc->color = lv_color_hex(C_REF_S);
            dsc->label_dsc->align = LV_TEXT_ALIGN_CENTER;
        } else if (col == 1 || col == 3 || col == 4) {
            dsc->label_dsc->color = lv_color_hex(C_TEXT);
            dsc->label_dsc->align = LV_TEXT_ALIGN_RIGHT;
        } else {
            dsc->label_dsc->color = lv_color_hex(C_TEXT);
            dsc->label_dsc->align = LV_TEXT_ALIGN_CENTER;
        }
        dsc->label_dsc->font = &lv_font_montserrat_14;
    }
    dsc->rect_dsc->border_color = lv_color_hex(C_BORDER);
    dsc->rect_dsc->border_width = 1;
}

static void pts_click_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    uint16_t row, col;
    lv_table_get_selected_cell(obj, &row, &col);
    if (row > 0 && (int)row <= pt_count) {
        int idx = pt_count - (int)row;
        sel_row = (sel_row == idx) ? -1 : idx;
        lv_obj_invalidate(obj);
    }
}

// ── Refresh ──────────────────────────────────────────────────────────────────
static void refresh_table()
{
    if (!ui_tbl_pts) return;
    lv_obj_t *t = ui_tbl_pts;
    lv_table_set_row_cnt(t, (uint16_t)(pt_count + 1));
    lv_table_set_cell_value(t, 0, 0, "Ref#");
    lv_table_set_cell_value(t, 0, 1, "Dist (m)");
    lv_table_set_cell_value(t, 0, 2, "E");
    lv_table_set_cell_value(t, 0, 3, "Azm \xC2\xB0");
    lv_table_set_cell_value(t, 0, 4, "Inc \xC2\xB0");

    char buf[24];
    /* Linha 1 = mais recente (pts[pt_count-1]); última linha = mais antiga (pts[0]). */
    for (int row = 1; row <= pt_count; row++) {
        int i = pt_count - row;
        snprintf(buf, sizeof(buf), "%lu", (unsigned long)pts[i].id);
        lv_table_set_cell_value(t, row, 0, buf);
        snprintf(buf, sizeof(buf), "%.3f", pts[i].dist);
        lv_table_set_cell_value(t, row, 1, buf);
        lv_table_set_cell_value(t, row, 2, pts[i].laser_ok ? "" : "E");
        snprintf(buf, sizeof(buf), "%.1f", pts[i].yaw);
        lv_table_set_cell_value(t, row, 3, buf);
        snprintf(buf, sizeof(buf), "%.1f", pts[i].pitch);
        lv_table_set_cell_value(t, row, 4, buf);
    }
    snprintf(buf, sizeof(buf), "PTS: %d", pt_count);
    if (ui_lbl_count) lv_label_set_text(ui_lbl_count, buf);
}

static void refresh_sensor_display()
{
    char buf[128];
    if (ui_lbl_tof_val) {
        if (tof_ok) {
            snprintf(buf, sizeof(buf), "%.3f m  (%lu mm)",
                     tof_dist_mm / 1000.0f, (unsigned long)tof_dist_mm);
        } else {
            snprintf(buf, sizeof(buf), "sem leitura");
        }
        lv_label_set_text(ui_lbl_tof_val, buf);
    }
    if (ui_lbl_imu_val) {
        snprintf(buf, sizeof(buf), "Az: %.1f\xC2\xB0   Inc: %.1f\xC2\xB0   Rol: %.1f\xC2\xB0",
                 imu_azimuth_deg, imu_inclination_deg, imu_roll);
        lv_label_set_text(ui_lbl_imu_val, buf);
    }
    if (ui_lbl_sens_stat) {
        snprintf(buf, sizeof(buf), "LASER: %s   IMU: %s",
                 tof_ok ? "OK" : "FAIL", imu_ok ? "OK" : "FAIL");
        lv_label_set_text(ui_lbl_sens_stat, buf);
    }
}

static void refresh_setup_display()
{
    if (!ui_lbl_setup_acc) return;
    char buf[192];
    if (!imu_ok) {
        snprintf(buf, sizeof(buf), "IMU: offline");
    } else if (isfinite(imu_rv_accuracy_deg)) {
        snprintf(buf, sizeof(buf), "Precisão estimada (heading): ±%.1f°\n"
            "(menor = melhor; movimento lento figura-8 melhora em zonas más)",
            imu_rv_accuracy_deg);
    } else {
        snprintf(buf, sizeof(buf), "Precisão estimada: calibrando…\n"
            "Mova o dispositivo devagar em todas as direções.");
    }
    lv_label_set_text(ui_lbl_setup_acc, buf);
}

static void update_active_lbl()
{
    if (!ui_lbl_active) return;
    const char *n = active_csv; if (n[0]=='/') n++;
    char buf[64];
    snprintf(buf, sizeof(buf), "Active: %s  (%d pts)", n, pt_count);
    lv_label_set_text(ui_lbl_active, buf);
}

static void fmt_bric_timestamp(uint32_t posix_sec, char *buf, size_t len)
{
    struct tm tm_out;
    time_t tt = (time_t)posix_sec;
#ifdef ARDUINO_ARCH_ESP32
    localtime_r(&tt, &tm_out);
#else
    gmtime_r(&tt, &tm_out);
#endif
    snprintf(buf, len, "%04d.%02d.%02d@%02d:%02d:%02d",
             tm_out.tm_year + 1900, tm_out.tm_mon + 1, tm_out.tm_mday,
             tm_out.tm_hour, tm_out.tm_min, tm_out.tm_sec);
}

static void append_point_csv_br(Print &pr, const MeasPoint &p)
{
    char ts[28];
    fmt_bric_timestamp(p.posix_sec, ts, sizeof(ts));
    float az = norm_deg360(p.yaw);
    float rol = norm_deg360(p.roll);
    const char *mtype = (p.type == PT_NAV) ? "Navigation" : "Regular";
    const char *elog = p.error_log[0] ? p.error_log : "";
    pr.printf("%s,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%s\n",
              ts,
              (unsigned long)p.posix_sec,
              (unsigned long)p.id,
              (double)p.dist, (double)az, (double)p.pitch, (double)p.dip_deg, (double)rol,
              (double)p.temp_c, mtype, elog);
}

static bool sniff_br_csv_row(const char *s)
{
    int dots = 0;
    bool atseen = false;
    for (; *s && *s != ','; s++) {
        if (*s == '.') dots++;
        if (*s == '@') atseen = true;
    }
    return dots >= 2 && atseen;
}

static bool parse_legacy_csv_row(char *buf, MeasPoint &p)
{
    memset(&p, 0, sizeof(p));
    char *tok = strtok(buf, ",");
    if (!tok) return false;
    p.id = (uint32_t)strtoul(tok, nullptr, 10);
    tok = strtok(nullptr, ",");
    if (!tok) return false;
    p.type = (tok[0] == 'N' || tok[0] == 'n') ? PT_NAV : PT_SAMPLE;
    tok = strtok(nullptr, ","); if (tok) p.dist = strtof(tok, nullptr);
    tok = strtok(nullptr, ","); if (tok) p.roll = strtof(tok, nullptr);
    tok = strtok(nullptr, ","); if (tok) p.pitch = strtof(tok, nullptr);
    tok = strtok(nullptr, ","); if (tok) p.yaw = strtof(tok, nullptr);
    tok = strtok(nullptr, ",\r\n");
    if (tok) p.laser_ok = (atoi(tok) != 0); else p.laser_ok = true;
    p.posix_sec = POSIX_FALLBACK_ANCHOR_SEC + p.id;
    p.dip_deg = EXPORT_BRIC_DIP_DEG;
    p.temp_c = 22.f;
    p.error_log[0] = '\0';
    if (!p.laser_ok)
        strlcpy(p.error_log, "Laser:N/A", sizeof(p.error_log));
    return true;
}

static bool parse_br_csv_row(char *work, MeasPoint &p)
{
    memset(&p, 0, sizeof(p));
    char *cols[14];
    int n = 0;
    cols[n++] = work;
    for (char *q = work; *q && n < 14; q++) {
        if (*q == ',') {
            *q = '\0';
            cols[n++] = q + 1;
        }
    }
    if (n < 11) return false;
    p.posix_sec = (uint32_t)strtoul(cols[1], nullptr, 10);
    p.id = (uint32_t)strtoul(cols[2], nullptr, 10);
    p.dist = strtof(cols[3], nullptr);
    p.yaw = strtof(cols[4], nullptr);
    p.pitch = strtof(cols[5], nullptr);
    p.dip_deg = strtof(cols[6], nullptr);
    p.roll = strtof(cols[7], nullptr);
    p.temp_c = strtof(cols[8], nullptr);
    while (cols[9][0] == ' ') cols[9]++;
    if (cols[9][0] == '\0')
        return false;
    p.type = (cols[9][0] == 'N' || cols[9][0] == 'n') ? PT_NAV : PT_SAMPLE;
    p.error_log[0] = '\0';
    if (n >= 11) {
        char *dst = p.error_log;
        size_t left = sizeof(p.error_log);
        for (int i = 10; i < n && left > 1; i++) {
            if (i > 10 && left > 2) {
                *dst++ = ',';
                left--;
            }
            size_t L = strlen(cols[i]);
            if (L >= left)
                L = left - 1;
            memcpy(dst, cols[i], L);
            dst += L;
            left -= L;
        }
        *dst = '\0';
    }
    p.laser_ok = true;
    if (strstr(p.error_log, "Laser"))
        p.laser_ok = false;
    return true;
}

// ── SD save / load ───────────────────────────────────────────────────────────
static void save_csv()
{
    if (!sd_ready) return;
    if (SD.exists(active_csv)) SD.remove(active_csv);
    File f = SD.open(active_csv, FILE_WRITE);
    if (!f) return;
    f.println(BRIC_CSV_HEADER);
    for (int i = 0; i < pt_count; i++)
        append_point_csv_br(f, pts[i]);
    f.close();
    DBG_PRINT("[SD] Saved %d pts to %s\n", pt_count, active_csv);
}

static void load_csv()
{
    if (!sd_ready) return;
    File f = SD.open(active_csv, FILE_READ);
    if (!f) return;
    pt_count = 0;
    next_id = 1;
    while (f.available() && pt_count < MAX_PTS) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;
        if (line.startsWith("#GPS")) continue;
        if (line.startsWith("#BRICS5_MM1")) continue;
        if (line.startsWith("Time-Stamp")) continue;
        if (line.startsWith("ID,")) continue;

        char buf[384];
        line.toCharArray(buf, sizeof(buf));
        MeasPoint &p = pts[pt_count];
        bool ok;
        if (sniff_br_csv_row(buf))
            ok = parse_br_csv_row(buf, p);
        else
            ok = parse_legacy_csv_row(buf, p);
        if (!ok) continue;
        if (p.id >= next_id) next_id = p.id + 1;
        pt_count++;
    }
    f.close();
    sel_row = -1;
    DBG_PRINT("[SD] Loaded %d pts from %s\n", pt_count, active_csv);
}

// ── File browser ─────────────────────────────────────────────────────────────
static void scan_csv_files()
{
    file_count = 0;
    if (!sd_ready) return;
    File root = SD.open("/");
    if (!root || !root.isDirectory()) return;
    File entry;
    while ((entry = root.openNextFile()) && file_count < MAX_FILES) {
        if (!entry.isDirectory()) {
            const char *n = entry.name();
            int len = strlen(n);
            const char *base = (n[0]=='/')? n+1 : n;
            if (len >= 4) {
                const char *ext = n+len-4;
                if (ext[0]=='.' && (ext[1]=='c'||ext[1]=='C') &&
                    (ext[2]=='s'||ext[2]=='S') && (ext[3]=='v'||ext[3]=='V')) {
                    snprintf(file_names[file_count], sizeof(file_names[0]), "/%s", base);
                    file_count++;
                }
            }
        }
        entry.close();
    }
    root.close();
}

static void refresh_file_list()
{
    scan_csv_files();
    if (!ui_tbl_files) return;
    lv_table_set_row_cnt(ui_tbl_files, (uint16_t)(file_count+1));
    lv_table_set_cell_value(ui_tbl_files, 0, 0, "File");
    lv_table_set_cell_value(ui_tbl_files, 0, 1, "Size");
    for (int i = 0; i < file_count; i++) {
        const char *n = file_names[i]; if (n[0]=='/') n++;
        lv_table_set_cell_value(ui_tbl_files, i+1, 0, n);
        File f = SD.open(file_names[i], FILE_READ);
        if (f) {
            char sz[12]; uint32_t b = f.size();
            if (b<1024) snprintf(sz,sizeof(sz),"%luB",b);
            else        snprintf(sz,sizeof(sz),"%lukB",b/1024);
            f.close();
            lv_table_set_cell_value(ui_tbl_files, i+1, 1, sz);
        } else lv_table_set_cell_value(ui_tbl_files, i+1, 1, "?");
    }
    lv_obj_invalidate(ui_tbl_files);
}

static void file_draw_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (dsc->part != LV_PART_ITEMS) return;
    uint32_t row = dsc->id / lv_table_get_col_cnt(obj);
    uint32_t col = dsc->id % lv_table_get_col_cnt(obj);
    if (row == 0) {
        dsc->rect_dsc->bg_color = lv_color_hex(C_TBL_HDR);
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;
        dsc->label_dsc->color = lv_color_hex(C_WHITE);
        dsc->label_dsc->align = LV_TEXT_ALIGN_CENTER;
        dsc->label_dsc->font  = &lv_font_montserrat_14;
    } else {
        int idx = (int)row-1;
        bool act = idx<file_count && strcmp(file_names[idx],active_csv)==0;
        bool sel = idx==file_sel;
        if (sel)      dsc->rect_dsc->bg_color = lv_color_hex(C_ROW_SEL);
        else if (act) dsc->rect_dsc->bg_color = lv_color_hex(C_FILE_ACT);
        else          dsc->rect_dsc->bg_color = (row%2==0)?lv_color_hex(C_ROW_EVEN):lv_color_hex(C_ROW_ODD);
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;
        dsc->label_dsc->color = (col==0)?lv_color_hex(C_REF_S):lv_color_hex(C_TEXT);
        dsc->label_dsc->align = (col==0)?LV_TEXT_ALIGN_LEFT:LV_TEXT_ALIGN_RIGHT;
        dsc->label_dsc->font  = &lv_font_montserrat_14;
    }
    dsc->rect_dsc->border_color = lv_color_hex(C_BORDER);
    dsc->rect_dsc->border_width = 1;
}

static void file_click_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    uint16_t row, col;
    lv_table_get_selected_cell(obj, &row, &col);
    if (row>0 && (int)(row-1)<file_count) {
        int idx = (int)row-1;
        file_sel = (file_sel==idx)?-1:idx;
        lv_obj_invalidate(obj);
    }
}

// ── Button callbacks ─────────────────────────────────────────────────────────
static void set_fstatus(const char *msg)
{
    if (ui_lbl_fstatus) lv_label_set_text(ui_lbl_fstatus, msg);
}

static void add_point(PtType type, bool sync_laser_before)
{
    if (pt_count >= MAX_PTS) { set_fstatus(LV_SYMBOL_WARNING " Full!"); return; }
    if (sync_laser_before)
        lzr_sync_for_capture();
    MeasPoint &p = pts[pt_count];
    p.id    = next_id++;
    p.type  = type;
    p.dist  = tof_dist_mm / 1000.0f;
    p.roll  = imu_roll;
    p.pitch = imu_inclination_deg;
    p.yaw   = imu_azimuth_deg;
    p.laser_ok = tof_ok;
    time_t tv = time(nullptr);
    if (tv > (time_t)1577836800L)
        p.posix_sec = (uint32_t)tv;
    else
        p.posix_sec = POSIX_FALLBACK_ANCHOR_SEC + (uint32_t)(millis() / 1000UL);
    p.dip_deg = EXPORT_BRIC_DIP_DEG;
#ifdef ARDUINO_ARCH_ESP32
    p.temp_c = (float)temperatureRead();
#else
    p.temp_c = 22.f;
#endif
    p.error_log[0] = '\0';
    if (!tof_ok)
        strlcpy(p.error_log, "Laser:N/A", sizeof(p.error_log));
    pt_count++;
    sel_row = -1;
    refresh_table();
    char buf[40];
    snprintf(buf, sizeof(buf), LV_SYMBOL_OK " %s Ref#%u  %.3fm %s",
             type==PT_NAV?"Nav":"Shot", p.id, p.dist, p.laser_ok?"":"E");
    set_fstatus(buf);
}

static void btn_del_cb(lv_event_t *e)
{
    if (sel_row < 0 || sel_row >= pt_count) {
        set_fstatus(LV_SYMBOL_WARNING " Select a row!"); return;
    }
    uint32_t did = pts[sel_row].id;
    for (int i = sel_row; i < pt_count-1; i++) pts[i] = pts[i+1];
    pt_count--;
    sel_row = -1;
    refresh_table();
    char buf[32]; snprintf(buf,sizeof(buf), LV_SYMBOL_TRASH " Del #%u", did);
    set_fstatus(buf);
}

static void btn_save_cb(lv_event_t *e)
{
    save_csv();
    char buf[32]; snprintf(buf,sizeof(buf), LV_SYMBOL_OK " Saved %d pts", pt_count);
    set_fstatus(buf);
}

static void btn_clear_cb(lv_event_t *e)
{
    pt_count = 0; sel_row = -1; next_id = 1;
    refresh_table();
    set_fstatus(LV_SYMBOL_TRASH " Cleared");
}

static void btn_load_cb(lv_event_t *e)
{
    if (!sd_ready) { set_fstatus(LV_SYMBOL_WARNING " No SD!"); return; }
    pt_count = 0; sel_row = -1; next_id = 1;
    load_csv();
    refresh_table();
    char buf[32]; snprintf(buf,sizeof(buf), LV_SYMBOL_DOWNLOAD " Loaded %d pts", pt_count);
    set_fstatus(buf);
}

// File CRUD
static void btn_new_file_cb(lv_event_t *e)
{
    if (!sd_ready) { set_fstatus(LV_SYMBOL_WARNING " No SD!"); return; }
    char path[32]; int n=1;
    do { snprintf(path,sizeof(path),"/brics%02d.csv",n++); }
    while (SD.exists(path) && n<100);
    File f=SD.open(path,FILE_WRITE);
    if (f) { f.println(BRIC_CSV_HEADER); f.close(); }
    strlcpy(active_csv,path,sizeof(active_csv));
    pt_count=0; sel_row=-1; next_id=1;
    refresh_table(); refresh_file_list(); update_active_lbl();
    char buf[48]; snprintf(buf,sizeof(buf), LV_SYMBOL_OK " Created %s", path+1);
    set_fstatus(buf);
}

static void btn_use_file_cb(lv_event_t *e)
{
    if (file_sel<0||file_sel>=file_count) { set_fstatus(LV_SYMBOL_WARNING " Select file!"); return; }
    strlcpy(active_csv,file_names[file_sel],sizeof(active_csv));
    pt_count=0; sel_row=-1; next_id=1;
    load_csv(); refresh_table();
    lv_obj_invalidate(ui_tbl_files); update_active_lbl();
    char buf[48]; snprintf(buf,sizeof(buf), LV_SYMBOL_OK " %s (%d)", active_csv+1, pt_count);
    set_fstatus(buf);
}

static void btn_del_file_cb(lv_event_t *e)
{
    if (file_sel<0||file_sel>=file_count) { set_fstatus(LV_SYMBOL_WARNING " Select file!"); return; }
    char path[32]; strlcpy(path,file_names[file_sel],sizeof(path));
    bool was_act = strcmp(path,active_csv)==0;
    SD.remove(path); file_sel=-1; refresh_file_list();
    if (was_act) {
        if (file_count>0) strlcpy(active_csv,file_names[0],sizeof(active_csv));
        else { strlcpy(active_csv,"/brics5_mm1.csv",sizeof(active_csv));
               File f=SD.open(active_csv,FILE_WRITE);
               if(f){f.println(BRIC_CSV_HEADER);f.close();}
               refresh_file_list(); }
        pt_count=0;sel_row=-1;next_id=1; load_csv(); refresh_table(); update_active_lbl();
    }
    char buf[48]; snprintf(buf,sizeof(buf), LV_SYMBOL_TRASH " Deleted %s", path+1);
    set_fstatus(buf);
}

static void tabview_changed_cb(lv_event_t *e)
{
    lv_obj_t *tv = lv_event_get_target(e);
    uint16_t tab = lv_tabview_get_tab_act(tv);
    ui_active_tab = (uint8_t)tab;
    lzr_sync_poll_gap_for_tab(ui_active_tab);
    lzr_next_poll_ms = millis();
    if (tab == 1) refresh_sensor_display();
    if (tab == 2) { refresh_file_list(); update_active_lbl(); }
    if (tab == 3) refresh_setup_display();
}

// ── Status / BT / periodic ───────────────────────────────────────────────────
static void bt_send_sd_file(const char *path_raw)
{
    char path[96];
    strlcpy(path, path_raw ? path_raw : "", sizeof(path));
    char *s = path;
    while (*s == ' ' || *s == '\t')
        s++;
    if (s != path)
        memmove(path, s, strlen(s) + 1);
    if (path[0] != '/') {
        char tmp[96];
        snprintf(tmp, sizeof(tmp), "/%s", path);
        strlcpy(path, tmp, sizeof(path));
    }
    if (!sd_ready) {
        SerialBT.println("ERR,NO_SD");
        return;
    }
    File f = SD.open(path, FILE_READ);
    if (!f) {
        SerialBT.printf("ERR,NO_FILE,%s\n", path);
        return;
    }
    SerialBT.printf("BEGIN,%s,%lu\n", path, (unsigned long)f.size());
    uint8_t chunk[160];
    while (f.available()) {
        size_t n = f.read(chunk, sizeof(chunk));
        if (n > 0) {
            SerialBT.write(chunk, n);
            delay(4);
            yield();
        }
    }
    SerialBT.print("\r\nEND_FILE\r\n");
    f.close();
}

static void update_status()
{
    bt_conn = SerialBT.connected();
    lv_label_set_text(ui_lbl_bt, bt_conn ? LV_SYMBOL_BLUETOOTH " BT" : LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(ui_lbl_bt, lv_color_hex(bt_conn?C_BT_ON:C_BT_OFF), 0);
    lv_label_set_text(ui_lbl_sd, sd_ready ? LV_SYMBOL_SD_CARD " SD" : LV_SYMBOL_SD_CARD);
    lv_obj_set_style_text_color(ui_lbl_sd, lv_color_hex(sd_ready?C_SD_ON:C_SD_OFF), 0);

    read_battery();
    char bb[16];
    snprintf(bb, sizeof(bb), LV_SYMBOL_BATTERY_FULL " %d%%", bat_pct);
    lv_label_set_text(ui_lbl_bat, bb);
    lv_obj_set_style_text_color(ui_lbl_bat,
        lv_color_hex(bat_pct > 20 ? C_BAT_OK : C_BAT_LOW), 0);

    uint32_t s = millis()/1000;
    char tb[12]; snprintf(tb,sizeof(tb),"%02lu:%02lu:%02lu",(s/3600)%24,(s/60)%60,s%60);
    lv_label_set_text(ui_lbl_time, tb);
}

static void handle_bt_cmd(const String &raw)
{
    String cmd = raw;
    cmd.trim();
    if (cmd.length() > 0 && cmd.charAt(cmd.length() - 1) == '\r')
        cmd.remove(cmd.length() - 1);

    String cup = cmd;
    cup.toUpperCase();
    if (cup.startsWith("FILE_SEND")) {
        String rest = cmd.substring(9);
        rest.trim();
        if (rest.startsWith(","))
            rest.remove(0, 1);
        rest.trim();
        if (rest.length() == 0) {
            SerialBT.println("ERR,USAGE,FILE_SEND,/nome.csv");
            return;
        }
        bt_send_sd_file(rest.c_str());
        return;
    }
    if (cmd.equalsIgnoreCase("FILES")) {
        scan_csv_files();
        SerialBT.printf("BEGIN,FILES,%d\n", file_count);
        for (int i = 0; i < file_count; i++)
            SerialBT.println(file_names[i]);
        SerialBT.println("END,FILES");
        return;
    }
    if (cmd.equalsIgnoreCase("MEAS")) {
        add_point(PT_SAMPLE, true);
        SerialBT.printf("OK,%d\n", pt_count);
        return;
    }
    if (cmd.equalsIgnoreCase("CLEAR")) {
        pt_count = 0;
        sel_row = -1;
        next_id = 1;
        refresh_table();
        SerialBT.println("OK");
        return;
    }
    if (cmd.equalsIgnoreCase("LIST") || cmd.equalsIgnoreCase("EXPORT")) {
        SerialBT.println(BRIC_CSV_HEADER);
        for (int i = 0; i < pt_count; i++)
            append_point_csv_br(SerialBT, pts[i]);
        SerialBT.println("END");
        return;
    }
}

static void periodic_cb(lv_timer_t*t)
{
    update_status();
    static String bt_line;
    while (SerialBT.available()) {
        char c=(char)SerialBT.read();
        if (c=='\n') { if (bt_line.length()>0) handle_bt_cmd(bt_line); bt_line=""; }
        else bt_line += c;
    }
}

static void sensor_timer_cb(lv_timer_t *t)
{
    (void)t;
    refresh_sensor_display();
    if (ui_active_tab == 3)
        refresh_setup_display();
}

// ── Build UI ─────────────────────────────────────────────────────────────────
static void build_ui()
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // ── Header (36px) ────────────────────────────────────────────────────
    lv_obj_t *hdr = lv_obj_create(scr);
    lv_obj_set_size(hdr, SCREEN_W, 36);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_style_bg_color(hdr, lv_color_hex(C_TOPBAR), 0);
    lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(hdr, 2, LV_PART_MAIN);
    lv_obj_set_style_border_side(hdr, LV_BORDER_SIDE_BOTTOM, LV_PART_MAIN);
    lv_obj_set_style_border_color(hdr, lv_color_hex(C_HDR_LINE), LV_PART_MAIN);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lt = lv_label_create(hdr);
    lv_label_set_text(lt, "  MM1-BLACK");
    lv_obj_align(lt, LV_ALIGN_LEFT_MID, 6, 0);
    lv_obj_set_style_text_color(lt, lv_color_hex(C_HDR_LINE), 0);
    lv_obj_set_style_text_font(lt, &lv_font_montserrat_16, 0);

    ui_lbl_count = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_count, "PTS: 0");
    /* Offset menor que 175: PTS + contagem não invadem ícones da bateria/SD/BT à direita. */
    lv_obj_align(ui_lbl_count, LV_ALIGN_LEFT_MID, 142, 0);
    lv_obj_set_style_text_color(ui_lbl_count, lv_color_hex(C_GREY), 0);

    ui_lbl_bat = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_bat, LV_SYMBOL_BATTERY_FULL " 100%");
    lv_obj_align(ui_lbl_bat, LV_ALIGN_RIGHT_MID, -200, 0);
    lv_obj_set_style_text_color(ui_lbl_bat, lv_color_hex(C_BAT_OK), 0);

    ui_lbl_sd = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_sd, LV_SYMBOL_SD_CARD);
    lv_obj_align(ui_lbl_sd, LV_ALIGN_RIGHT_MID, -150, 0);

    ui_lbl_bt = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_bt, LV_SYMBOL_BLUETOOTH);
    lv_obj_align(ui_lbl_bt, LV_ALIGN_RIGHT_MID, -100, 0);

    ui_lbl_time = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_time, "00:00:00");
    lv_obj_align(ui_lbl_time, LV_ALIGN_RIGHT_MID, -6, 0);
    lv_obj_set_style_text_color(ui_lbl_time, lv_color_hex(C_TEXT), 0);

    // ── Tabs ─────────────────────────────────────────────────────────────
    const int TAB_Y=36, TAB_H=SCREEN_H-TAB_Y, STRIP_H=28;
    const int CONTENT_H=TAB_H-STRIP_H, ACTION_H=40;
    const int TABLE_H=CONTENT_H-ACTION_H;

    lv_obj_t *tv = lv_tabview_create(scr, LV_DIR_TOP, STRIP_H);
    lv_obj_set_size(tv, SCREEN_W, TAB_H);
    lv_obj_set_pos(tv, 0, TAB_Y);
    lv_obj_set_style_bg_color(tv, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_opa(tv, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(tv, 0, 0);
    lv_obj_clear_flag(tv, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(tv, tabview_changed_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *tbtns = lv_tabview_get_tab_btns(tv);
    lv_obj_set_style_bg_color(tbtns, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_opa(tbtns, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(tbtns, lv_color_hex(C_GREY), 0);
    lv_obj_set_style_text_color(tbtns, lv_color_hex(C_HDR_LINE), LV_PART_ITEMS|LV_STATE_CHECKED);
    lv_obj_set_style_border_color(tbtns, lv_color_hex(C_HDR_LINE), LV_PART_ITEMS|LV_STATE_CHECKED);
    lv_obj_set_style_border_side(tbtns, LV_BORDER_SIDE_BOTTOM, LV_PART_ITEMS|LV_STATE_CHECKED);
    lv_obj_set_style_border_width(tbtns, 2, LV_PART_ITEMS|LV_STATE_CHECKED);

    auto make_btn = [&](lv_obj_t *par, uint32_t col, const char *txt, lv_event_cb_t cb) {
        lv_obj_t *btn = lv_btn_create(par);
        lv_obj_set_size(btn, 72, 32);
        lv_obj_set_style_bg_color(btn, lv_color_hex(col), 0);
        lv_obj_set_style_bg_color(btn, lv_color_darken(lv_color_hex(col),LV_OPA_20), LV_STATE_PRESSED);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_set_style_shadow_width(btn, 3, 0);
        lv_obj_set_style_shadow_opa(btn, LV_OPA_30, 0);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_t *l = lv_label_create(btn);
        lv_label_set_text(l, txt);
        lv_obj_center(l);
        lv_obj_set_style_text_color(l, lv_color_hex(C_WHITE), 0);
    };

    auto make_bar = [&](lv_obj_t *par, int y) -> lv_obj_t* {
        lv_obj_t *bar = lv_obj_create(par);
        lv_obj_set_size(bar, SCREEN_W, ACTION_H);
        lv_obj_set_pos(bar, 0, y);
        lv_obj_set_style_bg_color(bar, lv_color_hex(C_TOPBAR), 0);
        lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(bar, 1, LV_PART_MAIN);
        lv_obj_set_style_border_side(bar, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
        lv_obj_set_style_border_color(bar, lv_color_hex(C_HDR_LINE), LV_PART_MAIN);
        lv_obj_set_style_radius(bar, 0, 0);
        lv_obj_set_style_pad_all(bar, 3, 0);
        lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_layout(bar, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(bar, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        return bar;
    };

    // ── POINTS tab ───────────────────────────────────────────────────────
    lv_obj_t *tp = lv_tabview_add_tab(tv, LV_SYMBOL_LIST " POINTS");
    lv_obj_set_style_bg_color(tp, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(tp, 0, 0);
    lv_obj_clear_flag(tp, LV_OBJ_FLAG_SCROLLABLE);

    ui_tbl_pts = lv_table_create(tp);
    lv_obj_set_size(ui_tbl_pts, SCREEN_W, TABLE_H);
    lv_obj_set_pos(ui_tbl_pts, 0, 0);
    lv_table_set_col_cnt(ui_tbl_pts, 5);
    lv_table_set_col_width(ui_tbl_pts, 0, 64);   // Ref#
    lv_table_set_col_width(ui_tbl_pts, 1, 108);  // Dist
    lv_table_set_col_width(ui_tbl_pts, 2, 40);   // E
    lv_table_set_col_width(ui_tbl_pts, 3, 92);   // Azm
    lv_table_set_col_width(ui_tbl_pts, 4, 92);   // Inc
    lv_obj_set_style_bg_color(ui_tbl_pts, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_top(ui_tbl_pts, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(ui_tbl_pts, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(ui_tbl_pts, 6, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(ui_tbl_pts, 6, LV_PART_ITEMS);
    lv_obj_add_event_cb(ui_tbl_pts, pts_draw_cb, LV_EVENT_DRAW_PART_BEGIN, nullptr);
    lv_obj_add_event_cb(ui_tbl_pts, pts_click_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *bar_p = make_bar(tp, TABLE_H);
    make_btn(bar_p, C_BTN_DEL,  LV_SYMBOL_TRASH " DEL",  btn_del_cb);
    make_btn(bar_p, C_BTN_DEL,  LV_SYMBOL_TRASH " CLR",  btn_clear_cb);
    make_btn(bar_p, C_BTN_SAVE, LV_SYMBOL_SAVE " SAVE",  btn_save_cb);

    // ── SENSOR tab ───────────────────────────────────────────────────────
    lv_obj_t *ts = lv_tabview_add_tab(tv, LV_SYMBOL_EYE_OPEN " SENSOR");
    lv_obj_set_style_bg_color(ts, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(ts, 10, 0);
    lv_obj_clear_flag(ts, LV_OBJ_FLAG_SCROLLABLE);

    auto sec_lbl = [&](lv_obj_t*p, int y, const char*t) {
        lv_obj_t*l=lv_label_create(p); lv_label_set_text(l,t);
        lv_obj_align(l,LV_ALIGN_TOP_LEFT,0,y);
        lv_obj_set_style_text_font(l,&lv_font_montserrat_14,0);
        lv_obj_set_style_text_color(l,lv_color_hex(C_HDR_LINE),0);
    };
    auto val_lbl = [&](lv_obj_t*p, int y) -> lv_obj_t* {
        lv_obj_t*l=lv_label_create(p);
        lv_label_set_long_mode(l,LV_LABEL_LONG_DOT);
        lv_obj_set_width(l,SCREEN_W-20);
        lv_obj_align(l,LV_ALIGN_TOP_LEFT,0,y);
        lv_label_set_text(l,"---");
        lv_obj_set_style_text_font(l,&lv_font_montserrat_16,0);
        lv_obj_set_style_text_color(l,lv_color_hex(C_TEXT),0);
        return l;
    };

    sec_lbl(ts, 0,
#if LZR_SHARE_USB_UART
            "LASER  UART0  RXD0=IO3  TXD0=IO1");
#else
            "UART LASER");
#endif
    ui_lbl_tof_val = val_lbl(ts, 22);
    sec_lbl(ts, 58, "AZIMUTE / INCLINA\xC3\xA7\xC3\xA3O / ROLAMENTO");
    ui_lbl_imu_val = val_lbl(ts, 80);
    sec_lbl(ts, 116, "STATUS");
    ui_lbl_sens_stat = val_lbl(ts, 138);

    // ── FILES tab ────────────────────────────────────────────────────────
    lv_obj_t *tf = lv_tabview_add_tab(tv, LV_SYMBOL_SD_CARD " FILES");
    lv_obj_set_style_bg_color(tf, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(tf, 6, 0);
    lv_obj_clear_flag(tf, LV_OBJ_FLAG_SCROLLABLE);

    ui_lbl_active = lv_label_create(tf);
    lv_label_set_long_mode(ui_lbl_active, LV_LABEL_LONG_DOT);
    lv_obj_set_width(ui_lbl_active, SCREEN_W-12);
    lv_obj_align(ui_lbl_active, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_color(ui_lbl_active, lv_color_hex(C_REF_S), 0);

    const int FTH = CONTENT_H - 12 - 22 - 4 - 44 - 4 - 22;
    ui_tbl_files = lv_table_create(tf);
    lv_obj_set_size(ui_tbl_files, SCREEN_W-12, FTH);
    lv_obj_align(ui_tbl_files, LV_ALIGN_TOP_LEFT, 0, 26);
    lv_table_set_col_cnt(ui_tbl_files, 2);
    lv_table_set_col_width(ui_tbl_files, 0, SCREEN_W-12-90);
    lv_table_set_col_width(ui_tbl_files, 1, 84);
    lv_obj_set_style_bg_color(ui_tbl_files, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_top(ui_tbl_files, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(ui_tbl_files, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(ui_tbl_files, 8, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(ui_tbl_files, 8, LV_PART_ITEMS);
    lv_obj_add_event_cb(ui_tbl_files, file_draw_cb, LV_EVENT_DRAW_PART_BEGIN, nullptr);
    lv_obj_add_event_cb(ui_tbl_files, file_click_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    auto fbtn = [&](lv_obj_t*p, uint32_t c, const char*t, lv_event_cb_t cb) {
        lv_obj_t*b=lv_btn_create(p); lv_obj_set_size(b,148,40);
        lv_obj_set_style_bg_color(b,lv_color_hex(c),0);
        lv_obj_set_style_radius(b,8,0); lv_obj_set_style_shadow_width(b,4,0);
        lv_obj_set_style_shadow_opa(b,LV_OPA_30,0);
        lv_obj_add_event_cb(b,cb,LV_EVENT_CLICKED,nullptr);
        lv_obj_t*l=lv_label_create(b); lv_label_set_text(l,t);
        lv_obj_center(l); lv_obj_set_style_text_color(l,lv_color_hex(C_WHITE),0);
    };
    int by = 26+FTH+4;
    lv_obj_t *fr = lv_obj_create(tf);
    lv_obj_set_size(fr, SCREEN_W-12, 44);
    lv_obj_align(fr, LV_ALIGN_TOP_LEFT, 0, by);
    lv_obj_set_style_bg_opa(fr,LV_OPA_TRANSP,0);
    lv_obj_set_style_border_width(fr,0,0);
    lv_obj_set_style_pad_all(fr,0,0);
    lv_obj_set_layout(fr, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(fr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(fr, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(fr, LV_OBJ_FLAG_SCROLLABLE);
    fbtn(fr, C_BTN_NEW, LV_SYMBOL_PLUS " NEW",  btn_new_file_cb);
    fbtn(fr, C_BTN_USE, LV_SYMBOL_OK " USE",    btn_use_file_cb);
    fbtn(fr, C_BTN_DEL, LV_SYMBOL_TRASH " DEL", btn_del_file_cb);

    ui_lbl_fstatus = lv_label_create(tf);
    lv_label_set_long_mode(ui_lbl_fstatus, LV_LABEL_LONG_DOT);
    lv_obj_set_width(ui_lbl_fstatus, SCREEN_W-12);
    lv_obj_align(ui_lbl_fstatus, LV_ALIGN_TOP_LEFT, 0, by+48);
    lv_label_set_text(ui_lbl_fstatus, "Ready");
    lv_obj_set_style_text_color(ui_lbl_fstatus, lv_color_hex(C_GREY), 0);

    // ── SETUP tab (Bluetooth / TopoDroid / IMU BNO086) ──────────────────
    {
        lv_obj_t *tsetup = lv_tabview_add_tab(tv, LV_SYMBOL_SETTINGS " SETUP");
        lv_obj_set_style_bg_color(tsetup, lv_color_hex(C_BG), 0);
        lv_obj_set_style_pad_all(tsetup, 2, 0);
        lv_obj_add_flag(tsetup, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(tsetup, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(tsetup, LV_SCROLLBAR_MODE_AUTO);

        lv_obj_t *lt_bt = lv_label_create(tsetup);
        lv_label_set_text(lt_bt,
            "Bluetooth (SPP) — TopoDroid / SexyTopo\n"
            "Emparelhe em Definições → Bluetooth e escolha BRICS5-MM1 (perfil porta série).\n"
            "No telemóvel, depois use importação CSV na app (LIST/EXPORT no MM1 ou ficheiro no SD).\n"
            "Comandos: MEAS, LIST, EXPORT, CLEAR, FILES, FILE_SEND,/caminho.csv\n\n"
            "Outros dispositivos (ex. Mr_Zappy) podem enviar linhas COMPASS/CLINO/DIST; "
            "este MM1 exporta CSV BRIC5 para TopoDroid.");
        lv_label_set_long_mode(lt_bt, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(lt_bt, SCREEN_W - 16);
        lv_obj_set_style_text_font(lt_bt, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lt_bt, lv_color_hex(C_TEXT), 0);
        lv_obj_align(lt_bt, LV_ALIGN_TOP_LEFT, 6, 4);

        lv_obj_t *lt_imu_t = lv_label_create(tsetup);
        lv_label_set_text(lt_imu_t, "IMU BNO086 — estado / calibração");
        lv_obj_set_style_text_font(lt_imu_t, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lt_imu_t, lv_color_hex(C_HDR_LINE), 0);
        lv_obj_align_to(lt_imu_t, lt_bt, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 12);

        ui_lbl_setup_acc = lv_label_create(tsetup);
        lv_label_set_long_mode(ui_lbl_setup_acc, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(ui_lbl_setup_acc, SCREEN_W - 16);
        lv_obj_set_style_text_font(ui_lbl_setup_acc, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ui_lbl_setup_acc, lv_color_hex(C_REF_S), 0);
        lv_obj_align_to(ui_lbl_setup_acc, lt_imu_t, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 6);
        refresh_setup_display();

        lv_obj_t *lt_imu_h = lv_label_create(tsetup);
        lv_label_set_text(lt_imu_h,
            "Calibração em ambientes com muita anomalia magnética\n"
            "• SH-2 calibra automaticamente; a precisão de heading (acima) deve baixar.\n"
            "• Evite ferragens, motores DC, colunas metálicas nos primeiros minutos.\n"
            "• Movimento lento: figura-8 e rotações completas em X/Y/Z durante 30–60 s.\n"
            "• Se a precisão não baixa (<~5°), repita ao ar livre longe de estruturas.\n"
            "• Com yaw irrecuperável, use medições relativas entre pontos; "
            "Game Rotation Vector (sem mag) é alternativa em firmware dedicado.");
        lv_label_set_long_mode(lt_imu_h, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(lt_imu_h, SCREEN_W - 16);
        lv_obj_set_style_text_font(lt_imu_h, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lt_imu_h, lv_color_hex(C_TEXT), 0);
        lv_obj_align_to(lt_imu_h, ui_lbl_setup_acc, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
    }

    // ── Render ───────────────────────────────────────────────────────────
    refresh_table();
    update_active_lbl();
    update_status();
}

// ── Init helpers ─────────────────────────────────────────────────────────────
static void sd_init()
{
#ifdef ARDUINO_ARCH_ESP32
    sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    sd_ready = SD.begin(SD_CS, sd_spi, 4000000U);
#else
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    sd_ready = SD.begin(SD_CS);
#endif
    DBG_PRINT("[SD] %s\n", sd_ready ? "OK" : "Not found");
}

static void sensor_init()
{
    Wire.begin(I2C_SDA, I2C_SCL, 100000);
    delay(200);
#if IMU_RST_PIN >= 0
    pinMode(IMU_RST_PIN, OUTPUT);
    digitalWrite(IMU_RST_PIN, LOW);
    delay(20);
    digitalWrite(IMU_RST_PIN, HIGH);
    delay(300);
#endif
    pinMode(IMU_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IMU_INT), imuISR, FALLING);

    imu_ok = bno08x.begin_I2C(IMU_ADDR, &Wire);
    if (imu_ok) {
        bno08x.enableReport(SH2_ROTATION_VECTOR, 20000);
        delay(50);
        bno08x.enableReport(SH2_ACCELEROMETER, 50000);
        delay(50);
    }
    DBG_PRINT("[IMU] %s\n", imu_ok ? "OK" : "FAIL");

    lzr_init();
    DBG_PRINT("[LASER] %s (UART RX=%d TX=%d)\n",
              tof_ok ? "OK" : "FAIL", LZR_PIN_RX, LZR_PIN_TX);
    lzr_post_init = true;
}

/** Splash antes da UI LVGL — mesmo pipeline de pixels que disp_flush (pushColors + swap),
 *  para cores iguais ao LVGL; não usar pushImage aqui (usa pushPixels/setSwapBytes e troca RB). */
static void show_boot_splash_tft(void)
{
    tft.startWrite();
    tft.setAddrWindow(0, 0, SCREEN_W, SCREEN_H);
    tft.pushColors(reinterpret_cast<uint16_t *>(const_cast<uint8_t *>(mira_splash_map)),
                   (uint32_t)SCREEN_W * SCREEN_H, true);
    tft.endWrite();
    delay(SPLASH_MS);
}

// ── setup / loop ─────────────────────────────────────────────────────────────
void setup()
{
#ifdef ARDUINO_ARCH_ESP32
#if LZR_SHARE_USB_UART
    esp_log_level_set("*", ESP_LOG_NONE);
#endif
#endif
#if !LZR_SHARE_USB_UART
    Serial.begin(115200);
    DBG_PRINT("\n[BRICS-5 MM1] Boot\n");
#endif

    tft.init();
    tft.setRotation(1);
    tft.setTouch(const_cast<uint16_t*>(TOUCH_CAL));
    tft.fillScreen(TFT_BLACK);
    show_boot_splash_tft();

#ifdef ARDUINO_ARCH_ESP32
    audio_init_hw();
#endif
    pinMode(USER_BUTTON_PIN, INPUT_PULLUP);

    sd_init();
    SerialBT.begin("BRICS5-MM1");
    sensor_init();

    lv_init();
    lvgl_buf = (lv_color_t*)malloc(SCREEN_W * 10 * sizeof(lv_color_t));
    if (!lvgl_buf) {
#if !LZR_SHARE_USB_UART
        Serial.println("FATAL");
#endif
        while (1) delay(1000);
    }
    lv_disp_draw_buf_init(&draw_buf, lvgl_buf, nullptr, SCREEN_W*10);

    static lv_disp_drv_t dd;
    lv_disp_drv_init(&dd);
    dd.hor_res=SCREEN_W; dd.ver_res=SCREEN_H;
    dd.flush_cb=disp_flush; dd.draw_buf=&draw_buf;
    lv_disp_t *disp = lv_disp_drv_register(&dd);

    lv_theme_t *th = lv_theme_default_init(disp,
        lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_TEAL),
        false, &lv_font_montserrat_14);
    lv_disp_set_theme(disp, th);

    if (sd_ready) {
        if (!SD.exists(active_csv)) {
            File f=SD.open(active_csv,FILE_WRITE);
            if(f){f.println(BRIC_CSV_HEADER);f.close();}
        }
        load_csv();
    }

    static lv_indev_drv_t id;
    lv_indev_drv_init(&id);
    id.type=LV_INDEV_TYPE_POINTER; id.read_cb=touch_read;
    lv_indev_drv_register(&id);

    build_ui();
    lv_timer_create(periodic_cb, 1000, nullptr);
    lv_timer_create(sensor_timer_cb, 250, nullptr);
#ifdef ARDUINO_ARCH_ESP32
    play_boot_chime();
#endif
#if !LZR_SHARE_USB_UART
    Serial.println("[BRICS-5 MM1] Ready");
#endif
}

/* Botão: mantém pressionado = pede medições laser; soltar grava na tabela (curto = shot, ≥650 ms = nav). */
static int user_btn_last_raw = HIGH;
static int user_btn_stable = HIGH;
static unsigned long user_btn_edge_ms = 0;
static unsigned long user_btn_press_t0 = 0;

void loop()
{
    unsigned long m = millis();
    int x = digitalRead(USER_BUTTON_PIN);
    if (x != user_btn_last_raw) {
        user_btn_last_raw = x;
        user_btn_edge_ms = m;
    }
    if ((m - user_btn_edge_ms) > 50UL && x != user_btn_stable) {
        int prev = user_btn_stable;
        user_btn_stable = x;
        if (user_btn_stable == LOW && prev == HIGH)
            user_btn_press_t0 = m;
        else if (user_btn_stable == HIGH && prev == LOW) {
            unsigned long dur = m - user_btn_press_t0;
#ifdef ARDUINO_ARCH_ESP32
            if (dur >= 650UL)
                add_point(PT_NAV, false);
            else if (dur >= 50UL)
                add_point(PT_SAMPLE, false);
            play_button_ack();
#endif
        }
    }

#ifdef ARDUINO_ARCH_ESP32
    lzr_service_hold_capture(user_btn_stable == LOW);
#endif

    lzr_loop_tick(millis());
    poll_imu();

    lv_timer_handler();
    delay(5);
}
