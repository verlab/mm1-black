/**
 * MM1-BLACK  —  Smart tape / UART laser range + IMU
 * ESP32 CYD  |  ST7796 480×320  |  LVGL 8.3
 *
 * Flat point list; TopoDroid-compatible CSV table. Physical button (BRIC5-style, issue #2):
 * 1st press → red laser on only (waiting); 2nd press → measure + success/error beep + laser off.
 * Debounce 50 ms; aim auto-cancels after BTN_CAP_ARM_TIMEOUT_MS.
 * Two point types:
 *   S (Sample)    – measurement samples
 *   N (Navigation) – reference points for transforms
 *
 * Tabs: POINTS | SENSOR | FILES | SETUP (sub: About, Bright, Cal, BT)
 *
 * BT BLE **SAP6** (CaveBLE GATT) for TopoDroid / SexyTopo / DiscoX-class apps.
 * Leg notify 17 B + ACK 0x55/0x56; queue + 5 s resend. CSV on SD + Wi‑Fi portal for file export.
 */

#include <Arduino.h>
#include <cstdarg>
#include <math.h>
#include <string.h>
#include <time.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_BNO08x.h>
#include <lvgl.h>
#include <SD.h>
#ifdef ARDUINO_ARCH_ESP32
#include <Preferences.h>
#include <cinttypes>
#include <esp_log.h>
#include <esp_gap_ble_api.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <esp_coexist.h>
#include "esp32-hal-ledc.h"
#include "extra/libs/qrcode/lv_qrcode.h"
#include "web_portal.h"
#include "mm1_geometry.h"
#include "sap6_ble.h"
#include "fw_update_url.h"
#include "serial_cmd.h"
#endif

#include "firmware_version.h"

/* Bitmap RGB565 em mira_splash_img.c (480×320); logo rotated in gen_mira_splash.py only. */
extern const uint8_t mira_splash_map[];
#define MIRA_SPLASH_W 480
#define MIRA_SPLASH_H 320

// ── Pins ─────────────────────────────────────────────────────────────────────
/* Factory landscape = rotation 1 (480×320). Portrait UI (rot 0) breaks LVGL on CYD. */
#ifndef TFT_ROTATION
#define TFT_ROTATION 1
#endif
#if TFT_ROTATION == 0 || TFT_ROTATION == 2
#define SCREEN_W  320
#define SCREEN_H  480
#else
#define SCREEN_W  480
#define SCREEN_H  320
#endif
/* Portrait / narrow width: compact header + icon-only top tabs. */
#define UI_COMPACT_HEADER (SCREEN_W < 400)
#ifndef SPLASH_MS
#define SPLASH_MS 500UL
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

/* Correção aplicada ao azimute fusion (aprox.: norte verdadeiro ≈ norte magnético + D).
 * Padrão = declinação WMM ~2025 para Belo Horizonte MG (~19.92°S 43.94°W), convenção NOAA (leste +).
 * Outra região: -DAZIMUTH_OFFSET_DEG=... no build; ou 0 se quiseres só norte magnético (bússola).
 * Valor na NVS (SETUP) substitui isto após primeiro ajuste gravado. */
#ifndef AZIMUTH_OFFSET_DEG
#define AZIMUTH_OFFSET_DEG (-21.7f)
#endif
/** BNO08x Rotation Vector → Android-style ENU: +X East, +Y North, +Z up.
 * Azimuth = atan2(wx, wy) = clockwise from magnetic north (TopoDroid-style), if fusion is mag-based.
 *
 * AZIMUTH_OFFSET_DEG — default BH acima = D (NOAA, leste +) para aproximar azimute ao norte geográfico.
 *   • Bússola / só magnético: define 0 no build ou repõe na SETUP (ou apaga chave NVS).
 *   • Confirma D anualmente: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
 *   • Empírico: OFFSET = B_conhecido − B_medido (ajusta na SETUP e grava).
 *   NVS Preferences substitui o macro após primeiro SAVE na SETUP (ESP32). */

#define PREFS_NAMESPACE "mm1blk"
#define PREFS_KEY_AZ_OFS "az_ofs"
#define PREFS_KEY_BL_PCT "bl_pct"
/** Default TFT backlight (GPIO 27, LEDC PWM) before first NVS save. */
#ifndef BL_DEFAULT_PCT
#define BL_DEFAULT_PCT 85
#endif
#ifndef TFT_BL_LEDC_CH
#define TFT_BL_LEDC_CH 6
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
/** 1 = modulo iliasam 701A/X-40 (ASCII 256000, calib zero com 'C'); 0 = M01/Egismos 0xAA */
#ifndef LZR_PROTO_ILIASAM
#define LZR_PROTO_ILIASAM 0
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
/* “Dip” column ≈ local magnetic inclination (nominal); no standalone magnetometer — nominal value for TopoDroid import. */
#ifndef EXPORT_DIP_DEG_NOMINAL
#define EXPORT_DIP_DEG_NOMINAL (29.88f)
#endif
#ifndef BT_DEVICE_NAME
/** BLE advertised name; protocol id is always "SAP6" in the Name characteristic. */
#define BT_DEVICE_NAME "SAP6_0001"
#endif
#ifndef POSIX_FALLBACK_ANCHOR_SEC
#define POSIX_FALLBACK_ANCHOR_SEC (1767225600UL)
#endif

/* XPT2046 cal for UI landscape (rotation 1). */
static const uint16_t TOUCH_CAL_ACTIVE[5] = { 254, 3643, 176, 3693, 7 };

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
/** LVGL Montserrat has no em-dash / many Unicode glyphs — use ASCII only on labels. */
#define UI_NA       "-"
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
#define C_BTN_BT    0x1565C0u
#define C_FILE_ACT  0xC8E6C9u
#define C_REF_S     0x1565C0u
#define C_TYPE_S    0x00695Cu   // sample text
#define C_TYPE_N    0x6A1B9Au   // nav text
#define C_BAT_OK    0x2E7D32u
#define C_BAT_LOW   0xD32F2Fu
#define C_WARN      0xF9A825u   /* portal / SoftAP warning */
#define C_CAP_AIM   0xF9A825u   /* button aim (BRIC5) */
#define C_CAP_MEAS  0xE65100u   /* button capture in progress */
#define C_HDR_BLINK_HI 0x42A5F5u /* table header blink (aim / measure) */
#define C_HDR_OK_ON    0x2E7D32u
#define C_HDR_OK_OFF   0x81C784u
#define C_HDR_FAIL_ON  0xD32F2Fu
#define C_HDR_FAIL_OFF 0xEF9A9Au

// ── Data / CSV ───────────────────────────────────────────────────────────────
// TopoDroid-style column header (two spaces before “Measurement Type”).
#define TD_CSV_HEADER \
    "Time-Stamp, POSIX Time, Index, Distance (meters), Azimuth (deg), Inclination (deg), Dip (deg), Roll (deg), Temperature (Celsius),  Measurement Type, Error Log"
#define MAX_PTS  100
/** Maximo de legs enviados num unico TX a partir do CSV no SD. */
#define MAX_CSV_STREAM_ROWS  5000
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

static void tft_touch_apply_cal(void)
{
    tft.setTouch(const_cast<uint16_t *>(TOUCH_CAL_ACTIVE));
}
#ifdef ARDUINO_ARCH_ESP32
/** SD no HSPI: nunca usar `SPI.begin(...)` no VSPI global — TFT_eSPI (display + XPT2046) usa VSPI em 12/13/14. */
static SPIClass sd_spi(HSPI);
#endif
#ifdef ARDUINO_ARCH_ESP32
static volatile bool g_bt_stack_ready = false;
static void ble_boot_service(void);
/** SETUP Wi-Fi — portal start/stop in `loop()` (not LVGL callback: stack + radio). */
static volatile bool g_wifi_portal_restart_req = false;
static volatile bool g_wifi_portal_stop_req    = false;
/** Defer SETUP label refresh out of tabview event (avoids UI freeze). */
static volatile bool g_setup_ui_refresh_req    = false;
#endif
static Adafruit_BNO08x   bno08x(-1);
static sh2_SensorValue_t sensorValue;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t        *lvgl_buf = nullptr;
static bool             g_ble_boot_pending = true;

static MeasPoint pts[MAX_PTS];
static int       pt_count = 0;
static int       sel_row  = -1;
static uint32_t  next_id  = 1;

// Sensors
static uint32_t tof_dist_mm = 0;
/** Azimute 0–360° e inclinação (° acima/baixo da horizontal) ao longo do eixo do laser. */
static float    imu_azimuth_deg = 0, imu_inclination_deg = 0;
static int      imu_rv_accuracy = -1;
static float    imu_grav_mag    = 0.f;
static float    imu_roll = 0, imu_pitch = 0, imu_yaw = 0;
static bool     imu_ok = false, tof_ok = false;
static volatile bool imu_irq = false;

// Battery (placeholder – reads ADC or shows 100%)
static int bat_pct = 100;
/** ESP32 die temperature (°C) — same source as TopoDroid CSV Temperature column. */
static float g_live_temp_c = NAN;

// SD / files
static bool sd_ready = false, bt_conn = false;
#ifdef ARDUINO_ARCH_ESP32
static bool                  g_bt_paired         = false;
static volatile bool         g_sap6_req_meas     = false;
static volatile bool         g_sap6_req_laser_on = false;
static volatile bool         g_sap6_req_laser_off = false;
static char                  g_bt_peer_mac[18]   = {0};   // "AA:BB:CC:DD:EE:FF" or "" if unknown
static char                  g_bt_local_mac[18]  = {0};   // device MAC (filled at boot)

/* WiFi soft-AP — only started on demand from the WIFI tab. */
#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID  "MM1-MIRA"
#endif
#ifndef WIFI_AP_PASS
#define WIFI_AP_PASS  "mira-mm1"   /* must be 8+ chars or empty for open AP */
#endif
#ifndef FW_VERSION_STR
#define FW_VERSION_STR "0.4.x"
#endif
static bool                  g_wifi_user_on      = false;
#endif
static char active_csv[40] = "/mm1_black_000.csv";
static char file_names[MAX_FILES][32];
static int  file_count = 0, file_sel = -1;

// UI handles
static lv_obj_t *ui_tbl_pts      = nullptr;
static lv_obj_t *ui_lbl_bt       = nullptr;
static lv_obj_t *ui_lbl_wifi     = nullptr;
static lv_obj_t *ui_lbl_sd       = nullptr;
static lv_obj_t *ui_lbl_bat      = nullptr;
static lv_obj_t *ui_lbl_sens_temp = nullptr;
static lv_obj_t *ui_lbl_time     = nullptr;
static lv_obj_t *ui_lbl_count    = nullptr;
static lv_obj_t *ui_hdr_bar      = nullptr;

typedef enum : uint8_t {
    CAP_UI_IDLE = 0,
    CAP_UI_AIM,
    CAP_UI_MEASURING,
    CAP_UI_SUCCESS,
    CAP_UI_FAIL,
} CapUiState;

static CapUiState  g_cap_ui_state       = CAP_UI_IDLE;
static unsigned long g_cap_ui_blink_ms  = 0;
static bool          g_cap_ui_blink_on  = false;

static void cap_ui_set_state(CapUiState st);
static void cap_ui_tick(unsigned long now);
static void cap_ui_result_pulse(bool ok);
static uint32_t cap_pts_hdr_color(void);
static void cap_ui_invalidate_pts_hdr(void);
static lv_obj_t *ui_lbl_tof_val  = nullptr;
static lv_obj_t *ui_lbl_imu_val  = nullptr;
static lv_obj_t *ui_lbl_sens_stat= nullptr;
static lv_obj_t *ui_tbl_files    = nullptr;
static lv_obj_t *ui_lbl_active   = nullptr;
static lv_obj_t *ui_lbl_fstatus  = nullptr;
static lv_obj_t *ui_lbl_setup_ack   = nullptr;
static lv_obj_t *ui_lbl_setup_cal_ack = nullptr;
static lv_obj_t *ui_lbl_setup_az_offs = nullptr;
static lv_obj_t *ui_lbl_setup_imu_head = nullptr;
static lv_obj_t *ui_lbl_setup_imu_qual = nullptr;
static lv_obj_t *ui_lbl_setup_imu_grav = nullptr;
static lv_obj_t *ui_lbl_setup_bt_stat   = nullptr;
static lv_obj_t *ui_lbl_setup_bt_mac    = nullptr;
static lv_obj_t *ui_lbl_setup_bt_pair   = nullptr;
static lv_obj_t *ui_lbl_setup_bt_peer   = nullptr;
static lv_obj_t *ui_lbl_setup_bt_diag   = nullptr;
static lv_obj_t *ui_lbl_setup_wifi_stat = nullptr;
static lv_obj_t *ui_lbl_setup_wifi_info = nullptr;
static lv_obj_t *ui_qr_wifi             = nullptr;
static lv_obj_t *ui_btn_wifi_portal     = nullptr;
static lv_obj_t *ui_lbl_setup_bl       = nullptr;
static lv_obj_t *ui_slider_bl         = nullptr;
static lv_obj_t *ui_lbl_setup_ver     = nullptr;
static lv_obj_t *ui_qr_fw_update      = nullptr;
static uint8_t   g_backlight_pct      = BL_DEFAULT_PCT;
static bool        g_bl_pwm_attached  = false;
static bool        g_bl_ui_sync       = false;
/** Azimuth offset added after atan2; NVS persists on ESP32 (SETUP tab). Loaded at boot. */
static float       g_azimuth_offset_deg = AZIMUTH_OFFSET_DEG;

// Tab order: 0=POINTS, 1=SENSOR, 2=FILES, 3=SETUP (lv_tabview_add_tab order).
static uint8_t     ui_active_tab    = 0;
/** SETUP sub-tab index (About | Bright | Cal | BT). */
static uint8_t     g_setup_sub_idx  = 0;
#define SETUP_SUB_ABOUT   0
#define SETUP_SUB_BRIGHT  1
#define SETUP_SUB_CAL     2
#define SETUP_SUB_BT      3

static inline bool ui_is_setup_sensor_tab(void)
{
    return ui_active_tab == 1;
}

static inline bool ui_is_setup_files_tab(void)
{
    return ui_active_tab == 2;
}
static uint32_t    lzr_poll_gap_ms  = POLL_INTERVAL_MS;

// Forward declarations
static void sd_init();
static void sensor_init();
static void prefs_load_az_offset();
static void prefs_load_backlight(void);
static void prefs_save_backlight(void);
static void tft_bl_init(void);
static void tft_bl_apply(uint8_t pct);
static void refresh_setup_bl_label(void);
static void refresh_table();
static void refresh_sensor_display();
static void refresh_setup_bt_status();
static void refresh_setup_az_offs_label();
static void refresh_setup_cal_display(void);
static void setup_tab_cal_ack(const char *msg);
static void setup_tab_bt_ack(const char *msg);
static void set_fstatus(const char *msg);
static void audio_init_hw();
static void play_boot_chime();
static void play_button_ack();
static void add_point(PtType type, bool sync_laser_before);
static void handle_bt_cmd(const String &raw);

#ifdef ARDUINO_ARCH_ESP32
static void sap6_process_pending_cmds(void);
static void load_csv(void);
static void setup_tab_bt_ack(const char *msg);
static bool ble_csv_tx_begin(void);
static void ble_csv_tx_poll(void);
static bool ble_csv_tx_busy(void);
#endif

void IRAM_ATTR imuISR() { imu_irq = true; }

#ifdef ARDUINO_ARCH_ESP32
/** Message queued for LVGL from Bluedroid callback (avoid touching LVGL off the GUI thread). */
static char g_bt_setup_banner[160];

static void bt_setup_banner_lv_async_cb(void *)
{
    if (ui_lbl_setup_ack)
        lv_label_set_text(ui_lbl_setup_ack, g_bt_setup_banner);
}

static void bt_post_setup_banner_fmt(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_bt_setup_banner, sizeof(g_bt_setup_banner), fmt, ap);
    va_end(ap);
    (void)lv_async_call(bt_setup_banner_lv_async_cb, nullptr);
}

static void bt_refresh_bond_state(void);

/** Refresh BLE bond list (TopoDroid / SexyTopo pair in Android settings first). */
static void bt_refresh_bond_state(void)
{
    int n = esp_ble_get_bond_device_num();
    if (n <= 0) {
        g_bt_paired = false;
        g_bt_peer_mac[0] = '\0';
        return;
    }
    esp_ble_bond_dev_t list[4];
    int dev_num = n > 4 ? 4 : n;
    if (esp_ble_get_bond_device_list(&dev_num, list) == ESP_OK && dev_num > 0) {
        g_bt_paired = true;
        snprintf(g_bt_peer_mac, sizeof(g_bt_peer_mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                 list[0].bd_addr[0], list[0].bd_addr[1], list[0].bd_addr[2],
                 list[0].bd_addr[3], list[0].bd_addr[4], list[0].bd_addr[5]);
    } else {
        g_bt_paired = false;
        g_bt_peer_mac[0] = '\0';
    }
}

static void bt_clear_all_bonds(void)
{
    sap6_ble_clear_bonds();
    g_bt_paired = false;
    g_bt_peer_mac[0] = '\0';
}

static void lzr_on();
static void lzr_off();

void sap6_on_command(uint8_t cmd)
{
    if (cmd == 0x38)
        g_sap6_req_meas = true;
    else if (cmd == 0x36)
        g_sap6_req_laser_on = true;
    else if (cmd == 0x37)
        g_sap6_req_laser_off = true;
}

static void sap6_process_pending_cmds(void)
{
    if (g_sap6_req_laser_on) {
        g_sap6_req_laser_on = false;
        lzr_on();
    }
    if (g_sap6_req_laser_off) {
        g_sap6_req_laser_off = false;
        lzr_off();
    }
    if (g_sap6_req_meas) {
        g_sap6_req_meas = false;
        add_point(PT_SAMPLE, true);
        setup_tab_bt_ack("Shot (SAP6)");
    }
}

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

/** 2nd button tap — success only (1st tap is silent, laser only). */
static void play_capture_sound()
{
    buzzer_note(988, 55);
    delay(18);
    buzzer_note(1568, 95);
    delay(22);
    buzzer_note(2093, 130);
}

/** Invalid laser reading or table full. */
static void play_error_sound()
{
    buzzer_note(494, 90);
    delay(30);
    buzzer_note(370, 90);
    delay(30);
    buzzer_note(262, 160);
}
#else
static void audio_init_hw() {}
static void play_boot_chime() {}
static void play_button_ack() {}
static void play_capture_sound() {}
static void play_error_sound() {}
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

/** Tab bar above scrollable content so taps register (not only horizontal swipe). */
static void tabview_enable_tab_taps(lv_obj_t *tv)
{
    lv_obj_t *tbtns = lv_tabview_get_tab_btns(tv);
    if (!tbtns)
        return;
    lv_obj_move_foreground(tbtns);
    lv_obj_add_flag(tbtns, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(tbtns, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_set_ext_click_area(tbtns, 12);
    lv_btnmatrix_set_btn_ctrl_all(tbtns, LV_BTNMATRIX_CTRL_CLICK_TRIG);
    lv_obj_t *cont = lv_tabview_get_content(tv);
    if (cont)
        lv_obj_add_flag(cont, LV_OBJ_FLAG_SCROLL_CHAIN_VER);
}

// ── Sensor functions ─────────────────────────────────────────────────────────
#if LZR_SHARE_USB_UART
static HardwareSerial &lzr_port = Serial;
#else
static HardwareSerial   lzr_hw(LZR_UART_NUM);
static HardwareSerial  &lzr_port = lzr_hw;
#endif

static const uint8_t CMD_LASER_ON[]  = {0xAA, 0x00, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x01, 0xC1};
static const uint8_t CMD_LASER_OFF[] = {0xAA, 0x00, 0x01, 0xBE, 0x00, 0x01, 0x00, 0x00, 0xC0};
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
/** True while user has armed aim (1st button tap) — red beam on until 2nd tap or timeout. */
static bool       lzr_btn_aim_active = false;
/** True during blocking capture — no UART drain, no stale recover. */
static bool       lzr_capture_busy    = false;
#if LZR_CONTINUOUS
static unsigned long lzr_keepalive_ms = 0;
#endif

#ifndef BTN_CAP_ARM_TIMEOUT_MS
#define BTN_CAP_ARM_TIMEOUT_MS 90000UL
#endif

static void lzr_on();
static void lzr_off();

static void lzr_sync_poll_gap_now(void)
{
    if (ui_is_setup_sensor_tab())
        lzr_poll_gap_ms = SENSOR_TAB_POLL_MS;
    else if (ui_is_setup_files_tab())
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

static void lzr_off()
{
    lzr_port.write(CMD_LASER_OFF, sizeof(CMD_LASER_OFF));
    lzr_port.flush();
}

/** Aim mode: red laser beam only — no UART measure commands until 2nd button tap. */
static void lzr_aim_on()
{
    lzr_on();
    delay(40);
    lzr_on();
}

/** Force beam off and stop aim/capture poll state (call after measure or cancel). */
static void lzr_shutdown_beam(void)
{
    lzr_btn_aim_active = false;
    lzr_one_shot_armed  = false;
#if !LZR_CONTINUOUS
    lzr_poll_state = 0;
#endif
    lzr_uart_drain();
    delay(30);
    lzr_off();
    delay(50);
    lzr_off();
}

static void lzr_aim_off()
{
    lzr_shutdown_beam();
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

static bool lzr_reading_fresh(unsigned long now)
{
    return isfinite(lzr_last_m) && lzr_last_m >= 0.001f
           && (now - lzr_last_valid_ms) <= STALE_MS;
}

static void lzr_poll_send_measure()
{
    /* While aiming / capturing, do not drain RX — may discard the module response. */
    const bool drain_rx = !lzr_btn_aim_active && !lzr_capture_busy;
    if (drain_rx)
        lzr_uart_drain();
    lzr_parse_pos = 0;
    lzr_decode_at_send = lzr_decode_tick;
    lzr_on();
    delay((lzr_btn_aim_active || lzr_capture_busy) ? 120 : 25);
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
    return ui_is_setup_sensor_tab();
}
#endif

static void lzr_poll_tick(unsigned long now)
{
#if LZR_CONTINUOUS
    (void)now;
    return;
#else
    if (lzr_poll_state == 0) {
        if (lzr_capture_busy && !lzr_one_shot_armed)
            return;
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
        if (!lzr_capture_busy && (now - lzr_last_recover_ms) >= RECOVER_MIN_MS) {
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
    if (lzr_capture_busy) return false;
    /* No range data while only the red laser is on — recover would spam CMD_SINGLE. */
    if (lzr_btn_aim_active) return false;
    return ui_is_setup_sensor_tab();
}
#endif

static void lzr_loop_tick(unsigned long now)
{
    lzr_process_incoming();
#if LZR_CONTINUOUS
    if (ui_is_setup_sensor_tab()) {
        if ((now - lzr_keepalive_ms) >= LZR_KEEPALIVE_MS) {
            lzr_keepalive_ms = now;
            lzr_on();
            delay(25);
            lzr_send_continuous();
        }
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
    /* BRIC5-style: laser off until user arms with the physical button (or SENSOR tab polls). */
    lzr_aim_off();
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
    imu_azimuth_deg = norm_deg360(atan2f(wx, wy) * r2d + g_azimuth_offset_deg);

    /* First-order pivot: M11 base is MM1_IMU_TO_M11_X_MM along body -X from IMU. */
    const float arm_m = MM1_IMU_TO_M11_X_MM * 0.001f;
    const float roll_r = imu_roll * (float)M_PI / 180.f;
    const float inc_r  = imu_inclination_deg * (float)M_PI / 180.f;
    const float d_inc  = atan2f(arm_m * sinf(roll_r) * cosf(inc_r), 1.f) * r2d;
    imu_inclination_deg += d_inc;
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
            imu_rv_accuracy = (int)sensorValue.un.rotationVector.accuracy;
            imu_update_angles_from_quat(qw, qx, qy, qz);
        } else if (sensorValue.sensorId == SH2_ACCELEROMETER) {
            const float ax = sensorValue.un.accelerometer.x;
            const float ay = sensorValue.un.accelerometer.y;
            const float az = sensorValue.un.accelerometer.z;
            imu_grav_mag = sqrtf(ax * ax + ay * ay + az * az);
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
    if (ui_is_setup_sensor_tab()) return;

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

#if !LZR_CONTINUOUS
/** Blocking measure (2nd button tap / MEAS): one-shot poll + fallbacks, same path as lzr_sync_for_capture. */
static bool lzr_measure_once_blocking(void)
{
    if (!lzr_post_init)
        return false;

    const unsigned long now0 = millis();
    if (lzr_reading_fresh(now0)) {
        lzr_apply_to_globals(now0);
        return true;
    }

    lzr_capture_busy = true;
    cap_ui_set_state(CAP_UI_MEASURING);
    const unsigned long tmax = POLL_TIMEOUT_MS + 1200UL;
    bool got = false;

    for (int attempt = 0; attempt < 3 && !got; attempt++) {
        lzr_poll_state     = 0;
        lzr_one_shot_armed = true;
        lzr_next_poll_ms   = millis();
        const uint32_t decode0 = lzr_decode_tick;
        unsigned long t0 = millis();

        while ((millis() - t0) < tmax) {
            const unsigned long now = millis();
            lzr_loop_tick(now);
            poll_imu();
            cap_ui_tick(now);
            lv_timer_handler();
            if (lzr_decode_tick != decode0) {
                lzr_apply_to_globals(millis());
                if (lzr_reading_fresh(millis())) {
                    got = true;
                    break;
                }
            }
            delay(2);
        }
        if (got)
            break;

        lzr_poll_fallback();
        unsigned long tf = millis();
        while ((millis() - tf) < 600UL) {
            const unsigned long now = millis();
            lzr_process_incoming();
            cap_ui_tick(now);
            lv_timer_handler();
            if (lzr_reading_fresh(millis())) {
                lzr_apply_to_globals(millis());
                got = true;
                break;
            }
            delay(2);
        }
        if (got)
            break;
    }

    lzr_capture_busy   = false;
    lzr_one_shot_armed = false;
    lzr_poll_state     = 0;
    lzr_apply_to_globals(millis());
    return got || lzr_reading_fresh(millis());
}
#else
static bool lzr_measure_once_blocking(void)
{
    lzr_apply_to_globals(millis());
    return isfinite(lzr_last_m) && tof_ok;
}
#endif

static uint32_t cap_pts_hdr_color(void)
{
    if (g_cap_ui_state == CAP_UI_IDLE)
        return C_TBL_HDR;

    uint32_t on, off;
    switch (g_cap_ui_state) {
    case CAP_UI_AIM:
    case CAP_UI_MEASURING:
        on  = C_TBL_HDR;
        off = C_HDR_BLINK_HI;
        break;
    case CAP_UI_SUCCESS:
        on  = C_HDR_OK_ON;
        off = C_HDR_OK_OFF;
        break;
    case CAP_UI_FAIL:
        on  = C_HDR_FAIL_ON;
        off = C_HDR_FAIL_OFF;
        break;
    default:
        return C_TBL_HDR;
    }
    return g_cap_ui_blink_on ? on : off;
}

static void cap_ui_invalidate_pts_hdr(void)
{
    if (ui_tbl_pts)
        lv_obj_invalidate(ui_tbl_pts);
}

static void cap_ui_set_state(CapUiState st)
{
    g_cap_ui_state    = st;
    g_cap_ui_blink_ms = millis();
    g_cap_ui_blink_on = true;
    cap_ui_invalidate_pts_hdr();
}

static void cap_ui_tick(unsigned long now)
{
    if (g_cap_ui_state == CAP_UI_IDLE)
        return;

    unsigned long period = 380UL;
    if (g_cap_ui_state == CAP_UI_MEASURING)
        period = 200UL;
    else if (g_cap_ui_state == CAP_UI_SUCCESS || g_cap_ui_state == CAP_UI_FAIL)
        period = 260UL;

    if ((now - g_cap_ui_blink_ms) < period)
        return;
    g_cap_ui_blink_ms = now;
    g_cap_ui_blink_on = !g_cap_ui_blink_on;
    cap_ui_invalidate_pts_hdr();
}

static void cap_ui_result_pulse(bool ok)
{
    cap_ui_set_state(ok ? CAP_UI_SUCCESS : CAP_UI_FAIL);
    const unsigned long t0 = millis();
    while ((millis() - t0) < 1000UL) {
        cap_ui_tick(millis());
        lv_timer_handler();
        delay(4);
    }
    cap_ui_set_state(CAP_UI_IDLE);
}

/** Re-send LASER_ON while waiting for 2nd tap (module may time out the beam). */
static void lzr_aim_laser_keepalive_tick(unsigned long now)
{
    if (!lzr_btn_aim_active)
        return;
    static unsigned long last_on_ms = 0;
    if ((now - last_on_ms) < 2500UL)
        return;
    last_on_ms = now;
    lzr_on();
}

static void read_battery()
{
    int raw = analogRead(BAT_ADC_PIN);
    // 12-bit ADC, 3.3V ref, typical voltage divider: adjust as needed
    float v = raw * 3.3f / 4095.0f * 2.0f;   // ×2 for divider
    bat_pct = constrain((int)((v - 3.0f) / (4.2f - 3.0f) * 100.0f), 0, 100);
}

static void read_device_temp_c(void)
{
#ifdef ARDUINO_ARCH_ESP32
    const float t = (float)temperatureRead();
    g_live_temp_c = (isfinite(t) && t > -40.f && t < 125.f) ? t : NAN;
#else
    g_live_temp_c = NAN;
#endif
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
        dsc->rect_dsc->bg_color = lv_color_hex(cap_pts_hdr_color());
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
    lv_table_set_cell_value(t, 0, 3, "Azm");
    lv_table_set_cell_value(t, 0, 4, "Inc");

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
    if (ui_lbl_count) {
        if (UI_COMPACT_HEADER)
            snprintf(buf, sizeof(buf), "#%d", pt_count);
        else
            snprintf(buf, sizeof(buf), "PTS:%d", pt_count);
        lv_label_set_text(ui_lbl_count, buf);
    }
}

static void refresh_sensor_display()
{
    read_device_temp_c();
    char buf[128];
    if (ui_lbl_tof_val) {
        if (tof_ok) {
            snprintf(buf, sizeof(buf), "%.3f m  (%lu mm)",
                     tof_dist_mm / 1000.0f, (unsigned long)tof_dist_mm);
        } else {
            snprintf(buf, sizeof(buf), "No reading");
        }
        lv_label_set_text(ui_lbl_tof_val, buf);
    }
    if (ui_lbl_imu_val) {
        snprintf(buf, sizeof(buf), "Az: %.1f deg  Inc: %.1f deg  Roll: %.1f deg",
                 imu_azimuth_deg, imu_inclination_deg, imu_roll);
        lv_label_set_text(ui_lbl_imu_val, buf);
    }
    if (ui_lbl_sens_stat) {
        snprintf(buf, sizeof(buf), "LASER: %s   IMU: %s",
                 tof_ok ? "OK" : "FAIL", imu_ok ? "OK" : "FAIL");
        lv_label_set_text(ui_lbl_sens_stat, buf);
    }
    if (ui_lbl_sens_temp) {
        if (isfinite(g_live_temp_c))
            snprintf(buf, sizeof(buf), "%.1f C  (ESP32 MCU)", (double)g_live_temp_c);
        else
            strlcpy(buf, UI_NA, sizeof(buf));
        lv_label_set_text(ui_lbl_sens_temp, buf);
    }
}

static void update_active_lbl()
{
    if (!ui_lbl_active) return;
    const char *n = active_csv; if (n[0]=='/') n++;
    char buf[64];
    snprintf(buf, sizeof(buf), "Active: %s  (%d pts)", n, pt_count);
    lv_label_set_text(ui_lbl_active, buf);
}

static void fmt_td_csv_timestamp(uint32_t posix_sec, char *buf, size_t len)
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
    fmt_td_csv_timestamp(p.posix_sec, ts, sizeof(ts));
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

/* ── CSV line helpers (SD stream + load) ───────────────────────────────────── */
static bool csv_read_line(File &f, char *buf, size_t sz)
{
    if (!f || !buf || sz < 2)
        return false;
    size_t n = 0;
    const size_t end = f.size();
    if (end > 0) {
        while ((size_t)f.position() < end) {
            const int c = f.read();
            if (c < 0)
                break;
            if (c == '\r')
                continue;
            if (c == '\n') {
                buf[n] = '\0';
                return n > 0;
            }
            if (n + 1 < sz)
                buf[n++] = (char)c;
        }
    } else {
        for (int g = 0; g < 512; g++) {
            const int c = f.read();
            if (c < 0) {
                buf[n] = '\0';
                return n > 0;
            }
            if (c == '\r')
                continue;
            if (c == '\n') {
                buf[n] = '\0';
                return n > 0;
            }
            if (n + 1 < sz)
                buf[n++] = (char)c;
        }
    }
    buf[n] = '\0';
    return false;
}

static bool tx_file_at_eof(File &f)
{
    if (!f)
        return true;
    const size_t sz = f.size();
    if (sz > 0)
        return (size_t)f.position() >= sz;
    return !f.available();
}

static bool csv_line_skipped(const char *line)
{
    while (*line == ' ' || *line == '\t')
        line++;
    if (!*line)
        return true;
    if (line[0] == '#')
        return true;
    if (strncmp(line, "Time-Stamp", 10) == 0)
        return true;
    if (strncmp(line, "ID,", 3) == 0)
        return true;
    return false;
}

static bool sniff_br_csv_row(const char *s);
static bool parse_legacy_csv_row(char *buf, MeasPoint &p);
static bool parse_br_csv_row(char *work, MeasPoint &p);

static bool csv_parse_line(char *buf, MeasPoint &p)
{
    if (csv_line_skipped(buf))
        return false;
    if (sniff_br_csv_row(buf))
        return parse_br_csv_row(buf, p);
    return parse_legacy_csv_row(buf, p);
}

#ifdef ARDUINO_ARCH_ESP32
/* ── BLE CSV TX: popup compacto (sem layer fullscreen) + prep em fases ─── */
static volatile bool g_tx_pending_start = false;
static char          g_tx_csv_path[48];
static char          g_tx_csv_line[384];
static long          g_tx_fpos = 0;
static long          g_tx_count_pos = 0;
static uint8_t       g_tx_phase = 0; /* 0 idle, 1 prep, 2 streaming */
static uint8_t       g_tx_prep_step = 0;
static int           g_tx_total = 0;
static int           g_tx_rows_sent = 0;
static int           g_tx_send_idx = 0;
static bool          g_tx_use_ram = false;
static bool          g_tx_sd_file = false;
static bool          g_tx_eof = false;
static uint32_t      g_tx_acks_base = 0;
static uint32_t      g_tx_last_progress_ms = 0;
static uint32_t      g_tx_last_acked = 0;
static uint32_t      g_tx_ui_ms = 0;
static lv_obj_t     *ui_tx_panel  = nullptr;
static lv_obj_t     *ui_tx_bar    = nullptr;
static lv_obj_t     *ui_tx_lbl    = nullptr;

static void ble_csv_tx_cancel(void);
static void ble_csv_tx_hard_reset(void);
static void ble_csv_tx_cancel_cb(lv_event_t *e);

static void ble_csv_tx_ui_hide(void)
{
    if (ui_tx_panel) {
        lv_obj_del(ui_tx_panel);
        ui_tx_panel = nullptr;
        ui_tx_bar = nullptr;
        ui_tx_lbl = nullptr;
    }
}

/** Painel solido no ecra activo — sem overlay semitransparente (evita layer 24KB + reset). */
static void ble_csv_tx_ui_show(void)
{
    ble_csv_tx_ui_hide();
    ui_tx_panel = lv_obj_create(lv_scr_act());
    lv_obj_set_size(ui_tx_panel, SCREEN_W - 32, 128);
    lv_obj_center(ui_tx_panel);
    lv_obj_set_style_bg_color(ui_tx_panel, lv_color_hex(C_TOPBAR), 0);
    lv_obj_set_style_bg_opa(ui_tx_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(ui_tx_panel, 8, 0);
    lv_obj_set_style_pad_all(ui_tx_panel, 10, 0);
    lv_obj_set_style_border_width(ui_tx_panel, 2, 0);
    lv_obj_set_style_border_color(ui_tx_panel, lv_color_hex(C_HDR_LINE), 0);
    lv_obj_set_layout(ui_tx_panel, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(ui_tx_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_tx_panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(ui_tx_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_move_foreground(ui_tx_panel);

    lv_obj_t *ttl = lv_label_create(ui_tx_panel);
    lv_label_set_text(ttl, "STREAM BLE");
    lv_obj_set_style_text_font(ttl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ttl, lv_color_hex(C_HDR_LINE), 0);

    ui_tx_lbl = lv_label_create(ui_tx_panel);
    lv_label_set_text(ui_tx_lbl, "Preparing...");
    lv_obj_set_style_text_align(ui_tx_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(ui_tx_lbl, lv_color_hex(C_TEXT), 0);
    lv_obj_set_width(ui_tx_lbl, SCREEN_W - 56);

    ui_tx_bar = lv_bar_create(ui_tx_panel);
    lv_obj_set_width(ui_tx_bar, SCREEN_W - 72);
    lv_obj_set_height(ui_tx_bar, 14);
    lv_bar_set_range(ui_tx_bar, 0, 100);
    lv_bar_set_value(ui_tx_bar, 0, LV_ANIM_OFF);

    lv_obj_t *btn_cancel = lv_btn_create(ui_tx_panel);
    lv_obj_set_size(btn_cancel, 96, 30);
    lv_obj_set_style_bg_color(btn_cancel, lv_color_hex(C_BTN_DEL), 0);
    lv_obj_add_event_cb(btn_cancel, ble_csv_tx_cancel_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *bl = lv_label_create(btn_cancel);
    lv_label_set_text(bl, "Cancel");
    lv_obj_center(bl);
}

static void ble_csv_tx_cancel_cb(lv_event_t *e)
{
    (void)e;
    ble_csv_tx_cancel();
}

static void ble_csv_tx_ui_update(const char *line1)
{
    if (!ui_tx_lbl || !line1)
        return;
    const uint32_t now = millis();
    if (g_tx_phase == 2 && now - g_tx_ui_ms < 120)
        return;
    g_tx_ui_ms = now;
    lv_label_set_text(ui_tx_lbl, line1);
    if (ui_tx_bar && g_tx_total > 0) {
        const uint32_t acked = sap6_ble_acks_ok() - g_tx_acks_base;
        int pct = (int)((acked * 100UL) / (unsigned long)g_tx_total);
        if (pct > 100)
            pct = 100;
        lv_bar_set_value(ui_tx_bar, pct, LV_ANIM_OFF);
    }
}

static void ble_csv_tx_finish(bool ok)
{
    g_tx_pending_start = false;
    sap6_ble_stream_cancel();
    sap6_ble_queue_reset();
    g_tx_phase = 0;
    g_tx_total = 0;
    g_tx_rows_sent = 0;
    g_tx_sd_file = false;
    g_tx_fpos = 0;
    g_tx_count_pos = 0;
    g_tx_eof = false;
    g_tx_prep_step = 0;
    g_tx_send_idx = 0;
    g_tx_use_ram = false;
    char b[72];
    if (ok) {
        const uint32_t acked = sap6_ble_acks_ok() - g_tx_acks_base;
        snprintf(b, sizeof(b), LV_SYMBOL_OK " STREAM fim %lu/%d",
                 (unsigned long)acked, g_tx_total > 0 ? g_tx_total : (int)acked);
    } else {
        snprintf(b, sizeof(b), LV_SYMBOL_WARNING " STREAM cancelled");
    }
    ble_csv_tx_ui_hide();
    set_fstatus(b);
    if (ui_lbl_count) {
        if (UI_COMPACT_HEADER)
            snprintf(b, sizeof(b), "#%d", pt_count);
        else
            snprintf(b, sizeof(b), "PTS:%d", pt_count);
        lv_label_set_text(ui_lbl_count, b);
    }
}

static void ble_csv_tx_cancel(void)
{
    ble_csv_tx_finish(false);
}

static void ble_csv_tx_hard_reset(void)
{
    g_tx_pending_start = false;
    sap6_ble_stream_cancel();
    sap6_ble_queue_reset();
    g_tx_phase = 0;
    g_tx_prep_step = 0;
    g_tx_total = 0;
    g_tx_rows_sent = 0;
    g_tx_send_idx = 0;
    g_tx_use_ram = false;
    g_tx_sd_file = false;
    g_tx_fpos = 0;
    g_tx_count_pos = 0;
    g_tx_eof = false;
    ble_csv_tx_ui_hide();
}

/** Conta linhas do CSV no SD em fatias (nao bloquear nem estourar stack). */
static bool ble_csv_tx_count_sd_slice(void)
{
    File f = SD.open(g_tx_csv_path, FILE_READ);
    if (!f)
        return true;
    if (g_tx_count_pos > 0)
        f.seek(g_tx_count_pos);
    for (int i = 0; i < 24 && !tx_file_at_eof(f); i++) {
        if (!csv_read_line(f, g_tx_csv_line, sizeof(g_tx_csv_line)))
            break;
        MeasPoint tmp;
        if (!csv_parse_line(g_tx_csv_line, tmp))
            continue;
        g_tx_total++;
        if (g_tx_total >= MAX_CSV_STREAM_ROWS)
            break;
    }
    g_tx_count_pos = f.position();
    const bool done = tx_file_at_eof(f) || g_tx_total >= MAX_CSV_STREAM_ROWS;
    f.close();
    if (done)
        g_tx_count_pos = 0;
    return done;
}

static void ble_csv_tx_prep_tick(void)
{
    char b[56];
    if (g_tx_prep_step == 0) {
        ble_csv_tx_ui_show();
        ble_csv_tx_ui_update("A preparar...");
        g_tx_prep_step = 1;
        return;
    }
    if (g_tx_prep_step == 1) {
        if (pt_count == 0 && sd_ready && active_csv[0])
            load_csv();
        if (pt_count > 0) {
            g_tx_use_ram = true;
            g_tx_sd_file = false;
            g_tx_total = pt_count;
            g_tx_prep_step = 3;
            return;
        }
        if (sd_ready && active_csv[0]) {
            strlcpy(g_tx_csv_path, active_csv, sizeof(g_tx_csv_path));
            g_tx_use_ram = false;
            g_tx_sd_file = true;
            g_tx_total = 0;
            g_tx_count_pos = 0;
            g_tx_prep_step = 2;
            snprintf(b, sizeof(b), "SD: counting...");
            ble_csv_tx_ui_update(b);
            return;
        }
        ble_csv_tx_finish(false);
        setup_tab_bt_ack("STREAM: no points");
        return;
    }
    if (g_tx_prep_step == 2) {
        if (!ble_csv_tx_count_sd_slice()) {
            snprintf(b, sizeof(b), "SD: count... %d", g_tx_total);
            ble_csv_tx_ui_update(b);
            return;
        }
        if (g_tx_total <= 0) {
            ble_csv_tx_finish(false);
            setup_tab_bt_ack("STREAM: empty CSV");
            return;
        }
        g_tx_prep_step = 3;
        return;
    }
    if (g_tx_prep_step == 3) {
        sap6_ble_stream_cancel();
        sap6_ble_queue_reset();
        g_tx_acks_base = sap6_ble_acks_ok();
        g_tx_rows_sent = 0;
        g_tx_send_idx = 0;
        g_tx_eof = false;
        g_tx_fpos = 0;
        g_tx_last_progress_ms = millis();
        g_tx_last_acked = 0;
        g_tx_ui_ms = 0;
        snprintf(b, sizeof(b), g_tx_use_ram ? "RAM: 0 / %d" : "SD: 0 / %d", g_tx_total);
        ble_csv_tx_ui_update(b);
        g_tx_phase = 2;
        g_tx_prep_step = 0;
    }
}

/** So marca prep — trabalho pesado em ble_csv_tx_prep_tick (1 passo/loop). */
static void ble_csv_tx_do_start(void)
{
    g_tx_pending_start = false;
    sap6_ble_stream_cancel();
    sap6_ble_queue_reset();
    g_tx_phase = 1;
    g_tx_prep_step = 0;
    g_tx_use_ram = false;
    g_tx_sd_file = false;
    g_tx_total = 0;
    g_tx_rows_sent = 0;
}

static bool ble_csv_tx_begin(void)
{
    if (!g_bt_stack_ready || !sap6_ble_connected())
        return false;
    if (g_tx_phase != 0 || g_tx_pending_start || sap6_ble_stream_active())
        ble_csv_tx_hard_reset();
    g_tx_pending_start = true;
    return true;
}

static bool ble_csv_tx_can_send_more(void)
{
    return !sap6_ble_waiting_ack() && sap6_ble_queue_depth() == 0;
}

static void ble_csv_tx_poll_streaming(void)
{
    if (g_tx_phase != 2)
        return;

    const uint32_t acked = sap6_ble_acks_ok() - g_tx_acks_base;
    if (acked > g_tx_last_acked) {
        g_tx_last_acked = acked;
        g_tx_last_progress_ms = millis();
    }

    if (g_tx_use_ram && g_tx_total > 0) {
        if (sap6_ble_waiting_ack() &&
            (millis() - g_tx_last_progress_ms) > 6000UL)
            sap6_ble_ack_stall_recover();

        /* So envia o proximo leg depois do ACK (evita 41/44 com fila a frente). */
        if (ble_csv_tx_can_send_more() && acked < (uint32_t)g_tx_total) {
            const int idx = (int)acked;
            if (idx >= 0 && idx < pt_count) {
                const MeasPoint &p = pts[idx];
                if (sap6_ble_try_send_leg(norm_deg360(p.yaw), p.pitch, p.roll, p.dist))
                    g_tx_last_progress_ms = millis();
            }
        }

        g_tx_send_idx = (int)acked;
        g_tx_rows_sent = g_tx_send_idx;

        char b[72];
        int pct = (int)((acked * 100UL) / (unsigned long)g_tx_total);
        if (pct > 100)
            pct = 100;
        snprintf(b, sizeof(b), "RAM %lu/%d  %d%%",
                 (unsigned long)acked, g_tx_total, pct);
        ble_csv_tx_ui_update(b);

        if (acked >= (uint32_t)g_tx_total &&
            sap6_ble_queue_depth() == 0 && !sap6_ble_waiting_ack())
            ble_csv_tx_finish(true);
        else if ((millis() - g_tx_last_progress_ms) > 120000UL) {
            snprintf(b, sizeof(b), "No ACK (%lu/%d) - Cancel",
                     (unsigned long)acked, g_tx_total);
            ble_csv_tx_ui_update(b);
            g_tx_last_progress_ms = millis() - 90000UL;
        }
        return;
    }

    if (!g_tx_sd_file || g_tx_total <= 0)
        return;

    if (!g_tx_eof && ble_csv_tx_can_send_more() &&
        acked >= (uint32_t)g_tx_rows_sent) {
        File f = SD.open(g_tx_csv_path, FILE_READ);
        if (!f) {
            ble_csv_tx_finish(false);
            setup_tab_bt_ack("STREAM: SD read fail");
            return;
        }
        if (g_tx_fpos > 0)
            f.seek(g_tx_fpos);

        if (tx_file_at_eof(f)) {
            g_tx_eof = true;
        } else if (csv_read_line(f, g_tx_csv_line, sizeof(g_tx_csv_line))) {
            MeasPoint p;
            if (csv_parse_line(g_tx_csv_line, p) &&
                sap6_ble_try_send_leg(norm_deg360(p.yaw), p.pitch, p.roll, p.dist))
                g_tx_rows_sent++;
            if (tx_file_at_eof(f))
                g_tx_eof = true;
        } else {
            g_tx_eof = true;
        }
        g_tx_fpos = f.position();
        f.close();
        g_tx_last_progress_ms = millis();
    }
    if (acked > g_tx_last_acked) {
        g_tx_last_acked = acked;
        g_tx_last_progress_ms = millis();
    }

    char b[72];
    int pct = (int)((acked * 100UL) / (unsigned long)g_tx_total);
    if (pct > 100)
        pct = 100;
    snprintf(b, sizeof(b), "SD %lu/%d  %d%%",
             (unsigned long)acked, g_tx_total, pct);
    ble_csv_tx_ui_update(b);

    if (sap6_ble_waiting_ack() && (millis() - g_tx_last_progress_ms) > 12000UL)
        sap6_ble_ack_stall_recover();

    if (g_tx_eof && g_tx_rows_sent > 0 && acked >= (uint32_t)g_tx_rows_sent &&
        sap6_ble_queue_depth() == 0 && !sap6_ble_waiting_ack())
        ble_csv_tx_finish(true);
    else if ((millis() - g_tx_last_progress_ms) > 120000UL) {
        snprintf(b, sizeof(b), "SD no ACK (%lu) - Cancel", (unsigned long)acked);
        ble_csv_tx_ui_update(b);
        g_tx_last_progress_ms = millis() - 90000UL;
    }
}

static void ble_csv_tx_poll(void)
{
    if (g_tx_pending_start)
        ble_csv_tx_do_start();

    if (!sap6_ble_connected()) {
        if (g_tx_phase != 0 || g_tx_pending_start)
            ble_csv_tx_cancel();
        return;
    }
    if (g_tx_phase == 1)
        ble_csv_tx_prep_tick();
    else if (g_tx_phase == 2)
        ble_csv_tx_poll_streaming();
}

static bool ble_csv_tx_busy(void)
{
    return g_tx_pending_start || g_tx_phase != 0 || sap6_ble_stream_active();
}
#endif /* ARDUINO_ARCH_ESP32 */

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
    p.dip_deg = EXPORT_DIP_DEG_NOMINAL;
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
    f.println(TD_CSV_HEADER);
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
    int skipped_extra = 0;
    int lines = 0;
    while (!tx_file_at_eof(f)) {
        if (pt_count >= MAX_PTS) {
            skipped_extra = 1;
            break;
        }
        char buf[256];
        if (!csv_read_line(f, buf, sizeof(buf)))
            break;
        MeasPoint &p = pts[pt_count];
        if (!csv_parse_line(buf, p))
            continue;
        if (p.id >= next_id) next_id = p.id + 1;
        pt_count++;
        if ((++lines & 7) == 0)
            yield();
    }
    f.close();
    sel_row = -1;
    DBG_PRINT("[SD] Loaded %d pts from %s\n", pt_count, active_csv);
    if (skipped_extra) {
        char msg[72];
        snprintf(msg, sizeof(msg),
                 LV_SYMBOL_WARNING " Table %d pts (max %d) — TX sends full CSV",
                 pt_count, MAX_PTS);
        set_fstatus(msg);
    }
}

#ifdef ARDUINO_ARCH_ESP32
/* ── Web portal callbacks — render device snapshot for the dashboard ────────── */
static void web_get_status_cb(web_portal::Status& st)
{
    st.wifi_running  = web_portal::running();
    st.bt_paired     = g_bt_stack_ready && g_bt_paired;
    st.bt_linked     = g_bt_stack_ready && sap6_ble_connected();
    st.wifi_clients  = web_portal::clients();
    st.point_count   = (uint32_t)pt_count;
    st.device_name   = BT_DEVICE_NAME;
    st.ap_ssid       = WIFI_AP_SSID;
    st.ap_password   = WIFI_AP_PASS;
    st.ap_ip         = web_portal::ap_ip();
    st.bt_local_mac  = g_bt_local_mac;
    st.bt_peer_mac   = g_bt_peer_mac;
    st.active_csv    = active_csv;
    st.fw_version    = FW_VERSION_STR;
}

namespace {
struct SinkCtx {
    web_portal::WriteChunk write;
    void*                  user;
    void append(const char* p, size_t n) { write(p, n, user); }
    void append(const char* p) { append(p, strlen(p)); }
};

/* Minimal Print-compatible adapter that forwards into the web sink. Reused for
 * the CSV snapshot so we can call the existing append_point_csv_br() helper. */
class WebPrint : public Print {
public:
    explicit WebPrint(SinkCtx* s) : sink(s) {}
    size_t write(uint8_t b) override          { char c = (char)b; sink->append(&c, 1); return 1; }
    size_t write(const uint8_t* b, size_t n) override
    { sink->append((const char*)b, n); return n; }
    SinkCtx* sink;
};

void json_emit_escaped(SinkCtx& s, const char* str)
{
    s.append("\"", 1);
    if (str) {
        for (const char* p = str; *p; ++p) {
            char c = *p;
            if (c == '\\' || c == '"') { s.append("\\", 1); s.append(&c, 1); }
            else if ((uint8_t)c < 0x20)  { s.append(" ", 1); }
            else                          { s.append(&c, 1); }
        }
    }
    s.append("\"", 1);
}
}  // anon namespace

static void web_write_points_csv_cb(web_portal::WriteChunk write, void* user)
{
    SinkCtx s{ write, user };
    WebPrint pr(&s);
    pr.println(TD_CSV_HEADER);
    for (int i = 0; i < pt_count; ++i)
        append_point_csv_br(pr, pts[i]);
}

static void web_write_points_json_cb(web_portal::WriteChunk write, void* user)
{
    SinkCtx s{ write, user };
    s.append("{\"points\":[", 11);
    char num[48];
    char ts[28];
    for (int i = 0; i < pt_count; ++i) {
        const MeasPoint& p = pts[i];
        if (i) s.append(",", 1);
        s.append("{\"id\":", 6);
        snprintf(num, sizeof(num), "%lu", (unsigned long)p.id);
        s.append(num);
        s.append(",\"type\":\"", 9);
        s.append(p.type == PT_NAV ? "Nav" : "Sample");
        s.append("\",\"dist\":", 9);
        snprintf(num, sizeof(num), "%.3f", (double)p.dist);
        s.append(num);
        s.append(",\"az\":", 6);
        snprintf(num, sizeof(num), "%.2f", (double)norm_deg360(p.yaw));
        s.append(num);
        s.append(",\"inc\":", 7);
        snprintf(num, sizeof(num), "%.2f", (double)p.pitch);
        s.append(num);
        s.append(",\"roll\":", 8);
        snprintf(num, sizeof(num), "%.2f", (double)norm_deg360(p.roll));
        s.append(num);
        s.append(",\"temp\":", 8);
        snprintf(num, sizeof(num), "%.1f", (double)p.temp_c);
        s.append(num);
        fmt_td_csv_timestamp(p.posix_sec, ts, sizeof(ts));
        s.append(",\"ts\":", 6);
        json_emit_escaped(s, ts);
        s.append("}", 1);
    }
    s.append("]}", 2);
}

static const web_portal::Callbacks g_web_cbs = {
    web_get_status_cb,
    web_write_points_csv_cb,
    web_write_points_json_cb,
};

static bool web_portal_enable(void)
{
    if (web_portal::running()) return true;
    bool ok = web_portal::start(WIFI_AP_SSID, WIFI_AP_PASS, g_web_cbs);
    if (ok) g_wifi_user_on = true;
    return ok;
}
static void web_portal_disable(void)
{
    web_portal::stop();
    g_wifi_user_on = false;
}
#endif // ARDUINO_ARCH_ESP32

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
    p.dist  = mm1_distance_at_m11_m(tof_dist_mm / 1000.0f);
    p.roll  = imu_roll;
    p.pitch = imu_inclination_deg;
    p.yaw   = imu_azimuth_deg;
    p.laser_ok = tof_ok;
    time_t tv = time(nullptr);
    if (tv > (time_t)1577836800L)
        p.posix_sec = (uint32_t)tv;
    else
        p.posix_sec = POSIX_FALLBACK_ANCHOR_SEC + (uint32_t)(millis() / 1000UL);
    p.dip_deg = EXPORT_DIP_DEG_NOMINAL;
    read_device_temp_c();
#ifdef ARDUINO_ARCH_ESP32
    p.temp_c = isfinite(g_live_temp_c) ? g_live_temp_c : (float)temperatureRead();
#else
    p.temp_c = 22.f;
#endif
    p.error_log[0] = '\0';
    if (!tof_ok)
        strlcpy(p.error_log, "Laser:N/A", sizeof(p.error_log));
    pt_count++;
    sel_row = -1;
    refresh_table();
#if defined(ARDUINO_ARCH_ESP32)
    sap6_ble_send_leg(norm_deg360(p.yaw), p.pitch, p.roll, p.dist);
#endif
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
    char path[40]; int n = 0;
    for (; n < 1000; n++) {
        snprintf(path, sizeof(path), "/mm1_black_%03d.csv", n);
        if (!SD.exists(path)) break;
    }
    File f=SD.open(path,FILE_WRITE);
    if (f) { f.println(TD_CSV_HEADER); f.close(); }
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
        else { strlcpy(active_csv,"/mm1_black_000.csv",sizeof(active_csv));
               File f=SD.open(active_csv,FILE_WRITE);
               if(f){f.println(TD_CSV_HEADER);f.close();}
               refresh_file_list(); }
        pt_count=0;sel_row=-1;next_id=1; load_csv(); refresh_table(); update_active_lbl();
    }
    char buf[48]; snprintf(buf,sizeof(buf), LV_SYMBOL_TRASH " Deleted %s", path+1);
    set_fstatus(buf);
}

static void refresh_setup_about_display(void)
{
    if (ui_lbl_setup_ver) {
        char b[48];
        snprintf(b, sizeof(b), "Firmware: %s", FW_VERSION);
        lv_label_set_text(ui_lbl_setup_ver, b);
    }
#ifdef ARDUINO_ARCH_ESP32
    if (ui_qr_fw_update) {
        lv_qrcode_update(ui_qr_fw_update, FW_UPDATE_URL,
                         (uint32_t)strlen(FW_UPDATE_URL));
    }
#endif
}

static void request_setup_sub_refresh(void)
{
    if (ui_active_tab == 3)
        g_setup_ui_refresh_req = true;
}

#ifdef ARDUINO_ARCH_ESP32
/** Refresh only the active SETUP sub-tab (called from loop, not tabview ISR path). */
static void refresh_setup_active_sub_tab(void)
{
    switch (g_setup_sub_idx) {
    case SETUP_SUB_ABOUT:
        refresh_setup_about_display();
        break;
    case SETUP_SUB_CAL:
        refresh_setup_cal_display();
        break;
    case SETUP_SUB_BT:
        refresh_setup_bt_status();
        break;
    default:
        break;
    }
}

static void setup_ui_refresh_service(void)
{
    if (!g_setup_ui_refresh_req)
        return;
    g_setup_ui_refresh_req = false;
    refresh_setup_active_sub_tab();
}
#endif

static void setup_sub_tab_changed_cb(lv_event_t *e)
{
    lv_obj_t *sub = lv_event_get_target(e);
    g_setup_sub_idx = (uint8_t)lv_tabview_get_tab_act(sub);
    if (ui_active_tab == 3) {
        lzr_sync_poll_gap_now();
        lzr_next_poll_ms = millis();
        request_setup_sub_refresh();
    }
}

static void tabview_changed_cb(lv_event_t *e)
{
    lv_obj_t *tv = lv_event_get_target(e);
    uint16_t tab = lv_tabview_get_tab_act(tv);
    ui_active_tab = (uint8_t)tab;
    lzr_sync_poll_gap_now();
    lzr_next_poll_ms = millis();
    if (tab == 1)
        refresh_sensor_display();
    else if (tab == 2) {
        refresh_file_list();
        update_active_lbl();
    } else if (tab == 3)
        request_setup_sub_refresh();
}

// ── Status / BT / periodic ───────────────────────────────────────────────────
static void bt_send_sd_file(const char *path_raw)
{
    (void)path_raw;
    setup_tab_bt_ack("Files: FILES tab or SD card");
}

static void update_status()
{
#ifdef ARDUINO_ARCH_ESP32
    bt_conn = g_bt_stack_ready && sap6_ble_connected();
    if (!g_bt_stack_ready) {
        lv_label_set_text(ui_lbl_bt, LV_SYMBOL_BLUETOOTH);
        lv_obj_set_style_text_color(ui_lbl_bt, lv_color_hex(C_BT_OFF), 0);
    } else {
        bt_refresh_bond_state();
        const char *bt_txt;
        uint32_t    bt_col;
        if (bt_conn) {
            bt_txt = UI_COMPACT_HEADER ? LV_SYMBOL_BLUETOOTH : LV_SYMBOL_BLUETOOTH " LINK";
            bt_col = C_SD_ON;
        } else if (g_bt_paired) {
            bt_txt = UI_COMPACT_HEADER ? LV_SYMBOL_BLUETOOTH : LV_SYMBOL_BLUETOOTH " KEY";
            bt_col = C_BT_ON;
        } else {
            bt_txt = LV_SYMBOL_BLUETOOTH;
            bt_col = C_BT_OFF;
        }
        lv_label_set_text(ui_lbl_bt, bt_txt);
        lv_obj_set_style_text_color(ui_lbl_bt, lv_color_hex(bt_col), 0);
    }
#else
    bt_conn = false;
    lv_label_set_text(ui_lbl_bt, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(ui_lbl_bt, lv_color_hex(C_BT_OFF), 0);
#endif
    lv_label_set_text(ui_lbl_sd,
        sd_ready ? (UI_COMPACT_HEADER ? LV_SYMBOL_SD_CARD : LV_SYMBOL_SD_CARD " SD")
                 : LV_SYMBOL_SD_CARD);
    lv_obj_set_style_text_color(ui_lbl_sd, lv_color_hex(sd_ready?C_SD_ON:C_SD_OFF), 0);

#ifdef ARDUINO_ARCH_ESP32
    if (ui_lbl_wifi) {
        if (web_portal::running()) {
            lv_label_set_text(ui_lbl_wifi, LV_SYMBOL_WIFI " AP");
            lv_obj_set_style_text_color(ui_lbl_wifi, lv_color_hex(C_SD_ON), 0);
            lv_obj_clear_flag(ui_lbl_wifi, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(ui_lbl_wifi, LV_OBJ_FLAG_HIDDEN);
        }
    }
#endif

    read_battery();
    char bb[16];
    if (UI_COMPACT_HEADER)
        snprintf(bb, sizeof(bb), "%d%%", bat_pct);
    else
        snprintf(bb, sizeof(bb), LV_SYMBOL_BATTERY_FULL " %d%%", bat_pct);
    lv_label_set_text(ui_lbl_bat, bb);
    lv_obj_set_style_text_color(ui_lbl_bat,
        lv_color_hex(bat_pct > 20 ? C_BAT_OK : C_BAT_LOW), 0);

    uint32_t s = millis() / 1000;
    char tb[12];
    if (UI_COMPACT_HEADER)
        snprintf(tb, sizeof(tb), "%02lu:%02lu", (unsigned long)((s / 60) % 60),
                 (unsigned long)(s % 60));
    else
        snprintf(tb, sizeof(tb), "%02lu:%02lu:%02lu", (unsigned long)((s / 3600) % 24),
                 (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
    lv_label_set_text(ui_lbl_time, tb);
}

static void handle_bt_cmd(const String &raw)
{
#ifdef ARDUINO_ARCH_ESP32
    if (!g_bt_stack_ready)
        return;
#endif
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
        bt_send_sd_file(rest.c_str());
        return;
    }
    if (cmd.equalsIgnoreCase("FILES")) {
        scan_csv_files();
        setup_tab_bt_ack("CSV list: FILES tab or SD");
        return;
    }
    if (cmd.equalsIgnoreCase("MEAS")) {
#ifdef ARDUINO_ARCH_ESP32
        if (ble_csv_tx_busy()) {
            setup_tab_bt_ack("Measure: wait for STREAM to finish");
            return;
        }
#endif
        if (pt_count >= MAX_PTS) {
            setup_tab_bt_ack("Measure: table full (max 100)");
            return;
        }
        add_point(PT_SAMPLE, true);
        char b[72];
#ifdef ARDUINO_ARCH_ESP32
        if (sap6_ble_connected())
            snprintf(b, sizeof(b), "Measure: shot #%d -> BLE (%d in table)",
                     pt_count > 0 ? pts[pt_count - 1].id : 0, pt_count);
        else
            snprintf(b, sizeof(b), "Measure: shot #%d (%d pts, BT not linked)",
                     pt_count > 0 ? pts[pt_count - 1].id : 0, pt_count);
#else
        snprintf(b, sizeof(b), "Measure: %d pts in table", pt_count);
#endif
        setup_tab_bt_ack(b);
        return;
    }
    if (cmd.equalsIgnoreCase("CLEAR")) {
        pt_count = 0;
        sel_row = -1;
        next_id = 1;
        refresh_table();
        setup_tab_bt_ack("Points cleared");
        return;
    }
    if (cmd.equalsIgnoreCase("LIST") || cmd.equalsIgnoreCase("EXPORT")) {
        setup_tab_bt_ack("Export CSV: WiFi portal or SD");
        return;
    }
#ifdef ARDUINO_ARCH_ESP32
    if (cmd.equalsIgnoreCase("STREAM")) {
        if (!ble_csv_tx_begin()) {
            setup_tab_bt_ack("STREAM: connect BT or no data");
            return;
        }
        setup_tab_bt_ack("STREAM: progress popup...");
        return;
    }
#endif
}

static void setup_tab_bt_ack(const char *msg)
{
    if (ui_lbl_setup_ack) lv_label_set_text(ui_lbl_setup_ack, msg);
}

static void setup_btn_meas_cb(lv_event_t *e)
{
    (void)e;
    handle_bt_cmd(String("MEAS"));
}

#ifdef ARDUINO_ARCH_ESP32
static void setup_btn_stream_cb(lv_event_t *e)
{
    (void)e;
    if (ble_csv_tx_busy())
        ble_csv_tx_hard_reset();
    if (!g_bt_stack_ready) {
        setup_tab_bt_ack("STREAM: BLE starting... try again.");
        return;
    }
    if (!sap6_ble_connected()) {
        setup_tab_bt_ack("STREAM: connect TopoDroid/SexyTopo");
        return;
    }
    handle_bt_cmd(String("STREAM"));
}

static void setup_btn_unpair_cb(lv_event_t *e)
{
    (void)e;
    bt_clear_all_bonds();
    setup_tab_bt_ack("Bonds cleared on MM1 - remove on phone and pair again.");
}

static void setup_btn_ble_restart_cb(lv_event_t *e)
{
    (void)e;
    setup_tab_bt_ack("Restarting BLE...");
    sap6_ble_restart(BT_DEVICE_NAME);
    sap6_ble_get_mac_str(g_bt_local_mac, sizeof(g_bt_local_mac));
    bt_refresh_bond_state();
    g_bt_stack_ready = sap6_ble_stack_ready();
    refresh_setup_bt_status();
    if (g_bt_stack_ready)
        setup_tab_bt_ack("BLE OK - look for SAP6_0001 in nRF/TopoDroid");
    else
        setup_tab_bt_ack("BLE failed - reboot MM1");
}

/** Portal soft-AP: off by default; toggle in SETUP WiFi (runs in loop()). */
static void setup_btn_wifi_restart_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;
#ifdef ARDUINO_ARCH_ESP32
    if (web_portal::running()) {
        g_wifi_portal_stop_req = true;
        setup_tab_bt_ack("Stopping portal...");
    } else {
        g_wifi_portal_restart_req = true;
        setup_tab_bt_ack("Starting AP portal...");
    }
#endif
}
#endif

/** Refresh BT / Wi-Fi labels on SETUP sub-tabs. */
static void refresh_setup_bt_status(void)
{
#ifdef ARDUINO_ARCH_ESP32
    const bool linked = g_bt_stack_ready && sap6_ble_connected();
    if (ui_lbl_setup_bt_stat) {
        const char *st;
        uint32_t    col;
        if (!g_bt_stack_ready) {
            st  = "Bluetooth: BLE off (reflash?)";
            col = C_GREY;
        } else if (linked) {
            st  = "Bluetooth: SAP6 connected";
            col = C_SD_ON;
        } else if (g_bt_paired) {
            st  = "Bluetooth: bond saved";
            col = C_BT_ON;
        } else {
            st  = "Bluetooth: advertising SAP6_0001 (BLE)";
            col = C_GREY;
        }
        lv_label_set_text(ui_lbl_setup_bt_stat, st);
        lv_obj_set_style_text_color(ui_lbl_setup_bt_stat, lv_color_hex(col), 0);
    }
    if (ui_lbl_setup_bt_mac)
        lv_label_set_text(ui_lbl_setup_bt_mac,
                          g_bt_local_mac[0] ? g_bt_local_mac : UI_NA);
    if (ui_lbl_setup_bt_pair)
        lv_label_set_text(ui_lbl_setup_bt_pair,
            linked ? "yes (session)" : (g_bt_paired ? "yes (saved)" : "no"));
    if (ui_lbl_setup_bt_peer)
        lv_label_set_text(ui_lbl_setup_bt_peer,
                          g_bt_peer_mac[0] ? g_bt_peer_mac : UI_NA);
    if (ui_lbl_setup_bt_diag) {
        char st[72];
        char buf[200];
        sap6_ble_format_status(st, sizeof(st));
        snprintf(buf, sizeof(buf),
                 "%s | legs %lu ACK %lu fila %lu",
                 st,
                 (unsigned long)sap6_ble_legs_sent(),
                 (unsigned long)sap6_ble_acks_ok(),
                 (unsigned long)sap6_ble_queue_depth());
        lv_label_set_text(ui_lbl_setup_bt_diag, buf);
    }
    if (ui_lbl_setup_wifi_stat) {
        const char *st;
        uint32_t    col;
        if (web_portal::running()) {
            st  = "Wi-Fi: AP portal on (BLE kept)";
            col = C_SD_ON;
        } else {
            st  = "Wi-Fi: off (default)";
            col = C_GREY;
        }
        lv_label_set_text(ui_lbl_setup_wifi_stat, st);
        lv_obj_set_style_text_color(ui_lbl_setup_wifi_stat, lv_color_hex(col), 0);
    }
    if (ui_lbl_setup_wifi_info) {
        char buf[200];
        if (web_portal::running()) {
            snprintf(buf, sizeof(buf),
                     "Portal AP %s\nPassword %s\nhttp://%s/\nClients: %u\n"
                     "(no Internet — phone file export only)",
                     WIFI_AP_SSID, WIFI_AP_PASS, web_portal::ap_ip(),
                     (unsigned)web_portal::clients());
        } else {
            snprintf(buf, sizeof(buf),
                     "Portal AP %s\nPassword %s\nTap Enable AP portal\n"
                     "TopoDroid BLE stays active.",
                     WIFI_AP_SSID, WIFI_AP_PASS);
        }
        lv_label_set_text(ui_lbl_setup_wifi_info, buf);
    }
    if (ui_btn_wifi_portal) {
        lv_obj_t *lbl = lv_obj_get_child(ui_btn_wifi_portal, 0);
        if (lbl)
            lv_label_set_text(lbl,
                web_portal::running() ? "Disable AP portal" : "Enable AP portal");
    }
#endif
}

#ifdef ARDUINO_ARCH_ESP32
static void wifi_portal_service_requests(void)
{
    if (g_wifi_portal_stop_req) {
        g_wifi_portal_stop_req = false;
        web_portal_disable();
        setup_tab_bt_ack("AP portal disabled");
        refresh_setup_bt_status();
        return;
    }
    if (!g_wifi_portal_restart_req)
        return;
    g_wifi_portal_restart_req = false;

    if (web_portal::running()) {
        refresh_setup_bt_status();
        return;
    }

    (void)esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
    delay(80);
    if (web_portal_enable()) {
        char b[96];
        snprintf(b, sizeof(b), "Portal AP http://%s/", web_portal::ap_ip());
        setup_tab_bt_ack(b);
    } else {
        setup_tab_bt_ack("AP portal failed - USB power or retry");
    }
    refresh_setup_bt_status();
}
#endif

static void periodic_cb(lv_timer_t*t)
{
    (void)t;
    update_status();
}

static void sensor_timer_cb(lv_timer_t *t)
{
    (void)t;
    if (ui_is_setup_sensor_tab())
        refresh_sensor_display();
#ifdef ARDUINO_ARCH_ESP32
    else if (ui_active_tab == 3) {
        static unsigned long last_setup_ms = 0;
        const unsigned long now = millis();
        if ((now - last_setup_ms) >= 2000UL) {
            last_setup_ms = now;
            request_setup_sub_refresh();
        }
    }
#endif
}

static void prefs_save_az_offset(void)
{
#ifdef ARDUINO_ARCH_ESP32
    Preferences p;
    if (p.begin(PREFS_NAMESPACE, false)) {
        p.putFloat(PREFS_KEY_AZ_OFS, g_azimuth_offset_deg);
        p.end();
    }
#endif
}

static void prefs_load_az_offset(void)
{
    g_azimuth_offset_deg = AZIMUTH_OFFSET_DEG;
#ifdef ARDUINO_ARCH_ESP32
    Preferences p;
    if (p.begin(PREFS_NAMESPACE, true)) {
        float v = p.getFloat(PREFS_KEY_AZ_OFS, NAN);
        p.end();
        if (isfinite(v) && v >= -180.f && v <= 180.f)
            g_azimuth_offset_deg = v;
    }
#endif
}

static void prefs_save_backlight(void)
{
#ifdef ARDUINO_ARCH_ESP32
    Preferences p;
    if (p.begin(PREFS_NAMESPACE, false)) {
        p.putUChar(PREFS_KEY_BL_PCT, g_backlight_pct);
        p.end();
    }
#endif
}

static void prefs_load_backlight(void)
{
    g_backlight_pct = BL_DEFAULT_PCT;
#ifdef ARDUINO_ARCH_ESP32
    Preferences p;
    if (p.begin(PREFS_NAMESPACE, true)) {
        const uint8_t v = p.getUChar(PREFS_KEY_BL_PCT, BL_DEFAULT_PCT);
        p.end();
        if (v >= 10 && v <= 100)
            g_backlight_pct = v;
    }
#endif
}

static void tft_bl_init(void)
{
#if defined(TFT_BL) && defined(ARDUINO_ARCH_ESP32)
    ledcSetup(TFT_BL_LEDC_CH, 5000, 8);
    ledcAttachPin(TFT_BL, TFT_BL_LEDC_CH);
    g_bl_pwm_attached = true;
#endif
}

static void tft_bl_apply(uint8_t pct)
{
    if (pct < 10)
        pct = 10;
    if (pct > 100)
        pct = 100;
    g_backlight_pct = pct;

#if defined(TFT_BL) && defined(ARDUINO_ARCH_ESP32)
    if (g_bl_pwm_attached) {
        const uint32_t pwm = ((uint32_t)pct * 255U + 50U) / 100U;
        ledcWrite(TFT_BL_LEDC_CH, (uint8_t)pwm);
    }
#elif defined(TFT_BL) && defined(TFT_BACKLIGHT_ON)
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, (pct >= 50) ? TFT_BACKLIGHT_ON
                                     : (TFT_BACKLIGHT_ON == HIGH ? LOW : HIGH));
#endif

    if (ui_slider_bl) {
        g_bl_ui_sync = true;
        lv_slider_set_value(ui_slider_bl, pct, LV_ANIM_OFF);
        g_bl_ui_sync = false;
    }
    refresh_setup_bl_label();
}

static void refresh_setup_bl_label(void)
{
    if (!ui_lbl_setup_bl)
        return;
    char b[96];
#ifdef ARDUINO_ARCH_ESP32
    snprintf(b, sizeof(b), "%u%%", (unsigned)g_backlight_pct);
#else
    snprintf(b, sizeof(b), "Brightness %u%%", (unsigned)g_backlight_pct);
#endif
    lv_label_set_text(ui_lbl_setup_bl, b);
}

static void bl_adjust(int delta_pct)
{
    int v = (int)g_backlight_pct + delta_pct;
    if (v < 10)
        v = 10;
    if (v > 100)
        v = 100;
    tft_bl_apply((uint8_t)v);
    prefs_save_backlight();
    setup_tab_bt_ack("Brightness saved");
#ifdef ARDUINO_ARCH_ESP32
    play_button_ack();
#endif
}

static void setup_bl_delta_cb(lv_event_t *e)
{
    const int delta = (int)(intptr_t)lv_event_get_user_data(e);
    bl_adjust(delta);
}

static void setup_bl_slider_cb(lv_event_t *e)
{
    if (g_bl_ui_sync)
        return;
    const lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_VALUE_CHANGED && code != LV_EVENT_RELEASED)
        return;
    const int v = (int)lv_slider_get_value(lv_event_get_target(e));
    tft_bl_apply((uint8_t)v);
    if (code == LV_EVENT_RELEASED)
        prefs_save_backlight();
}

static void setup_bl_rst_cb(lv_event_t *e)
{
    (void)e;
    tft_bl_apply(BL_DEFAULT_PCT);
    prefs_save_backlight();
    refresh_setup_bl_label();
    setup_tab_bt_ack("Default brightness restored");
#ifdef ARDUINO_ARCH_ESP32
    play_button_ack();
#endif
}

static void refresh_setup_az_offs_label(void)
{
    if (!ui_lbl_setup_az_offs) return;
    char b[120];
#ifdef ARDUINO_ARCH_ESP32
    snprintf(b, sizeof(b), "Azimuth %+0.1f deg", (double)g_azimuth_offset_deg);
#else
    snprintf(b, sizeof(b), "Azimuth %+0.1f deg", (double)g_azimuth_offset_deg);
#endif
    lv_label_set_text(ui_lbl_setup_az_offs, b);
}

static void az_offs_adjust(float delta_deg)
{
    g_azimuth_offset_deg += delta_deg;
    if (g_azimuth_offset_deg < -180.f) g_azimuth_offset_deg = -180.f;
    if (g_azimuth_offset_deg > 180.f) g_azimuth_offset_deg = 180.f;
    prefs_save_az_offset();
    refresh_setup_az_offs_label();
    setup_tab_cal_ack("Azimuth saved");
#ifdef ARDUINO_ARCH_ESP32
    play_button_ack();
#endif
}

static void setup_az_offs_delta_cb(lv_event_t *e)
{
    int cdeg = (int)(intptr_t)lv_event_get_user_data(e); /* cents of degree */
    az_offs_adjust(cdeg / 100.0f);
}

static void setup_az_offs_rst_cb(lv_event_t *e)
{
    (void)e;
    g_azimuth_offset_deg = AZIMUTH_OFFSET_DEG;
    prefs_save_az_offset();
    refresh_setup_az_offs_label();
    setup_tab_cal_ack("Default azimuth restored");
#ifdef ARDUINO_ARCH_ESP32
    play_button_ack();
#endif
}

static void setup_tab_cal_ack(const char *msg)
{
    if (ui_lbl_setup_cal_ack)
        lv_label_set_text(ui_lbl_setup_cal_ack, msg);
}

static const char *imu_fusion_quality_str(int acc)
{
    switch (acc) {
    case 3: return "excellent";
    case 2: return "good";
    case 1: return "fair";
    case 0: return "low";
    case -1: return "waiting for fusion";
    default: return "unknown";
    }
}

static void refresh_setup_cal_display(void)
{
    if (!ui_lbl_setup_imu_head && !ui_lbl_setup_imu_qual && !ui_lbl_setup_imu_grav)
        return;

    char b[96];
    if (!imu_ok) {
        if (ui_lbl_setup_imu_head)
            lv_label_set_text(ui_lbl_setup_imu_head, "IMU: offline");
        if (ui_lbl_setup_imu_qual)
            lv_label_set_text(ui_lbl_setup_imu_qual, "Fusion: " UI_NA);
        if (ui_lbl_setup_imu_grav)
            lv_label_set_text(ui_lbl_setup_imu_grav, "|g|: " UI_NA);
        return;
    }

    if (ui_lbl_setup_imu_head) {
        snprintf(b, sizeof(b), "Hdg %.1f  inc %.1f",
                 (double)imu_azimuth_deg, (double)imu_inclination_deg);
        lv_label_set_text(ui_lbl_setup_imu_head, b);
    }
    if (ui_lbl_setup_imu_qual) {
        if (imu_rv_accuracy < 0) {
            lv_label_set_text(ui_lbl_setup_imu_qual,
                "Fusion: move device (figure-8) until acc 0-3");
            lv_obj_set_style_text_color(ui_lbl_setup_imu_qual, lv_color_hex(C_HDR_LINE), 0);
        } else {
            snprintf(b, sizeof(b), "Fusion: %s (acc %d)",
                     imu_fusion_quality_str(imu_rv_accuracy), imu_rv_accuracy);
            lv_label_set_text(ui_lbl_setup_imu_qual, b);
            const uint32_t col = (imu_rv_accuracy >= 2) ? C_SD_ON
                                 : (imu_rv_accuracy >= 1) ? C_HDR_LINE : C_SD_OFF;
            lv_obj_set_style_text_color(ui_lbl_setup_imu_qual, lv_color_hex(col), 0);
        }
    }
    if (ui_lbl_setup_imu_grav) {
        if (imu_grav_mag < 0.5f) {
            lv_label_set_text(ui_lbl_setup_imu_grav,
                "|g| waiting (accel report) - open SENSOR tab");
            lv_obj_set_style_text_color(ui_lbl_setup_imu_grav, lv_color_hex(C_HDR_LINE), 0);
        } else {
            snprintf(b, sizeof(b), "|g| %.2f m/s2 (target ~9.81)", (double)imu_grav_mag);
            lv_label_set_text(ui_lbl_setup_imu_grav, b);
            const bool ok_g = (imu_grav_mag >= 8.5f && imu_grav_mag <= 11.0f);
            lv_obj_set_style_text_color(ui_lbl_setup_imu_grav,
                                        lv_color_hex(ok_g ? C_SD_ON : C_SD_OFF), 0);
        }
    }
}

static void setup_az_zero_here_cb(lv_event_t *e)
{
    (void)e;
    if (!imu_ok) {
        setup_tab_cal_ack("IMU offline");
        return;
    }
    g_azimuth_offset_deg -= imu_azimuth_deg;
    while (g_azimuth_offset_deg > 180.f)
        g_azimuth_offset_deg -= 360.f;
    while (g_azimuth_offset_deg < -180.f)
        g_azimuth_offset_deg += 360.f;
    prefs_save_az_offset();
    refresh_setup_az_offs_label();
    refresh_setup_cal_display();
    setup_tab_cal_ack("Current heading = 0");
#ifdef ARDUINO_ARCH_ESP32
    play_button_ack();
#endif
}

static void setup_btn_lzr_test_cb(lv_event_t *e)
{
    (void)e;
    if (!lzr_post_init) {
        setup_tab_cal_ack("Laser not started");
        return;
    }
    const bool ok = lzr_measure_once_blocking();
    char b[72];
    if (ok && isfinite(lzr_last_m))
        snprintf(b, sizeof(b), "Laser OK: %.3f m", (double)lzr_last_m);
    else
        snprintf(b, sizeof(b), "Laser FAIL - bright target >10 cm");
    setup_tab_cal_ack(b);
#ifdef ARDUINO_ARCH_ESP32
    play_button_ack();
#endif
}

#if LZR_PROTO_ILIASAM
static void setup_btn_lzr_zero_cb(lv_event_t *e)
{
    (void)e;
    lzr_uart_drain();
    lzr_port.print('C');
    lzr_port.flush();
    setup_tab_cal_ack("Calib zero (C) sent - white target >10 cm");
#ifdef ARDUINO_ARCH_ESP32
    play_button_ack();
#endif
}
#endif

/** Linha de botoes compactos (3 colunas, ~32% largura) para caber em 480px. */
static lv_obj_t *setup_mk_btn_row(lv_obj_t *parent, int h)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_width(row, SCREEN_W - 24);
    lv_obj_set_height(row, h);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_style_pad_column(row, 4, 0);
    lv_obj_set_layout(row, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    return row;
}

static lv_obj_t *setup_mk_btn(lv_obj_t *row, const char *lbl,
                              lv_event_cb_t cb, void *ud, uint32_t col)
{
    lv_obj_t *b = lv_btn_create(row);
    lv_obj_set_width(b, lv_pct(32));
    lv_obj_set_height(b, LV_PCT(100));
    lv_obj_set_style_bg_color(b, lv_color_hex(col), 0);
    lv_obj_set_style_radius(b, 4, 0);
    lv_obj_set_style_pad_all(b, 2, 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t *lb = lv_label_create(b);
    lv_label_set_text(lb, lbl);
    lv_obj_set_style_text_font(lb, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lb, lv_color_hex(C_WHITE), 0);
    lv_obj_center(lb);
    return b;
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
    ui_hdr_bar = hdr;
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
    lv_obj_set_layout(hdr, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(hdr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(hdr, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_left(hdr, 4, 0);
    lv_obj_set_style_pad_right(hdr, 4, 0);

    auto hdr_mk_strip = [](lv_obj_t *parent) -> lv_obj_t * {
        lv_obj_t *s = lv_obj_create(parent);
        lv_obj_set_height(s, LV_PCT(100));
        lv_obj_set_style_bg_opa(s, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(s, 0, 0);
        lv_obj_set_style_pad_all(s, 0, 0);
        lv_obj_set_style_pad_column(s, UI_COMPACT_HEADER ? 4 : 8, 0);
        lv_obj_set_layout(s, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(s, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(s, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(s, LV_OBJ_FLAG_SCROLLABLE);
        return s;
    };

    lv_obj_t *hdr_left = hdr_mk_strip(hdr);
    lv_obj_set_flex_grow(hdr_left, 1);

    lv_obj_t *lt = lv_label_create(hdr_left);
    lv_label_set_text(lt, "MM1-BLACK");
    lv_obj_set_style_text_color(lt, lv_color_hex(C_HDR_LINE), 0);
    lv_obj_set_style_text_font(lt, &lv_font_montserrat_12, 0);

    ui_lbl_count = lv_label_create(hdr_left);
    lv_label_set_text(ui_lbl_count, "0");
    lv_obj_set_style_text_color(ui_lbl_count, lv_color_hex(C_GREY), 0);
    lv_obj_set_style_text_font(ui_lbl_count, &lv_font_montserrat_12, 0);

    lv_obj_t *hdr_right = hdr_mk_strip(hdr);
    lv_obj_set_flex_grow(hdr_right, 0);
    lv_obj_set_flex_align(hdr_right, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    ui_lbl_bat = lv_label_create(hdr_right);
    lv_label_set_text(ui_lbl_bat, "100%");
    lv_obj_set_style_text_color(ui_lbl_bat, lv_color_hex(C_BAT_OK), 0);
    lv_obj_set_style_text_font(ui_lbl_bat, &lv_font_montserrat_12, 0);

    ui_lbl_sd = lv_label_create(hdr_right);
    lv_label_set_text(ui_lbl_sd, LV_SYMBOL_SD_CARD);
    lv_obj_set_style_text_font(ui_lbl_sd, &lv_font_montserrat_12, 0);

#ifdef ARDUINO_ARCH_ESP32
    ui_lbl_wifi = lv_label_create(hdr_right);
    lv_label_set_text(ui_lbl_wifi, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_font(ui_lbl_wifi, &lv_font_montserrat_12, 0);
    lv_obj_add_flag(ui_lbl_wifi, LV_OBJ_FLAG_HIDDEN);
#endif

    ui_lbl_bt = lv_label_create(hdr_right);
    lv_label_set_text(ui_lbl_bt, LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_font(ui_lbl_bt, &lv_font_montserrat_12, 0);

    ui_lbl_time = lv_label_create(hdr_right);
    lv_label_set_text(ui_lbl_time, "00:00");
    lv_obj_set_style_text_color(ui_lbl_time, lv_color_hex(C_TEXT), 0);
    lv_obj_set_style_text_font(ui_lbl_time, &lv_font_montserrat_12, 0);

    // ── Tabs ─────────────────────────────────────────────────────────────
    const int TAB_Y=36, TAB_H=SCREEN_H-TAB_Y, STRIP_H=36;
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
    lv_obj_set_style_pad_all(tbtns, 2, LV_PART_ITEMS);
    lv_obj_set_style_pad_gap(tbtns, 0, LV_PART_MAIN);
    lv_obj_set_style_text_font(tbtns,
        UI_COMPACT_HEADER ? &lv_font_montserrat_14 : &lv_font_montserrat_16, LV_PART_ITEMS);
    lv_obj_set_width(tbtns, SCREEN_W);
    tabview_enable_tab_taps(tv);

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
    lv_obj_t *tp = lv_tabview_add_tab(tv,
        UI_COMPACT_HEADER ? LV_SYMBOL_LIST : LV_SYMBOL_LIST " PTS");
    lv_obj_set_style_bg_color(tp, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(tp, 0, 0);
    lv_obj_clear_flag(tp, LV_OBJ_FLAG_SCROLLABLE);

    ui_tbl_pts = lv_table_create(tp);
    lv_obj_set_size(ui_tbl_pts, SCREEN_W, TABLE_H);
    lv_obj_set_pos(ui_tbl_pts, 0, 0);
    lv_table_set_col_cnt(ui_tbl_pts, 5);
    if (UI_COMPACT_HEADER) {
        lv_table_set_col_width(ui_tbl_pts, 0, 44);
        lv_table_set_col_width(ui_tbl_pts, 1, 88);
        lv_table_set_col_width(ui_tbl_pts, 2, 28);
        lv_table_set_col_width(ui_tbl_pts, 3, 76);
        lv_table_set_col_width(ui_tbl_pts, 4, 76);
    } else {
        lv_table_set_col_width(ui_tbl_pts, 0, 64);
        lv_table_set_col_width(ui_tbl_pts, 1, 108);
        lv_table_set_col_width(ui_tbl_pts, 2, 40);
        lv_table_set_col_width(ui_tbl_pts, 3, 92);
        lv_table_set_col_width(ui_tbl_pts, 4, 92);
    }
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
#ifdef ARDUINO_ARCH_ESP32
    /* STREAM: CSV activo no SD (ou RAM) -> legs SAP6; TopoDroid ligado. */
    make_btn(bar_p, C_BTN_BT,   LV_SYMBOL_UPLOAD " TX",  setup_btn_stream_cb);
#endif

    // ── SENSOR tab ───────────────────────────────────────────────────────
    lv_obj_t *ts = lv_tabview_add_tab(tv,
        UI_COMPACT_HEADER ? LV_SYMBOL_EYE_OPEN : LV_SYMBOL_EYE_OPEN " SNS");
    lv_obj_set_style_bg_color(ts, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(ts, 10, 0);
    lv_obj_clear_flag(ts, LV_OBJ_FLAG_SCROLLABLE);

    auto sec_lbl = [&](lv_obj_t *p, int y, const char *t) {
        lv_obj_t *l = lv_label_create(p);
        lv_label_set_text(l, t);
        lv_obj_align(l, LV_ALIGN_TOP_LEFT, 0, y);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(C_HDR_LINE), 0);
    };
    auto val_lbl = [&](lv_obj_t *p, int y) -> lv_obj_t * {
        lv_obj_t *l = lv_label_create(p);
        lv_label_set_long_mode(l, LV_LABEL_LONG_DOT);
        lv_obj_set_width(l, SCREEN_W - 20);
        lv_obj_align(l, LV_ALIGN_TOP_LEFT, 0, y);
        lv_label_set_text(l, "---");
        lv_obj_set_style_text_font(l, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(C_TEXT), 0);
        return l;
    };

    sec_lbl(ts, 0,
#if LZR_SHARE_USB_UART
            "LASER  UART0  RXD0=IO3  TXD0=IO1");
#else
            "UART LASER");
#endif
    ui_lbl_tof_val = val_lbl(ts, 22);
    sec_lbl(ts, 58, "AZIMUTH / INCLINATION / ROLL");
    ui_lbl_imu_val = val_lbl(ts, 80);
    sec_lbl(ts, 116, "STATUS");
    ui_lbl_sens_stat = val_lbl(ts, 138);
    sec_lbl(ts, 174, "TEMPERATURE (MCU)");
    ui_lbl_sens_temp = val_lbl(ts, 196);

    // ── FILES tab ────────────────────────────────────────────────────────
    lv_obj_t *tf = lv_tabview_add_tab(tv,
        UI_COMPACT_HEADER ? LV_SYMBOL_SD_CARD : LV_SYMBOL_SD_CARD " FILE");
    lv_obj_set_style_bg_color(tf, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(tf, 6, 0);
    lv_obj_clear_flag(tf, LV_OBJ_FLAG_SCROLLABLE);

    ui_lbl_active = lv_label_create(tf);
    lv_label_set_long_mode(ui_lbl_active, LV_LABEL_LONG_DOT);
    lv_obj_set_width(ui_lbl_active, SCREEN_W - 12);
    lv_obj_align(ui_lbl_active, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_color(ui_lbl_active, lv_color_hex(C_REF_S), 0);

    const int FTH = CONTENT_H - 12 - 22 - 4 - 44 - 4 - 22;
    ui_tbl_files = lv_table_create(tf);
    lv_obj_set_size(ui_tbl_files, SCREEN_W - 12, FTH);
    lv_obj_align(ui_tbl_files, LV_ALIGN_TOP_LEFT, 0, 26);
    lv_table_set_col_cnt(ui_tbl_files, 2);
    lv_table_set_col_width(ui_tbl_files, 0, SCREEN_W - 12 - 90);
    lv_table_set_col_width(ui_tbl_files, 1, 84);
    lv_obj_set_style_bg_color(ui_tbl_files, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_top(ui_tbl_files, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(ui_tbl_files, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(ui_tbl_files, 8, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(ui_tbl_files, 8, LV_PART_ITEMS);
    lv_obj_add_event_cb(ui_tbl_files, file_draw_cb, LV_EVENT_DRAW_PART_BEGIN, nullptr);
    lv_obj_add_event_cb(ui_tbl_files, file_click_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    auto fbtn = [&](lv_obj_t *p, uint32_t c, const char *t, lv_event_cb_t cb) {
        lv_obj_t *b = lv_btn_create(p);
        lv_obj_set_size(b, 148, 40);
        lv_obj_set_style_bg_color(b, lv_color_hex(c), 0);
        lv_obj_set_style_radius(b, 8, 0);
        lv_obj_set_style_shadow_width(b, 4, 0);
        lv_obj_set_style_shadow_opa(b, LV_OPA_30, 0);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_t *l = lv_label_create(b);
        lv_label_set_text(l, t);
        lv_obj_center(l);
        lv_obj_set_style_text_color(l, lv_color_hex(C_WHITE), 0);
    };
    int by = 26 + FTH + 4;
    lv_obj_t *fr = lv_obj_create(tf);
    lv_obj_set_size(fr, SCREEN_W - 12, 44);
    lv_obj_align(fr, LV_ALIGN_TOP_LEFT, 0, by);
    lv_obj_set_style_bg_opa(fr, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(fr, 0, 0);
    lv_obj_set_style_pad_all(fr, 0, 0);
    lv_obj_set_layout(fr, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(fr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(fr, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(fr, LV_OBJ_FLAG_SCROLLABLE);
    fbtn(fr, C_BTN_NEW, LV_SYMBOL_PLUS " NEW", btn_new_file_cb);
    fbtn(fr, C_BTN_USE, LV_SYMBOL_OK " USE", btn_use_file_cb);
    fbtn(fr, C_BTN_DEL, LV_SYMBOL_TRASH " DEL", btn_del_file_cb);

    ui_lbl_fstatus = lv_label_create(tf);
    lv_label_set_long_mode(ui_lbl_fstatus, LV_LABEL_LONG_DOT);
    lv_obj_set_width(ui_lbl_fstatus, SCREEN_W - 12);
    lv_obj_align(ui_lbl_fstatus, LV_ALIGN_TOP_LEFT, 0, by + 48);
    lv_label_set_text(ui_lbl_fstatus, "Ready");
    lv_obj_set_style_text_color(ui_lbl_fstatus, lv_color_hex(C_GREY), 0);

    // ── SETUP — About | Bright | Cal | BT ───────────────────────────────
    {
        lv_obj_t *tsetup = lv_tabview_add_tab(tv,
            UI_COMPACT_HEADER ? LV_SYMBOL_SETTINGS : LV_SYMBOL_SETTINGS " SET");
        lv_obj_set_style_bg_color(tsetup, lv_color_hex(C_BG), 0);
        lv_obj_set_style_pad_all(tsetup, 2, 0);
        lv_obj_clear_flag(tsetup, LV_OBJ_FLAG_SCROLLABLE);

        const int SUB_STRIP = 32;
        lv_obj_t *sub_tv = lv_tabview_create(tsetup, LV_DIR_TOP, SUB_STRIP);
        lv_obj_set_size(sub_tv, SCREEN_W - 8, CONTENT_H - 8);
        lv_obj_align(sub_tv, LV_ALIGN_TOP_MID, 0, 4);
        lv_obj_t *stb = lv_tabview_get_tab_btns(sub_tv);
        lv_obj_set_style_bg_color(stb, lv_color_hex(C_TOPBAR), 0);
        lv_obj_set_style_bg_opa(stb, LV_OPA_COVER, 0);
        lv_obj_set_style_border_side(stb, LV_BORDER_SIDE_BOTTOM, 0);
        lv_obj_set_style_border_width(stb, 1, 0);
        lv_obj_set_style_border_color(stb, lv_color_hex(C_BORDER), 0);
        lv_obj_set_style_pad_hor(stb, 2, LV_PART_ITEMS);
        lv_obj_add_event_cb(sub_tv, setup_sub_tab_changed_cb, LV_EVENT_VALUE_CHANGED, nullptr);
        tabview_enable_tab_taps(sub_tv);

        /* About: version + hint on top, QR at bottom (portrait-safe). */
        lv_obj_t *t_about = lv_tabview_add_tab(sub_tv, "About");
        lv_obj_set_layout(t_about, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(t_about, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_all(t_about, 8, 0);
        lv_obj_set_style_pad_row(t_about, 6, 0);
        lv_obj_set_style_pad_bottom(t_about, 12, 0);
        lv_obj_add_flag(t_about, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(t_about, LV_DIR_VER);

        ui_lbl_setup_ver = lv_label_create(t_about);
        lv_obj_set_width(ui_lbl_setup_ver, SCREEN_W - 24);
        lv_label_set_long_mode(ui_lbl_setup_ver, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(ui_lbl_setup_ver, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ui_lbl_setup_ver, lv_color_hex(C_TEXT), 0);

        lv_obj_t *about_hint = lv_label_create(t_about);
        lv_label_set_text(about_hint,
            "MM1-BLACK - USB firmware update\n"
            "Scan QR on a PC (Chrome/Edge).\n"
            "Serial: send VERSION at 9600 baud.");
        lv_obj_set_width(about_hint, SCREEN_W - 24);
        lv_label_set_long_mode(about_hint, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_color(about_hint, lv_color_hex(C_GREY), 0);
        lv_obj_set_style_text_font(about_hint, &lv_font_montserrat_12, 0);

        lv_obj_t *about_spacer = lv_obj_create(t_about);
        lv_obj_set_width(about_spacer, 1);
        lv_obj_set_height(about_spacer, 4);
        lv_obj_set_flex_grow(about_spacer, 1);
        lv_obj_set_style_bg_opa(about_spacer, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(about_spacer, 0, 0);
        lv_obj_clear_flag(about_spacer, LV_OBJ_FLAG_SCROLLABLE);

#ifdef ARDUINO_ARCH_ESP32
        {
            const int qr_sz = UI_COMPACT_HEADER ? 96 : 108;
            ui_qr_fw_update = lv_qrcode_create(t_about, qr_sz,
                                               lv_color_hex(0x111827),
                                               lv_color_hex(0xFFFFFF));
            lv_obj_set_style_border_color(ui_qr_fw_update, lv_color_hex(C_BORDER), 0);
            lv_obj_set_style_border_width(ui_qr_fw_update, 1, 0);
            lv_obj_set_style_pad_all(ui_qr_fw_update, 4, 0);
        }
#endif
        refresh_setup_about_display();

        /* --- Brightness --- */
        lv_obj_t *t_disp = lv_tabview_add_tab(sub_tv, "Bright");
        lv_obj_set_layout(t_disp, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(t_disp, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_all(t_disp, 12, 0);
        lv_obj_set_style_pad_row(t_disp, 10, 0);
        lv_obj_clear_flag(t_disp, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *bl_ttl = lv_label_create(t_disp);
        lv_label_set_text(bl_ttl, "Display brightness");
        lv_obj_set_style_text_font(bl_ttl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(bl_ttl, lv_color_hex(C_HDR_LINE), 0);

        ui_lbl_setup_bl = lv_label_create(t_disp);
        lv_label_set_text(ui_lbl_setup_bl, UI_NA);
        lv_obj_set_style_text_font(ui_lbl_setup_bl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(ui_lbl_setup_bl, lv_color_hex(C_TEXT), 0);
        refresh_setup_bl_label();

        ui_slider_bl = lv_slider_create(t_disp);
        lv_obj_set_width(ui_slider_bl, SCREEN_W - 40);
        lv_slider_set_range(ui_slider_bl, 10, 100);
        lv_slider_set_value(ui_slider_bl, g_backlight_pct, LV_ANIM_OFF);
        lv_obj_add_event_cb(ui_slider_bl, setup_bl_slider_cb, LV_EVENT_VALUE_CHANGED, nullptr);
        lv_obj_add_event_cb(ui_slider_bl, setup_bl_slider_cb, LV_EVENT_RELEASED, nullptr);

        lv_obj_t *bl_hint = lv_label_create(t_disp);
        lv_label_set_text(bl_hint, "Saved automatically when you release the slider.");
        lv_obj_set_width(bl_hint, SCREEN_W - 32);
        lv_label_set_long_mode(bl_hint, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_color(bl_hint, lv_color_hex(C_GREY), 0);

        /* --- Calibracao: IMU BNO086 + azimute + laser --- */
        lv_obj_t *t_cal = lv_tabview_add_tab(sub_tv, "Cal");
        lv_obj_set_layout(t_cal, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(t_cal, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_all(t_cal, 8, 0);
        lv_obj_set_style_pad_row(t_cal, 5, 0);
        lv_obj_add_flag(t_cal, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(t_cal, LV_DIR_VER);

        lv_obj_t *imu_ttl = lv_label_create(t_cal);
        lv_label_set_text(imu_ttl, "IMU (BNO086)");
        lv_obj_set_style_text_font(imu_ttl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(imu_ttl, lv_color_hex(C_HDR_LINE), 0);

        ui_lbl_setup_imu_head = lv_label_create(t_cal);
        lv_label_set_text(ui_lbl_setup_imu_head, UI_NA);
        lv_obj_set_style_text_font(ui_lbl_setup_imu_head, &lv_font_montserrat_14, 0);

        ui_lbl_setup_imu_qual = lv_label_create(t_cal);
        lv_label_set_text(ui_lbl_setup_imu_qual, UI_NA);
        lv_obj_set_style_text_font(ui_lbl_setup_imu_qual, &lv_font_montserrat_12, 0);

        ui_lbl_setup_imu_grav = lv_label_create(t_cal);
        lv_label_set_text(ui_lbl_setup_imu_grav, UI_NA);
        lv_obj_set_style_text_font(ui_lbl_setup_imu_grav, &lv_font_montserrat_12, 0);

        lv_obj_t *geom_lbl = lv_label_create(t_cal);
        {
            char geom_buf[128];
            snprintf(geom_buf, sizeof(geom_buf),
                "M11 ref: laser -%.2f mm, base -%.2f mm (total %.2f mm); "
                "IMU pivot -%.2f mm on -X",
                (double)MM1_LASER_BASE_OFFSET_MM, (double)MM1_M11_BASE_OFFSET_MM,
                (double)(MM1_LASER_BASE_OFFSET_MM + MM1_M11_BASE_OFFSET_MM),
                (double)MM1_IMU_TO_M11_X_MM);
            lv_label_set_text(geom_lbl, geom_buf);
        }
        lv_obj_set_width(geom_lbl, SCREEN_W - 24);
        lv_label_set_long_mode(geom_lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_color(geom_lbl, lv_color_hex(C_GREY), 0);
        lv_obj_set_style_text_font(geom_lbl, &lv_font_montserrat_12, 0);

        lv_obj_t *imu_hint = lv_label_create(t_cal);
        lv_label_set_text(imu_hint,
            "BNO086: no factory button - rotate in figure-8 (~30 s) until "
            "Fusion acc is 2-3. acc -1 = not ready yet (not broken). "
            "|g| needs accel report (stay on this tab or SENSOR).");
        lv_obj_set_width(imu_hint, SCREEN_W - 24);
        lv_label_set_long_mode(imu_hint, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_color(imu_hint, lv_color_hex(C_GREY), 0);
        lv_obj_set_style_text_font(imu_hint, &lv_font_montserrat_12, 0);

        lv_obj_t *az_ttl = lv_label_create(t_cal);
        lv_label_set_text(az_ttl, "Azimuth offset");
        lv_obj_set_style_text_font(az_ttl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(az_ttl, lv_color_hex(C_HDR_LINE), 0);

        ui_lbl_setup_az_offs = lv_label_create(t_cal);
        lv_label_set_text(ui_lbl_setup_az_offs, UI_NA);
        lv_obj_set_style_text_font(ui_lbl_setup_az_offs, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ui_lbl_setup_az_offs, lv_color_hex(C_TEXT), 0);
        refresh_setup_az_offs_label();

        lv_obj_t *az_row1 = setup_mk_btn_row(t_cal, 34);
        setup_mk_btn(az_row1, "-5", setup_az_offs_delta_cb, (void *)(intptr_t)-500, C_HDR_LINE);
        setup_mk_btn(az_row1, "-1", setup_az_offs_delta_cb, (void *)(intptr_t)-100, C_HDR_LINE);
        setup_mk_btn(az_row1, "-.1", setup_az_offs_delta_cb, (void *)(intptr_t)-10, C_HDR_LINE);

        lv_obj_t *az_row2 = setup_mk_btn_row(t_cal, 34);
        setup_mk_btn(az_row2, "+.1", setup_az_offs_delta_cb, (void *)(intptr_t)10, C_HDR_LINE);
        setup_mk_btn(az_row2, "+1", setup_az_offs_delta_cb, (void *)(intptr_t)100, C_HDR_LINE);
        setup_mk_btn(az_row2, "+5", setup_az_offs_delta_cb, (void *)(intptr_t)500, C_HDR_LINE);

        lv_obj_t *az_act = setup_mk_btn_row(t_cal, 34);
        setup_mk_btn(az_act, "Az=0", setup_az_zero_here_cb, nullptr, C_BTN_BT);
        setup_mk_btn(az_act, "Default", setup_az_offs_rst_cb, nullptr, C_GREY);

        lv_obj_t *lzr_ttl = lv_label_create(t_cal);
        lv_label_set_text(lzr_ttl, "Laser");
        lv_obj_set_style_text_font(lzr_ttl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lzr_ttl, lv_color_hex(C_HDR_LINE), 0);

        lv_obj_t *lzr_hint = lv_label_create(t_cal);
#if LZR_PROTO_ILIASAM
        lv_label_set_text(lzr_hint,
            "X-40/701A: white target >10 cm, then Zero (C). Test confirms DIST.");
#else
        lv_label_set_text(lzr_hint,
            "M01/U86 (0xAA): no factory zero in firmware; use bright target >10 cm "
            "and SENSOR tab. Calib C only on X-40 (see MEMORY_LASER.md).");
#endif
        lv_obj_set_width(lzr_hint, SCREEN_W - 24);
        lv_label_set_long_mode(lzr_hint, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_color(lzr_hint, lv_color_hex(C_GREY), 0);
        lv_obj_set_style_text_font(lzr_hint, &lv_font_montserrat_12, 0);

        lv_obj_t *lzr_row = setup_mk_btn_row(t_cal, 34);
        setup_mk_btn(lzr_row, "Test", setup_btn_lzr_test_cb, nullptr, C_BTN_BT);
#if LZR_PROTO_ILIASAM
        setup_mk_btn(lzr_row, "Zero C", setup_btn_lzr_zero_cb, nullptr, C_HDR_LINE);
#endif

        ui_lbl_setup_cal_ack = lv_label_create(t_cal);
        lv_label_set_long_mode(ui_lbl_setup_cal_ack, LV_LABEL_LONG_DOT);
        lv_obj_set_width(ui_lbl_setup_cal_ack, SCREEN_W - 20);
        lv_label_set_text(ui_lbl_setup_cal_ack, UI_NA);
        lv_obj_set_style_text_color(ui_lbl_setup_cal_ack, lv_color_hex(C_GREY), 0);

        refresh_setup_cal_display();

#ifdef ARDUINO_ARCH_ESP32
        auto setup_kv = [](lv_obj_t *parent, const char *key,
                           const char *val_def) -> lv_obj_t * {
            lv_obj_t *row = lv_obj_create(parent);
            lv_obj_set_width(row, SCREEN_W - 28);
            lv_obj_set_height(row, LV_SIZE_CONTENT);
            lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
            lv_obj_set_style_border_width(row, 0, 0);
            lv_obj_set_style_pad_all(row, 0, 0);
            lv_obj_set_layout(row, LV_LAYOUT_FLEX);
            lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
            lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_t *kl = lv_label_create(row);
            lv_label_set_text(kl, key);
            lv_obj_set_width(kl, 88);
            lv_obj_set_style_text_color(kl, lv_color_hex(C_GREY), 0);
            lv_obj_t *vl = lv_label_create(row);
            lv_label_set_text(vl, val_def);
            lv_obj_set_width(vl, SCREEN_W - 28 - 88);
            lv_label_set_long_mode(vl, LV_LABEL_LONG_DOT);
            return vl;
        };

        /* --- Bluetooth --- */
        lv_obj_t *t_bt = lv_tabview_add_tab(sub_tv, "BT");
        lv_obj_set_layout(t_bt, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(t_bt, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_all(t_bt, 6, 0);
        lv_obj_set_style_pad_row(t_bt, 4, 0);
        lv_obj_add_flag(t_bt, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(t_bt, LV_DIR_VER);

        ui_lbl_setup_bt_stat = lv_label_create(t_bt);
        lv_label_set_text(ui_lbl_setup_bt_stat, UI_NA);
        lv_obj_set_width(ui_lbl_setup_bt_stat, SCREEN_W - 24);
        lv_obj_set_style_text_font(ui_lbl_setup_bt_stat, &lv_font_montserrat_14, 0);

        lv_obj_t *bt_info = lv_obj_create(t_bt);
        lv_obj_set_width(bt_info, SCREEN_W - 16);
        lv_obj_set_height(bt_info, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(bt_info, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(bt_info, 0, 0);
        lv_obj_set_style_pad_all(bt_info, 0, 0);
        lv_obj_set_layout(bt_info, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(bt_info, LV_FLEX_FLOW_COLUMN);
        lv_obj_clear_flag(bt_info, LV_OBJ_FLAG_SCROLLABLE);

        (void)setup_kv(bt_info, "Name", BT_DEVICE_NAME);
        ui_lbl_setup_bt_mac  = setup_kv(bt_info, "MAC", UI_NA);
        ui_lbl_setup_bt_pair = setup_kv(bt_info, "Bond", UI_NA);
        ui_lbl_setup_bt_peer = setup_kv(bt_info, "Peer", UI_NA);

        lv_obj_t *btbar1 = setup_mk_btn_row(t_bt, 36);
        setup_mk_btn(btbar1, "Measure", setup_btn_meas_cb, nullptr, C_BTN_BT);
        setup_mk_btn(btbar1, "TX", setup_btn_stream_cb, nullptr, C_BTN_BT);
        lv_obj_t *btbar2 = setup_mk_btn_row(t_bt, 36);
        setup_mk_btn(btbar2, "Restart BLE", setup_btn_ble_restart_cb, nullptr, C_BTN_BT);
        setup_mk_btn(btbar2, "Unpair", setup_btn_unpair_cb, nullptr, C_BTN_DEL);

        ui_lbl_setup_ack = lv_label_create(t_bt);
        lv_label_set_long_mode(ui_lbl_setup_ack, LV_LABEL_LONG_DOT);
        lv_obj_set_width(ui_lbl_setup_ack, SCREEN_W - 20);
        lv_label_set_text(ui_lbl_setup_ack,
            "Measure=1 shot (laser+BLE). TX=CSV on SD. Files: FILES tab.");
        lv_obj_set_style_text_color(ui_lbl_setup_ack, lv_color_hex(C_GREY), 0);

        ui_lbl_setup_bt_diag = lv_label_create(t_bt);
        lv_label_set_text(ui_lbl_setup_bt_diag, UI_NA);
        lv_obj_set_width(ui_lbl_setup_bt_diag, SCREEN_W - 24);
        lv_label_set_long_mode(ui_lbl_setup_bt_diag, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_color(ui_lbl_setup_bt_diag, lv_color_hex(C_GREY), 0);

        refresh_setup_bt_status();
#endif
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

/** Splash 480×320 always at rotation 1; restore TFT_ROTATION before UI (see e504a60). */
static void show_boot_splash_tft(void)
{
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.startWrite();
    tft.setAddrWindow(0, 0, MIRA_SPLASH_W, MIRA_SPLASH_H);
    tft.pushColors(reinterpret_cast<uint16_t *>(const_cast<uint8_t *>(mira_splash_map)),
                   (uint32_t)MIRA_SPLASH_W * MIRA_SPLASH_H, true);
    tft.endWrite();
    tft.setRotation(TFT_ROTATION);
#ifdef ARDUINO_ARCH_ESP32
    const unsigned long t0 = millis();
    play_boot_chime();
    const unsigned long el = millis() - t0;
    if (el < SPLASH_MS)
        delay(SPLASH_MS - el);
#else
    delay(SPLASH_MS);
#endif
}

static void ble_boot_service(void)
{
    if (!g_ble_boot_pending)
        return;
    g_ble_boot_pending = false;
    sap6_ble_begin(BT_DEVICE_NAME);
    sap6_ble_get_mac_str(g_bt_local_mac, sizeof(g_bt_local_mac));
    bt_refresh_bond_state();
    g_bt_stack_ready = sap6_ble_stack_ready();
}


void setup()
{
#ifdef ARDUINO_ARCH_ESP32
#if LZR_SHARE_USB_UART
    esp_log_level_set("*", ESP_LOG_NONE);
#endif
#endif
#if !LZR_SHARE_USB_UART
    Serial.begin(115200);
    DBG_PRINT("\n[MM1-BLACK] Boot\n");
#endif

    tft.init();
#ifdef ARDUINO_ARCH_ESP32
    prefs_load_backlight();
    tft_bl_init();
    tft_bl_apply(g_backlight_pct);
#endif
    tft.fillScreen(TFT_BLACK);
#ifdef ARDUINO_ARCH_ESP32
    audio_init_hw();
#endif

    show_boot_splash_tft();
    tft.fillScreen(TFT_BLACK);
    tft_touch_apply_cal();
    pinMode(USER_BUTTON_PIN, INPUT_PULLUP);

#ifdef ARDUINO_ARCH_ESP32
    WiFi.persistent(false);
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
#endif

    /* LVGL before SD/IMU/laser — sensor_init blocks up to ~4 s; UI must paint first (5bb9b56). */
    lv_init();
    lvgl_buf = (lv_color_t *)malloc((size_t)SCREEN_W * 10 * sizeof(lv_color_t));
    if (!lvgl_buf) {
        tft.fillScreen(TFT_RED);
        tft.setTextColor(TFT_WHITE, TFT_RED);
        tft.drawString("LVGL mem fail", 10, 10, 2);
        while (1)
            delay(1000);
    }
    lv_disp_draw_buf_init(&draw_buf, lvgl_buf, nullptr, SCREEN_W * 10);

    static lv_disp_drv_t dd;
    lv_disp_drv_init(&dd);
    dd.hor_res=SCREEN_W; dd.ver_res=SCREEN_H;
    dd.flush_cb=disp_flush; dd.draw_buf=&draw_buf;
    lv_disp_t *disp = lv_disp_drv_register(&dd);

    lv_theme_t *th = lv_theme_default_init(disp,
        lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_TEAL),
        false, &lv_font_montserrat_14);
    lv_disp_set_theme(disp, th);

    static lv_indev_drv_t id;
    lv_indev_drv_init(&id);
    id.type=LV_INDEV_TYPE_POINTER; id.read_cb=touch_read;
    lv_indev_drv_register(&id);

    prefs_load_az_offset();
#ifndef ARDUINO_ARCH_ESP32
    prefs_load_backlight();
    tft_bl_apply(g_backlight_pct);
#endif
    build_ui();
    lv_obj_invalidate(lv_scr_act());
    lv_refr_now(nullptr);
    lv_timer_handler();

    sd_init();
    sensor_init();

    if (sd_ready) {
        if (!SD.exists(active_csv)) {
            File f = SD.open(active_csv, FILE_WRITE);
            if (f) {
                f.println(TD_CSV_HEADER);
                f.close();
            }
        }
        load_csv();
        refresh_table();
        update_active_lbl();
    }

    lv_timer_create(periodic_cb, 1000, nullptr);
    lv_timer_create(sensor_timer_cb, 250, nullptr);
#if !LZR_SHARE_USB_UART
    Serial.println("[MM1-BLACK] Ready");
#endif
}

/* Botão físico — BRIC5: 1.º toque = só laser vermelho; 2.º toque = bip + medida + apaga laser. */
static int user_btn_last_raw = HIGH;
static int user_btn_stable = HIGH;
static unsigned long user_btn_edge_ms = 0;
static bool user_btn_cap_armed = false;
static unsigned long user_btn_cap_armed_ms = 0;
static unsigned long user_btn_last_tap_ms = 0;

#ifndef BTN_TAP_MIN_MS
#define BTN_TAP_MIN_MS 280UL
#endif

static void user_btn_cap_arm(void)
{
    user_btn_cap_armed = true;
    user_btn_cap_armed_ms = millis();
    lzr_btn_aim_active = true;
    lzr_aim_on();
    cap_ui_set_state(CAP_UI_AIM);
    set_fstatus(LV_SYMBOL_EYE_OPEN " AIM - press again to CAPTURE");
}

static void user_btn_cap_disarm(const char *msg)
{
    user_btn_cap_armed = false;
    lzr_shutdown_beam();
    cap_ui_set_state(CAP_UI_IDLE);
    if (msg)
        set_fstatus(msg);
}

static void user_btn_cap_do_capture(void)
{
    user_btn_cap_armed = false;
    /* Keep beam + LASER_ON keepalive during UART measure (do not clear aim yet). */

    const bool got = lzr_measure_once_blocking();
    lzr_shutdown_beam();

    if (pt_count >= MAX_PTS) {
        play_error_sound();
        cap_ui_result_pulse(false);
        set_fstatus(LV_SYMBOL_WARNING " Table full");
        return;
    }
    if (!got) {
        play_error_sound();
        cap_ui_result_pulse(false);
        char buf[72];
        snprintf(buf, sizeof(buf),
                 LV_SYMBOL_WARNING " No reading (rx=%lu) - SETUP Sensor?",
                 (unsigned long)lzr_rx_bytes_total);
        set_fstatus(buf);
        return;
    }

    play_capture_sound();
    const int n0 = pt_count;
    add_point(PT_SAMPLE, false);
    cap_ui_result_pulse(true);
    if (pt_count > n0) {
        char buf[56];
        snprintf(buf, sizeof(buf), LV_SYMBOL_OK " Shot #%u  %.3f m",
                 (unsigned)pts[pt_count - 1].id, (double)pts[pt_count - 1].dist);
        set_fstatus(buf);
    }
}

/** One debounced press (LOW): 1st = laser only; 2nd = capture sound + measure. */
static void user_btn_on_press(void)
{
#ifdef ARDUINO_ARCH_ESP32
    unsigned long now = millis();
    if ((now - user_btn_last_tap_ms) < BTN_TAP_MIN_MS)
        return;
    user_btn_last_tap_ms = now;

    if (!user_btn_cap_armed) {
        user_btn_cap_arm();
        return;
    }

    user_btn_cap_do_capture();
#else
    (void)0;
#endif
}

static void user_btn_cap_tick(unsigned long now)
{
    if (!user_btn_cap_armed)
        return;
    if ((now - user_btn_cap_armed_ms) >= BTN_CAP_ARM_TIMEOUT_MS)
        user_btn_cap_disarm(LV_SYMBOL_CLOSE " Aim cancelled (timeout)");
}

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
            user_btn_on_press();
    }

    user_btn_cap_tick(m);

#ifdef ARDUINO_ARCH_ESP32
    cap_ui_tick(m);
    lzr_aim_laser_keepalive_tick(m);
#endif

    lzr_loop_tick(millis());
    poll_imu();

#ifdef ARDUINO_ARCH_ESP32
    ble_boot_service();
    wifi_portal_service_requests();
    setup_ui_refresh_service();
    serial_cmd_poll();
    sap6_ble_poll();
    ble_csv_tx_poll();
    sap6_process_pending_cmds();
    /* Web portal HTTP server (no-op when softAP is off). */
    web_portal::loop();
#endif

    lv_timer_handler();
    delay(5);
}
