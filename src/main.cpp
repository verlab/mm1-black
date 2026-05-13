/**
 * MM1-BLACK  —  Smart tape / UART laser range + IMU
 * ESP32 CYD  |  ST7796 480×320  |  LVGL 8.3
 *
 * Flat point list; TopoDroid-compatible CSV table. Physical button: hold for laser polls;
 * release to log a point (short = shot, ≥650 ms = nav). Debounce 50 ms.
 * Two point types:
 *   S (Sample)    – measurement samples
 *   N (Navigation) – reference points for transforms
 *
 * Tabs: POINTS | SENSOR | FILES | SETUP
 *
 * BT SPP defaults to TopoDroid “DistoX” (classic A3) name & 8‑byte DISTO_PACKET_DATA shots; override BT_DEVICE_NAME.
 * Also: MEAS | CLEAR | LIST | EXPORT | FILES | FILE_SEND,<path> text commands (newline-terminated).
 * TopoDroid CSV on SD as mm1_black_XXX.csv for backup/import.
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
#include <BluetoothSerial.h>
#ifdef ARDUINO_ARCH_ESP32
#include <Preferences.h>
#include <cinttypes>
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
/* “Dip” column ≈ local magnetic inclination (nominal); no standalone magnetometer — nominal value for TopoDroid import. */
#ifndef EXPORT_DIP_DEG_NOMINAL
#define EXPORT_DIP_DEG_NOMINAL (29.88f)
#endif
#ifndef BT_DEVICE_NAME
/** TopoDroid classic BT only treats model string **exactly** `DistoX` as DISTO_A3 RFCOMM — same UUID as generic SPP. */
#define BT_DEVICE_NAME "DistoX"
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
#define C_BTN_BT    0x1565C0u
#define C_FILE_ACT  0xC8E6C9u
#define C_REF_S     0x1565C0u
#define C_TYPE_S    0x00695Cu   // sample text
#define C_TYPE_N    0x6A1B9Au   // nav text
#define C_BAT_OK    0x2E7D32u
#define C_BAT_LOW   0xD32F2Fu

// ── Data / CSV ───────────────────────────────────────────────────────────────
// TopoDroid-style column header (two spaces before “Measurement Type”).
#define TD_CSV_HEADER \
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
#ifdef ARDUINO_ARCH_ESP32
/** Unget for BT stream mux (DistoX reply vs TopoDroid ACK). */
static int16_t               g_bt_spill_byte       = -1;
static uint8_t               g_bt_dx_stage       = 0;
static uint8_t               g_bt_dx_b0           = 0;
static uint32_t              g_bt_dx_partial_ms = 0;
/** Sequence bit toggle for TopoDroid DistoX PACKET_DATA (bit7). */
static uint8_t               g_distox_pkt_seq       = 0;
#endif
static char active_csv[40] = "/mm1_black_000.csv";
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
static lv_obj_t *ui_lbl_setup_deg   = nullptr;
static lv_obj_t *ui_lbl_setup_qual  = nullptr;
static lv_obj_t *ui_lbl_setup_grav  = nullptr;
static lv_obj_t *ui_lbl_setup_imu   = nullptr;
static lv_obj_t *ui_lbl_setup_lzr   = nullptr;
static lv_obj_t *ui_lbl_setup_hint  = nullptr;
static lv_obj_t *ui_lbl_setup_ack   = nullptr;
static lv_obj_t *ui_lbl_setup_az_offs = nullptr;

/** Precisão estimada do vetor de rotação SH-2 (rad → graus no ecrã SETUP). */
static float       imu_rv_accuracy_deg = NAN;
/** Accelerometer magnitude (m/s²); ~9.81 at rest for gravity sanity on SETUP tab. */
static float       imu_accel_mag_mss = NAN;
/** Azimuth offset added after atan2; NVS persists on ESP32 (SETUP tab). Loaded at boot. */
static float       g_azimuth_offset_deg = AZIMUTH_OFFSET_DEG;

// Tab order: 0=POINTS, 1=SENSOR, 2=FILES, 3=SETUP (must match lv_tabview_add_tab order).
static uint8_t     ui_active_tab    = 0;
static uint32_t    lzr_poll_gap_ms  = POLL_INTERVAL_MS;

// Forward declarations
static void sd_init();
static void sensor_init();
static void prefs_load_az_offset();
static void refresh_table();
static void refresh_sensor_display();
static void refresh_setup_display();
static void refresh_setup_az_offs_label();
static void set_fstatus(const char *msg);
static void audio_init_hw();
static void play_boot_chime();
static void play_button_ack();
static void add_point(PtType type, bool sync_laser_before);
static void handle_bt_cmd(const String &raw);

#ifdef ARDUINO_ARCH_ESP32
static void poll_bt_ascii_and_distox(void);
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

/** SSP numeric comparison — phone shows a 6-digit code; MCU auto-accepts AND mirrors it on SETUP BT. */
static void bt_ssp_confirm_cb(uint32_t pin)
{
#if !LZR_SHARE_USB_UART
    Serial.printf("[BT] SSP confirm code %" PRIu32 " (reply yes)\n", pin);
#endif
    bt_post_setup_banner_fmt(LV_SYMBOL_BLUETOOTH
                             " Pair code %06" PRIu32 " — confirming (must match phone).",
                             pin);
    SerialBT.confirmReply(true);
}

static void bt_auth_cmpl_cb(boolean success)
{
#if !LZR_SHARE_USB_UART
    Serial.printf("[BT] pairing %s\n", success ? "OK" : "FAILED");
#endif
    if (success)
        bt_post_setup_banner_fmt(LV_SYMBOL_OK " Bluetooth paired.");
    else
        bt_post_setup_banner_fmt(LV_SYMBOL_WARNING " Pairing failed — remove MM1/DistoX pair on phone,"
                                " reboot device, retry.");
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
    imu_azimuth_deg = norm_deg360(atan2f(wx, wy) * r2d + g_azimuth_offset_deg);
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
        } else if (sensorValue.sensorId == SH2_ACCELEROMETER) {
            float ax = sensorValue.un.accelerometer.x;
            float ay = sensorValue.un.accelerometer.y;
            float az = sensorValue.un.accelerometer.z;
            imu_accel_mag_mss = sqrtf(ax * ax + ay * ay + az * az);
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
            snprintf(buf, sizeof(buf), "No reading");
        }
        lv_label_set_text(ui_lbl_tof_val, buf);
    }
    if (ui_lbl_imu_val) {
        snprintf(buf, sizeof(buf), "Az: %.1f\xC2\xB0   Inc: %.1f\xC2\xB0   Roll: %.1f\xC2\xB0",
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
    if (!ui_lbl_setup_deg) return;

    if (!imu_ok) {
        lv_label_set_text(ui_lbl_setup_deg, "Hdg err: — (IMU off)");
        lv_label_set_text(ui_lbl_setup_qual, "Q: —");
        lv_label_set_text(ui_lbl_setup_grav, "|g|: —");
    } else if (isfinite(imu_rv_accuracy_deg)) {
        char b[96];
        snprintf(b, sizeof(b), "Hdg err: ±%.1f°", (double)imu_rv_accuracy_deg);
        lv_label_set_text(ui_lbl_setup_deg, b);
        const char *band = "Q: calibrating…";
        if (imu_rv_accuracy_deg < 3.f)
            band = "Q: excellent";
        else if (imu_rv_accuracy_deg < 5.f)
            band = "Q: good";
        else if (imu_rv_accuracy_deg < 10.f)
            band = "Q: fair";
        else
            band = "Q: poor";
        lv_label_set_text(ui_lbl_setup_qual, band);
        if (!isfinite(imu_accel_mag_mss)) {
            lv_label_set_text(ui_lbl_setup_grav, "|g|: …");
        } else {
            float e = fabsf(imu_accel_mag_mss - 9.81f);
            if (e < 2.0f)
                snprintf(b, sizeof(b), "|g|: %.2f m/s²", (double)imu_accel_mag_mss);
            else if (e < 4.5f)
                snprintf(b, sizeof(b), "|g|: %.2f (still)", (double)imu_accel_mag_mss);
            else
                snprintf(b, sizeof(b), "|g|: %.2f ?", (double)imu_accel_mag_mss);
            lv_label_set_text(ui_lbl_setup_grav, b);
        }
    } else {
        lv_label_set_text(ui_lbl_setup_deg, "Hdg err: — (move IMU)");
        lv_label_set_text(ui_lbl_setup_qual, "Q: acquiring…");
        if (!isfinite(imu_accel_mag_mss))
            lv_label_set_text(ui_lbl_setup_grav, "|g|: …");
        else {
            char b[96];
            snprintf(b, sizeof(b), "|g|: %.2f m/s²", (double)imu_accel_mag_mss);
            lv_label_set_text(ui_lbl_setup_grav, b);
        }
    }

    if (ui_lbl_setup_imu) {
        char b[128];
        if (imu_ok)
            snprintf(b, sizeof(b), "Az %.1f°  Inc %.1f°  Roll %.1f°",
                     (double)imu_azimuth_deg, (double)imu_inclination_deg, (double)imu_roll);
        else
            strlcpy(b, "Az / Inc / Roll: —", sizeof(b));
        lv_label_set_text(ui_lbl_setup_imu, b);
    }
    if (ui_lbl_setup_lzr) {
        char b[96];
        if (tof_ok)
            snprintf(b, sizeof(b), "LZR: %.3f m", (double)(tof_dist_mm / 1000.f));
        else
            strlcpy(b, "LZR: —", sizeof(b));
        lv_label_set_text(ui_lbl_setup_lzr, b);
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
    while (f.available() && pt_count < MAX_PTS) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;
        if (line.startsWith("#GPS")) continue;
        if (line.length() > 0 && line.charAt(0) == '#')
            continue;
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

#ifdef ARDUINO_ARCH_ESP32
// ── TopoDroid DistoX A3 classic BT ────────────────────────────────────────────
// Device.btnameToType("DistoX") → DISTO_A3 → DistoXA3Comm RFCOMM (standard SPP UUID).

static constexpr uint8_t kTopoDistoxPktData = 0x01;

static inline uint16_t topo_distox_bearing_word(float yaw_deg360)
{
    float bn = yaw_deg360;
    if (!isfinite(bn))
        bn = 0.f;
    bn = norm_deg360(bn);
    unsigned long w = (unsigned long)((double)bn * 32768.0 / 180.0 + 0.5);
    return (uint16_t)(w & 0xffffu);
}

/** Inclination (degrees); matches TopoDroidProtocol clino decoding (16384 ≡ 90°). */
static inline uint16_t topo_distox_clino_word(float incl_deg)
{
    float x = incl_deg;
    if (!isfinite(x))
        x = 0.f;
    if (x > 89.99f)
        x = 89.99f;
    if (x < -89.99f)
        x = -89.99f;
    if (x >= 0.f) {
        unsigned long u = (unsigned long)((double)x * 16384.0 / 90.0 + 0.5);
        return (uint16_t)(u & 0xffffu);
    }
    unsigned long m = (unsigned long)((double)(-x) * 16384.0 / 90.0 + 0.5);
    m %= 65536u;
    return (uint16_t)((65536u - m) & 0xffffu);
}

static inline uint8_t topo_distox_roll_byte(float roll_deg)
{
    if (!isfinite(roll_deg))
        roll_deg = 0.f;
    long z = (long)((double)roll_deg * 128.0 / 180.0 + (roll_deg >= 0 ? 0.5 : -0.5));
    if (z < 0)
        z = 0;
    if (z > 255)
        z = 255;
    return (uint8_t)z;
}

/** 3-byte read 0x38,addr → 8-byte reply (MemoryOctet BYTE_PACKET_REPLY). */
static void topo_distox_answer_mem_read(uint8_t /*tag*/, uint8_t lo, uint8_t hi)
{
    uint16_t addr = (uint16_t)lo | ((uint16_t)hi << 8u);
    uint8_t rep[8];
    memset(rep, 0, sizeof(rep));
    rep[0] = 0x38;
    rep[1] = lo;
    rep[2] = hi;
    if (addr == 0x8000u) {
        rep[3] = 0;
    } else if (addr == 0xc020u) {
        uint16_t head = 8, tail = 8;
        rep[3] = (uint8_t)(head & 0xffu);
        rep[4] = (uint8_t)(head >> 8);
        rep[5] = (uint8_t)(tail & 0xffu);
        rep[6] = (uint8_t)(tail >> 8);
        rep[7] = 0;
    }
    SerialBT.write(rep, (int)sizeof(rep));
}

static void topo_distox_maybe_emit_shot(const MeasPoint &p)
{
    /* hasClient()==true quando há sessão SPP RFCOMM (TopoDroid ligado ao canal série).
     * connected() sem timeout pode falhar em algumas builds Arduino-ESP32. */
    if (!SerialBT.hasClient())
        return;

    double dm = (double)p.dist * 1000.0;
    long dmm = (long)((dm >= 0.0 ? dm + 0.5 : dm - 0.5));
    if (dmm < 0L)
        dmm = 0L;
    if (dmm > 65535L)
        dmm = 65535L;
    uint16_t ud = (uint16_t)dmm;

    uint16_t br = topo_distox_bearing_word(p.yaw);
    uint16_t cl = topo_distox_clino_word(p.pitch);
    uint8_t rb = topo_distox_roll_byte(p.roll);
    uint8_t b0 = (uint8_t)(kTopoDistoxPktData | ((g_distox_pkt_seq++ & 1u) ? 0x80u : 0u));

    uint8_t pkt[8] = {
        b0,
        (uint8_t)(ud & 0xffu),
        (uint8_t)(ud >> 8),
        (uint8_t)(br & 0xffu),
        (uint8_t)(br >> 8),
        (uint8_t)(cl & 0xffu),
        (uint8_t)(cl >> 8),
        rb,
    };
    SerialBT.write(pkt, (int)sizeof(pkt));
    SerialBT.flush();

    /* TopoDroid DistoXProtocol.readPacket: ACK = (packet[0] & 0x80) | 0x55 → 0x55 or 0xD5 (not 0x56). */
    uint8_t want_ack = (uint8_t)((b0 & 0x80u) | 0x55u);
    uint32_t t0 = millis();
    while ((int32_t)(millis() - t0) < 450) {
        while (SerialBT.available()) {
            int rr = SerialBT.read();
            if (rr < 0)
                break;
            auto u = (uint8_t)rr;
            if (u == want_ack)
                return;
            if (u == 0x38u || u == 0x39u) {
                if (g_bt_spill_byte < 0)
                    g_bt_spill_byte = (int16_t)u;
                return;
            }
        }
        delay(2);
        yield();
    }
}

/** Text commands (newline) + TopoDroid memory read stubs. */
static void poll_bt_ascii_and_distox(void)
{
    static constexpr int32_t kDistoxPartialMs = 140;
    static String bt_line;

    if (!SerialBT.hasClient()) {
        bt_line = "";
        g_bt_dx_stage = 0;
        g_bt_spill_byte = -1;
        return;
    }

    if (g_bt_dx_stage == 1) {
        if ((int32_t)(millis() - g_bt_dx_partial_ms) > kDistoxPartialMs)
            g_bt_dx_stage = 0;
        else if (SerialBT.available() >= 2) {
            uint8_t b1 = SerialBT.read();
            uint8_t b2 = SerialBT.read();
            topo_distox_answer_mem_read(g_bt_dx_b0, b1, b2);
            g_bt_dx_stage = 0;
        }
    }

    for (;;) {
        int ci = -1;
        if (g_bt_spill_byte >= 0) {
            ci = (int)g_bt_spill_byte;
            g_bt_spill_byte = -1;
        } else {
            if (!SerialBT.available())
                break;
            ci = SerialBT.read();
        }
        if (ci < 0)
            break;
        uint8_t b = (uint8_t)ci;

        if (isprint((int)b) || b == '\r') {
            bt_line += (char)b;
            continue;
        }

        if (b == '\n') {
            if (bt_line.length() > 0)
                handle_bt_cmd(bt_line);
            bt_line = "";
            continue;
        }

        if (b == 0x38u) {
            if (SerialBT.available() >= 2) {
                uint8_t b1 = SerialBT.read();
                uint8_t b2 = SerialBT.read();
                topo_distox_answer_mem_read(b, b1, b2);
            } else {
                g_bt_dx_b0 = b;
                g_bt_dx_stage = 1;
                g_bt_dx_partial_ms = millis();
            }
            continue;
        }

        if (b == 0x39u && SerialBT.available() >= 6) {
            uint8_t r6[6];
            for (int i = 0; i < 6; ++i)
                r6[i] = (uint8_t)SerialBT.read();
            uint8_t ack[8] = { 0x38, r6[0], r6[1], r6[2], r6[3], r6[4], r6[5], 0 };
            SerialBT.write(ack, (int)sizeof(ack));
            continue;
        }
    }
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
    p.dip_deg = EXPORT_DIP_DEG_NOMINAL;
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
#if defined(ARDUINO_ARCH_ESP32)
    topo_distox_maybe_emit_shot(pts[pt_count - 1]);
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
    bt_conn = SerialBT.hasClient();
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
        SerialBT.println(TD_CSV_HEADER);
        for (int i = 0; i < pt_count; i++)
            append_point_csv_br(SerialBT, pts[i]);
        SerialBT.println("END");
        return;
    }
}

static void setup_tab_bt_ack(const char *msg)
{
    if (ui_lbl_setup_ack) lv_label_set_text(ui_lbl_setup_ack, msg);
}

static void setup_btn_list_cb(lv_event_t *e)
{
    (void)e;
    handle_bt_cmd(String("LIST"));
    setup_tab_bt_ack(LV_SYMBOL_DOWNLOAD "  CSV export (LIST)");
}

static void setup_btn_files_cb(lv_event_t *e)
{
    (void)e;
    handle_bt_cmd(String("FILES"));
    setup_tab_bt_ack(LV_SYMBOL_LIST "  File index sent");
}

static void setup_btn_meas_cb(lv_event_t *e)
{
    (void)e;
    handle_bt_cmd(String("MEAS"));
    char b[56];
    snprintf(b, sizeof(b), LV_SYMBOL_OK "  MEAS — %d pts total", pt_count);
    setup_tab_bt_ack(b);
}

static void periodic_cb(lv_timer_t*t)
{
    (void)t;
    update_status();
#ifdef ARDUINO_ARCH_ESP32
    poll_bt_ascii_and_distox();
#endif
}

static void sensor_timer_cb(lv_timer_t *t)
{
    (void)t;
    refresh_sensor_display();
    if (ui_active_tab == 3)
        refresh_setup_display();
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

static void refresh_setup_az_offs_label(void)
{
    if (!ui_lbl_setup_az_offs) return;
    char b[120];
#ifdef ARDUINO_ARCH_ESP32
    snprintf(b, sizeof(b), "Offset = %+0.2f deg (NVS saved, build default %+0.2f)",
             (double)g_azimuth_offset_deg, (double)AZIMUTH_OFFSET_DEG);
#else
    snprintf(b, sizeof(b), "Offset = %+0.2f deg (volatile, build %+0.2f)",
             (double)g_azimuth_offset_deg, (double)AZIMUTH_OFFSET_DEG);
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
    refresh_setup_display();
    setup_tab_bt_ack(LV_SYMBOL_SAVE "  Azimuth correction saved");
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
    refresh_setup_display();
    setup_tab_bt_ack(LV_SYMBOL_REFRESH "  Restored firmware default (+ saved)");
#ifdef ARDUINO_ARCH_ESP32
    play_button_ack();
#endif
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
    sec_lbl(ts, 58, "AZIMUTH / INCLINATION / ROLL");
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

    // ── SETUP tab — nested BT | Az | Live (compact) ───────────────────────
    {
        lv_obj_t *tsetup = lv_tabview_add_tab(tv, LV_SYMBOL_SETTINGS " SETUP");
        lv_obj_set_style_bg_color(tsetup, lv_color_hex(C_BG), 0);
        lv_obj_set_style_pad_all(tsetup, 2, 0);
        lv_obj_clear_flag(tsetup, LV_OBJ_FLAG_SCROLLABLE);

        const int SUB_STRIP = 30;
        lv_obj_t *sub_tv = lv_tabview_create(tsetup, LV_DIR_TOP, SUB_STRIP);
        lv_obj_set_size(sub_tv, SCREEN_W - 8, CONTENT_H - 8);
        lv_obj_align(sub_tv, LV_ALIGN_TOP_MID, 0, 4);
        lv_obj_t *stb = lv_tabview_get_tab_btns(sub_tv);
        lv_obj_set_style_bg_color(stb, lv_color_hex(C_TOPBAR), 0);
        lv_obj_set_style_bg_opa(stb, LV_OPA_COVER, 0);
        lv_obj_set_style_border_side(stb, LV_BORDER_SIDE_BOTTOM, 0);
        lv_obj_set_style_border_width(stb, 1, 0);
        lv_obj_set_style_border_color(stb, lv_color_hex(C_BORDER), 0);
        lv_obj_set_style_pad_hor(stb, 4, LV_PART_ITEMS);

        /* --- BT: RF name + actions --- */
        lv_obj_t *t_bt = lv_tabview_add_tab(sub_tv, LV_SYMBOL_BLUETOOTH " BT");
        lv_obj_set_layout(t_bt, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(t_bt, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(t_bt, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
        lv_obj_set_style_pad_all(t_bt, 6, 0);
        lv_obj_set_style_pad_row(t_bt, 6, 0);
        lv_obj_add_flag(t_bt, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(t_bt, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(t_bt, LV_SCROLLBAR_MODE_AUTO);

        {
            char rf[72];
            snprintf(rf, sizeof(rf), LV_SYMBOL_WIFI "  RF name: \"%s\"", BT_DEVICE_NAME);
            lv_obj_t *lrf = lv_label_create(t_bt);
            lv_label_set_text(lrf, rf);
            lv_obj_set_width(lrf, SCREEN_W - 28);
            lv_label_set_long_mode(lrf, LV_LABEL_LONG_DOT);
            lv_obj_set_style_text_font(lrf, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(lrf, lv_color_hex(C_TEXT), 0);
        }
        lv_obj_t *ltd = lv_label_create(t_bt);
        lv_label_set_text(ltd,
                          LV_SYMBOL_PLAY "  TopoDroid: enable continuous data reception on the survey.\n"
                          LV_SYMBOL_EYE_OPEN "  Shoot on MM1 while the app listens — no Disto memory for batch import.\n"
                          LV_SYMBOL_SETTINGS "  Connection fails? DistoX prefs: insecure socket or RFCOMM channel 1.\n"
                          LV_SYMBOL_LIST "  CSV: buttons above or LIST from the phone.\n"
                          LV_SYMBOL_KEYBOARD " If the phone asks a 4-digit PIN (legacy), firmware answers — try 1234.\n"
                          LV_SYMBOL_WARNING " Device type: Disto XA classic (RFCOMM), not BLE SAP6 (Mr Zappy uses BLE).");
        lv_obj_set_width(ltd, SCREEN_W - 28);
        lv_label_set_long_mode(ltd, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_font(ltd, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ltd, lv_color_hex(C_GREY), 0);

        lv_obj_t *btbar = lv_obj_create(t_bt);
        lv_obj_set_width(btbar, SCREEN_W - 16);
        lv_obj_set_height(btbar, 44);
        lv_obj_set_style_bg_opa(btbar, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btbar, 0, 0);
        lv_obj_set_style_pad_all(btbar, 0, 0);
        lv_obj_set_style_pad_column(btbar, 6, 0);
        lv_obj_set_layout(btbar, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(btbar, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(btbar, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(btbar, LV_OBJ_FLAG_SCROLLABLE);

        auto mk_setup_btn = [&](const char *txt, lv_event_cb_t cb) {
            lv_obj_t *b = lv_btn_create(btbar);
            lv_obj_set_flex_grow(b, 1);
            lv_obj_set_height(b, 40);
            lv_obj_set_style_bg_color(b, lv_color_hex(C_BTN_BT), 0);
            lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
            lv_obj_set_style_radius(b, 6, 0);
            lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
            lv_obj_t *lb = lv_label_create(b);
            lv_label_set_text(lb, txt);
            lv_obj_center(lb);
            lv_obj_set_style_text_color(lb, lv_color_hex(C_WHITE), 0);
            lv_obj_set_style_text_font(lb, &lv_font_montserrat_14, 0);
        };
        mk_setup_btn(LV_SYMBOL_DOWNLOAD " CSV", setup_btn_list_cb);
        mk_setup_btn(LV_SYMBOL_LIST " SD", setup_btn_files_cb);
        mk_setup_btn(LV_SYMBOL_GPS " Shot", setup_btn_meas_cb);

        ui_lbl_setup_ack = lv_label_create(t_bt);
        lv_label_set_long_mode(ui_lbl_setup_ack, LV_LABEL_LONG_DOT);
        lv_obj_set_width(ui_lbl_setup_ack, SCREEN_W - 20);
        lv_label_set_text(ui_lbl_setup_ack, "Pair your phone over Bluetooth.");
        lv_obj_set_style_text_font(ui_lbl_setup_ack, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ui_lbl_setup_ack, lv_color_hex(C_GREY), 0);

        /* --- Azimuth: correction --- */
        lv_obj_t *t_az = lv_tabview_add_tab(sub_tv, LV_SYMBOL_EDIT " Az");
        lv_obj_set_layout(t_az, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(t_az, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_all(t_az, 6, 0);
        lv_obj_set_style_pad_row(t_az, 6, 0);
        lv_obj_add_flag(t_az, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(t_az, LV_DIR_VER);

        lv_obj_t *az_wrap = lv_obj_create(t_az);
        lv_obj_set_width(az_wrap, SCREEN_W - 20);
        lv_obj_set_height(az_wrap, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(az_wrap, lv_color_hex(C_TOPBAR), 0);
        lv_obj_set_style_bg_opa(az_wrap, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(az_wrap, lv_color_hex(C_BORDER), 0);
        lv_obj_set_style_border_width(az_wrap, 1, 0);
        lv_obj_set_style_radius(az_wrap, 8, 0);
        lv_obj_set_style_pad_all(az_wrap, 8, 0);
        lv_obj_set_layout(az_wrap, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(az_wrap, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(az_wrap, 6, 0);
        lv_obj_clear_flag(az_wrap, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *az_ttl = lv_label_create(az_wrap);
        lv_label_set_text(az_ttl, LV_SYMBOL_EDIT "  Azimuth correction (deg)");
        lv_obj_set_style_text_font(az_ttl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(az_ttl, lv_color_hex(C_HDR_LINE), 0);

        ui_lbl_setup_az_offs = lv_label_create(az_wrap);
        lv_label_set_long_mode(ui_lbl_setup_az_offs, LV_LABEL_LONG_CLIP);
        lv_obj_set_width(ui_lbl_setup_az_offs, SCREEN_W - 40);
        lv_obj_set_style_text_font(ui_lbl_setup_az_offs, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ui_lbl_setup_az_offs, lv_color_hex(C_REF_S), 0);
        lv_label_set_text(ui_lbl_setup_az_offs, "—");
        refresh_setup_az_offs_label();

        lv_obj_t *az_row = lv_obj_create(az_wrap);
        lv_obj_set_width(az_row, SCREEN_W - 36);
        lv_obj_set_height(az_row, 36);
        lv_obj_set_style_bg_opa(az_row, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(az_row, 0, 0);
        lv_obj_set_style_pad_all(az_row, 0, 0);
        lv_obj_set_layout(az_row, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(az_row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(az_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_clear_flag(az_row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_column(az_row, 2, 0);

        auto mk_az_btn = [&](const char *lbl, int cdeg_cent) {
            lv_obj_t *b = lv_btn_create(az_row);
            lv_obj_set_flex_grow(b, 1);
            lv_obj_set_height(b, 32);
            lv_obj_set_style_bg_color(b, lv_color_hex(C_HDR_LINE), 0);
            lv_obj_set_style_radius(b, 4, 0);
            lv_obj_add_event_cb(b, setup_az_offs_delta_cb, LV_EVENT_CLICKED,
                                (void *)(intptr_t)cdeg_cent);
            lv_obj_t *lb = lv_label_create(b);
            lv_label_set_text(lb, lbl);
            lv_obj_center(lb);
            lv_obj_set_style_text_font(lb, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(lb, lv_color_hex(C_WHITE), 0);
        };
        mk_az_btn("-5", -500);
        mk_az_btn("-1", -100);
        mk_az_btn("-0.1", -10);
        mk_az_btn("+0.1", 10);
        mk_az_btn("+1", 100);
        mk_az_btn("+5", 500);

        lv_obj_t *az_rst = lv_btn_create(az_wrap);
        lv_obj_set_width(az_rst, SCREEN_W - 40);
        lv_obj_set_height(az_rst, 30);
        lv_obj_set_style_bg_color(az_rst, lv_color_hex(C_GREY), 0);
        lv_obj_set_style_radius(az_rst, 4, 0);
        lv_obj_add_event_cb(az_rst, setup_az_offs_rst_cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_t *az_rst_lbl = lv_label_create(az_rst);
        lv_label_set_text(az_rst_lbl, LV_SYMBOL_REFRESH " Firmware default");
        lv_obj_center(az_rst_lbl);
        lv_obj_set_style_text_color(az_rst_lbl, lv_color_hex(C_WHITE), 0);
        lv_obj_set_style_text_font(az_rst_lbl, &lv_font_montserrat_14, 0);

        /* --- Live sensors --- */
        lv_obj_t *t_live = lv_tabview_add_tab(sub_tv, LV_SYMBOL_EYE_OPEN " Live");
        lv_obj_set_layout(t_live, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(t_live, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_all(t_live, 6, 0);
        lv_obj_set_style_pad_row(t_live, 6, 0);
        lv_obj_add_flag(t_live, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(t_live, LV_DIR_VER);

        lv_obj_t *card = lv_obj_create(t_live);
        lv_obj_set_width(card, SCREEN_W - 20);
        lv_obj_set_height(card, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_color(card, lv_color_hex(C_TOPBAR), 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(card, lv_color_hex(C_BORDER), 0);
        lv_obj_set_style_border_width(card, 1, 0);
        lv_obj_set_style_radius(card, 8, 0);
        lv_obj_set_style_pad_all(card, 10, 0);
        lv_obj_set_layout(card, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_style_pad_row(card, 4, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        ui_lbl_setup_deg = lv_label_create(card);
        lv_label_set_text(ui_lbl_setup_deg, "Heading err: —");
        lv_obj_set_width(ui_lbl_setup_deg, SCREEN_W - 44);
        lv_label_set_long_mode(ui_lbl_setup_deg, LV_LABEL_LONG_DOT);
        lv_obj_set_style_text_font(ui_lbl_setup_deg, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ui_lbl_setup_deg, lv_color_hex(C_TEXT), 0);

        ui_lbl_setup_qual = lv_label_create(card);
        lv_label_set_text(ui_lbl_setup_qual, "Quality: —");
        lv_obj_set_width(ui_lbl_setup_qual, SCREEN_W - 44);
        lv_obj_set_style_text_font(ui_lbl_setup_qual, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ui_lbl_setup_qual, lv_color_hex(C_TEXT), 0);

        ui_lbl_setup_grav = lv_label_create(card);
        lv_label_set_text(ui_lbl_setup_grav, "|g|: —");
        lv_obj_set_width(ui_lbl_setup_grav, SCREEN_W - 44);
        lv_obj_set_style_text_font(ui_lbl_setup_grav, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ui_lbl_setup_grav, lv_color_hex(C_TEXT), 0);

        lv_obj_t *sep = lv_obj_create(card);
        lv_obj_set_size(sep, SCREEN_W - 48, 1);
        lv_obj_set_style_bg_color(sep, lv_color_hex(C_BORDER), 0);
        lv_obj_set_style_bg_opa(sep, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(sep, 0, 0);

        ui_lbl_setup_imu = lv_label_create(card);
        lv_label_set_text(ui_lbl_setup_imu, "Az / Inc / Roll: —");
        lv_obj_set_width(ui_lbl_setup_imu, SCREEN_W - 44);
        lv_label_set_long_mode(ui_lbl_setup_imu, LV_LABEL_LONG_DOT);
        lv_obj_set_style_text_font(ui_lbl_setup_imu, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ui_lbl_setup_imu, lv_color_hex(C_TEXT), 0);

        ui_lbl_setup_lzr = lv_label_create(card);
        lv_label_set_text(ui_lbl_setup_lzr, "Laser: —");
        lv_obj_set_width(ui_lbl_setup_lzr, SCREEN_W - 44);
        lv_obj_set_style_text_font(ui_lbl_setup_lzr, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ui_lbl_setup_lzr, lv_color_hex(C_TEXT), 0);

        ui_lbl_setup_hint = lv_label_create(t_live);
        lv_label_set_text(ui_lbl_setup_hint,
                          LV_SYMBOL_WARNING "  Magnetic noise: check Quality before trusting azimuth.");
        lv_obj_set_width(ui_lbl_setup_hint, SCREEN_W - 24);
        lv_label_set_long_mode(ui_lbl_setup_hint, LV_LABEL_LONG_DOT);
        lv_obj_set_style_text_font(ui_lbl_setup_hint, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(ui_lbl_setup_hint, lv_color_hex(C_GREY), 0);

        refresh_setup_display();
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
    DBG_PRINT("\n[MM1-BLACK] Boot\n");
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
#ifdef ARDUINO_ARCH_ESP32
    /* SSP + callbacks registered before begin (see Arduino BluetoothSerial). */
    SerialBT.enableSSP();
    SerialBT.onConfirmRequest(bt_ssp_confirm_cb);
    SerialBT.onAuthComplete(bt_auth_cmpl_cb);
#endif
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
            if(f){f.println(TD_CSV_HEADER);f.close();}
        }
        load_csv();
    }

    static lv_indev_drv_t id;
    lv_indev_drv_init(&id);
    id.type=LV_INDEV_TYPE_POINTER; id.read_cb=touch_read;
    lv_indev_drv_register(&id);

    prefs_load_az_offset();
    build_ui();
#ifdef ARDUINO_ARCH_ESP32
    SerialBT.begin(BT_DEVICE_NAME);
#endif
    lv_timer_create(periodic_cb, 1000, nullptr);
    lv_timer_create(sensor_timer_cb, 250, nullptr);
#ifdef ARDUINO_ARCH_ESP32
    play_boot_chime();
#endif
#if !LZR_SHARE_USB_UART
    Serial.println("[MM1-BLACK] Ready");
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
