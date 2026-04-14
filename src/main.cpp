/**
 * BRICS-5 MM1  –  Field Measurement Tool
 * ESP32 CYD  |  ST7796 480×320  |  LVGL 8.3
 *
 * Layout (landscape 480×320):
 *  ┌─────────────────────── 36px Header ────────────────────────────┐
 *  │  BRICS-5 MM1   PTS:N      [SD●]  [BT●]      HH:MM:SS          │
 *  ├────────────────────── 28px Tab Bar ────────────────────────────┤
 *  │  [ SURVEY ]    [ POSITION ]                                     │
 *  ├────────────────── Scrollable Table ────────────────────────────┤
 *  │  REF │ DIST(m) │ AZMO°  │ INCLO°                               │
 *  │  ...rows...                                                     │
 *  ├──────────────────── Scrollable Table ────────────────────────────┤
 *  │  (no footer – all actions live in the MANAGE tab)              │
 *  └────────────────────────────────────────────────────────────────┘
 *
 * Bluetooth Serial commands (newline-terminated):
 *   ADD,<ref>,<dist>,<azmo>,<inclo>,<x>,<y>,<z>,<roll>,<pitch>,<yaw>
 *   CLEAR
 *   LIST   -> responds with CSV dump
 */

#include <Arduino.h>
#include <math.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <lvgl.h>
#include <SD.h>
#include <BluetoothSerial.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Hardware pinout  (4.0" ST7796  ESP32-WROOM-32E)
//  TFT + Touch share HSPI (pins 12/13/14), touch CS=33 – managed by TFT_eSPI.
//  SD card uses the default VSPI (pins 18/19/23), CS=5  – no conflict.
//  Source: board datasheet / demos in 4.0inch_ESP32-32E_ST7796_E32R40T_V1.0
// ─────────────────────────────────────────────────────────────────────────────
#define SCREEN_W  480
#define SCREEN_H  320

// SD card – default VSPI pins
#define SD_CS    5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

// Touch calibration for tft.setRotation(1)  (landscape 480×320).
// Obtained from the official RGB_LED_TOUCH demo shipped with the board.
static const uint16_t TOUCH_CAL[5] = { 254, 3643, 176, 3693, 7 };

// ─────────────────────────────────────────────────────────────────────────────
//  Colour palette  –  light engineering theme
// ─────────────────────────────────────────────────────────────────────────────
#define C_BG          0xF0F4F8u   // light blue-grey background
#define C_TOPBAR      0xFFFFFFu   // white header / footer bar
#define C_TBL_HDR     0x1E3A5Fu   // dark navy  – table header row
#define C_ROW_ODD     0xFFFFFFu   // white rows
#define C_ROW_EVEN    0xEEF2F7u   // pale blue-grey rows
#define C_ROW_SEL     0xBBDEFBu   // sky-blue highlight
#define C_ACCENT_S    0xFFFFFFu   // white – text on dark survey header
#define C_ACCENT_P    0xFFFFFFu   // white – text on dark position header
#define C_REF_S       0x1565C0u   // deep blue  – REF column / survey
#define C_REF_P       0x00695Cu   // deep teal  – REF column / position
#define C_TEXT        0x1A2027u   // near-black – data cell text
#define C_HDR_LINE    0x1565C0u   // blue accent line on header / footer
#define C_BORDER      0xCFD8DCu   // soft blue-grey cell border
#define C_WHITE       0xFFFFFFu
#define C_GREY        0x546E7Au   // muted blue-grey
#define C_BT_ON       0x1565C0u   // Bluetooth connected
#define C_BT_OFF      0x90A4AEu   // Bluetooth off
#define C_SD_ON       0x2E7D32u   // SD present
#define C_SD_OFF      0xD32F2Fu   // SD missing
#define C_BTN_ADD     0x00897Bu   // teal      – add point
#define C_BTN_SAVE    0x1565C0u   // blue      – save SD
#define C_BTN_LOAD    0x1B5E20u   // dark green– load SD
#define C_BTN_DEL     0xC62828u   // crimson   – delete row / file
#define C_BTN_CLEAR   0xB71C1Cu   // dark red  – clear all
#define C_BTN_USE     0x4A148Cu   // deep purple – use / select file
#define C_FILE_ACTIVE 0xC8E6C9u   // light green – active file row

// ─────────────────────────────────────────────────────────────────────────────
//  Data structure
// ─────────────────────────────────────────────────────────────────────────────
#define MAX_POINTS 30
#define MAX_FILES  16

struct MeasPoint {
    uint16_t ref;
    float    dist;           // distance  (m)
    float    azmo;           // azimuth   (°)
    float    inclo;          // inclination (°)
    float    x, y, z;        // position  (m)
    float    roll, pitch, yaw;   // orientation (°)
    char     marker[4];      // e.g. "E", "W", ""
};

// ─────────────────────────────────────────────────────────────────────────────
//  Globals
// ─────────────────────────────────────────────────────────────────────────────
static TFT_eSPI        tft;
static BluetoothSerial SerialBT;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t        *lvgl_buf = nullptr;   // allocated on heap in setup()

static MeasPoint  pts[MAX_POINTS];
static int        pt_count    = 0;
static int        sel_row     = -1;
static bool       sd_ready    = false;
static bool       bt_conn     = false;

// Active CSV file (absolute path on SD card, e.g. "/brics5_mm1.csv")
static char       active_csv[32] = "/brics5_mm1.csv";

// SD file-browser state
static char       file_names[MAX_FILES][32];   // filenames found on SD
static int        file_count = 0;
static int        file_sel   = -1;             // highlighted row in file list

// UI handles
static lv_obj_t *ui_tbl_survey   = nullptr;
static lv_obj_t *ui_tbl_position = nullptr;
static lv_obj_t *ui_lbl_bt       = nullptr;
static lv_obj_t *ui_lbl_sd       = nullptr;
static lv_obj_t *ui_lbl_time     = nullptr;
static lv_obj_t *ui_lbl_count    = nullptr;
static lv_obj_t *ui_tbl_files    = nullptr;   // file-list table in MANAGE tab
static lv_obj_t *ui_lbl_active   = nullptr;   // shows active_csv in MANAGE tab
static lv_obj_t *ui_lbl_manage_status = nullptr;

// Forward declaration for SD helper (defined near setup())
static void sd_init();

// ─────────────────────────────────────────────────────────────────────────────
//  LVGL display flush (TFT_eSPI backend)
// ─────────────────────────────────────────────────────────────────────────────
static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors(reinterpret_cast<uint16_t *>(color_p), w * h, true);
    tft.endWrite();
    lv_disp_flush_ready(drv);
}

// ─────────────────────────────────────────────────────────────────────────────
//  LVGL touch read  –  uses TFT_eSPI built-in XPT2046 support.
//  Touch CS (pin 33) shares the HSPI bus with the TFT; no separate library
//  is required.  Calibration is applied once in setup() via tft.setTouch().
// ─────────────────────────────────────────────────────────────────────────────
static void touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t tx = 0, ty = 0;
    if (tft.getTouch(&tx, &ty)) {
        data->point.x = (int16_t)tx;
        data->point.y = (int16_t)ty;
        data->state   = LV_INDEV_STATE_PR;
        static uint32_t last_dbg = 0;
        if (millis() - last_dbg > 300) {
            last_dbg = millis();
            Serial.printf("[TOUCH] x=%u  y=%u\n", tx, ty);
        }
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Table draw callbacks  (custom row/column styling via LVGL draw events)
// ─────────────────────────────────────────────────────────────────────────────
static void survey_draw_cb(lv_event_t *e)
{
    lv_obj_t               *obj = lv_event_get_target(e);
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (dsc->part != LV_PART_ITEMS) return;

    uint32_t row = dsc->id / lv_table_get_col_cnt(obj);
    uint32_t col = dsc->id % lv_table_get_col_cnt(obj);

    if (row == 0) {
        // ── Header row ──
        dsc->rect_dsc->bg_color  = lv_color_hex(C_TBL_HDR);
        dsc->rect_dsc->bg_opa    = LV_OPA_COVER;
        dsc->label_dsc->color    = lv_color_hex(C_ACCENT_S);
        dsc->label_dsc->align    = LV_TEXT_ALIGN_CENTER;
        dsc->label_dsc->font     = &lv_font_montserrat_14;
    } else {
        // ── Data rows ──
        bool is_selected = (static_cast<int>(row) - 1) == sel_row;
        if (is_selected) {
            dsc->rect_dsc->bg_color = lv_color_hex(C_ROW_SEL);
        } else {
            dsc->rect_dsc->bg_color = (row % 2 == 0)
                                      ? lv_color_hex(C_ROW_EVEN)
                                      : lv_color_hex(C_ROW_ODD);
        }
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;

        if (col == 0) {
            dsc->label_dsc->align = LV_TEXT_ALIGN_CENTER;
            dsc->label_dsc->color = lv_color_hex(C_REF_S);
        } else {
            dsc->label_dsc->align = LV_TEXT_ALIGN_RIGHT;
            dsc->label_dsc->color = lv_color_hex(C_TEXT);
        }
        dsc->label_dsc->font = &lv_font_montserrat_14;
    }
    dsc->rect_dsc->border_color = lv_color_hex(C_BORDER);
    dsc->rect_dsc->border_width = 1;
}

static void position_draw_cb(lv_event_t *e)
{
    lv_obj_t               *obj = lv_event_get_target(e);
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (dsc->part != LV_PART_ITEMS) return;

    uint32_t row = dsc->id / lv_table_get_col_cnt(obj);
    uint32_t col = dsc->id % lv_table_get_col_cnt(obj);

    if (row == 0) {
        dsc->rect_dsc->bg_color  = lv_color_hex(C_TBL_HDR);
        dsc->rect_dsc->bg_opa    = LV_OPA_COVER;
        dsc->label_dsc->color    = lv_color_hex(C_ACCENT_P);
        dsc->label_dsc->align    = LV_TEXT_ALIGN_CENTER;
        dsc->label_dsc->font     = &lv_font_montserrat_14;
    } else {
        bool is_selected = (static_cast<int>(row) - 1) == sel_row;
        if (is_selected) {
            dsc->rect_dsc->bg_color = lv_color_hex(C_ROW_SEL);
        } else {
            dsc->rect_dsc->bg_color = (row % 2 == 0)
                                      ? lv_color_hex(C_ROW_EVEN)
                                      : lv_color_hex(C_ROW_ODD);
        }
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;

        if (col == 0) {
            dsc->label_dsc->align = LV_TEXT_ALIGN_CENTER;
            dsc->label_dsc->color = lv_color_hex(C_REF_P);
        } else {
            dsc->label_dsc->align = LV_TEXT_ALIGN_RIGHT;
            dsc->label_dsc->color = lv_color_hex(C_TEXT);
        }
        dsc->label_dsc->font = &lv_font_montserrat_14;
    }
    dsc->rect_dsc->border_color = lv_color_hex(C_BORDER);
    dsc->rect_dsc->border_width = 1;
}

// Row selection on tap
static void survey_click_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    uint16_t  row, col;
    lv_table_get_selected_cell(obj, &row, &col);
    if (row > 0 && (int)(row - 1) < pt_count) {
        sel_row = (sel_row == (int)(row - 1)) ? -1 : (int)(row - 1);
        lv_obj_invalidate(obj);
        if (ui_tbl_position) lv_obj_invalidate(ui_tbl_position);
    }
}

static void position_click_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    uint16_t  row, col;
    lv_table_get_selected_cell(obj, &row, &col);
    if (row > 0 && (int)(row - 1) < pt_count) {
        sel_row = (sel_row == (int)(row - 1)) ? -1 : (int)(row - 1);
        lv_obj_invalidate(obj);
        if (ui_tbl_survey) lv_obj_invalidate(ui_tbl_survey);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Refresh tables
// ─────────────────────────────────────────────────────────────────────────────
static void refresh_survey()
{
    lv_obj_t *t = ui_tbl_survey;
    // Set row count to exactly header + data rows — removes any previously
    // visible extra rows (critical for Clear All to work visually).
    lv_table_set_row_cnt(t, (uint16_t)(pt_count + 1));

    lv_table_set_cell_value(t, 0, 0, "REF");
    lv_table_set_cell_value(t, 0, 1, "DIST (m)");
    lv_table_set_cell_value(t, 0, 2, "AZMO \xC2\xB0");
    lv_table_set_cell_value(t, 0, 3, "INCLO \xC2\xB0");

    char buf[24];
    for (int i = 0; i < pt_count; i++) {
        MeasPoint &p = pts[i];
        snprintf(buf, sizeof(buf), "%s%u", p.marker, p.ref);
        lv_table_set_cell_value(t, i + 1, 0, buf);
        snprintf(buf, sizeof(buf), "%.1f", p.dist);
        lv_table_set_cell_value(t, i + 1, 1, buf);
        snprintf(buf, sizeof(buf), "%.1f", p.azmo);
        lv_table_set_cell_value(t, i + 1, 2, buf);
        snprintf(buf, sizeof(buf), "%.1f", p.inclo);
        lv_table_set_cell_value(t, i + 1, 3, buf);
    }

    snprintf(buf, sizeof(buf), "PTS: %d", pt_count);
    lv_label_set_text(ui_lbl_count, buf);
}

static void refresh_position()
{
    lv_obj_t *t = ui_tbl_position;
    lv_table_set_row_cnt(t, (uint16_t)(pt_count + 1));

    lv_table_set_cell_value(t, 0, 0, "REF");
    lv_table_set_cell_value(t, 0, 1, "X (m)");
    lv_table_set_cell_value(t, 0, 2, "Y (m)");
    lv_table_set_cell_value(t, 0, 3, "Z (m)");
    lv_table_set_cell_value(t, 0, 4, "R \xC2\xB0");
    lv_table_set_cell_value(t, 0, 5, "P \xC2\xB0");
    lv_table_set_cell_value(t, 0, 6, "Y \xC2\xB0");

    char buf[24];
    for (int i = 0; i < pt_count; i++) {
        MeasPoint &p = pts[i];
        snprintf(buf, sizeof(buf), "%s%u", p.marker, p.ref);
        lv_table_set_cell_value(t, i + 1, 0, buf);
        snprintf(buf, sizeof(buf), "%.2f", p.x);
        lv_table_set_cell_value(t, i + 1, 1, buf);
        snprintf(buf, sizeof(buf), "%.2f", p.y);
        lv_table_set_cell_value(t, i + 1, 2, buf);
        snprintf(buf, sizeof(buf), "%.2f", p.z);
        lv_table_set_cell_value(t, i + 1, 3, buf);
        snprintf(buf, sizeof(buf), "%.1f", p.roll);
        lv_table_set_cell_value(t, i + 1, 4, buf);
        snprintf(buf, sizeof(buf), "%.1f", p.pitch);
        lv_table_set_cell_value(t, i + 1, 5, buf);
        snprintf(buf, sizeof(buf), "%.1f", p.yaw);
        lv_table_set_cell_value(t, i + 1, 6, buf);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SD card  –  save / load
// ─────────────────────────────────────────────────────────────────────────────
static void save_csv()
{
    if (!sd_ready) {
        Serial.println("[SD] save_csv: card not ready");
        return;
    }
    if (SD.exists(active_csv)) SD.remove(active_csv);
    File f = SD.open(active_csv, FILE_WRITE);
    if (!f) {
        Serial.printf("[SD] save_csv: could not open %s\n", active_csv);
        return;
    }
    f.println("REF,MARKER,DIST,AZMO,INCLO,X,Y,Z,ROLL,PITCH,YAW");
    for (int i = 0; i < pt_count; i++) {
        MeasPoint &p = pts[i];
        f.printf("%u,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                 p.ref, p.marker,
                 p.dist, p.azmo, p.inclo,
                 p.x, p.y, p.z,
                 p.roll, p.pitch, p.yaw);
    }
    f.close();
    Serial.printf("[SD] Saved %d points to %s\n", pt_count, active_csv);
}

static void load_csv()
{
    if (!sd_ready) {
        Serial.println("[SD] load_csv: card not ready");
        return;
    }
    File f = SD.open(active_csv, FILE_READ);
    if (!f) {
        Serial.printf("[SD] load_csv: %s not found\n", active_csv);
        return;
    }
    Serial.printf("[SD] load_csv: %s  size=%u bytes\n", active_csv, (unsigned)f.size());
    f.readStringUntil('\n'); // skip CSV header line
    pt_count = 0;
    while (f.available() && pt_count < MAX_POINTS) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;
        MeasPoint &p = pts[pt_count];
        char buf[128];
        line.toCharArray(buf, sizeof(buf));
        char *tok = strtok(buf, ",");
        if (tok) { p.ref   = (uint16_t)atoi(tok); tok = strtok(NULL, ","); }
        if (tok) { strlcpy(p.marker, tok, sizeof(p.marker)); tok = strtok(NULL, ","); }
        if (tok) { p.dist  = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { p.azmo  = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { p.inclo = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { p.x     = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { p.y     = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { p.z     = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { p.roll  = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { p.pitch = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { p.yaw   = atof(tok); }
        pt_count++;
    }
    f.close();
    Serial.printf("[SD] Loaded %d points from %s\n", pt_count, active_csv);
}

// ─────────────────────────────────────────────────────────────────────────────
//  File-browser helpers
// ─────────────────────────────────────────────────────────────────────────────

// Scan root directory of SD for *.csv files and fill file_names[].
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
            int len = (int)strlen(n);
            // name() may or may not include the leading '/'
            const char *base = (n[0] == '/') ? n + 1 : n;
            if (len >= 4) {
                const char *ext = n + len - 4;
                if (ext[0] == '.' &&
                    (ext[1] == 'c' || ext[1] == 'C') &&
                    (ext[2] == 's' || ext[2] == 'S') &&
                    (ext[3] == 'v' || ext[3] == 'V')) {
                    snprintf(file_names[file_count], sizeof(file_names[0]),
                             "/%s", base);
                    file_count++;
                }
            }
        }
        entry.close();
    }
    root.close();
}

// Refresh the MANAGE file-list table from the current scan results.
static void refresh_file_list()
{
    scan_csv_files();
    if (!ui_tbl_files) return;
    lv_table_set_row_cnt(ui_tbl_files, (uint16_t)(file_count + 1));
    lv_table_set_cell_value(ui_tbl_files, 0, 0, "File");
    lv_table_set_cell_value(ui_tbl_files, 0, 1, "Size");
    for (int i = 0; i < file_count; i++) {
        // Strip leading '/' for display
        const char *name = file_names[i];
        if (name[0] == '/') name++;
        lv_table_set_cell_value(ui_tbl_files, i + 1, 0, name);
        File f = SD.open(file_names[i], FILE_READ);
        if (f) {
            char sz[12];
            uint32_t bytes = f.size();
            if (bytes < 1024)
                snprintf(sz, sizeof(sz), "%luB", bytes);
            else
                snprintf(sz, sizeof(sz), "%lukB", bytes / 1024);
            f.close();
            lv_table_set_cell_value(ui_tbl_files, i + 1, 1, sz);
        } else {
            lv_table_set_cell_value(ui_tbl_files, i + 1, 1, "?");
        }
    }
    lv_obj_invalidate(ui_tbl_files);
}

// Update the "Active:" label in the MANAGE tab.
static void update_active_lbl()
{
    if (!ui_lbl_active) return;
    const char *name = active_csv;
    if (name[0] == '/') name++;
    char buf[64];
    snprintf(buf, sizeof(buf), "Active: %s   (%d pts)", name, pt_count);
    lv_label_set_text(ui_lbl_active, buf);
}

// Draw callback – file-list table (header + rows, highlight active file).
static void file_list_draw_cb(lv_event_t *e)
{
    lv_obj_t               *obj = lv_event_get_target(e);
    lv_obj_draw_part_dsc_t *dsc = lv_event_get_draw_part_dsc(e);
    if (dsc->part != LV_PART_ITEMS) return;

    uint32_t row = dsc->id / lv_table_get_col_cnt(obj);
    uint32_t col = dsc->id % lv_table_get_col_cnt(obj);

    if (row == 0) {
        dsc->rect_dsc->bg_color  = lv_color_hex(C_TBL_HDR);
        dsc->rect_dsc->bg_opa    = LV_OPA_COVER;
        dsc->label_dsc->color    = lv_color_hex(C_WHITE);
        dsc->label_dsc->align    = LV_TEXT_ALIGN_CENTER;
        dsc->label_dsc->font     = &lv_font_montserrat_14;
    } else {
        int idx = (int)row - 1;
        bool is_active   = (idx < file_count) &&
                           (strcmp(file_names[idx], active_csv) == 0);
        bool is_selected = (idx == file_sel);

        if (is_selected)
            dsc->rect_dsc->bg_color = lv_color_hex(C_ROW_SEL);
        else if (is_active)
            dsc->rect_dsc->bg_color = lv_color_hex(C_FILE_ACTIVE);
        else
            dsc->rect_dsc->bg_color = (row % 2 == 0)
                                      ? lv_color_hex(C_ROW_EVEN)
                                      : lv_color_hex(C_ROW_ODD);
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;

        dsc->label_dsc->color = (col == 0) ? lv_color_hex(C_REF_S)
                                           : lv_color_hex(C_TEXT);
        dsc->label_dsc->align = (col == 0) ? LV_TEXT_ALIGN_LEFT
                                           : LV_TEXT_ALIGN_RIGHT;
        dsc->label_dsc->font  = &lv_font_montserrat_14;
    }
    dsc->rect_dsc->border_color = lv_color_hex(C_BORDER);
    dsc->rect_dsc->border_width = 1;
}

// Click callback – select a row in the file list.
static void file_list_click_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    uint16_t  row, col;
    lv_table_get_selected_cell(obj, &row, &col);
    if (row > 0 && (int)(row - 1) < file_count) {
        int idx  = (int)row - 1;
        file_sel = (file_sel == idx) ? -1 : idx;
        lv_obj_invalidate(obj);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Button callbacks  (MANAGE tab)
// ─────────────────────────────────────────────────────────────────────────────

// Helper – post a message to the MANAGE tab status label
static void set_status(const char *msg)
{
    if (ui_lbl_manage_status) lv_label_set_text(ui_lbl_manage_status, msg);
    Serial.printf("[STATUS] %s\n", msg);
}

static void btn_save_cb(lv_event_t *e)
{
    save_csv();
    char buf[40];
    snprintf(buf, sizeof(buf), LV_SYMBOL_OK " Saved %d pts to SD", pt_count);
    set_status(buf);
}

static void btn_load_cb(lv_event_t *e)
{
    if (!sd_ready) { set_status(LV_SYMBOL_WARNING " No SD card!"); return; }
    pt_count = 0;
    sel_row  = -1;
    load_csv();
    refresh_survey();
    refresh_position();
    char buf[40];
    snprintf(buf, sizeof(buf), LV_SYMBOL_DOWNLOAD " Loaded %d pts from SD", pt_count);
    set_status(buf);
}

static void btn_add_cb(lv_event_t *e)
{
    if (pt_count >= MAX_POINTS) { set_status(LV_SYMBOL_WARNING " Table full!"); return; }
    MeasPoint &p = pts[pt_count];

    auto rnd = [](float lo, float hi) -> float {
        return lo + ((float)(esp_random() & 0xFFFF) / 65535.0f) * (hi - lo);
    };

    p.ref   = (pt_count > 0) ? (pts[pt_count - 1].ref - 1) : 100;
    p.dist  = rnd(5.0f,   50.0f);
    p.azmo  = rnd(0.0f,  360.0f);
    p.inclo = rnd(-15.0f, 35.0f);

    float az_r  = p.azmo  * (float)M_PI / 180.0f;
    float inc_r = p.inclo * (float)M_PI / 180.0f;
    p.x = p.dist * cosf(inc_r) * sinf(az_r);
    p.y = p.dist * cosf(inc_r) * cosf(az_r);
    p.z = p.dist * sinf(inc_r);
    p.roll  = rnd(-5.0f,  5.0f);
    p.pitch = rnd(-5.0f,  5.0f);
    p.yaw   = p.azmo;
    strcpy(p.marker, "");

    pt_count++;
    refresh_survey();
    refresh_position();

    char buf[40];
    snprintf(buf, sizeof(buf), LV_SYMBOL_PLUS " Added ref=%u  pts=%d", p.ref, pt_count);
    set_status(buf);
}

static void btn_del_row_cb(lv_event_t *e)
{
    if (sel_row < 0 || sel_row >= pt_count) {
        set_status(LV_SYMBOL_WARNING " Select a row first!");
        return;
    }
    uint16_t deleted_ref = pts[sel_row].ref;
    // Shift array left
    for (int i = sel_row; i < pt_count - 1; i++) pts[i] = pts[i + 1];
    pt_count--;
    sel_row = -1;
    refresh_survey();
    refresh_position();

    char buf[40];
    snprintf(buf, sizeof(buf), LV_SYMBOL_TRASH " Deleted ref=%u  pts=%d", deleted_ref, pt_count);
    set_status(buf);
}

static void btn_clear_cb(lv_event_t *e)
{
    pt_count = 0;
    sel_row  = -1;
    refresh_survey();
    refresh_position();
    set_status(LV_SYMBOL_TRASH " Table cleared");
}

// ── File-browser callbacks (MANAGE tab) ─────────────────────────────────────

// Create a new empty CSV on the SD card with an auto-generated name.
static void btn_new_file_cb(lv_event_t *e)
{
    if (!sd_ready) { set_status(LV_SYMBOL_WARNING " No SD card!"); return; }
    char path[32];
    int n = 1;
    do {
        snprintf(path, sizeof(path), "/brics%02d.csv", n++);
    } while (SD.exists(path) && n < 100);
    if (n >= 100) { set_status(LV_SYMBOL_WARNING " Too many files!"); return; }

    File f = SD.open(path, FILE_WRITE);
    if (!f) { set_status(LV_SYMBOL_WARNING " Create failed!"); return; }
    f.println("REF,MARKER,DIST,AZMO,INCLO,X,Y,Z,ROLL,PITCH,YAW");
    f.close();

    // Switch to the new file (starts empty)
    strlcpy(active_csv, path, sizeof(active_csv));
    pt_count = 0;
    sel_row  = -1;
    refresh_survey();
    refresh_position();
    refresh_file_list();
    update_active_lbl();

    char buf[48];
    snprintf(buf, sizeof(buf), LV_SYMBOL_OK " Created & using %s", path + 1);
    set_status(buf);
}

// Load the selected file and make it the active CSV.
static void btn_use_file_cb(lv_event_t *e)
{
    if (file_sel < 0 || file_sel >= file_count) {
        set_status(LV_SYMBOL_WARNING " Select a file first!");
        return;
    }
    strlcpy(active_csv, file_names[file_sel], sizeof(active_csv));
    pt_count = 0;
    sel_row  = -1;
    load_csv();
    refresh_survey();
    refresh_position();
    lv_obj_invalidate(ui_tbl_files);   // refresh active highlight
    update_active_lbl();

    char buf[48];
    snprintf(buf, sizeof(buf), LV_SYMBOL_OK " Using %s  (%d pts)",
             active_csv + 1, pt_count);
    set_status(buf);
}

// Delete the selected file from the SD card.
static void btn_del_file_cb(lv_event_t *e)
{
    if (file_sel < 0 || file_sel >= file_count) {
        set_status(LV_SYMBOL_WARNING " Select a file first!");
        return;
    }
    char path[32];
    strlcpy(path, file_names[file_sel], sizeof(path));
    bool was_active = (strcmp(path, active_csv) == 0);

    SD.remove(path);
    file_sel = -1;
    refresh_file_list();

    if (was_active) {
        // Fall back to first remaining file, or re-create the default
        if (file_count > 0) {
            strlcpy(active_csv, file_names[0], sizeof(active_csv));
        } else {
            strlcpy(active_csv, "/brics5_mm1.csv", sizeof(active_csv));
            File f = SD.open(active_csv, FILE_WRITE);
            if (f) { f.println("REF,MARKER,DIST,AZMO,INCLO,X,Y,Z,ROLL,PITCH,YAW"); f.close(); }
            refresh_file_list();
        }
        pt_count = 0;
        sel_row  = -1;
        load_csv();
        refresh_survey();
        refresh_position();
        update_active_lbl();
    }

    char buf[48];
    snprintf(buf, sizeof(buf), LV_SYMBOL_TRASH " Deleted %s", path + 1);
    set_status(buf);
}

// Refresh the file list whenever the user switches to the MANAGE tab.
static void tabview_changed_cb(lv_event_t *e)
{
    lv_obj_t *tv  = lv_event_get_target(e);
    uint16_t  tab = lv_tabview_get_tab_act(tv);
    if (tab == 2) {
        refresh_file_list();
        update_active_lbl();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Status bar update
// ─────────────────────────────────────────────────────────────────────────────
static void update_status()
{
    bt_conn = SerialBT.connected();

    lv_label_set_text(ui_lbl_bt, bt_conn
                      ? LV_SYMBOL_BLUETOOTH " BT"
                      : LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(ui_lbl_bt,
                                lv_color_hex(bt_conn ? C_BT_ON : C_BT_OFF), 0);

    lv_label_set_text(ui_lbl_sd, sd_ready
                      ? LV_SYMBOL_SD_CARD " SD"
                      : LV_SYMBOL_SD_CARD);
    lv_obj_set_style_text_color(ui_lbl_sd,
                                lv_color_hex(sd_ready ? C_SD_ON : C_SD_OFF), 0);

    uint32_t s = millis() / 1000;
    char     tbuf[12];
    snprintf(tbuf, sizeof(tbuf), "%02lu:%02lu:%02lu",
             (s / 3600) % 24, (s / 60) % 60, s % 60);
    lv_label_set_text(ui_lbl_time, tbuf);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Bluetooth command parser
//  Supported commands (newline-terminated):
//    ADD,ref,dist,azmo,inclo,x,y,z,roll,pitch,yaw
//    CLEAR
//    LIST
// ─────────────────────────────────────────────────────────────────────────────
static void handle_bt_cmd(const String &raw)
{
    String cmd = raw;
    cmd.trim();

    if (cmd.equalsIgnoreCase("CLEAR")) {
        pt_count = 0;
        sel_row  = -1;
        refresh_survey();
        refresh_position();
        SerialBT.println("OK,CLEARED");
        return;
    }

    if (cmd.equalsIgnoreCase("LIST")) {
        SerialBT.println("REF,MARKER,DIST,AZMO,INCLO,X,Y,Z,ROLL,PITCH,YAW");
        for (int i = 0; i < pt_count; i++) {
            MeasPoint &p = pts[i];
            SerialBT.printf("%u,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                            p.ref, p.marker,
                            p.dist, p.azmo, p.inclo,
                            p.x, p.y, p.z,
                            p.roll, p.pitch, p.yaw);
        }
        SerialBT.println("END");
        return;
    }

    if (cmd.startsWith("ADD,") && pt_count < MAX_POINTS) {
        char     buf[128];
        MeasPoint &p = pts[pt_count];
        cmd.toCharArray(buf, sizeof(buf));
        char *tok = strtok(buf + 4, ",");
        if (tok) { p.ref   = (uint16_t)atoi(tok); tok = strtok(NULL, ","); }
        if (tok) { p.dist  = atof(tok);            tok = strtok(NULL, ","); }
        if (tok) { p.azmo  = atof(tok);            tok = strtok(NULL, ","); }
        if (tok) { p.inclo = atof(tok);            tok = strtok(NULL, ","); }
        if (tok) { p.x     = atof(tok);            tok = strtok(NULL, ","); }
        if (tok) { p.y     = atof(tok);            tok = strtok(NULL, ","); }
        if (tok) { p.z     = atof(tok);            tok = strtok(NULL, ","); }
        if (tok) { p.roll  = atof(tok);            tok = strtok(NULL, ","); }
        if (tok) { p.pitch = atof(tok);            tok = strtok(NULL, ","); }
        if (tok) { p.yaw   = atof(tok); }
        strcpy(p.marker, "");
        pt_count++;
        refresh_survey();
        refresh_position();
        SerialBT.printf("OK,ADDED,%d\n", pt_count);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Periodic LVGL timer  (1 s)
// ─────────────────────────────────────────────────────────────────────────────
static void periodic_cb(lv_timer_t *t)
{
    update_status();

    // Drain BT serial buffer
    static String bt_line;
    while (SerialBT.available()) {
        char c = (char)SerialBT.read();
        if (c == '\n') {
            if (bt_line.length() > 0) handle_bt_cmd(bt_line);
            bt_line = "";
        } else {
            bt_line += c;
        }
    }
}


// ─────────────────────────────────────────────────────────────────────────────
//  Build UI
// ─────────────────────────────────────────────────────────────────────────────
static void build_ui()
{
    // ── Screen background ────────────────────────────────────────────────
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // ── Top header bar  (36 px) ──────────────────────────────────────────
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

    lv_obj_t *lbl_title = lv_label_create(hdr);
    lv_label_set_text(lbl_title, "  BRICS-5  MM1");
    lv_obj_align(lbl_title, LV_ALIGN_LEFT_MID, 6, 0);
    lv_obj_set_style_text_color(lbl_title, lv_color_hex(C_HDR_LINE), 0);
    lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_16, 0);

    ui_lbl_count = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_count, "PTS: 0");
    lv_obj_align(ui_lbl_count, LV_ALIGN_LEFT_MID, 185, 0);
    lv_obj_set_style_text_color(ui_lbl_count, lv_color_hex(C_GREY), 0);
    lv_obj_set_style_text_font(ui_lbl_count, &lv_font_montserrat_14, 0);

    ui_lbl_sd = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_sd, LV_SYMBOL_SD_CARD);
    lv_obj_align(ui_lbl_sd, LV_ALIGN_RIGHT_MID, -155, 0);
    lv_obj_set_style_text_color(ui_lbl_sd, lv_color_hex(C_SD_OFF), 0);
    lv_obj_set_style_text_font(ui_lbl_sd, &lv_font_montserrat_14, 0);

    ui_lbl_bt = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_bt, LV_SYMBOL_BLUETOOTH);
    lv_obj_align(ui_lbl_bt, LV_ALIGN_RIGHT_MID, -100, 0);
    lv_obj_set_style_text_color(ui_lbl_bt, lv_color_hex(C_BT_OFF), 0);
    lv_obj_set_style_text_font(ui_lbl_bt, &lv_font_montserrat_14, 0);

    ui_lbl_time = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_time, "00:00:00");
    lv_obj_align(ui_lbl_time, LV_ALIGN_RIGHT_MID, -6, 0);
    lv_obj_set_style_text_color(ui_lbl_time, lv_color_hex(C_TEXT), 0);
    lv_obj_set_style_text_font(ui_lbl_time, &lv_font_montserrat_14, 0);

    // ── Tab view  (header to bottom of screen) ────────────────────────────
    // Tab content height: 320 - 36 header - 28 tab-strip = 256 px
    // Action bar at bottom of data tabs: 40 px
    // Table height inside data tabs: 256 - 40 = 216 px
    const int TAB_Y        = 36;
    const int TAB_H        = SCREEN_H - TAB_Y;   // 284 px
    const int STRIP_H      = 28;
    const int CONTENT_H    = TAB_H - STRIP_H;    // 256 px
    const int ACTION_H     = 40;
    const int TABLE_H      = CONTENT_H - ACTION_H;  // 216 px

    lv_obj_t *tv = lv_tabview_create(scr, LV_DIR_TOP, STRIP_H);
    lv_obj_set_size(tv, SCREEN_W, TAB_H);
    lv_obj_set_pos(tv, 0, TAB_Y);
    lv_obj_set_style_bg_color(tv, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_opa(tv, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(tv, 0, 0);
    lv_obj_clear_flag(tv, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(tv, tabview_changed_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *tab_btns = lv_tabview_get_tab_btns(tv);
    lv_obj_set_style_bg_color(tab_btns, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_opa(tab_btns, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(tab_btns, lv_color_hex(C_GREY), 0);
    lv_obj_set_style_text_color(tab_btns, lv_color_hex(C_HDR_LINE),
                                LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(tab_btns, lv_color_hex(C_HDR_LINE),
                                  LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_side(tab_btns, LV_BORDER_SIDE_BOTTOM,
                                 LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(tab_btns, 2,
                                  LV_PART_ITEMS | LV_STATE_CHECKED);

    // ── Shared helpers ────────────────────────────────────────────────────

    // Action bar at the bottom of a data tab (5 buttons × 88 px).
    // Returns the bar object so callers can attach buttons to it.
    auto make_action_bar = [&](lv_obj_t *parent) -> lv_obj_t * {
        lv_obj_t *bar = lv_obj_create(parent);
        lv_obj_set_size(bar, SCREEN_W, ACTION_H);
        lv_obj_set_pos(bar, 0, TABLE_H);
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
        lv_obj_set_flex_align(bar,
            LV_FLEX_ALIGN_SPACE_EVENLY,
            LV_FLEX_ALIGN_CENTER,
            LV_FLEX_ALIGN_CENTER);
        return bar;
    };

    // Small action button for the 40px bar.
    auto make_action_btn = [&](lv_obj_t *bar, uint32_t col,
                               const char *text, lv_event_cb_t cb) {
        lv_obj_t *btn = lv_btn_create(bar);
        lv_obj_set_size(btn, 86, 32);
        lv_obj_set_style_bg_color(btn, lv_color_hex(col), 0);
        lv_obj_set_style_bg_color(btn,
            lv_color_darken(lv_color_hex(col), LV_OPA_20), LV_STATE_PRESSED);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_set_style_shadow_width(btn, 3, 0);
        lv_obj_set_style_shadow_opa(btn, LV_OPA_30, 0);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, text);
        lv_obj_center(lbl);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(C_WHITE), 0);
    };

    // ── SURVEY tab ────────────────────────────────────────────────────────
    lv_obj_t *tab_s = lv_tabview_add_tab(tv, LV_SYMBOL_LIST " SURVEY");
    lv_obj_set_style_bg_color(tab_s, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(tab_s, 0, 0);
    lv_obj_clear_flag(tab_s, LV_OBJ_FLAG_SCROLLABLE);

    ui_tbl_survey = lv_table_create(tab_s);
    lv_obj_set_size(ui_tbl_survey, SCREEN_W, TABLE_H);
    lv_obj_set_pos(ui_tbl_survey, 0, 0);
    lv_table_set_col_cnt(ui_tbl_survey, 4);
    lv_table_set_col_width(ui_tbl_survey, 0,  76);
    lv_table_set_col_width(ui_tbl_survey, 1, 130);
    lv_table_set_col_width(ui_tbl_survey, 2, 130);
    lv_table_set_col_width(ui_tbl_survey, 3, 130);
    lv_obj_set_style_bg_color(ui_tbl_survey, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_top(ui_tbl_survey,    5, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(ui_tbl_survey, 5, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(ui_tbl_survey,   8, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(ui_tbl_survey,  8, LV_PART_ITEMS);
    lv_obj_add_event_cb(ui_tbl_survey, survey_draw_cb,  LV_EVENT_DRAW_PART_BEGIN, nullptr);
    lv_obj_add_event_cb(ui_tbl_survey, survey_click_cb, LV_EVENT_VALUE_CHANGED,   nullptr);

    lv_obj_t *bar_s = make_action_bar(tab_s);
    make_action_btn(bar_s, C_BTN_ADD,   LV_SYMBOL_PLUS  " ADD",  btn_add_cb);
    make_action_btn(bar_s, C_BTN_DEL,   LV_SYMBOL_CLOSE " DEL",  btn_del_row_cb);
    make_action_btn(bar_s, C_BTN_CLEAR, LV_SYMBOL_TRASH " CLR",  btn_clear_cb);
    make_action_btn(bar_s, C_BTN_SAVE,  LV_SYMBOL_SAVE  " SAVE", btn_save_cb);
    make_action_btn(bar_s, C_BTN_LOAD,  LV_SYMBOL_DOWNLOAD " LOAD", btn_load_cb);

    // ── POSITION tab ──────────────────────────────────────────────────────
    lv_obj_t *tab_p = lv_tabview_add_tab(tv, LV_SYMBOL_GPS " POSITION");
    lv_obj_set_style_bg_color(tab_p, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(tab_p, 0, 0);
    lv_obj_clear_flag(tab_p, LV_OBJ_FLAG_SCROLLABLE);

    ui_tbl_position = lv_table_create(tab_p);
    lv_obj_set_size(ui_tbl_position, SCREEN_W, TABLE_H);
    lv_obj_set_pos(ui_tbl_position, 0, 0);
    lv_table_set_col_cnt(ui_tbl_position, 7);
    lv_table_set_col_width(ui_tbl_position, 0,  56);
    lv_table_set_col_width(ui_tbl_position, 1,  72);
    lv_table_set_col_width(ui_tbl_position, 2,  72);
    lv_table_set_col_width(ui_tbl_position, 3,  72);
    lv_table_set_col_width(ui_tbl_position, 4,  52);
    lv_table_set_col_width(ui_tbl_position, 5,  52);
    lv_table_set_col_width(ui_tbl_position, 6,  52);
    lv_obj_set_style_bg_color(ui_tbl_position, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_top(ui_tbl_position,    4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(ui_tbl_position, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(ui_tbl_position,   5, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(ui_tbl_position,  5, LV_PART_ITEMS);
    lv_obj_add_event_cb(ui_tbl_position, position_draw_cb,  LV_EVENT_DRAW_PART_BEGIN, nullptr);
    lv_obj_add_event_cb(ui_tbl_position, position_click_cb, LV_EVENT_VALUE_CHANGED,   nullptr);

    lv_obj_t *bar_p = make_action_bar(tab_p);
    make_action_btn(bar_p, C_BTN_ADD,   LV_SYMBOL_PLUS  " ADD",  btn_add_cb);
    make_action_btn(bar_p, C_BTN_DEL,   LV_SYMBOL_CLOSE " DEL",  btn_del_row_cb);
    make_action_btn(bar_p, C_BTN_CLEAR, LV_SYMBOL_TRASH " CLR",  btn_clear_cb);
    make_action_btn(bar_p, C_BTN_SAVE,  LV_SYMBOL_SAVE  " SAVE", btn_save_cb);
    make_action_btn(bar_p, C_BTN_LOAD,  LV_SYMBOL_DOWNLOAD " LOAD", btn_load_cb);

    // ── MANAGE tab  (CSV file browser) ────────────────────────────────────
    lv_obj_t *tab_m = lv_tabview_add_tab(tv, LV_SYMBOL_SD_CARD " FILES");
    lv_obj_set_style_bg_color(tab_m, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(tab_m, 6, 0);
    lv_obj_clear_flag(tab_m, LV_OBJ_FLAG_SCROLLABLE);

    // ── Active file indicator ─────────────────────────────────────────────
    ui_lbl_active = lv_label_create(tab_m);
    lv_label_set_long_mode(ui_lbl_active, LV_LABEL_LONG_DOT);
    lv_obj_set_width(ui_lbl_active, SCREEN_W - 12);
    lv_obj_align(ui_lbl_active, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_font(ui_lbl_active, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ui_lbl_active, lv_color_hex(C_REF_S), 0);

    // ── File list table ────────────────────────────────────────────────────
    // Height: CONTENT_H(256) - 6pad - 22lbl - 4gap - 44btn - 4gap - 22status - 6pad = 148 px
    const int FILE_TBL_H = CONTENT_H - 12 - 22 - 4 - 44 - 4 - 22;   // ~148 px
    ui_tbl_files = lv_table_create(tab_m);
    lv_obj_set_size(ui_tbl_files, SCREEN_W - 12, FILE_TBL_H);
    lv_obj_align(ui_tbl_files, LV_ALIGN_TOP_LEFT, 0, 26);
    lv_table_set_col_cnt(ui_tbl_files, 2);
    lv_table_set_col_width(ui_tbl_files, 0, SCREEN_W - 12 - 90);  // filename
    lv_table_set_col_width(ui_tbl_files, 1, 84);                   // size
    lv_obj_set_style_bg_color(ui_tbl_files, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_top(ui_tbl_files,    4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(ui_tbl_files, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(ui_tbl_files,   8, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(ui_tbl_files,  8, LV_PART_ITEMS);
    lv_obj_add_event_cb(ui_tbl_files, file_list_draw_cb,  LV_EVENT_DRAW_PART_BEGIN, nullptr);
    lv_obj_add_event_cb(ui_tbl_files, file_list_click_cb, LV_EVENT_VALUE_CHANGED,   nullptr);

    // ── File operation buttons row ────────────────────────────────────────
    auto make_file_btn = [&](lv_obj_t *parent, uint32_t col,
                             const char *text, lv_event_cb_t cb) {
        lv_obj_t *btn = lv_btn_create(parent);
        lv_obj_set_size(btn, 148, 40);
        lv_obj_set_style_bg_color(btn, lv_color_hex(col), 0);
        lv_obj_set_style_bg_color(btn,
            lv_color_darken(lv_color_hex(col), LV_OPA_20), LV_STATE_PRESSED);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_set_style_shadow_width(btn, 4, 0);
        lv_obj_set_style_shadow_opa(btn, LV_OPA_30, 0);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, text);
        lv_obj_center(lbl);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(C_WHITE), 0);
    };

    int btn_y = 26 + FILE_TBL_H + 4;
    lv_obj_t *btn_row = lv_obj_create(tab_m);
    lv_obj_set_size(btn_row, SCREEN_W - 12, 44);
    lv_obj_align(btn_row, LV_ALIGN_TOP_LEFT, 0, btn_y);
    lv_obj_set_style_bg_opa(btn_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btn_row, 0, 0);
    lv_obj_set_style_pad_all(btn_row, 0, 0);
    lv_obj_set_layout(btn_row, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row,
        LV_FLEX_ALIGN_SPACE_EVENLY,
        LV_FLEX_ALIGN_CENTER,
        LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(btn_row, LV_OBJ_FLAG_SCROLLABLE);

    make_file_btn(btn_row, C_BTN_ADD, LV_SYMBOL_PLUS " NEW FILE",   btn_new_file_cb);
    make_file_btn(btn_row, C_BTN_USE, LV_SYMBOL_OK   " USE FILE",   btn_use_file_cb);
    make_file_btn(btn_row, C_BTN_DEL, LV_SYMBOL_TRASH " DEL FILE",  btn_del_file_cb);

    // ── Status / feedback label ───────────────────────────────────────────
    ui_lbl_manage_status = lv_label_create(tab_m);
    lv_label_set_long_mode(ui_lbl_manage_status, LV_LABEL_LONG_DOT);
    lv_obj_set_width(ui_lbl_manage_status, SCREEN_W - 12);
    lv_obj_align(ui_lbl_manage_status, LV_ALIGN_TOP_LEFT, 0, btn_y + 48);
    lv_label_set_text(ui_lbl_manage_status,
                      "Select a file then tap USE FILE to load it");
    lv_obj_set_style_text_font(ui_lbl_manage_status, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ui_lbl_manage_status, lv_color_hex(C_GREY), 0);

    // ── Initial data render ───────────────────────────────────────────────
    refresh_survey();
    refresh_position();
    update_active_lbl();
    update_status();
}

// ─────────────────────────────────────────────────────────────────────────────
//  SD card init helper  (VSPI default pins – no conflict with touch)
// ─────────────────────────────────────────────────────────────────────────────
static void sd_init()
{
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    sd_ready = SD.begin(SD_CS);
    if (sd_ready) {
        uint8_t ct = SD.cardType();
        Serial.printf("[SD] OK  type=%s  size=%llu MB\n",
                      ct == CARD_MMC  ? "MMC"  :
                      ct == CARD_SD   ? "SD"   :
                      ct == CARD_SDHC ? "SDHC" : "UNKNOWN",
                      SD.cardSize() / (1024ULL * 1024ULL));
    } else {
        Serial.println("[SD] Not found – check wiring / FAT32 format");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    Serial.println("\n[BRICS-5 MM1] Booting...");

    // ── TFT + Touch (HSPI pins 12/13/14, touch CS=33) ───────────────────
    tft.init();
    tft.setRotation(1);   // landscape 480×320
    tft.setTouch(const_cast<uint16_t *>(TOUCH_CAL));  // calibration for rotation=1
    tft.fillScreen(TFT_BLACK);

    // ── SD card (VSPI default pins 18/19/23/5 – completely separate bus) ─
    sd_init();

    // ── Bluetooth Serial ─────────────────────────────────────────────────
    SerialBT.begin("BRICS5-MM1");
    Serial.println("[BT] 'BRICS5-MM1' advertising");

    // ── LVGL – display driver ────────────────────────────────────────────
    lv_init();
    lvgl_buf = (lv_color_t *)malloc(SCREEN_W * 10 * sizeof(lv_color_t));
    if (!lvgl_buf) {
        Serial.println("[LVGL] FATAL: draw buffer allocation failed");
        while (true) delay(1000);
    }
    lv_disp_draw_buf_init(&draw_buf, lvgl_buf, nullptr, SCREEN_W * 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = SCREEN_W;
    disp_drv.ver_res  = SCREEN_H;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    lv_theme_t *th = lv_theme_default_init(
        disp,
        lv_palette_main(LV_PALETTE_BLUE),
        lv_palette_main(LV_PALETTE_TEAL),
        false,
        &lv_font_montserrat_14);
    lv_disp_set_theme(disp, th);

    // ── Load data from SD; create active CSV if it doesn't exist yet ────
    if (sd_ready) {
        if (!SD.exists(active_csv)) {
            File f = SD.open(active_csv, FILE_WRITE);
            if (f) { f.println("REF,MARKER,DIST,AZMO,INCLO,X,Y,Z,ROLL,PITCH,YAW"); f.close(); }
            Serial.printf("[SD] Created empty %s\n", active_csv);
        }
        load_csv();
    }

    // ── LVGL touch input device ───────────────────────────────────────────
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read;
    lv_indev_drv_register(&indev_drv);

    build_ui();

    lv_timer_create(periodic_cb, 1000, nullptr);

    Serial.println("[BRICS-5 MM1] Ready");
}

// ─────────────────────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    lv_timer_handler();
    delay(5);
}
