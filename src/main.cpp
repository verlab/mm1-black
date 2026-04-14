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
 *  ├─────────────────────── 40px Footer ────────────────────────────┤
 *  │  [+ NEW POINT]      [💾 SAVE SD]      [🗑 CLEAR ALL]           │
 *  └────────────────────────────────────────────────────────────────┘
 *
 * Bluetooth Serial commands (newline-terminated):
 *   ADD,<ref>,<dist>,<azmo>,<inclo>,<x>,<y>,<z>,<roll>,<pitch>,<yaw>
 *   CLEAR
 *   LIST   -> responds with CSV dump
 */

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <lvgl.h>
#include <SD.h>
#include <BluetoothSerial.h>

// ─────────────────────────────────────────────────────────────────────────────
//  Hardware pinout  (CYD  ESP32-2432S028R / Random Nerd Tutorials layout)
// ─────────────────────────────────────────────────────────────────────────────
#define SCREEN_W  480
#define SCREEN_H  320

// Touch  (VSPI  – custom pins)
#define XPT2046_CLK   25
#define XPT2046_MOSI  32
#define XPT2046_MISO  39
#define XPT2046_CS    33
#define XPT2046_IRQ   36

// SD card (shares VSPI bus with touch)
#define SD_CS   5

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
#define C_BTN_ADD     0x00897Bu   // teal
#define C_BTN_SAVE    0x1565C0u   // blue
#define C_BTN_CLEAR   0xB71C1Cu   // red

// ─────────────────────────────────────────────────────────────────────────────
//  Data structure
// ─────────────────────────────────────────────────────────────────────────────
#define MAX_POINTS 30

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
static TFT_eSPI           tft;
static SPIClass           touchSPI(VSPI);
static XPT2046_Touchscreen touch(XPT2046_CS, XPT2046_IRQ);
static BluetoothSerial    SerialBT;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t        *lvgl_buf = nullptr;   // allocated on heap in setup()

static MeasPoint  pts[MAX_POINTS];
static int        pt_count    = 0;
static int        sel_row     = -1;
static bool       sd_ready    = false;
static bool       bt_conn     = false;

// UI handles
static lv_obj_t *ui_tbl_survey   = nullptr;
static lv_obj_t *ui_tbl_position = nullptr;
static lv_obj_t *ui_lbl_bt       = nullptr;
static lv_obj_t *ui_lbl_sd       = nullptr;
static lv_obj_t *ui_lbl_time     = nullptr;
static lv_obj_t *ui_lbl_count    = nullptr;
static lv_obj_t *ui_save_lbl     = nullptr;  // label inside save button

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
//  LVGL touch read (XPT2046 backend)
//  Touch calibration is for CYD landscape rotation=1.
//  If touches are inverted/swapped, adjust the map() calls below.
// ─────────────────────────────────────────────────────────────────────────────
static void touch_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    // GPIO 36 (IRQ) is an input-only pin on ESP32; tirqTouched() can miss
    // events.  Polling touch.touched() directly via SPI is reliable.
    if (touch.touched()) {
        TS_Point p = touch.getPoint();
        // Landscape rotation 1: swap axes and remap to screen pixels.
        // Adjust the raw min/max values if the calibration is off for your
        // specific unit (typical CYD range: x 200-3900, y 240-3800).
        int16_t tx = map(p.y, 3800, 240,  0, SCREEN_W - 1);
        int16_t ty = map(p.x, 200,  3900, 0, SCREEN_H - 1);
        data->point.x = (int16_t)constrain(tx, 0, SCREEN_W - 1);
        data->point.y = (int16_t)constrain(ty, 0, SCREEN_H - 1);
        data->state   = LV_INDEV_STATE_PR;
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
    // Keep one empty row past the data so the table always fills visually
    if (pt_count < MAX_POINTS) {
        lv_table_set_cell_value(t, pt_count + 1, 0, "");
        lv_table_set_cell_value(t, pt_count + 1, 1, "");
        lv_table_set_cell_value(t, pt_count + 1, 2, "");
        lv_table_set_cell_value(t, pt_count + 1, 3, "");
    }

    snprintf(buf, sizeof(buf), "PTS: %d", pt_count);
    lv_label_set_text(ui_lbl_count, buf);
}

static void refresh_position()
{
    lv_obj_t *t = ui_tbl_position;
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
    // FILE_WRITE appends; remove first so each save is a clean overwrite.
    if (SD.exists("/brics5_mm1.csv")) {
        SD.remove("/brics5_mm1.csv");
    }
    File f = SD.open("/brics5_mm1.csv", FILE_WRITE);
    if (!f) {
        Serial.println("[SD] save_csv: could not open file for writing");
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
    Serial.printf("[SD] Saved %d points to /brics5_mm1.csv\n", pt_count);
}

static void load_csv()
{
    if (!sd_ready) {
        Serial.println("[SD] load_csv: card not ready");
        return;
    }
    File f = SD.open("/brics5_mm1.csv", FILE_READ);
    if (!f) {
        Serial.println("[SD] load_csv: file not found");
        return;
    }
    Serial.printf("[SD] load_csv: size %u bytes\n", (unsigned)f.size());
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
    Serial.printf("[SD] Loaded %d points from /brics5_mm1.csv\n", pt_count);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Button callbacks
// ─────────────────────────────────────────────────────────────────────────────
static void restore_save_lbl_cb(lv_timer_t *t)
{
    lv_label_set_text(ui_save_lbl, LV_SYMBOL_SAVE " SAVE SD");
    lv_timer_del(t);
}

static void btn_save_cb(lv_event_t *e)
{
    save_csv();
    lv_label_set_text(ui_save_lbl, LV_SYMBOL_OK " SAVED!");
    lv_timer_create(restore_save_lbl_cb, 2000, nullptr);
}

static void btn_add_cb(lv_event_t *e)
{
    if (pt_count >= MAX_POINTS) return;
    // Add an empty placeholder point; in production, replace with an
    // input dialog or receive data via Bluetooth.
    MeasPoint &p = pts[pt_count];
    p.ref   = (uint16_t)(pt_count + 1);
    p.dist  = 0.0f; p.azmo  = 0.0f; p.inclo = 0.0f;
    p.x     = 0.0f; p.y     = 0.0f; p.z     = 0.0f;
    p.roll  = 0.0f; p.pitch = 0.0f; p.yaw   = 0.0f;
    strcpy(p.marker, "");
    pt_count++;
    refresh_survey();
    refresh_position();
}

static void btn_clear_cb(lv_event_t *e)
{
    pt_count = 0;
    sel_row  = -1;
    refresh_survey();
    refresh_position();
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
//  Load factory sample data  (matches the photo)
// ─────────────────────────────────────────────────────────────────────────────
static void load_sample_data()
{
    auto add = [](uint16_t ref, const char *m,
                  float dist, float azmo, float inclo,
                  float x,    float y,    float z,
                  float r,    float p,    float yw) {
        if (pt_count >= MAX_POINTS) return;
        MeasPoint &pt = pts[pt_count++];
        pt.ref = ref;
        strlcpy(pt.marker, m, sizeof(pt.marker));
        pt.dist = dist; pt.azmo = azmo; pt.inclo = inclo;
        pt.x = x;  pt.y = y;  pt.z = z;
        pt.roll = r; pt.pitch = p; pt.yaw = yw;
    };

    //      ref   mrk   dist   azmo   inclo     x       y       z      r      p      yw
    add(90, "",   12.4, 102.1, 25.1,  11.90, -2.60,   5.30,   2.1,  -3.2, 102.1);
    add(89, "",   39.3, 137.1,  5.6,  28.70,-26.70,   3.80,   0.8,   1.2, 137.1);
    add(88, "",   39.2, 133.3,  2.4,  27.00,-28.50,   1.60,  -0.4,   0.3, 133.3);
    add(87, "E",  17.4, 149.5, 11.5,  14.90,-11.20,   3.50,   1.5,  -2.1, 149.5);
    add(86, "",    6.4,  66.5, 16.5,   2.60,  5.90,   1.80,   0.9,   3.4,  66.5);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Build UI
// ─────────────────────────────────────────────────────────────────────────────
static void build_ui()
{
    // ── Screen background ──────────────────────────────────────────────────
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // ── Top header bar  (36 px) ────────────────────────────────────────────
    lv_obj_t *hdr = lv_obj_create(scr);
    lv_obj_set_size(hdr, SCREEN_W, 36);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_style_bg_color(hdr, lv_color_hex(C_TOPBAR), 0);
    lv_obj_set_style_bg_opa(hdr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_border_side(hdr, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_color(hdr, lv_color_hex(C_HDR_LINE), LV_PART_MAIN);
    lv_obj_set_style_border_width(hdr, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 0, 0);
    lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t *lbl_title = lv_label_create(hdr);
    lv_label_set_text(lbl_title, "  BRICS-5  MM1");
    lv_obj_align(lbl_title, LV_ALIGN_LEFT_MID, 6, 0);
    lv_obj_set_style_text_color(lbl_title, lv_color_hex(C_HDR_LINE), 0);
    lv_obj_set_style_text_font(lbl_title, &lv_font_montserrat_16, 0);

    // Point count
    ui_lbl_count = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_count, "PTS: 0");
    lv_obj_align(ui_lbl_count, LV_ALIGN_LEFT_MID, 185, 0);
    lv_obj_set_style_text_color(ui_lbl_count, lv_color_hex(C_GREY), 0);
    lv_obj_set_style_text_font(ui_lbl_count, &lv_font_montserrat_14, 0);

    // SD status
    ui_lbl_sd = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_sd, LV_SYMBOL_SD_CARD);
    lv_obj_align(ui_lbl_sd, LV_ALIGN_RIGHT_MID, -155, 0);
    lv_obj_set_style_text_color(ui_lbl_sd, lv_color_hex(C_SD_OFF), 0);
    lv_obj_set_style_text_font(ui_lbl_sd, &lv_font_montserrat_14, 0);

    // Bluetooth status
    ui_lbl_bt = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_bt, LV_SYMBOL_BLUETOOTH);
    lv_obj_align(ui_lbl_bt, LV_ALIGN_RIGHT_MID, -100, 0);
    lv_obj_set_style_text_color(ui_lbl_bt, lv_color_hex(C_BT_OFF), 0);
    lv_obj_set_style_text_font(ui_lbl_bt, &lv_font_montserrat_14, 0);

    // Clock
    ui_lbl_time = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_time, "00:00:00");
    lv_obj_align(ui_lbl_time, LV_ALIGN_RIGHT_MID, -6, 0);
    lv_obj_set_style_text_color(ui_lbl_time, lv_color_hex(C_TEXT), 0);
    lv_obj_set_style_text_font(ui_lbl_time, &lv_font_montserrat_14, 0);

    // ── Bottom footer bar  (40 px) ─────────────────────────────────────────
    lv_obj_t *ftr = lv_obj_create(scr);
    lv_obj_set_size(ftr, SCREEN_W, 40);
    lv_obj_set_pos(ftr, 0, SCREEN_H - 40);
    lv_obj_set_style_bg_color(ftr, lv_color_hex(C_TOPBAR), 0);
    lv_obj_set_style_bg_opa(ftr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(ftr, 1, LV_PART_MAIN);
    lv_obj_set_style_border_side(ftr, LV_BORDER_SIDE_TOP, LV_PART_MAIN);
    lv_obj_set_style_border_color(ftr, lv_color_hex(C_HDR_LINE), LV_PART_MAIN);
    lv_obj_set_style_radius(ftr, 0, 0);
    lv_obj_set_style_pad_all(ftr, 4, 0);
    lv_obj_clear_flag(ftr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(ftr, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(ftr, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ftr,
                          LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    auto make_btn = [&](lv_obj_t *parent, uint32_t col,
                        const char *text, lv_event_cb_t cb,
                        lv_obj_t **lbl_out) -> lv_obj_t * {
        lv_obj_t *btn = lv_btn_create(parent);
        lv_obj_set_size(btn, 145, 32);
        lv_obj_set_style_bg_color(btn, lv_color_hex(col), 0);
        lv_obj_set_style_bg_color(btn, lv_color_darken(lv_color_hex(col), LV_OPA_20),
                                  LV_STATE_PRESSED);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_set_style_shadow_width(btn, 4, 0);
        lv_obj_set_style_shadow_color(btn, lv_color_hex(0x000000), 0);
        lv_obj_set_style_shadow_opa(btn, LV_OPA_40, 0);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, text);
        lv_obj_center(lbl);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        if (lbl_out) *lbl_out = lbl;
        return btn;
    };

    make_btn(ftr, C_BTN_ADD,   LV_SYMBOL_PLUS " NEW POINT", btn_add_cb,   nullptr);
    make_btn(ftr, C_BTN_SAVE,  LV_SYMBOL_SAVE " SAVE SD",   btn_save_cb,  &ui_save_lbl);
    make_btn(ftr, C_BTN_CLEAR, LV_SYMBOL_TRASH " CLEAR ALL", btn_clear_cb, nullptr);

    // ── Tab view  (fills the space between header and footer) ─────────────
    const int TAB_Y = 36;
    const int TAB_H = SCREEN_H - 36 - 40;   // 244 px

    lv_obj_t *tv = lv_tabview_create(scr, LV_DIR_TOP, 28);
    lv_obj_set_size(tv, SCREEN_W, TAB_H);
    lv_obj_set_pos(tv, 0, TAB_Y);
    lv_obj_set_style_bg_color(tv, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_opa(tv, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(tv, 0, 0);
    lv_obj_clear_flag(tv, LV_OBJ_FLAG_SCROLLABLE);

    // Tab button strip
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

    // ── SURVEY tab ────────────────────────────────────────────────────────
    lv_obj_t *tab_s = lv_tabview_add_tab(tv, LV_SYMBOL_LIST " SURVEY");
    lv_obj_set_style_bg_color(tab_s, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(tab_s, 0, 0);
    lv_obj_clear_flag(tab_s, LV_OBJ_FLAG_SCROLLABLE);

    ui_tbl_survey = lv_table_create(tab_s);
    lv_obj_set_size(ui_tbl_survey, SCREEN_W, TAB_H - 28);
    lv_obj_set_pos(ui_tbl_survey, 0, 0);
    lv_table_set_col_cnt(ui_tbl_survey, 4);
    lv_table_set_col_width(ui_tbl_survey, 0,  76);   // REF
    lv_table_set_col_width(ui_tbl_survey, 1, 130);   // DIST
    lv_table_set_col_width(ui_tbl_survey, 2, 130);   // AZMO
    lv_table_set_col_width(ui_tbl_survey, 3, 130);   // INCLO
    lv_obj_set_style_bg_color(ui_tbl_survey, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_top(ui_tbl_survey,    5, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(ui_tbl_survey, 5, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(ui_tbl_survey,   8, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(ui_tbl_survey,  8, LV_PART_ITEMS);
    lv_obj_add_event_cb(ui_tbl_survey, survey_draw_cb,  LV_EVENT_DRAW_PART_BEGIN, nullptr);
    lv_obj_add_event_cb(ui_tbl_survey, survey_click_cb, LV_EVENT_VALUE_CHANGED,   nullptr);

    // ── POSITION tab ──────────────────────────────────────────────────────
    lv_obj_t *tab_p = lv_tabview_add_tab(tv, LV_SYMBOL_GPS " POSITION");
    lv_obj_set_style_bg_color(tab_p, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(tab_p, 0, 0);
    lv_obj_clear_flag(tab_p, LV_OBJ_FLAG_SCROLLABLE);

    ui_tbl_position = lv_table_create(tab_p);
    lv_obj_set_size(ui_tbl_position, SCREEN_W, TAB_H - 28);
    lv_obj_set_pos(ui_tbl_position, 0, 0);
    lv_table_set_col_cnt(ui_tbl_position, 7);
    lv_table_set_col_width(ui_tbl_position, 0,  56);   // REF
    lv_table_set_col_width(ui_tbl_position, 1,  72);   // X
    lv_table_set_col_width(ui_tbl_position, 2,  72);   // Y
    lv_table_set_col_width(ui_tbl_position, 3,  72);   // Z
    lv_table_set_col_width(ui_tbl_position, 4,  52);   // R
    lv_table_set_col_width(ui_tbl_position, 5,  52);   // P
    lv_table_set_col_width(ui_tbl_position, 6,  52);   // Y(aw)
    lv_obj_set_style_bg_color(ui_tbl_position, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_top(ui_tbl_position,    4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(ui_tbl_position, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(ui_tbl_position,   5, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(ui_tbl_position,  5, LV_PART_ITEMS);
    lv_obj_add_event_cb(ui_tbl_position, position_draw_cb,  LV_EVENT_DRAW_PART_BEGIN, nullptr);
    lv_obj_add_event_cb(ui_tbl_position, position_click_cb, LV_EVENT_VALUE_CHANGED,   nullptr);

    // ── Initial data render ───────────────────────────────────────────────
    refresh_survey();
    refresh_position();
    update_status();
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    Serial.println("\n[BRICS-5 MM1] Booting...");

    // TFT – landscape
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    // Touch (VSPI with custom pins)
    touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    touch.begin(touchSPI);

    // SD card – shares VSPI bus with touch (CS on separate pin)
    sd_ready = SD.begin(SD_CS, touchSPI);
    if (sd_ready) {
        uint8_t ctype = SD.cardType();
        const char *ct = (ctype == CARD_MMC)  ? "MMC"  :
                         (ctype == CARD_SD)   ? "SD"   :
                         (ctype == CARD_SDHC) ? "SDHC" : "UNKNOWN";
        Serial.printf("[SD] OK  type=%s  size=%llu MB\n",
                      ct, SD.cardSize() / (1024ULL * 1024ULL));
    } else {
        Serial.println("[SD] Not found – check wiring or FAT32 format");
    }

    // Bluetooth Serial
    SerialBT.begin("BRICS5-MM1");
    Serial.println("[BT] 'BRICS5-MM1' advertising");

    // LVGL
    lv_init();
    // Allocate the draw buffer on the heap to avoid consuming BSS/DRAM segment.
    // SCREEN_W * 10 lines  ≈ 9 600 bytes  –  sufficient for smooth rendering.
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

    // Light theme – blue primary, teal secondary
    lv_theme_t *th = lv_theme_default_init(
        disp,
        lv_palette_main(LV_PALETTE_BLUE),
        lv_palette_main(LV_PALETTE_TEAL),
        false, // light mode
        &lv_font_montserrat_14);
    lv_disp_set_theme(disp, th);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read;
    lv_indev_drv_register(&indev_drv);

    // Load data: try SD first, then fall back to sample data
    if (sd_ready && SD.exists("/brics5_mm1.csv")) {
        load_csv();
    } else {
        load_sample_data();
    }

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
