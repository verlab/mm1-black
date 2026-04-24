/**
 * BRICS-5 MM1  –  Smart Tape / 1D LiDAR + IMU
 * ESP32 CYD  |  ST7796 480×320  |  LVGL 8.3
 *
 * Flat point list with a single station (GPS entered via popup / BT).
 * Two point types:
 *   S (Sample)    – measurement samples
 *   N (Navigation) – reference points for transforms
 *
 * Tabs: POINTS | SENSOR | FILES
 *
 * BT commands:  GPS,lat,lon,alt  |  MEAS  |  CLEAR  |  LIST
 */

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_BNO08x.h>
#include <lvgl.h>
#include <SD.h>
#include <BluetoothSerial.h>

// ── Pins ─────────────────────────────────────────────────────────────────────
#define SCREEN_W  480
#define SCREEN_H  320
#define SD_CS 5
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23
#define I2C_SDA 32
#define I2C_SCL 25
#define IMU_RST 17
#define IMU_INT 16
#define IMU_ADDR 0x4B
#define TOF_ADDR 0x08
#define TOF_REG  0x24
#define BAT_ADC_PIN 34

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

// ── Data ─────────────────────────────────────────────────────────────────────
#define MAX_PTS  50
#define MAX_FILES 16

enum PtType : uint8_t { PT_SAMPLE = 0, PT_NAV = 1 };

struct MeasPoint {
    uint16_t id;
    PtType   type;
    float    dist, roll, pitch, yaw;
};

// ── Globals ──────────────────────────────────────────────────────────────────
static TFT_eSPI          tft;
static BluetoothSerial   SerialBT;
static Adafruit_BNO08x   bno08x(-1);
static sh2_SensorValue_t sensorValue;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t        *lvgl_buf = nullptr;

static MeasPoint pts[MAX_PTS];
static int       pt_count = 0;
static int       sel_row  = -1;
static uint16_t  next_id  = 1;

// Station GPS (single station)
static float stn_lat = 0, stn_lon = 0, stn_alt = 0;

// Sensors
static uint32_t tof_dist_mm = 0;
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
static lv_obj_t *ui_lbl_gps_disp = nullptr;
static lv_obj_t *ui_tbl_files    = nullptr;
static lv_obj_t *ui_lbl_active   = nullptr;
static lv_obj_t *ui_lbl_fstatus  = nullptr;
static lv_obj_t *ui_gps_overlay  = nullptr;   // GPS popup
static lv_obj_t *ui_lbl_gps_lat  = nullptr;
static lv_obj_t *ui_lbl_gps_lon  = nullptr;
static lv_obj_t *ui_lbl_gps_alt  = nullptr;

// Forward declarations
static void sd_init();
static void sensor_init();
static void refresh_table();
static void refresh_sensor_display();
static void set_fstatus(const char *msg);

void IRAM_ATTR imuISR() { imu_irq = true; }

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
    uint16_t tx = 0, ty = 0;
    if (tft.getTouch(&tx, &ty)) {
        data->point.x = (int16_t)tx;
        data->point.y = (int16_t)ty;
        data->state   = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// ── Sensor functions ─────────────────────────────────────────────────────────
static bool read_tof(uint32_t &d)
{
    Wire.beginTransmission(TOF_ADDR);
    Wire.write(TOF_REG);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)TOF_ADDR, 4) != 4) return false;
    uint8_t b0=Wire.read(), b1=Wire.read(), b2=Wire.read(), b3=Wire.read();
    d = b0 | ((uint32_t)b1<<8) | ((uint32_t)b2<<16) | ((uint32_t)b3<<24);
    return true;
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
            float qw=sensorValue.un.rotationVector.real;
            float qx=sensorValue.un.rotationVector.i;
            float qy=sensorValue.un.rotationVector.j;
            float qz=sensorValue.un.rotationVector.k;
            imu_yaw   = atan2f(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))*180.f/(float)M_PI;
            imu_pitch = asinf(fmaxf(-1,fminf(1, 2*(qw*qy-qz*qx))))*180.f/(float)M_PI;
            imu_roll  = atan2f(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))*180.f/(float)M_PI;
        }
    }
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
        int idx = (int)row - 1;
        bool selected = (idx == sel_row);
        bool is_nav   = (idx < pt_count && pts[idx].type == PT_NAV);
        if (selected)
            dsc->rect_dsc->bg_color = lv_color_hex(C_ROW_SEL);
        else if (is_nav)
            dsc->rect_dsc->bg_color = lv_color_hex(C_ROW_NAV);
        else
            dsc->rect_dsc->bg_color = (row%2==0) ? lv_color_hex(C_ROW_EVEN)
                                                  : lv_color_hex(C_ROW_ODD);
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;
        if (col == 0) {
            dsc->label_dsc->color = is_nav ? lv_color_hex(C_TYPE_N)
                                           : lv_color_hex(C_TYPE_S);
            dsc->label_dsc->align = LV_TEXT_ALIGN_CENTER;
        } else if (col == 1) {
            dsc->label_dsc->color = lv_color_hex(C_REF_S);
            dsc->label_dsc->align = LV_TEXT_ALIGN_CENTER;
        } else {
            dsc->label_dsc->color = lv_color_hex(C_TEXT);
            dsc->label_dsc->align = LV_TEXT_ALIGN_RIGHT;
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
    if (row > 0 && (int)(row-1) < pt_count) {
        int idx = (int)row - 1;
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
    lv_table_set_cell_value(t, 0, 0, "TYPE");
    lv_table_set_cell_value(t, 0, 1, "#");
    lv_table_set_cell_value(t, 0, 2, "DIST (m)");
    lv_table_set_cell_value(t, 0, 3, "ROLL \xC2\xB0");
    lv_table_set_cell_value(t, 0, 4, "PITCH \xC2\xB0");
    lv_table_set_cell_value(t, 0, 5, "YAW \xC2\xB0");

    char buf[24];
    for (int i = 0; i < pt_count; i++) {
        lv_table_set_cell_value(t, i+1, 0,
            pts[i].type == PT_NAV ? "NAV" : "SMP");
        snprintf(buf, sizeof(buf), "%u", pts[i].id);
        lv_table_set_cell_value(t, i+1, 1, buf);
        snprintf(buf, sizeof(buf), "%.3f", pts[i].dist);
        lv_table_set_cell_value(t, i+1, 2, buf);
        snprintf(buf, sizeof(buf), "%.1f", pts[i].roll);
        lv_table_set_cell_value(t, i+1, 3, buf);
        snprintf(buf, sizeof(buf), "%.1f", pts[i].pitch);
        lv_table_set_cell_value(t, i+1, 4, buf);
        snprintf(buf, sizeof(buf), "%.1f", pts[i].yaw);
        lv_table_set_cell_value(t, i+1, 5, buf);
    }
    snprintf(buf, sizeof(buf), "PTS: %d", pt_count);
    if (ui_lbl_count) lv_label_set_text(ui_lbl_count, buf);
}

static void refresh_sensor_display()
{
    char buf[80];
    if (ui_lbl_tof_val) {
        snprintf(buf, sizeof(buf), "%.3f m   (%lu mm)",
                 tof_dist_mm/1000.0f, (unsigned long)tof_dist_mm);
        lv_label_set_text(ui_lbl_tof_val, buf);
    }
    if (ui_lbl_imu_val) {
        snprintf(buf, sizeof(buf), "R: %.1f\xC2\xB0   P: %.1f\xC2\xB0   Y: %.1f\xC2\xB0",
                 imu_roll, imu_pitch, imu_yaw);
        lv_label_set_text(ui_lbl_imu_val, buf);
    }
    if (ui_lbl_sens_stat) {
        snprintf(buf, sizeof(buf), "TOF: %s   IMU: %s",
                 tof_ok?"OK":"FAIL", imu_ok?"OK":"FAIL");
        lv_label_set_text(ui_lbl_sens_stat, buf);
    }
    if (ui_lbl_gps_disp) {
        snprintf(buf, sizeof(buf), "Station: %.6f, %.6f  Alt: %.1fm",
                 stn_lat, stn_lon, stn_alt);
        lv_label_set_text(ui_lbl_gps_disp, buf);
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

// ── SD save / load ───────────────────────────────────────────────────────────
static void save_csv()
{
    if (!sd_ready) return;
    if (SD.exists(active_csv)) SD.remove(active_csv);
    File f = SD.open(active_csv, FILE_WRITE);
    if (!f) return;
    f.printf("#GPS,%.6f,%.6f,%.2f\n", stn_lat, stn_lon, stn_alt);
    f.println("ID,TYPE,DIST,ROLL,PITCH,YAW");
    for (int i = 0; i < pt_count; i++) {
        f.printf("%u,%c,%.4f,%.2f,%.2f,%.2f\n",
                 pts[i].id, pts[i].type==PT_NAV?'N':'S',
                 pts[i].dist, pts[i].roll, pts[i].pitch, pts[i].yaw);
    }
    f.close();
    Serial.printf("[SD] Saved %d pts to %s\n", pt_count, active_csv);
}

static void load_csv()
{
    if (!sd_ready) return;
    File f = SD.open(active_csv, FILE_READ);
    if (!f) return;
    pt_count = 0; next_id = 1;
    while (f.available() && pt_count < MAX_PTS) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;
        if (line.startsWith("#GPS,")) {
            char buf[80]; line.toCharArray(buf, sizeof(buf));
            char *tok = strtok(buf+5, ",");
            if (tok) { stn_lat = atof(tok); tok = strtok(NULL, ","); }
            if (tok) { stn_lon = atof(tok); tok = strtok(NULL, ","); }
            if (tok) { stn_alt = atof(tok); }
            continue;
        }
        if (line.startsWith("ID,")) continue; // header
        char buf[100]; line.toCharArray(buf, sizeof(buf));
        MeasPoint &p = pts[pt_count];
        char *tok = strtok(buf, ",");
        if (tok) { p.id = (uint16_t)atoi(tok); tok = strtok(NULL, ","); }
        if (tok) { p.type = (tok[0]=='N'||tok[0]=='n') ? PT_NAV : PT_SAMPLE;
                   tok = strtok(NULL, ","); }
        if (tok) { p.dist  = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { p.roll  = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { p.pitch = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { p.yaw   = atof(tok); }
        if (p.id >= next_id) next_id = p.id + 1;
        pt_count++;
    }
    f.close();
    sel_row = -1;
    Serial.printf("[SD] Loaded %d pts from %s\n", pt_count, active_csv);
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

// ── GPS popup helpers ────────────────────────────────────────────────────────
static void update_gps_labels()
{
    char buf[32];
    snprintf(buf, sizeof(buf), "Lat: %.6f", stn_lat);
    if (ui_lbl_gps_lat) lv_label_set_text(ui_lbl_gps_lat, buf);
    snprintf(buf, sizeof(buf), "Lon: %.6f", stn_lon);
    if (ui_lbl_gps_lon) lv_label_set_text(ui_lbl_gps_lon, buf);
    snprintf(buf, sizeof(buf), "Alt: %.1f m", stn_alt);
    if (ui_lbl_gps_alt) lv_label_set_text(ui_lbl_gps_alt, buf);
}

static void gps_close_cb(lv_event_t *e)
{
    if (ui_gps_overlay) {
        lv_obj_del(ui_gps_overlay);
        ui_gps_overlay = nullptr;
        ui_lbl_gps_lat = ui_lbl_gps_lon = ui_lbl_gps_alt = nullptr;
    }
}

// Increment helpers for GPS nudge buttons
static void gps_lat_inc(lv_event_t*e) { stn_lat += 0.0001f; update_gps_labels(); }
static void gps_lat_dec(lv_event_t*e) { stn_lat -= 0.0001f; update_gps_labels(); }
static void gps_lon_inc(lv_event_t*e) { stn_lon += 0.0001f; update_gps_labels(); }
static void gps_lon_dec(lv_event_t*e) { stn_lon -= 0.0001f; update_gps_labels(); }
static void gps_alt_inc(lv_event_t*e) { stn_alt += 1.0f;    update_gps_labels(); }
static void gps_alt_dec(lv_event_t*e) { stn_alt -= 1.0f;    update_gps_labels(); }

static void show_gps_popup()
{
    if (ui_gps_overlay) return;

    lv_obj_t *scr = lv_scr_act();
    ui_gps_overlay = lv_obj_create(scr);
    lv_obj_set_size(ui_gps_overlay, 360, 230);
    lv_obj_center(ui_gps_overlay);
    lv_obj_set_style_bg_color(ui_gps_overlay, lv_color_hex(C_WHITE), 0);
    lv_obj_set_style_bg_opa(ui_gps_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(ui_gps_overlay, 12, 0);
    lv_obj_set_style_shadow_width(ui_gps_overlay, 20, 0);
    lv_obj_set_style_shadow_opa(ui_gps_overlay, LV_OPA_40, 0);
    lv_obj_set_style_border_color(ui_gps_overlay, lv_color_hex(C_HDR_LINE), 0);
    lv_obj_set_style_border_width(ui_gps_overlay, 2, 0);
    lv_obj_set_style_pad_all(ui_gps_overlay, 12, 0);
    lv_obj_clear_flag(ui_gps_overlay, LV_OBJ_FLAG_SCROLLABLE);

    // Title
    lv_obj_t *title = lv_label_create(ui_gps_overlay);
    lv_label_set_text(title, LV_SYMBOL_GPS " Station GPS");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(C_HDR_LINE), 0);

    // Row builder: label + [-] + [+]
    auto make_gps_row = [&](int y, lv_obj_t **lbl_out,
                            lv_event_cb_t dec_cb, lv_event_cb_t inc_cb) {
        lv_obj_t *lbl = lv_label_create(ui_gps_overlay);
        lv_obj_set_width(lbl, 200);
        lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 0, y);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        *lbl_out = lbl;

        lv_obj_t *bm = lv_btn_create(ui_gps_overlay);
        lv_obj_set_size(bm, 50, 30);
        lv_obj_align(bm, LV_ALIGN_TOP_RIGHT, -60, y-2);
        lv_obj_set_style_bg_color(bm, lv_color_hex(C_BTN_DEL), 0);
        lv_obj_set_style_radius(bm, 6, 0);
        lv_obj_add_event_cb(bm, dec_cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_t *lm = lv_label_create(bm);
        lv_label_set_text(lm, LV_SYMBOL_MINUS);
        lv_obj_center(lm);
        lv_obj_set_style_text_color(lm, lv_color_hex(C_WHITE), 0);

        lv_obj_t *bp = lv_btn_create(ui_gps_overlay);
        lv_obj_set_size(bp, 50, 30);
        lv_obj_align(bp, LV_ALIGN_TOP_RIGHT, 0, y-2);
        lv_obj_set_style_bg_color(bp, lv_color_hex(C_SD_ON), 0);
        lv_obj_set_style_radius(bp, 6, 0);
        lv_obj_add_event_cb(bp, inc_cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_t *lp = lv_label_create(bp);
        lv_label_set_text(lp, LV_SYMBOL_PLUS);
        lv_obj_center(lp);
        lv_obj_set_style_text_color(lp, lv_color_hex(C_WHITE), 0);
    };

    make_gps_row(30,  &ui_lbl_gps_lat, gps_lat_dec, gps_lat_inc);
    make_gps_row(68,  &ui_lbl_gps_lon, gps_lon_dec, gps_lon_inc);
    make_gps_row(106, &ui_lbl_gps_alt, gps_alt_dec, gps_alt_inc);
    update_gps_labels();

    // Hint
    lv_obj_t *hint = lv_label_create(ui_gps_overlay);
    lv_label_set_text(hint, "BT: GPS,lat,lon,alt");
    lv_obj_align(hint, LV_ALIGN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(C_GREY), 0);

    // Close button
    lv_obj_t *btn = lv_btn_create(ui_gps_overlay);
    lv_obj_set_size(btn, 80, 34);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_RIGHT, 0, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(C_HDR_LINE), 0);
    lv_obj_set_style_radius(btn, 8, 0);
    lv_obj_add_event_cb(btn, gps_close_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *bl = lv_label_create(btn);
    lv_label_set_text(bl, LV_SYMBOL_OK " OK");
    lv_obj_center(bl);
    lv_obj_set_style_text_color(bl, lv_color_hex(C_WHITE), 0);
}

// ── Button callbacks ─────────────────────────────────────────────────────────
static void set_fstatus(const char *msg)
{
    if (ui_lbl_fstatus) lv_label_set_text(ui_lbl_fstatus, msg);
}

static void add_point(PtType type)
{
    if (pt_count >= MAX_PTS) { set_fstatus(LV_SYMBOL_WARNING " Full!"); return; }
    MeasPoint &p = pts[pt_count];
    p.id    = next_id++;
    p.type  = type;
    p.dist  = tof_dist_mm / 1000.0f;
    p.roll  = imu_roll;
    p.pitch = imu_pitch;
    p.yaw   = imu_yaw;
    pt_count++;
    sel_row = -1;
    refresh_table();
    char buf[40];
    snprintf(buf, sizeof(buf), LV_SYMBOL_OK " %s #%u  d=%.3fm",
             type==PT_NAV?"NAV":"SMP", p.id, p.dist);
    set_fstatus(buf);
}

static void btn_add_samp_cb(lv_event_t*e) { add_point(PT_SAMPLE); }
static void btn_add_nav_cb(lv_event_t*e)  { add_point(PT_NAV); }

static void btn_del_cb(lv_event_t *e)
{
    if (sel_row < 0 || sel_row >= pt_count) {
        set_fstatus(LV_SYMBOL_WARNING " Select a row!"); return;
    }
    uint16_t did = pts[sel_row].id;
    for (int i = sel_row; i < pt_count-1; i++) pts[i] = pts[i+1];
    pt_count--;
    sel_row = -1;
    refresh_table();
    char buf[32]; snprintf(buf,sizeof(buf), LV_SYMBOL_TRASH " Del #%u", did);
    set_fstatus(buf);
}

static void btn_gps_cb(lv_event_t*e)   { show_gps_popup(); }

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
    if (f) { f.printf("#GPS,0,0,0\n"); f.println("ID,TYPE,DIST,ROLL,PITCH,YAW"); f.close(); }
    strlcpy(active_csv,path,sizeof(active_csv));
    pt_count=0; sel_row=-1; next_id=1; stn_lat=stn_lon=stn_alt=0;
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
               if(f){f.printf("#GPS,0,0,0\n");f.println("ID,TYPE,DIST,ROLL,PITCH,YAW");f.close();}
               refresh_file_list(); }
        pt_count=0;sel_row=-1;next_id=1; load_csv(); refresh_table(); update_active_lbl();
    }
    char buf[48]; snprintf(buf,sizeof(buf), LV_SYMBOL_TRASH " Deleted %s", path+1);
    set_fstatus(buf);
}

static void tabview_changed_cb(lv_event_t *e)
{
    uint16_t tab = lv_tabview_get_tab_act(lv_event_get_target(e));
    if (tab==1) refresh_sensor_display();
    if (tab==2) { refresh_file_list(); update_active_lbl(); }
}

// ── Status / BT / periodic ───────────────────────────────────────────────────
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
    String cmd = raw; cmd.trim();
    if (cmd.startsWith("GPS,")) {
        char buf[80]; cmd.toCharArray(buf,sizeof(buf));
        char *tok = strtok(buf+4, ",");
        if (tok) { stn_lat=atof(tok); tok=strtok(NULL,","); }
        if (tok) { stn_lon=atof(tok); tok=strtok(NULL,","); }
        if (tok) { stn_alt=atof(tok); }
        SerialBT.printf("OK,GPS,%.6f,%.6f,%.2f\n", stn_lat, stn_lon, stn_alt);
        if (ui_gps_overlay) update_gps_labels();
        refresh_sensor_display();
        return;
    }
    if (cmd.equalsIgnoreCase("MEAS")) { add_point(PT_SAMPLE); SerialBT.printf("OK,%d\n",pt_count); return; }
    if (cmd.equalsIgnoreCase("CLEAR")) { pt_count=0;sel_row=-1;next_id=1; refresh_table(); SerialBT.println("OK"); return; }
    if (cmd.equalsIgnoreCase("LIST")) {
        SerialBT.printf("#GPS,%.6f,%.6f,%.2f\n", stn_lat, stn_lon, stn_alt);
        for (int i=0;i<pt_count;i++)
            SerialBT.printf("%u,%c,%.4f,%.2f,%.2f,%.2f\n",
                pts[i].id, pts[i].type==PT_NAV?'N':'S',
                pts[i].dist, pts[i].roll, pts[i].pitch, pts[i].yaw);
        SerialBT.println("END"); return;
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

static void sensor_timer_cb(lv_timer_t*t) { refresh_sensor_display(); }

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
    lv_label_set_text(lt, "  BRICS-5  MM1");
    lv_obj_align(lt, LV_ALIGN_LEFT_MID, 6, 0);
    lv_obj_set_style_text_color(lt, lv_color_hex(C_HDR_LINE), 0);
    lv_obj_set_style_text_font(lt, &lv_font_montserrat_16, 0);

    ui_lbl_count = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_count, "PTS: 0");
    lv_obj_align(ui_lbl_count, LV_ALIGN_LEFT_MID, 175, 0);
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
    lv_table_set_col_cnt(ui_tbl_pts, 6);
    lv_table_set_col_width(ui_tbl_pts, 0, 50);  // TYPE
    lv_table_set_col_width(ui_tbl_pts, 1, 40);  // #
    lv_table_set_col_width(ui_tbl_pts, 2, 90);  // DIST
    lv_table_set_col_width(ui_tbl_pts, 3, 80);  // ROLL
    lv_table_set_col_width(ui_tbl_pts, 4, 80);  // PITCH
    lv_table_set_col_width(ui_tbl_pts, 5, 80);  // YAW
    lv_obj_set_style_bg_color(ui_tbl_pts, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_top(ui_tbl_pts, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(ui_tbl_pts, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(ui_tbl_pts, 6, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(ui_tbl_pts, 6, LV_PART_ITEMS);
    lv_obj_add_event_cb(ui_tbl_pts, pts_draw_cb, LV_EVENT_DRAW_PART_BEGIN, nullptr);
    lv_obj_add_event_cb(ui_tbl_pts, pts_click_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *bar_p = make_bar(tp, TABLE_H);
    make_btn(bar_p, C_BTN_SAMP, LV_SYMBOL_PLUS " SMP",  btn_add_samp_cb);
    make_btn(bar_p, C_BTN_NAV,  LV_SYMBOL_PLUS " NAV",  btn_add_nav_cb);
    make_btn(bar_p, C_BTN_DEL,  LV_SYMBOL_TRASH " DEL",  btn_del_cb);
    make_btn(bar_p, C_BTN_GPS,  LV_SYMBOL_GPS " GPS",    btn_gps_cb);
    make_btn(bar_p, C_BTN_SAVE, LV_SYMBOL_SAVE " SAVE",  btn_save_cb);
    make_btn(bar_p, C_BTN_DEL,  LV_SYMBOL_TRASH " CLR",  btn_clear_cb);

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

    sec_lbl(ts, 0,  "TOF LIDAR");
    ui_lbl_tof_val = val_lbl(ts, 22);
    sec_lbl(ts, 58, "IMU ORIENTATION");
    ui_lbl_imu_val = val_lbl(ts, 80);
    sec_lbl(ts, 116,"STATUS");
    ui_lbl_sens_stat = val_lbl(ts, 138);
    sec_lbl(ts, 174,"STATION GPS");
    ui_lbl_gps_disp = val_lbl(ts, 196);

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

    // ── Render ───────────────────────────────────────────────────────────
    refresh_table();
    update_active_lbl();
    update_status();
}

// ── Init helpers ─────────────────────────────────────────────────────────────
static void sd_init()
{
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    sd_ready = SD.begin(SD_CS);
    Serial.printf("[SD] %s\n", sd_ready ? "OK" : "Not found");
}

static void sensor_init()
{
    Wire.begin(I2C_SDA, I2C_SCL, 100000);
    delay(200);
    pinMode(IMU_RST, OUTPUT);
    digitalWrite(IMU_RST, LOW); delay(20);
    digitalWrite(IMU_RST, HIGH); delay(300);
    pinMode(IMU_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IMU_INT), imuISR, FALLING);

    imu_ok = bno08x.begin_I2C(IMU_ADDR, &Wire);
    if (imu_ok) {
        bno08x.enableReport(SH2_ROTATION_VECTOR, 20000);
        delay(50);
        bno08x.enableReport(SH2_ACCELEROMETER, 50000);
        delay(50);
    }
    Serial.printf("[IMU] %s\n", imu_ok ? "OK" : "FAIL");

    uint32_t d;
    tof_ok = read_tof(d);
    if (tof_ok) tof_dist_mm = d;
    Serial.printf("[TOF] %s\n", tof_ok ? "OK" : "FAIL");
}

// ── setup / loop ─────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    Serial.println("\n[BRICS-5 MM1] Boot");

    tft.init();
    tft.setRotation(1);
    tft.setTouch(const_cast<uint16_t*>(TOUCH_CAL));
    tft.fillScreen(TFT_BLACK);

    sd_init();
    sensor_init();
    SerialBT.begin("BRICS5-MM1");

    lv_init();
    lvgl_buf = (lv_color_t*)malloc(SCREEN_W * 10 * sizeof(lv_color_t));
    if (!lvgl_buf) { Serial.println("FATAL"); while(1) delay(1000); }
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
            if(f){f.printf("#GPS,0,0,0\n");f.println("ID,TYPE,DIST,ROLL,PITCH,YAW");f.close();}
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
    Serial.println("[BRICS-5 MM1] Ready");
}

void loop()
{
    uint32_t d;
    if (read_tof(d)) { tof_dist_mm=d; tof_ok=true; }
    poll_imu();
    lv_timer_handler();
    delay(5);
}
