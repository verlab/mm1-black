/**
 * BRICS-5 MM1  –  Smart Tape / Field Measurement Tool
 * ESP32 CYD  |  ST7796 480×320  |  LVGL 8.3
 *
 * Tree-based point structure:
 *   Root stations have GPS coordinates (entered via BT).
 *   From each station, measurements (dist + IMU angles) form child branches.
 *   Navigate INTO any child to make it the new station and add sub-measurements.
 *
 * Tabs:
 *   TREE   – breadcrumb + children table + action bar [MEAS/INTO/BACK/DEL/SAVE]
 *   SENSOR – live TOF + IMU readings
 *   FILES  – CSV file browser on SD card
 *
 * Hardware:
 *   TFT + Touch  : HSPI (TFT_eSPI)  pins 12/13/14, touch CS=33
 *   SD card       : VSPI             pins 18/19/23, CS=5
 *   BNO08x IMU    : I2C (Wire)       SDA=32, SCL=25, RST=17, INT=16
 *   TOFSense LiDAR: I2C (Wire)       addr 0x08
 *   Bluetooth Serial
 *
 * BT commands (newline-terminated):
 *   GPS,<lat>,<lon>,<alt>   – set GPS for current root station
 *   MEAS                    – take measurement (same as button)
 *   CLEAR                   – delete all nodes
 *   LIST                    – dump tree CSV
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

// ─────────────────────────────────────────────────────────────────────────────
//  Pin definitions
// ─────────────────────────────────────────────────────────────────────────────
#define SCREEN_W  480
#define SCREEN_H  320

#define SD_CS    5
#define SD_SCK  18
#define SD_MISO 19
#define SD_MOSI 23

#define I2C_SDA  32
#define I2C_SCL  25
#define IMU_RST  17
#define IMU_INT  16
#define IMU_ADDR 0x4B

#define TOF_ADDR 0x08
#define TOF_REG  0x24

static const uint16_t TOUCH_CAL[5] = { 254, 3643, 176, 3693, 7 };

// ─────────────────────────────────────────────────────────────────────────────
//  Colour palette
// ─────────────────────────────────────────────────────────────────────────────
#define C_BG          0xF0F4F8u
#define C_TOPBAR      0xFFFFFFu
#define C_TBL_HDR     0x1E3A5Fu
#define C_ROW_ODD     0xFFFFFFu
#define C_ROW_EVEN    0xEEF2F7u
#define C_ROW_SEL     0xBBDEFBu
#define C_TEXT        0x1A2027u
#define C_HDR_LINE    0x1565C0u
#define C_BORDER      0xCFD8DCu
#define C_WHITE       0xFFFFFFu
#define C_GREY        0x546E7Au
#define C_BT_ON       0x1565C0u
#define C_BT_OFF      0x90A4AEu
#define C_SD_ON       0x2E7D32u
#define C_SD_OFF      0xD32F2Fu
#define C_BTN_MEAS    0x00897Bu
#define C_BTN_INTO    0x1565C0u
#define C_BTN_BACK    0x546E7Au
#define C_BTN_DEL     0xC62828u
#define C_BTN_SAVE    0x1565C0u
#define C_BTN_NEW     0x00897Bu
#define C_BTN_USE     0x4A148Cu
#define C_FILE_ACTIVE 0xC8E6C9u
#define C_REF_S       0x1565C0u

// ─────────────────────────────────────────────────────────────────────────────
//  Data structure  –  tree of measurement nodes
// ─────────────────────────────────────────────────────────────────────────────
#define MAX_NODES 30
#define MAX_FILES 16

struct MeasNode {
    int16_t  parent;        // array index of parent (-1 = root station)
    uint16_t id;            // unique auto-increment ID
    uint8_t  depth;
    float    gps_lat, gps_lon, gps_alt;   // GPS (meaningful only for roots)
    float    dist;          // distance from parent (m)
    float    roll, pitch, yaw;            // orientation (°)
    float    x, y, z;       // computed absolute position (m)
};

// ─────────────────────────────────────────────────────────────────────────────
//  Globals
// ─────────────────────────────────────────────────────────────────────────────
static TFT_eSPI          tft;
static BluetoothSerial   SerialBT;
static Adafruit_BNO08x   bno08x(-1);
static sh2_SensorValue_t sensorValue;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t        *lvgl_buf = nullptr;

// Tree
static MeasNode  nodes[MAX_NODES];
static int       node_count  = 0;
static int       cur_node    = -1;   // which station we're inside (-1 = top)
static int       sel_child   = -1;   // highlighted child index in table
static uint16_t  next_id     = 1;

// Sensors
static uint32_t  tof_dist_mm = 0;
static float     imu_roll = 0, imu_pitch = 0, imu_yaw = 0;
static float     acc_x = 0, acc_y = 0, acc_z = 0;
static bool      imu_ok  = false;
static bool      tof_ok  = false;
static volatile bool imu_irq = false;

// SD / files
static bool      sd_ready    = false;
static bool      bt_conn     = false;
static char      active_csv[32] = "/brics5_mm1.csv";
static char      file_names[MAX_FILES][32];
static int       file_count  = 0;
static int       file_sel    = -1;

// UI handles
static lv_obj_t *ui_tbl_tree      = nullptr;
static lv_obj_t *ui_lbl_bread     = nullptr;
static lv_obj_t *ui_lbl_meas_btn  = nullptr;
static lv_obj_t *ui_lbl_bt        = nullptr;
static lv_obj_t *ui_lbl_sd        = nullptr;
static lv_obj_t *ui_lbl_time      = nullptr;
static lv_obj_t *ui_lbl_count     = nullptr;
static lv_obj_t *ui_lbl_tof_val   = nullptr;
static lv_obj_t *ui_lbl_imu_val   = nullptr;
static lv_obj_t *ui_lbl_sens_stat = nullptr;
static lv_obj_t *ui_lbl_gps_val   = nullptr;
static lv_obj_t *ui_tbl_files     = nullptr;
static lv_obj_t *ui_lbl_active    = nullptr;
static lv_obj_t *ui_lbl_fstatus   = nullptr;

// Forward declarations
static void sd_init();
static void sensor_init();
static void refresh_tree();
static void refresh_sensor_display();
static void set_fstatus(const char *msg);

// ─────────────────────────────────────────────────────────────────────────────
//  ISR
// ─────────────────────────────────────────────────────────────────────────────
void IRAM_ATTR imuISR() { imu_irq = true; }

// ─────────────────────────────────────────────────────────────────────────────
//  LVGL display flush + touch read
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

// ─────────────────────────────────────────────────────────────────────────────
//  Sensor functions
// ─────────────────────────────────────────────────────────────────────────────
static bool read_tof(uint32_t &dist_mm)
{
    Wire.beginTransmission(TOF_ADDR);
    Wire.write(TOF_REG);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)TOF_ADDR, 4) != 4) return false;
    uint8_t b0 = Wire.read(), b1 = Wire.read(), b2 = Wire.read(), b3 = Wire.read();
    dist_mm = b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
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
        switch (sensorValue.sensorId) {
        case SH2_ROTATION_VECTOR: {
            float qw = sensorValue.un.rotationVector.real;
            float qx = sensorValue.un.rotationVector.i;
            float qy = sensorValue.un.rotationVector.j;
            float qz = sensorValue.un.rotationVector.k;
            imu_yaw   = atan2f(2.0f*(qw*qz + qx*qy),
                                1.0f - 2.0f*(qy*qy + qz*qz)) * 180.0f / (float)M_PI;
            imu_pitch = asinf(fmaxf(-1.0f, fminf(1.0f,
                                2.0f*(qw*qy - qz*qx)))) * 180.0f / (float)M_PI;
            imu_roll  = atan2f(2.0f*(qw*qx + qy*qz),
                                1.0f - 2.0f*(qx*qx + qy*qy)) * 180.0f / (float)M_PI;
            break;
        }
        case SH2_ACCELEROMETER:
            acc_x = sensorValue.un.accelerometer.x;
            acc_y = sensorValue.un.accelerometer.y;
            acc_z = sensorValue.un.accelerometer.z;
            break;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tree helpers
// ─────────────────────────────────────────────────────────────────────────────
static int count_children(int parent_idx)
{
    int c = 0;
    for (int i = 0; i < node_count; i++)
        if (nodes[i].parent == parent_idx) c++;
    return c;
}

static int get_nth_child(int parent_idx, int n)
{
    int c = 0;
    for (int i = 0; i < node_count; i++) {
        if (nodes[i].parent == parent_idx) {
            if (c == n) return i;
            c++;
        }
    }
    return -1;
}

static int find_by_id(uint16_t id)
{
    for (int i = 0; i < node_count; i++)
        if (nodes[i].id == id) return i;
    return -1;
}

static int find_root(int idx)
{
    while (idx >= 0 && nodes[idx].parent >= 0)
        idx = nodes[idx].parent;
    return idx;
}

static void compute_position(int idx)
{
    if (idx < 0 || idx >= node_count) return;
    if (nodes[idx].parent < 0) {
        nodes[idx].x = nodes[idx].y = nodes[idx].z = 0;
        return;
    }
    int p = nodes[idx].parent;
    compute_position(p);
    float d  = nodes[idx].dist;
    float yr = nodes[idx].yaw   * (float)M_PI / 180.0f;
    float pr = nodes[idx].pitch * (float)M_PI / 180.0f;
    nodes[idx].x = nodes[p].x + d * cosf(pr) * sinf(yr);
    nodes[idx].y = nodes[p].y + d * cosf(pr) * cosf(yr);
    nodes[idx].z = nodes[p].z + d * sinf(pr);
}

static void build_breadcrumb(int idx, char *buf, int bufsize)
{
    if (idx < 0) { strlcpy(buf, "STATIONS", bufsize); return; }
    int chain[10];
    int depth = 0;
    int p = idx;
    while (p >= 0 && depth < 10) { chain[depth++] = p; p = nodes[p].parent; }
    buf[0] = '\0';
    for (int i = depth - 1; i >= 0; i--) {
        char tmp[16];
        if (nodes[chain[i]].parent < 0)
            snprintf(tmp, sizeof(tmp), "STN-%u", nodes[chain[i]].id);
        else
            snprintf(tmp, sizeof(tmp), "#%u", nodes[chain[i]].id);
        strlcat(buf, tmp, bufsize);
        if (i > 0) strlcat(buf, " > ", bufsize);
    }
}

static void delete_subtree(int idx)
{
    if (idx < 0 || idx >= node_count) return;
    bool mark[MAX_NODES] = {};
    mark[idx] = true;
    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 0; i < node_count; i++) {
            if (!mark[i] && nodes[i].parent >= 0 && mark[nodes[i].parent]) {
                mark[i] = true;
                changed = true;
            }
        }
    }
    int new_idx[MAX_NODES];
    int new_count = 0;
    for (int i = 0; i < node_count; i++) {
        if (!mark[i]) {
            new_idx[i] = new_count;
            if (new_count != i) nodes[new_count] = nodes[i];
            new_count++;
        } else {
            new_idx[i] = -1;
        }
    }
    for (int i = 0; i < new_count; i++) {
        if (nodes[i].parent >= 0)
            nodes[i].parent = new_idx[nodes[i].parent];
    }
    node_count = new_count;
}

static int add_node(int parent_idx)
{
    if (node_count >= MAX_NODES) return -1;
    MeasNode &n = nodes[node_count];
    memset(&n, 0, sizeof(MeasNode));
    n.parent = parent_idx;
    n.id     = next_id++;
    n.depth  = (parent_idx >= 0) ? (nodes[parent_idx].depth + 1) : 0;
    return node_count++;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tree table draw/click callbacks
// ─────────────────────────────────────────────────────────────────────────────
static void tree_draw_cb(lv_event_t *e)
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
        bool is_selected = ((int)row - 1) == sel_child;
        dsc->rect_dsc->bg_color = is_selected ? lv_color_hex(C_ROW_SEL)
                                  : (row % 2 == 0) ? lv_color_hex(C_ROW_EVEN)
                                  : lv_color_hex(C_ROW_ODD);
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;
        dsc->label_dsc->color = (col == 0) ? lv_color_hex(C_REF_S)
                                           : lv_color_hex(C_TEXT);
        dsc->label_dsc->align = (col == 0) ? LV_TEXT_ALIGN_CENTER
                                           : LV_TEXT_ALIGN_RIGHT;
        dsc->label_dsc->font  = &lv_font_montserrat_14;
    }
    dsc->rect_dsc->border_color = lv_color_hex(C_BORDER);
    dsc->rect_dsc->border_width = 1;
}

static void tree_click_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    uint16_t row, col;
    lv_table_get_selected_cell(obj, &row, &col);
    int nch = count_children(cur_node);
    if (row > 0 && (int)(row - 1) < nch) {
        int idx = (int)row - 1;
        sel_child = (sel_child == idx) ? -1 : idx;
        lv_obj_invalidate(obj);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Refresh tree table + breadcrumb
// ─────────────────────────────────────────────────────────────────────────────
static void refresh_tree()
{
    if (!ui_tbl_tree) return;
    lv_obj_t *t = ui_tbl_tree;
    int nch = count_children(cur_node);
    lv_table_set_row_cnt(t, (uint16_t)(nch + 1));

    char buf[64];
    if (cur_node < 0) {
        lv_table_set_cell_value(t, 0, 0, "STN");
        lv_table_set_cell_value(t, 0, 1, "LAT");
        lv_table_set_cell_value(t, 0, 2, "LON");
        lv_table_set_cell_value(t, 0, 3, "ALT (m)");
        lv_table_set_cell_value(t, 0, 4, "PTS");
        for (int i = 0; i < nch; i++) {
            int idx = get_nth_child(-1, i);
            if (idx < 0) continue;
            snprintf(buf, sizeof(buf), "S%u", nodes[idx].id);
            lv_table_set_cell_value(t, i + 1, 0, buf);
            snprintf(buf, sizeof(buf), "%.6f", nodes[idx].gps_lat);
            lv_table_set_cell_value(t, i + 1, 1, buf);
            snprintf(buf, sizeof(buf), "%.6f", nodes[idx].gps_lon);
            lv_table_set_cell_value(t, i + 1, 2, buf);
            snprintf(buf, sizeof(buf), "%.1f", nodes[idx].gps_alt);
            lv_table_set_cell_value(t, i + 1, 3, buf);
            snprintf(buf, sizeof(buf), "%d", count_children(idx));
            lv_table_set_cell_value(t, i + 1, 4, buf);
        }
    } else {
        lv_table_set_cell_value(t, 0, 0, "#");
        lv_table_set_cell_value(t, 0, 1, "DIST (m)");
        lv_table_set_cell_value(t, 0, 2, "YAW \xC2\xB0");
        lv_table_set_cell_value(t, 0, 3, "PITCH \xC2\xB0");
        lv_table_set_cell_value(t, 0, 4, "PTS");
        for (int i = 0; i < nch; i++) {
            int idx = get_nth_child(cur_node, i);
            if (idx < 0) continue;
            snprintf(buf, sizeof(buf), "#%u", nodes[idx].id);
            lv_table_set_cell_value(t, i + 1, 0, buf);
            snprintf(buf, sizeof(buf), "%.3f", nodes[idx].dist);
            lv_table_set_cell_value(t, i + 1, 1, buf);
            snprintf(buf, sizeof(buf), "%.1f", nodes[idx].yaw);
            lv_table_set_cell_value(t, i + 1, 2, buf);
            snprintf(buf, sizeof(buf), "%.1f", nodes[idx].pitch);
            lv_table_set_cell_value(t, i + 1, 3, buf);
            snprintf(buf, sizeof(buf), "%d", count_children(idx));
            lv_table_set_cell_value(t, i + 1, 4, buf);
        }
    }

    build_breadcrumb(cur_node, buf, sizeof(buf));
    if (ui_lbl_bread) lv_label_set_text(ui_lbl_bread, buf);

    if (ui_lbl_meas_btn) {
        lv_label_set_text(ui_lbl_meas_btn,
                          (cur_node < 0) ? LV_SYMBOL_PLUS " +STN"
                                         : LV_SYMBOL_PLUS " MEAS");
    }

    snprintf(buf, sizeof(buf), "N: %d", node_count);
    if (ui_lbl_count) lv_label_set_text(ui_lbl_count, buf);
}

static void refresh_sensor_display()
{
    char buf[80];
    if (ui_lbl_tof_val) {
        snprintf(buf, sizeof(buf), "%.3f m   (%lu mm)",
                 tof_dist_mm / 1000.0f, (unsigned long)tof_dist_mm);
        lv_label_set_text(ui_lbl_tof_val, buf);
    }
    if (ui_lbl_imu_val) {
        snprintf(buf, sizeof(buf),
                 "Roll: %.1f\xC2\xB0   Pitch: %.1f\xC2\xB0   Yaw: %.1f\xC2\xB0",
                 imu_roll, imu_pitch, imu_yaw);
        lv_label_set_text(ui_lbl_imu_val, buf);
    }
    if (ui_lbl_sens_stat) {
        snprintf(buf, sizeof(buf), "TOF: %s   IMU: %s",
                 tof_ok ? "OK" : "FAIL", imu_ok ? "OK" : "FAIL");
        lv_label_set_text(ui_lbl_sens_stat, buf);
    }
    if (ui_lbl_gps_val) {
        int root = (cur_node >= 0) ? find_root(cur_node) : -1;
        if (root >= 0)
            snprintf(buf, sizeof(buf), "Root GPS: %.6f, %.6f  Alt: %.1fm",
                     nodes[root].gps_lat, nodes[root].gps_lon, nodes[root].gps_alt);
        else
            strlcpy(buf, "GPS: no station selected", sizeof(buf));
        lv_label_set_text(ui_lbl_gps_val, buf);
    }
}

static void update_active_lbl()
{
    if (!ui_lbl_active) return;
    const char *name = active_csv;
    if (name[0] == '/') name++;
    char buf[64];
    snprintf(buf, sizeof(buf), "Active: %s   (%d nodes)", name, node_count);
    lv_label_set_text(ui_lbl_active, buf);
}

// ─────────────────────────────────────────────────────────────────────────────
//  SD card – save / load  (tree CSV format)
// ─────────────────────────────────────────────────────────────────────────────
static void save_tree()
{
    if (!sd_ready) { Serial.println("[SD] save: card not ready"); return; }
    if (SD.exists(active_csv)) SD.remove(active_csv);
    File f = SD.open(active_csv, FILE_WRITE);
    if (!f) { Serial.println("[SD] save: open failed"); return; }
    f.println("ID,PARENT_ID,LAT,LON,ALT,DIST,ROLL,PITCH,YAW,X,Y,Z");
    for (int i = 0; i < node_count; i++) {
        MeasNode &n = nodes[i];
        int pid = (n.parent >= 0) ? (int)nodes[n.parent].id : -1;
        f.printf("%u,%d,%.6f,%.6f,%.2f,%.4f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n",
                 n.id, pid,
                 n.gps_lat, n.gps_lon, n.gps_alt,
                 n.dist, n.roll, n.pitch, n.yaw,
                 n.x, n.y, n.z);
    }
    f.close();
    Serial.printf("[SD] Saved %d nodes to %s\n", node_count, active_csv);
}

static void load_tree()
{
    if (!sd_ready) return;
    File f = SD.open(active_csv, FILE_READ);
    if (!f) { Serial.printf("[SD] %s not found\n", active_csv); return; }
    f.readStringUntil('\n'); // skip header
    int parent_ids[MAX_NODES];
    node_count = 0;
    while (f.available() && node_count < MAX_NODES) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;
        MeasNode &n = nodes[node_count];
        memset(&n, 0, sizeof(MeasNode));
        char buf[200];
        line.toCharArray(buf, sizeof(buf));
        char *tok = strtok(buf, ",");
        if (tok) { n.id      = (uint16_t)atoi(tok); tok = strtok(NULL, ","); }
        if (tok) { parent_ids[node_count] = atoi(tok); tok = strtok(NULL, ","); }
        if (tok) { n.gps_lat = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { n.gps_lon = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { n.gps_alt = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { n.dist    = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { n.roll    = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { n.pitch   = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { n.yaw     = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { n.x       = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { n.y       = atof(tok); tok = strtok(NULL, ","); }
        if (tok) { n.z       = atof(tok); }
        node_count++;
    }
    f.close();
    // Resolve parent IDs → array indices
    next_id = 1;
    for (int i = 0; i < node_count; i++) {
        if (parent_ids[i] < 0) {
            nodes[i].parent = -1;
        } else {
            nodes[i].parent = -1;
            for (int j = 0; j < node_count; j++) {
                if ((int)nodes[j].id == parent_ids[i]) { nodes[i].parent = j; break; }
            }
        }
        if (nodes[i].id >= next_id) next_id = nodes[i].id + 1;
    }
    for (int i = 0; i < node_count; i++) {
        int d = 0, p = nodes[i].parent;
        while (p >= 0) { d++; p = nodes[p].parent; }
        nodes[i].depth = d;
    }
    cur_node  = -1;
    sel_child = -1;
    Serial.printf("[SD] Loaded %d nodes from %s\n", node_count, active_csv);
}

// ─────────────────────────────────────────────────────────────────────────────
//  File browser helpers
// ─────────────────────────────────────────────────────────────────────────────
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
            const char *base = (n[0] == '/') ? n + 1 : n;
            if (len >= 4) {
                const char *ext = n + len - 4;
                if (ext[0] == '.' &&
                    (ext[1] == 'c' || ext[1] == 'C') &&
                    (ext[2] == 's' || ext[2] == 'S') &&
                    (ext[3] == 'v' || ext[3] == 'V')) {
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
    lv_table_set_row_cnt(ui_tbl_files, (uint16_t)(file_count + 1));
    lv_table_set_cell_value(ui_tbl_files, 0, 0, "File");
    lv_table_set_cell_value(ui_tbl_files, 0, 1, "Size");
    for (int i = 0; i < file_count; i++) {
        const char *name = file_names[i];
        if (name[0] == '/') name++;
        lv_table_set_cell_value(ui_tbl_files, i + 1, 0, name);
        File f = SD.open(file_names[i], FILE_READ);
        if (f) {
            char sz[12];
            uint32_t bytes = f.size();
            if (bytes < 1024) snprintf(sz, sizeof(sz), "%luB", bytes);
            else              snprintf(sz, sizeof(sz), "%lukB", bytes / 1024);
            f.close();
            lv_table_set_cell_value(ui_tbl_files, i + 1, 1, sz);
        } else {
            lv_table_set_cell_value(ui_tbl_files, i + 1, 1, "?");
        }
    }
    lv_obj_invalidate(ui_tbl_files);
}

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
        bool is_active   = (idx < file_count) && (strcmp(file_names[idx], active_csv) == 0);
        bool is_selected = (idx == file_sel);
        if (is_selected)        dsc->rect_dsc->bg_color = lv_color_hex(C_ROW_SEL);
        else if (is_active)     dsc->rect_dsc->bg_color = lv_color_hex(C_FILE_ACTIVE);
        else                    dsc->rect_dsc->bg_color = (row % 2 == 0)
                                    ? lv_color_hex(C_ROW_EVEN) : lv_color_hex(C_ROW_ODD);
        dsc->rect_dsc->bg_opa = LV_OPA_COVER;
        dsc->label_dsc->color = (col == 0) ? lv_color_hex(C_REF_S) : lv_color_hex(C_TEXT);
        dsc->label_dsc->align = (col == 0) ? LV_TEXT_ALIGN_LEFT : LV_TEXT_ALIGN_RIGHT;
        dsc->label_dsc->font  = &lv_font_montserrat_14;
    }
    dsc->rect_dsc->border_color = lv_color_hex(C_BORDER);
    dsc->rect_dsc->border_width = 1;
}

static void file_list_click_cb(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target(e);
    uint16_t row, col;
    lv_table_get_selected_cell(obj, &row, &col);
    if (row > 0 && (int)(row - 1) < file_count) {
        int idx  = (int)row - 1;
        file_sel = (file_sel == idx) ? -1 : idx;
        lv_obj_invalidate(obj);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Button callbacks – TREE tab actions
// ─────────────────────────────────────────────────────────────────────────────
static void set_fstatus(const char *msg)
{
    if (ui_lbl_fstatus) lv_label_set_text(ui_lbl_fstatus, msg);
    Serial.printf("[STATUS] %s\n", msg);
}

static void btn_meas_cb(lv_event_t *e)
{
    if (node_count >= MAX_NODES) { set_fstatus(LV_SYMBOL_WARNING " Tree full!"); return; }

    if (cur_node < 0) {
        // Top level: create new root station
        int idx = add_node(-1);
        if (idx < 0) return;
        // GPS defaults to 0; user sets via BT: GPS,lat,lon,alt
        set_fstatus(LV_SYMBOL_OK " New station created (set GPS via BT)");
    } else {
        // Inside a station: take sensor measurement
        int idx = add_node(cur_node);
        if (idx < 0) return;
        nodes[idx].dist  = tof_dist_mm / 1000.0f;
        nodes[idx].roll  = imu_roll;
        nodes[idx].pitch = imu_pitch;
        nodes[idx].yaw   = imu_yaw;
        // Copy root GPS
        int root = find_root(cur_node);
        if (root >= 0) {
            nodes[idx].gps_lat = nodes[root].gps_lat;
            nodes[idx].gps_lon = nodes[root].gps_lon;
            nodes[idx].gps_alt = nodes[root].gps_alt;
        }
        compute_position(idx);

        char buf[60];
        snprintf(buf, sizeof(buf), LV_SYMBOL_OK " #%u  d=%.3fm  y=%.1f\xC2\xB0",
                 nodes[idx].id, nodes[idx].dist, nodes[idx].yaw);
        set_fstatus(buf);
    }
    sel_child = -1;
    refresh_tree();
}

static void btn_into_cb(lv_event_t *e)
{
    if (sel_child < 0) { set_fstatus(LV_SYMBOL_WARNING " Select a row first!"); return; }
    int child_idx = get_nth_child(cur_node, sel_child);
    if (child_idx < 0) return;
    cur_node  = child_idx;
    sel_child = -1;
    refresh_tree();
    refresh_sensor_display();

    char buf[40];
    build_breadcrumb(cur_node, buf, sizeof(buf));
    Serial.printf("[NAV] INTO %s\n", buf);
}

static void btn_back_cb(lv_event_t *e)
{
    if (cur_node < 0) return;
    cur_node  = nodes[cur_node].parent;
    sel_child = -1;
    refresh_tree();
    refresh_sensor_display();
}

static void btn_del_cb(lv_event_t *e)
{
    if (sel_child < 0) { set_fstatus(LV_SYMBOL_WARNING " Select a row first!"); return; }
    int child_idx = get_nth_child(cur_node, sel_child);
    if (child_idx < 0) return;

    uint16_t del_id = nodes[child_idx].id;
    uint16_t cur_id = (cur_node >= 0) ? nodes[cur_node].id : 0;

    delete_subtree(child_idx);

    // Recover cur_node index (may have shifted)
    if (cur_id > 0) cur_node = find_by_id(cur_id);
    else            cur_node = -1;
    sel_child = -1;
    refresh_tree();

    char buf[40];
    snprintf(buf, sizeof(buf), LV_SYMBOL_TRASH " Deleted #%u", del_id);
    set_fstatus(buf);
}

static void btn_save_cb(lv_event_t *e)
{
    save_tree();
    char buf[40];
    snprintf(buf, sizeof(buf), LV_SYMBOL_OK " Saved %d nodes", node_count);
    set_fstatus(buf);
}

// ─────────────────────────────────────────────────────────────────────────────
//  File CRUD callbacks – FILES tab
// ─────────────────────────────────────────────────────────────────────────────
static void btn_new_file_cb(lv_event_t *e)
{
    if (!sd_ready) { set_fstatus(LV_SYMBOL_WARNING " No SD card!"); return; }
    char path[32];
    int n = 1;
    do { snprintf(path, sizeof(path), "/brics%02d.csv", n++); }
    while (SD.exists(path) && n < 100);
    if (n >= 100) { set_fstatus(LV_SYMBOL_WARNING " Too many files!"); return; }

    File f = SD.open(path, FILE_WRITE);
    if (!f) { set_fstatus(LV_SYMBOL_WARNING " Create failed!"); return; }
    f.println("ID,PARENT_ID,LAT,LON,ALT,DIST,ROLL,PITCH,YAW,X,Y,Z");
    f.close();

    strlcpy(active_csv, path, sizeof(active_csv));
    node_count = 0; cur_node = -1; sel_child = -1; next_id = 1;
    refresh_tree();
    refresh_file_list();
    update_active_lbl();

    char buf[48];
    snprintf(buf, sizeof(buf), LV_SYMBOL_OK " Created %s", path + 1);
    set_fstatus(buf);
}

static void btn_use_file_cb(lv_event_t *e)
{
    if (file_sel < 0 || file_sel >= file_count) {
        set_fstatus(LV_SYMBOL_WARNING " Select a file first!"); return;
    }
    strlcpy(active_csv, file_names[file_sel], sizeof(active_csv));
    node_count = 0; cur_node = -1; sel_child = -1; next_id = 1;
    load_tree();
    refresh_tree();
    lv_obj_invalidate(ui_tbl_files);
    update_active_lbl();

    char buf[48];
    snprintf(buf, sizeof(buf), LV_SYMBOL_OK " Using %s (%d nodes)", active_csv + 1, node_count);
    set_fstatus(buf);
}

static void btn_del_file_cb(lv_event_t *e)
{
    if (file_sel < 0 || file_sel >= file_count) {
        set_fstatus(LV_SYMBOL_WARNING " Select a file first!"); return;
    }
    char path[32];
    strlcpy(path, file_names[file_sel], sizeof(path));
    bool was_active = (strcmp(path, active_csv) == 0);
    SD.remove(path);
    file_sel = -1;
    refresh_file_list();

    if (was_active) {
        if (file_count > 0) strlcpy(active_csv, file_names[0], sizeof(active_csv));
        else {
            strlcpy(active_csv, "/brics5_mm1.csv", sizeof(active_csv));
            File f = SD.open(active_csv, FILE_WRITE);
            if (f) { f.println("ID,PARENT_ID,LAT,LON,ALT,DIST,ROLL,PITCH,YAW,X,Y,Z"); f.close(); }
            refresh_file_list();
        }
        node_count = 0; cur_node = -1; sel_child = -1; next_id = 1;
        load_tree();
        refresh_tree();
        update_active_lbl();
    }
    char buf[48];
    snprintf(buf, sizeof(buf), LV_SYMBOL_TRASH " Deleted %s", path + 1);
    set_fstatus(buf);
}

static void tabview_changed_cb(lv_event_t *e)
{
    lv_obj_t *tv  = lv_event_get_target(e);
    uint16_t  tab = lv_tabview_get_tab_act(tv);
    if (tab == 1) refresh_sensor_display();
    if (tab == 2) { refresh_file_list(); update_active_lbl(); }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Status bar + BT + periodic callbacks
// ─────────────────────────────────────────────────────────────────────────────
static void update_status()
{
    bt_conn = SerialBT.connected();
    lv_label_set_text(ui_lbl_bt, bt_conn ? LV_SYMBOL_BLUETOOTH " BT" : LV_SYMBOL_BLUETOOTH);
    lv_obj_set_style_text_color(ui_lbl_bt,
        lv_color_hex(bt_conn ? C_BT_ON : C_BT_OFF), 0);
    lv_label_set_text(ui_lbl_sd, sd_ready ? LV_SYMBOL_SD_CARD " SD" : LV_SYMBOL_SD_CARD);
    lv_obj_set_style_text_color(ui_lbl_sd,
        lv_color_hex(sd_ready ? C_SD_ON : C_SD_OFF), 0);
    uint32_t s = millis() / 1000;
    char tbuf[12];
    snprintf(tbuf, sizeof(tbuf), "%02lu:%02lu:%02lu", (s/3600)%24, (s/60)%60, s%60);
    lv_label_set_text(ui_lbl_time, tbuf);
}

static void handle_bt_cmd(const String &raw)
{
    String cmd = raw;
    cmd.trim();

    if (cmd.startsWith("GPS,") && cur_node >= 0) {
        int root = find_root(cur_node);
        if (root >= 0) {
            char buf[128];
            cmd.toCharArray(buf, sizeof(buf));
            char *tok = strtok(buf + 4, ",");
            if (tok) { nodes[root].gps_lat = atof(tok); tok = strtok(NULL, ","); }
            if (tok) { nodes[root].gps_lon = atof(tok); tok = strtok(NULL, ","); }
            if (tok) { nodes[root].gps_alt = atof(tok); }
            SerialBT.printf("OK,GPS,%.6f,%.6f,%.2f\n",
                            nodes[root].gps_lat, nodes[root].gps_lon, nodes[root].gps_alt);
            refresh_tree();
            refresh_sensor_display();
        }
        return;
    }
    if (cmd.equalsIgnoreCase("MEAS")) {
        // Simulate pressing MEAS button
        btn_meas_cb(nullptr);
        SerialBT.printf("OK,MEAS,%d\n", node_count);
        return;
    }
    if (cmd.equalsIgnoreCase("CLEAR")) {
        node_count = 0; cur_node = -1; sel_child = -1; next_id = 1;
        refresh_tree();
        SerialBT.println("OK,CLEARED");
        return;
    }
    if (cmd.equalsIgnoreCase("LIST")) {
        SerialBT.println("ID,PARENT_ID,LAT,LON,ALT,DIST,ROLL,PITCH,YAW,X,Y,Z");
        for (int i = 0; i < node_count; i++) {
            MeasNode &n = nodes[i];
            int pid = (n.parent >= 0) ? (int)nodes[n.parent].id : -1;
            SerialBT.printf("%u,%d,%.6f,%.6f,%.2f,%.4f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n",
                            n.id, pid, n.gps_lat, n.gps_lon, n.gps_alt,
                            n.dist, n.roll, n.pitch, n.yaw, n.x, n.y, n.z);
        }
        SerialBT.println("END");
        return;
    }
}

static void periodic_cb(lv_timer_t *t)
{
    update_status();
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

static void sensor_timer_cb(lv_timer_t *t)
{
    refresh_sensor_display();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Build UI
// ─────────────────────────────────────────────────────────────────────────────
static void build_ui()
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(C_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // ── Header bar (36px) ────────────────────────────────────────────────
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
    lv_label_set_text(ui_lbl_count, "N: 0");
    lv_obj_align(ui_lbl_count, LV_ALIGN_LEFT_MID, 185, 0);
    lv_obj_set_style_text_color(ui_lbl_count, lv_color_hex(C_GREY), 0);
    lv_obj_set_style_text_font(ui_lbl_count, &lv_font_montserrat_14, 0);

    ui_lbl_sd = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_sd, LV_SYMBOL_SD_CARD);
    lv_obj_align(ui_lbl_sd, LV_ALIGN_RIGHT_MID, -155, 0);
    lv_obj_set_style_text_color(ui_lbl_sd, lv_color_hex(C_SD_OFF), 0);

    ui_lbl_bt = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_bt, LV_SYMBOL_BLUETOOTH);
    lv_obj_align(ui_lbl_bt, LV_ALIGN_RIGHT_MID, -100, 0);
    lv_obj_set_style_text_color(ui_lbl_bt, lv_color_hex(C_BT_OFF), 0);

    ui_lbl_time = lv_label_create(hdr);
    lv_label_set_text(ui_lbl_time, "00:00:00");
    lv_obj_align(ui_lbl_time, LV_ALIGN_RIGHT_MID, -6, 0);
    lv_obj_set_style_text_color(ui_lbl_time, lv_color_hex(C_TEXT), 0);

    // ── Tab view ─────────────────────────────────────────────────────────
    const int TAB_Y     = 36;
    const int TAB_H     = SCREEN_H - TAB_Y;     // 284
    const int STRIP_H   = 28;
    const int CONTENT_H = TAB_H - STRIP_H;      // 256
    const int ACTION_H  = 40;
    const int BREAD_H   = 22;
    const int TABLE_H   = CONTENT_H - ACTION_H - BREAD_H;  // 194

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

    // -- Action button helper --
    auto make_btn = [&](lv_obj_t *par, uint32_t col, const char *txt,
                        lv_event_cb_t cb, lv_obj_t **lbl_out = nullptr) {
        lv_obj_t *btn = lv_btn_create(par);
        lv_obj_set_size(btn, 86, 32);
        lv_obj_set_style_bg_color(btn, lv_color_hex(col), 0);
        lv_obj_set_style_bg_color(btn,
            lv_color_darken(lv_color_hex(col), LV_OPA_20), LV_STATE_PRESSED);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_set_style_shadow_width(btn, 3, 0);
        lv_obj_set_style_shadow_opa(btn, LV_OPA_30, 0);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, txt);
        lv_obj_center(lbl);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(C_WHITE), 0);
        if (lbl_out) *lbl_out = lbl;
    };

    auto make_action_bar = [&](lv_obj_t *par, int y_pos) -> lv_obj_t * {
        lv_obj_t *bar = lv_obj_create(par);
        lv_obj_set_size(bar, SCREEN_W, ACTION_H);
        lv_obj_set_pos(bar, 0, y_pos);
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
        lv_obj_set_flex_align(bar, LV_FLEX_ALIGN_SPACE_EVENLY,
                              LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        return bar;
    };

    // ── TREE tab ─────────────────────────────────────────────────────────
    lv_obj_t *tab_t = lv_tabview_add_tab(tv, LV_SYMBOL_LIST " TREE");
    lv_obj_set_style_bg_color(tab_t, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(tab_t, 0, 0);
    lv_obj_clear_flag(tab_t, LV_OBJ_FLAG_SCROLLABLE);

    // Breadcrumb
    ui_lbl_bread = lv_label_create(tab_t);
    lv_label_set_long_mode(ui_lbl_bread, LV_LABEL_LONG_DOT);
    lv_obj_set_size(ui_lbl_bread, SCREEN_W - 8, BREAD_H);
    lv_obj_set_pos(ui_lbl_bread, 4, 2);
    lv_label_set_text(ui_lbl_bread, "STATIONS");
    lv_obj_set_style_text_font(ui_lbl_bread, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ui_lbl_bread, lv_color_hex(C_HDR_LINE), 0);

    // Tree table
    ui_tbl_tree = lv_table_create(tab_t);
    lv_obj_set_size(ui_tbl_tree, SCREEN_W, TABLE_H);
    lv_obj_set_pos(ui_tbl_tree, 0, BREAD_H);
    lv_table_set_col_cnt(ui_tbl_tree, 5);
    lv_table_set_col_width(ui_tbl_tree, 0,  58);
    lv_table_set_col_width(ui_tbl_tree, 1, 110);
    lv_table_set_col_width(ui_tbl_tree, 2, 105);
    lv_table_set_col_width(ui_tbl_tree, 3, 105);
    lv_table_set_col_width(ui_tbl_tree, 4,  60);
    lv_obj_set_style_bg_color(ui_tbl_tree, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_top(ui_tbl_tree,    4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(ui_tbl_tree, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(ui_tbl_tree,   6, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(ui_tbl_tree,  6, LV_PART_ITEMS);
    lv_obj_add_event_cb(ui_tbl_tree, tree_draw_cb,  LV_EVENT_DRAW_PART_BEGIN, nullptr);
    lv_obj_add_event_cb(ui_tbl_tree, tree_click_cb, LV_EVENT_VALUE_CHANGED,   nullptr);

    // Action bar
    lv_obj_t *bar_t = make_action_bar(tab_t, BREAD_H + TABLE_H);
    make_btn(bar_t, C_BTN_MEAS, LV_SYMBOL_PLUS " +STN", btn_meas_cb, &ui_lbl_meas_btn);
    make_btn(bar_t, C_BTN_INTO, LV_SYMBOL_RIGHT " INTO", btn_into_cb);
    make_btn(bar_t, C_BTN_BACK, LV_SYMBOL_LEFT " BACK",  btn_back_cb);
    make_btn(bar_t, C_BTN_DEL,  LV_SYMBOL_TRASH " DEL",  btn_del_cb);
    make_btn(bar_t, C_BTN_SAVE, LV_SYMBOL_SAVE " SAVE",  btn_save_cb);

    // ── SENSOR tab ───────────────────────────────────────────────────────
    lv_obj_t *tab_s = lv_tabview_add_tab(tv, LV_SYMBOL_EYE_OPEN " SENSOR");
    lv_obj_set_style_bg_color(tab_s, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(tab_s, 10, 0);
    lv_obj_clear_flag(tab_s, LV_OBJ_FLAG_SCROLLABLE);

    auto make_section_lbl = [&](lv_obj_t *par, int y, const char *txt) {
        lv_obj_t *lbl = lv_label_create(par);
        lv_label_set_text(lbl, txt);
        lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 0, y);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(C_HDR_LINE), 0);
    };

    auto make_value_lbl = [&](lv_obj_t *par, int y) -> lv_obj_t * {
        lv_obj_t *lbl = lv_label_create(par);
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_DOT);
        lv_obj_set_width(lbl, SCREEN_W - 20);
        lv_obj_align(lbl, LV_ALIGN_TOP_LEFT, 0, y);
        lv_label_set_text(lbl, "---");
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(C_TEXT), 0);
        return lbl;
    };

    make_section_lbl(tab_s, 0,  "TOF LIDAR (distance)");
    ui_lbl_tof_val = make_value_lbl(tab_s, 22);

    make_section_lbl(tab_s, 58, "IMU ORIENTATION");
    ui_lbl_imu_val = make_value_lbl(tab_s, 80);

    make_section_lbl(tab_s, 116, "SENSOR STATUS");
    ui_lbl_sens_stat = make_value_lbl(tab_s, 138);

    make_section_lbl(tab_s, 174, "ROOT GPS");
    ui_lbl_gps_val = make_value_lbl(tab_s, 196);

    // ── FILES tab ────────────────────────────────────────────────────────
    lv_obj_t *tab_f = lv_tabview_add_tab(tv, LV_SYMBOL_SD_CARD " FILES");
    lv_obj_set_style_bg_color(tab_f, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_all(tab_f, 6, 0);
    lv_obj_clear_flag(tab_f, LV_OBJ_FLAG_SCROLLABLE);

    ui_lbl_active = lv_label_create(tab_f);
    lv_label_set_long_mode(ui_lbl_active, LV_LABEL_LONG_DOT);
    lv_obj_set_width(ui_lbl_active, SCREEN_W - 12);
    lv_obj_align(ui_lbl_active, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_text_font(ui_lbl_active, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ui_lbl_active, lv_color_hex(C_REF_S), 0);

    const int FILE_TBL_H = CONTENT_H - 12 - 22 - 4 - 44 - 4 - 22;
    ui_tbl_files = lv_table_create(tab_f);
    lv_obj_set_size(ui_tbl_files, SCREEN_W - 12, FILE_TBL_H);
    lv_obj_align(ui_tbl_files, LV_ALIGN_TOP_LEFT, 0, 26);
    lv_table_set_col_cnt(ui_tbl_files, 2);
    lv_table_set_col_width(ui_tbl_files, 0, SCREEN_W - 12 - 90);
    lv_table_set_col_width(ui_tbl_files, 1, 84);
    lv_obj_set_style_bg_color(ui_tbl_files, lv_color_hex(C_BG), 0);
    lv_obj_set_style_pad_top(ui_tbl_files,    4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(ui_tbl_files, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(ui_tbl_files,   8, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(ui_tbl_files,  8, LV_PART_ITEMS);
    lv_obj_add_event_cb(ui_tbl_files, file_list_draw_cb,  LV_EVENT_DRAW_PART_BEGIN, nullptr);
    lv_obj_add_event_cb(ui_tbl_files, file_list_click_cb, LV_EVENT_VALUE_CHANGED,   nullptr);

    auto make_file_btn = [&](lv_obj_t *par, uint32_t col,
                             const char *txt, lv_event_cb_t cb) {
        lv_obj_t *btn = lv_btn_create(par);
        lv_obj_set_size(btn, 148, 40);
        lv_obj_set_style_bg_color(btn, lv_color_hex(col), 0);
        lv_obj_set_style_bg_color(btn,
            lv_color_darken(lv_color_hex(col), LV_OPA_20), LV_STATE_PRESSED);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_set_style_shadow_width(btn, 4, 0);
        lv_obj_set_style_shadow_opa(btn, LV_OPA_30, 0);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, txt);
        lv_obj_center(lbl);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(C_WHITE), 0);
    };

    int btn_y = 26 + FILE_TBL_H + 4;
    lv_obj_t *btn_row = lv_obj_create(tab_f);
    lv_obj_set_size(btn_row, SCREEN_W - 12, 44);
    lv_obj_align(btn_row, LV_ALIGN_TOP_LEFT, 0, btn_y);
    lv_obj_set_style_bg_opa(btn_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btn_row, 0, 0);
    lv_obj_set_style_pad_all(btn_row, 0, 0);
    lv_obj_set_layout(btn_row, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_row, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(btn_row, LV_OBJ_FLAG_SCROLLABLE);

    make_file_btn(btn_row, C_BTN_NEW, LV_SYMBOL_PLUS  " NEW FILE",  btn_new_file_cb);
    make_file_btn(btn_row, C_BTN_USE, LV_SYMBOL_OK    " USE FILE",  btn_use_file_cb);
    make_file_btn(btn_row, C_BTN_DEL, LV_SYMBOL_TRASH " DEL FILE",  btn_del_file_cb);

    ui_lbl_fstatus = lv_label_create(tab_f);
    lv_label_set_long_mode(ui_lbl_fstatus, LV_LABEL_LONG_DOT);
    lv_obj_set_width(ui_lbl_fstatus, SCREEN_W - 12);
    lv_obj_align(ui_lbl_fstatus, LV_ALIGN_TOP_LEFT, 0, btn_y + 48);
    lv_label_set_text(ui_lbl_fstatus, "Ready");
    lv_obj_set_style_text_font(ui_lbl_fstatus, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ui_lbl_fstatus, lv_color_hex(C_GREY), 0);

    // ── Initial render ───────────────────────────────────────────────────
    refresh_tree();
    update_active_lbl();
    update_status();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Hardware init helpers
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
                      ct == CARD_SDHC ? "SDHC" : "UNK",
                      SD.cardSize() / (1024ULL * 1024ULL));
    } else {
        Serial.println("[SD] Not found");
    }
}

static void sensor_init()
{
    Wire.begin(I2C_SDA, I2C_SCL, 100000);
    delay(200);

    // BNO08x reset
    pinMode(IMU_RST, OUTPUT);
    digitalWrite(IMU_RST, LOW);
    delay(20);
    digitalWrite(IMU_RST, HIGH);
    delay(300);

    pinMode(IMU_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IMU_INT), imuISR, FALLING);

    imu_ok = bno08x.begin_I2C(IMU_ADDR, &Wire);
    if (imu_ok) {
        bno08x.enableReport(SH2_ROTATION_VECTOR, 20000);
        delay(50);
        bno08x.enableReport(SH2_ACCELEROMETER, 50000);
        delay(50);
        Serial.println("[IMU] BNO08x OK");
    } else {
        Serial.println("[IMU] BNO08x FAIL – angles will be 0");
    }

    uint32_t d;
    tof_ok = read_tof(d);
    if (tof_ok) tof_dist_mm = d;
    Serial.printf("[TOF] %s\n", tof_ok ? "OK" : "FAIL");
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    Serial.println("\n[BRICS-5 MM1] Booting...");

    tft.init();
    tft.setRotation(1);
    tft.setTouch(const_cast<uint16_t *>(TOUCH_CAL));
    tft.fillScreen(TFT_BLACK);

    sd_init();
    sensor_init();

    SerialBT.begin("BRICS5-MM1");
    Serial.println("[BT] 'BRICS5-MM1' advertising");

    // LVGL
    lv_init();
    lvgl_buf = (lv_color_t *)malloc(SCREEN_W * 10 * sizeof(lv_color_t));
    if (!lvgl_buf) { Serial.println("[LVGL] FATAL: alloc failed"); while (true) delay(1000); }
    lv_disp_draw_buf_init(&draw_buf, lvgl_buf, nullptr, SCREEN_W * 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = SCREEN_W;
    disp_drv.ver_res  = SCREEN_H;
    disp_drv.flush_cb = disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    lv_theme_t *th = lv_theme_default_init(
        disp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_TEAL),
        false, &lv_font_montserrat_14);
    lv_disp_set_theme(disp, th);

    // Load tree from SD
    if (sd_ready) {
        if (!SD.exists(active_csv)) {
            File f = SD.open(active_csv, FILE_WRITE);
            if (f) { f.println("ID,PARENT_ID,LAT,LON,ALT,DIST,ROLL,PITCH,YAW,X,Y,Z"); f.close(); }
            Serial.printf("[SD] Created empty %s\n", active_csv);
        }
        load_tree();
    }

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read;
    lv_indev_drv_register(&indev_drv);

    build_ui();

    lv_timer_create(periodic_cb, 1000, nullptr);
    lv_timer_create(sensor_timer_cb, 250, nullptr);

    Serial.println("[BRICS-5 MM1] Ready");
}

// ─────────────────────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    uint32_t d;
    if (read_tof(d)) { tof_dist_mm = d; tof_ok = true; }
    poll_imu();

    lv_timer_handler();
    delay(5);
}
