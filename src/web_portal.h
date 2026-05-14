#pragma once

#include <Arduino.h>

#ifdef ARDUINO_ARCH_ESP32

namespace web_portal {

struct Status {
    bool        wifi_running;
    bool        bt_paired;
    bool        bt_linked;
    uint8_t     wifi_clients;
    uint32_t    point_count;
    const char* device_name;
    const char* ap_ssid;
    const char* ap_password;
    const char* ap_ip;
    const char* bt_local_mac;
    const char* bt_peer_mac;
    const char* active_csv;
    const char* fw_version;
};

using WriteChunk = void (*)(const char*, size_t, void*);

/* Callbacks the UI side must supply so the web layer stays decoupled from the
 * LVGL globals in main.cpp. All callbacks must be cheap (run on web task). */
struct Callbacks {
    void (*get_status)(Status&);
    /* Snapshot current in-RAM points as a TopoDroid-compatible CSV (with header). */
    void (*write_points_csv)(WriteChunk write_chunk, void* user);
    /* Snapshot current in-RAM points as a JSON array { "points":[ {...}, ... ] }. */
    void (*write_points_json)(WriteChunk write_chunk, void* user);
};

bool start(const char* ssid, const char* password, const Callbacks& cb);
void stop();
bool running();
uint8_t clients();
const char* ap_ip();

void loop();   /* call from main loop while running; no radio until start() from device */

}  // namespace web_portal

#endif  // ARDUINO_ARCH_ESP32
