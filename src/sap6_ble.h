#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef ARDUINO_ARCH_ESP32

/** CaveBLE / SAP6 survey protocol (TopoDroid, SexyTopo, DiscoX family). */
void sap6_ble_begin(const char *device_name);
/** Full BT stack reset + advertise again (SETUP button). */
void sap6_ble_restart(const char *device_name);
void sap6_ble_poll(void);

bool sap6_ble_stack_ready(void);
bool sap6_ble_connected(void);
void sap6_ble_get_mac_str(char *buf, size_t len);
/** ctrl status + free heap for SETUP diagnostics */
void sap6_ble_format_status(char *buf, size_t len);

/** Queue one leg (azimuth°, inclination°, roll°, distance m). Sent when central ACKs prior leg. */
void sap6_ble_send_leg(float azimuth_deg, float inclination_deg, float roll_deg, float distance_m);

/** Replay all points in RAM (SETUP STREAM). */
int sap6_ble_stream_points(int count, const void *pts, size_t pt_stride,
                           float (*get_az)(const void *), float (*get_inc)(const void *),
                           float (*get_roll)(const void *), float (*get_dist)(const void *));

void sap6_ble_clear_bonds(void);

uint32_t sap6_ble_legs_sent(void);
uint32_t sap6_ble_acks_ok(void);
uint32_t sap6_ble_acks_wrong(void);
uint32_t sap6_ble_resends(void);
uint32_t sap6_ble_queue_depth(void);

#endif
