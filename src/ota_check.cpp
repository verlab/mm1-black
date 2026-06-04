/**
 * Firmware update check — placeholder until Wi-Fi STA + HTTPS OTA.
 * See docs/OTA.md
 */

#include "ota_check.h"

#ifdef ARDUINO_ARCH_ESP32

#include <Arduino.h>
#include "firmware_version.h"

static char g_ota_status[96] = "Toque Verificar para mais info";

const char *ota_check_status(void)
{
    return g_ota_status;
}

void ota_check_request(void)
{
    snprintf(g_ota_status, sizeof(g_ota_status),
             "v%s: use Release GitHub ou portal Wi-Fi",
             FW_VERSION);
}

void ota_check_poll(void)
{
}

#endif
