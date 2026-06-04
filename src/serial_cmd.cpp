#include "serial_cmd.h"

#ifdef ARDUINO_ARCH_ESP32

#include <Arduino.h>
#include <ctype.h>
#include "firmware_version.h"

static char g_cmd_buf[24];
static uint8_t g_cmd_len = 0;

void serial_cmd_poll(void)
{
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();
        if (c == '\r')
            continue;
        if (c == '\n') {
            g_cmd_buf[g_cmd_len] = '\0';
            if (g_cmd_len > 0) {
                for (uint8_t i = 0; i < g_cmd_len; i++)
                    g_cmd_buf[i] = (char)toupper((unsigned char)g_cmd_buf[i]);
                if (strcmp(g_cmd_buf, "VERSION") == 0 ||
                    strcmp(g_cmd_buf, "FW_VERSION") == 0) {
                    Serial.print("MM1_FW_VERSION=");
                    Serial.println(FW_VERSION);
                }
            }
            g_cmd_len = 0;
            continue;
        }
        if (g_cmd_len < sizeof(g_cmd_buf) - 1)
            g_cmd_buf[g_cmd_len++] = c;
        else
            g_cmd_len = 0;
    }
}

#endif
