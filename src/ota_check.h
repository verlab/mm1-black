#pragma once

#ifdef ARDUINO_ARCH_ESP32

/** Human-readable status after ota_check_poll() / user tap. */
const char *ota_check_status(void);

/** Start async check (STA/internet not implemented — sets informative status). */
void ota_check_request(void);

/** Call from loop() while check pending (reserved). */
void ota_check_poll(void);

#endif
