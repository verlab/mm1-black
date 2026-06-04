#pragma once

#ifdef ARDUINO_ARCH_ESP32

/** Handle VERSION command on USB/UART0 (9600 when LZR_SHARE_USB_UART). */
void serial_cmd_poll(void);

#endif
