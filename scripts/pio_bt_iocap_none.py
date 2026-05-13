"""Patch Arduino BluetoothSerial SSP IO capability to NoInputNoOutput.

With ESP_BT_IO_CAP_IO (DisplayYesNo), the stack expects TWO visible confirmations during
SSP numeric comparison — the phone shows a code while the MCU had nothing on TFT. Many
embedded SPP slaves use IO_CAP_NONE so pairing is confirmed mainly on the phone (same idea as
arduino-esp32 SerialToSerialBT_SSP example with OUTPUT_CAPABILITY=false).

Patches the global framework copy under ~/.platformio (shared across PIO projects).
"""

Import("env")

from pathlib import Path

OLD = "esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;"
NEW = "esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;"

platform = env.PioPlatform()
fd = platform.get_package_dir("framework-arduinoespressif32")
if not fd:
    print("[pio_bt_iocap] skip: framework-arduinoespressif32 path unknown")
else:
    path = Path(fd) / "libraries" / "BluetoothSerial" / "src" / "BluetoothSerial.cpp"
    if not path.is_file():
        print(f"[pio_bt_iocap] skip: missing {path}")
    else:
        text = path.read_text(encoding="utf-8")
        if NEW in text:
            print("[pio_bt_iocap] already ESP_BT_IO_CAP_NONE")
        elif OLD in text:
            path.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
            print("[pio_bt_iocap] BluetoothSerial SSP: IO_CAP_IO -> IO_CAP_NONE")
        else:
            print("[pio_bt_iocap] WARNING: IOCAP assignment line not found (core updated?)")
