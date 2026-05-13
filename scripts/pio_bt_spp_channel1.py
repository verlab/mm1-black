"""Patch framework BluetoothSerial so SPP listens on RFCOMM channel 1.

TopoDroid can use prefs «RFCOMM channel» / DISTOX_SOCKET_TYPE port modes that dial SCN 1.
UUID-based sockets still work: SDP lists the announced channel.
Patches the Espressif Arduino core package on disk (shared across PIO projects).
"""

Import("env")

from pathlib import Path

MARKER_OLD = "esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, _spp_server_name)"
MARKER_NEW = "esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 1, _spp_server_name)"

platform = env.PioPlatform()
fd = platform.get_package_dir("framework-arduinoespressif32")
if not fd:
    print("[pio_bt_spp_ch1] skip: framework-arduinoespressif32 path unknown")
else:
    path = Path(fd) / "libraries" / "BluetoothSerial" / "src" / "BluetoothSerial.cpp"
    if not path.is_file():
        print(f"[pio_bt_spp_ch1] skip: missing {path}")
    else:
        text = path.read_text(encoding="utf-8")
        if MARKER_NEW in text:
            print("[pio_bt_spp_ch1] already SCN=1")
        elif MARKER_OLD in text:
            path.write_text(text.replace(MARKER_OLD, MARKER_NEW, 1), encoding="utf-8")
            print("[pio_bt_spp_ch1] BluetoothSerial: SPP server SCN 0 -> 1")
        else:
            print("[pio_bt_spp_ch1] WARNING: expected line not found (Arduino core updated?)")
