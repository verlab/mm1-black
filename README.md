# MM1-BLACK — Smart Tape

A handheld 1D surveying tool built on the **ESP32 CYD** (4.0" ST7796 480×320 touch display). Combines a **TOFSense LiDAR** for distance and a **BNO08x IMU** for orientation to capture georeferenced measurement points in the field.

![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-blue)
![LVGL](https://img.shields.io/badge/LVGL-8.3-green)

## Features

- **Two point types** — *Sample* (SMP) for measurement data, *Navigation* (NAV) for reference/transform points
- **Live sensor display** — real-time TOF distance and IMU roll/pitch/yaw
- **Station GPS** — single station with coordinates entered via on-screen popup (+/- buttons) or Bluetooth
- **SD card persistence** — save/load CSV files, create and manage multiple datasets
- **Bluetooth Serial** — remote control and data export via classic BT
- **Battery indicator** — header bar with ADC-based battery percentage
- **Touch UI** — LVGL-based interface with three tabs (Points, Sensor, Files)

## Hardware

| Component | Interface | Pins |
|---|---|---|
| ST7796 TFT (480×320) | HSPI | CLK=14, MOSI=13, MISO=12, CS=15, DC=2 |
| XPT2046 Touch | HSPI (shared) | CS=33 |
| SD Card (FAT32) | VSPI | CLK=18, MOSI=23, MISO=19, CS=5 |
| BNO08x IMU | I2C (Wire) | SDA=32, SCL=25, RST=17, INT=16, addr=0x4B |
| TOFSense LiDAR | I2C (Wire) | addr=0x08 |
| Battery ADC | Analog | GPIO 34 |
| Bluetooth Serial | — | Classic BT SPP scan name **`MM1BLACK`** (override `BT_DEVICE_NAME`) |

> Based on the **4.0" ESP32-WROOM-32E ST7796** board (E32R40T / E32N40T).

## UI Layout

```
┌──────────── 36px Header ──────────────────────────────────┐
│  MM1-BLACK    │ PTS:N │ BAT% │ SD │ BT │  HH:MM:SS      │
├──────────── 28px Tab Bar ─────────────────────────────────┤
│  [ POINTS ]     [ SENSOR ]     [ FILES ]                  │
├──────────── Scrollable Table ─────────────────────────────┤
│  TYPE │ # │ DIST(m) │ ROLL° │ PITCH° │ YAW°              │
│  SMP  │ 1 │  12.345 │  1.2  │  -3.4  │  45.6            │
│  NAV  │ 2 │   8.765 │  0.5  │   2.1  │ 123.4            │
├──────────── 40px Action Bar ──────────────────────────────┤
│  [+SMP] [+NAV] [DEL] [GPS] [SAVE] [CLR]                  │
└───────────────────────────────────────────────────────────┘
```

## CSV Format

```csv
#GPS,-23.550000,-46.730000,740.00
ID,TYPE,DIST,ROLL,PITCH,YAW
1,S,12.3450,1.20,-3.40,45.60
2,N,8.7650,0.50,2.10,123.40
```

The first line stores station GPS coordinates. Point type is `S` (Sample) or `N` (Navigation).

## Bluetooth Commands

| Command | Description |
|---|---|
| `GPS,lat,lon,alt` | Set station GPS coordinates |
| `MEAS` | Take a sample measurement (reads sensors) |
| `CLEAR` | Delete all points |
| `LIST` | Dump all data as CSV |

## Building

### Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- USB cable connected to the ESP32 board

### Build & Upload

```bash
# Build
pio run

# Upload
pio run --target upload --upload-port /dev/ttyUSB0

# Serial monitor
pio device monitor -b 115200
```

### Dependencies (auto-installed by PlatformIO)

| Library | Purpose |
|---|---|
| `TFT_eSPI` | Display + touch driver |
| `LVGL 8.3` | UI framework |
| `Adafruit BNO08x` | IMU driver (I2C) |
| `SD` | SD card file system |
| `BluetoothSerial` | Classic BT serial |

### TFT_eSPI Configuration

The `TFT_eSPI` library must be configured for the ST7796 board. The board-specific `User_Setup.h` is provided by the board manufacturer under `4.0inch_ESP32-32E_ST7796_E32R40T_V1.0/` and should be copied to the TFT_eSPI library directory (`.pio/libdeps/denky32/TFT_eSPI/`).

## Project Structure

```
cyd_brics5_mm1/
├── src/
│   ├── main.cpp        # Application code
│   └── lv_conf.h       # LVGL configuration
├── platformio.ini       # Build configuration
├── include/
├── lib/
└── test/
```

## License

Hardware/software for field surveying with the MM1-BLACK handheld unit (ESP32 + display + laser + IMU).
