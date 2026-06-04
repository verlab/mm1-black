# MM1-BLACK

Handheld 1D cave survey tool: **laser distance**, **BNO08x IMU** (azimuth, inclination, roll), **touch UI**, **SD card** CSV, **SAP6 BLE** for **TopoDroid** and **SexyTopo**.

[![CI](https://github.com/verlab/mm1-black/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/verlab/mm1-black/actions/workflows/ci.yml)
[![Release](https://img.shields.io/badge/dynamic/json?label=release&query=%24.tag_name&url=https%3A%2F%2Fapi.github.com%2Frepos%2Fverlab%2Fmm1-black%2Freleases%2Flatest&color=orange&logo=github)](https://github.com/verlab/mm1-black/releases/latest)
[![Firmware installer](https://img.shields.io/badge/firmware-install%20page-blue)](https://verlab.github.io/mm1-black/)
![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-blue)

**Firmware update (USB):** [verlab.github.io/mm1-black](https://verlab.github.io/mm1-black/) — also via QR on the device (**SETUP → About**).

## Features

- **POINTS** — measurement table; physical button for aim + capture (laser + IMU)
- **SENSOR** — live laser distance, orientation, MCU temperature
- **FILES** — SD card CSV: create, select, delete, stream to BLE
- **SETUP** — About (version + installer QR), brightness, calibration (IMU + azimuth + laser), Bluetooth (SAP6)
- **SAP6 BLE** — CaveBLE GATT legs to TopoDroid / SexyTopo ([docs/SAP6_BLE.md](docs/SAP6_BLE.md))
- **STREAM** — send the active CSV file as SAP6 legs (SETUP → BT or POINTS)
- **CI / Releases** — `MM1-BLACK-denky32-v*.bin` on each tag `v*`

## Hardware (reference board)

ESP32 with **4.0″ ST7796** 480×320 touch (manufacturer demo bundle under `docs/4.0inch_*`).

| Function | Interface | Notes |
|----------|-----------|--------|
| Display / touch | HSPI | ST7796 + XPT2046 |
| Backlight | PWM GPIO 27 | 10–100%, NVS, SETUP → Bright |
| SD card | SPI | FAT32, CSV storage |
| IMU | I2C BNO08x | Azimuth, inclination, roll |
| Laser | UART | Distance (module-dependent) |
| BLE | SAP6 | `SAP6_MM1` — TopoDroid / SexyTopo |

Pin details: `platformio.ini`, `board_support/TFT_eSPI_User_Setup.h`, [docs/MEMORY_LASER.md](docs/MEMORY_LASER.md).

## UI

```
┌──────────── Header: MM1-BLACK · PTS · BAT · SD · BT · clock ────────────┐
│  [ POINTS ] [ SENSOR ] [ FILES ] [ SETUP ]                              │
├─────────────────────────────────────────────────────────────────────────┤
│  Points table · actions: measure / save / stream / GPS / …              │
└─────────────────────────────────────────────────────────────────────────┘
```

## CSV (TopoDroid-style)

Files on SD use a fixed header compatible with TopoDroid import:

```text
Time-Stamp, POSIX Time, Index, Distance (meters), Azimuth (deg), Inclination (deg), Dip (deg), Roll (deg), Temperature (Celsius),  Measurement Type, Error Log
```

Comment lines starting with `#` are ignored when loading.

## Bluetooth

Compatible with **SAP6** clients: **TopoDroid** and **SexyTopo** (not legacy SPP tape emulators).

| Action | Where |
|--------|--------|
| Single shot + leg notify | SETUP → BT **Measure**, or physical button |
| Stream CSV legs | POINTS **STREAM** or SETUP → BT **TX** |
| Pairing / status | SETUP → BT |

See [docs/SAP6_BLE.md](docs/SAP6_BLE.md) for pairing steps.

## Build & flash

```bash
pio run -e denky32
pio run -t upload -e denky32
pio device monitor -b 115200
```

### Releases & web installer

- Tag `v*` → GitHub Release with `MM1-BLACK-denky32-vX.Y.Z.bin`
- **[Web installer](https://verlab.github.io/mm1-black/)** — Chrome/Edge, Web Serial ([docs/OTA.md](docs/OTA.md), [docs/flasher/DEPLOY.md](docs/flasher/DEPLOY.md))
- CI artifacts: firmware per commit on Actions

## Project layout

```
mm1-black/
├── src/main.cpp          # Application
├── include/              # Headers (e.g. fw_update_url.h)
├── platformio.ini
├── docs/                 # OTA, SAP6, flasher, board vendor bundle
└── .github/workflows/    # CI, Release, Pages
```

## License

Firmware for the MM1-BLACK field survey handset (VERLAB / open development).

---

<p align="center">
  <a href="https://www.mirarobotica.com/">
    <img src="docs/flasher/assets/mira-logo.png" alt="MIRA — Mapeamento e Robótica" width="220" />
  </a>
</p>

<p align="center">
  <a href="https://www.mirarobotica.com/"><strong>MIRA — Mapeamento e Robótica</strong></a><br />
  Soluções em mapeamento e robótica para inspeção de espaços confinados.
</p>
