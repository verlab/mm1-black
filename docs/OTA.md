# Firmware update

## On-device (SETUP → App)

Shows **FW_VERSION** and a **QR code** to the updater page:

**https://verlab.github.io/cyd_brics5_mm1/**

## Official path: USB + Web Serial (issue #15)

1. Connect MM1-BLACK by **USB** to a PC (Chrome or Edge).
2. Open the [firmware updater](https://verlab.github.io/cyd_brics5_mm1/).
3. **Connect USB & read version** (sends `VERSION` at 9600 baud; device replies `MM1_FW_VERSION=…`).
4. Pick a **GitHub Release** and **Flash firmware** (confirm disclaimer).

Firmware images: `MM1-BLACK-denky32-vX.Y.Z.bin` from [Releases](https://github.com/verlab/cyd_brics5_mm1/releases).

## Not supported

- In-device OTA (Wi‑Fi scan, WPA, HTTPS download, `esp_ota`)
- Updating through the MM1-MIRA soft-AP (CSV export only)

## Alternative (developers)

```bash
pio run -t upload -e denky32
```

Related: issue #12 (CI/CD + release binaries).
