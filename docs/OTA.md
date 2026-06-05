# Firmware update

## On device (SETUP → About)

Shows **FW_VERSION** and a QR code to:

**https://verlab.github.io/mm1-black/**

## USB installer (recommended)

1. Connect **MM1-BLACK** by USB to a PC (**Chrome** or **Edge**).
2. Open the **[firmware installer](https://verlab.github.io/mm1-black/)**.
3. Select a release, confirm the checkbox, click **Install**, pick the serial port.
4. Optional: **Read** sends `VERSION` at 9600 baud → `MM1_FW_VERSION=…`

Images: `MM1-BLACK-denky32-vX.Y.Z.bin` on [GitHub Releases](https://github.com/verlab/mm1-black/releases).

## Not supported

- Over-the-air update inside the handset (no Wi‑Fi OTA download)
- Flashing from the optional Wi‑Fi file portal (SD export only)

## Developers

```bash
pio run -t upload -e denky32
```

See [docs/CI.md](docs/CI.md) and [docs/flasher/DEPLOY.md](flasher/DEPLOY.md).
