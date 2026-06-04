# Deploying the flasher page

1. **Settings → Pages → Source: GitHub Actions** (not “Deploy from branch”).
2. Workflow **Deploy flasher (GitHub Pages)** publishes `docs/flasher/` at:

   **https://verlab.github.io/cyd_brics5_mm1/**

## Firmware list (private repo)

The browser cannot call the GitHub Releases API on a **private** repo without a token.
Firmware images are hosted on Pages:

- `docs/flasher/releases.json` — manifest (tag, URL, size)
- `docs/flasher/bins/*.bin` — binaries (updated by the **Release** workflow on each `v*` tag)

After changing the flasher, push to `main` so Pages redeploys.

## USB connect

Use **Chrome** or **Edge** on a desktop PC. Click **Connect USB** and pick the CYD serial port in the browser dialog.
The log panel at the bottom of the Flash card shows progress and errors.
