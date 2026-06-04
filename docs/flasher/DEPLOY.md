# Deploying the installer page

Same model as [Tasmota Install](https://tasmota.github.io/install/): static page on GitHub Pages, firmware binaries on **GitHub Releases**.

## Requirements

1. **Repository must be public** (or release assets publicly downloadable).  
   Private repos return HTTP 404 to the browser for both the Releases API and `.bin` downloads — the installer cannot list or fetch firmware.

2. **Settings → Pages → Source: GitHub Actions** — workflow `Deploy flasher (GitHub Pages)`.

3. Site URL: **https://verlab.github.io/cyd_brics5_mm1/**

## Releases

Tag `v*` → workflow **Release** attaches `MM1-BLACK-denky32-v*.bin`.  
The installer loads the list from `api.github.com/repos/verlab/cyd_brics5_mm1/releases` automatically.

No firmware files are stored under `docs/flasher/`.
