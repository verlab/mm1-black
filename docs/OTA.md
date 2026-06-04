# Atualização de firmware

## Versão na UI

**SETUP → App** mostra `FW_VERSION` (definida no build; tag Git no CI de release).

## Caminho oficial (planeado)

**USB + GitHub Pages + WebSerial** — ver [issue #15](https://github.com/verlab/cyd_brics5_mm1/issues/15).

Utilizador liga o MM1 por cabo USB, abre a página no Chrome e grava o `.bin` do [Release](https://github.com/verlab/cyd_brics5_mm1/releases).

## O que não vamos fazer

**OTA dentro do ESP32** (scan de redes, senha WPA, download HTTPS, `esp_ota`) — descartado por RAM, UI e coexistência BLE/Wi‑Fi.

O portal **MM1-MIRA** (soft-AP) serve só para **ficheiros CSV**, não para atualizar firmware.

## Fluxos disponíveis hoje

1. GitHub **Releases** → `MM1-BLACK-denky32-vX.Y.Z.bin`
2. `pio run -t upload -e denky32` ou `esptool.py write_flash`

Relacionado: issue #12 (CI/CD + binário no release).
