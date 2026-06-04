# Atualização de firmware (OTA)

## Versão na UI

**SETUP → App** mostra `FW_VERSION` (definida no build; tag Git no CI de release).

## É possível o ESP32 verificar e atualizar sozinho?

**Sim, com Wi‑Fi que tenha Internet**, mas hoje o MM1 expõe sobretudo:

| Modo | Internet | OTA automático |
|------|----------|----------------|
| **Portal Wi‑Fi** (soft-AP `MM1-MIRA`) | Não (só telemóvel ↔ ESP) | Descarregar `.bin` no browser e gravar com `esptool` / PlatformIO |
| **Wi‑Fi STA** (rede com router) | Sim | Possível com `HTTPClient` + `esp_ota` (planeado) |

A **AP do portal não roteia** para a Internet — o telemóvel não “puxa” GitHub através do ESP32.

### Fluxo recomendado hoje

1. GitHub **Releases** → descarregar `MM1-BLACK-denky32-vX.Y.Z.bin`
2. `pio run -t upload -e denky32` ou `esptool.py write_flash` na flash do app

### Próximo passo (firmware)

- Botão **Verificar atualização** com Wi‑Fi STA (credenciais em NVS)
- Comparar tag com `https://api.github.com/repos/verlab/cyd_brics5_mm1/releases/latest`
- Gravar partição OTA com `esp_ota` após download HTTPS

Relacionado: issue #12 (CI/CD + binário no release).
