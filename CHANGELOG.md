# Changelog

Mudanças relevantes do **MM1-BLACK** (formato baseado em [Keep a Changelog](https://keepachangelog.com/pt-BR/1.1.0/)).

## [0.3.0] — 2026-05-12

### Corrigido

- **Toque (XPT2046)**: o cartão SD usava `SPI.begin()` no **VSPI** global, o mesmo barramento do TFT e do controlador de toque — o VSPI era remapeado para os pinos do SD e o toque deixava de responder. O SD passa a usar **`SPIClass sd_spi(HSPI)`** com `SD.begin(CS, sd_spi)`.
- **Build TFT_eSPI**: `include/User_Setup.h` (ST7796, `TOUCH_CS`, SPI) é copiado para `.pio/libdeps/.../TFT_eSPI/` em cada build via `scripts/pio_tft_setup_copy.py` e `extra_scripts` no `platformio.ini`, para o toque e o driver compilarem com os pinos corretos.
- **Leitura LVGL**: `touch_read` com limiar `350`, última coordenada em `REL` e `continue_reading = false` para estabilidade do ponteiro.

### Adicionado

- **Splash de arranque** MIRA (`mira_splash_map` / `show_boot_splash_tft`) com o mesmo pipeline de cor que o flush LVGL (`pushColors`, swap de bytes).
- **Asset** `assets/MIRA_principal_R.png` e ferramenta `tools/gen_mira_splash.py` para regenerar `src/mira_splash_img.c`.
- **`include/User_Setup.h`** alinhado ao painel E32R40T (demo ST7796).

### Alterado

- **IMU / laser**: azimute e inclinação ao longo do eixo do laser; política de polling do laser por separador; captura com botão físico (hold/release); cabeçalho **MM1-BLACK**; tabela de pontos com linha mais recente no topo (só UI); alinhamento do rótulo **PTS:** para não sobrepor ícones da barra.

---

## [0.2.0] — _(anterior)_

Versões anteriores não tinham changelog no repositório; usar `git log` para histórico fino.
