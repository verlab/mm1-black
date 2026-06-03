# Changelog

Mudanças relevantes do **MM1-BLACK** (formato baseado em [Keep a Changelog](https://keepachangelog.com/pt-BR/1.1.0/)).

## [Unreleased]

### Adicionado

- **Brilho da tela (#4)**: PWM em `TFT_BL` (GPIO 27), slider em SETUP; valor 10–100% gravado em NVS (`bl_pct`).
- **SETUP**: página única rolável (sem sub-abas/QR); temperatura só na aba SENSOR; menos carga ao abrir SETUP.

### Alterado

- **UI**: temperatura só na aba SENSOR; barra superior clássica (ícones restaurados).
- **Captura botão**: blink no cabeçalho azul da tabela (mira/medir = azul; OK = verde; falha = vermelho); sem overlay na tela inteira.

## [0.4.2] — 2026-06-03

### Adicionado

- **Botão BRIC5 (#2)**: feedback visual — badge **AIM** / **CAPTURE** na barra (todas as abas), faixa laranja durante medição, backlight TFT (GPIO 27) com pulso lento (mira) e rápido (captura).

## [0.4.1] — 2026-06-03

### Adicionado

- **Temperatura MCU** na barra superior, aba SENSOR (status) e SETUP → Live; mesma leitura da coluna Temperature do CSV (`temperatureRead()`).
- **`docs/MEMORY_LASER.md`**: protocolos M01 vs X-40/701A, notas sobre baud e temperatura.

### Alterado

- **Boot**: chime com o logo; splash mínimo ~0,9 s (antes 3 s).
- **PlatformIO**: `upload_speed = 921600`.
- **Botão físico (BRIC5)**: captura laser com one-shot/retry sem dreno UART com feixe ligado; sons sucesso/erro; `LASER_OFF` ao fim.
- **Laser**: `LASER_OFF` após boot; UART dedicada recomendada (`LZR_SHARE_USB_UART=0`) para evitar conflito com USB.

## [0.4.0] — 2026-05-12

### Adicionado

- **SETUP**: painel “Live measurements” (heading, qualidade, gravidade, eixo laser/IMU), botões **Export** / **Files** / **Meas** (mesmos comandos Bluetooth) e feedback na linha de estado.
- **Sanidade IMU**: magnitude do acelerómetro (`imu_accel_mag_mss`) para cruzar com ~1 g.

### Alterado

- **Branding**: nome Bluetooth e strings de produto **`MM1-BLACK`** (sem BRICS); macros CSV **`TD_CSV_HEADER`**, **`EXPORT_DIP_DEG_NOMINAL`**; comentários de ficheiro: ignorar qualquer linha `#…` ao carregar.
- **SD**: ficheiros predefinidos **`mm1_black_XXX.csv`**.
- **Documentação**: `README`, `CHANGELOG`, `lv_conf.h` alinhados ao MM1-BLACK.

### Outros

- Build **denky32** com partição `huge_app` (3 MB app).
