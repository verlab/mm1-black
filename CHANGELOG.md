# Changelog

Mudanças relevantes do **MM1-BLACK** (formato baseado em [Keep a Changelog](https://keepachangelog.com/pt-BR/1.1.0/)).

## [Unreleased]

### Adicionado

- **SAP6 BLE**: protocolo CaveBLE (GATT) em vez de DistoX SPP; fila + reenvio 5 s; doc [docs/SAP6_BLE.md](docs/SAP6_BLE.md).
- **Brilho da tela (#4)**: PWM em `TFT_BL` (GPIO 27), slider em SETUP; valor 10–100% gravado em NVS (`bl_pct`).
- **SETUP**: sub-abas Brilho / **Cal** / BT / WiFi; QR Wi-Fi e BT; brilho só com slider; botões compactos (3 colunas, fonte 12) para não cortar texto.
- **SETUP → Cal**: saúde BNO086 (direção, qualidade fusão, |g|), guia figura‑8, offset azimute em 2 linhas, **Az=0**, teste laser; zero **C** (iliasam X‑40) com `-D LZR_PROTO_ILIASAM=1`.

### Alterado

- **STREAM / TX**: CSV no SD enviado em lotes (até **5000** legs) com popup e barra de progresso (`ACK / total`, fila, linhas lidas); tabela em RAM até **100** pontos; sem SD usa só RAM.
- **SETUP → BT**: botão **CSV** substituído por **TX** (STREAM); **Medir** com mensagens corretas (sem falso “CSV enviado”).
- **Fix TX/STREAM**: crash/reboot ao carregar TX (ponteiros lambda inválidos no stream RAM; `File` SD global; overlay LVGL branco); arranque do STREAM na `loop()`.
- **Fix TX/STREAM**: envio parava no 1º ponto (EOF SD falso com `available()`; fim prematuro do stream; fila BLE enche por lote).
- **Fix TX/STREAM**: `csv_read_line` usa `position/size` em todo o SD (antes só lia ~2 linhas).
- **Fix TX/STREAM**: fila BLE cheia nao descarta linhas; fim parcial/timeout; linhas com erro no CSV; `ign` no popup.
- **Fix TX/STREAM**: overlay preto sem envio (SD `size()==0`, reabrir ficheiro cada tick, fila bloqueada); botao Cancelar; ficheiro SD aberto uma vez.
- **Fix TX/STREAM**: remove fase “contar linhas” (bloqueava em 0); leitura SD byte-a-byte; envio imediato; painel branco visivel.
- **Fix TX/STREAM**: prioridade RAM (`load_csv` + `sap6_ble_stream_start`); SD so se tabela vazia; timeout se zero envio em 20 s.
- **Fix TX/STREAM**: revert ao fluxo 49eb747 (SD contar + enviar, popup %); `stream_tick` um leg/ACK para nao travar.
- **Fix TX/STREAM**: remove overlay `lv_layer_top` (tela branca); progresso na barra POINTS; `load_csv` antes do envio RAM; um leg por ACK (`try_send_leg`); leitura SD com `size()==0`.
- **Bluetooth**: apenas **BLE SAP6** (`SAP6_MM1`); parear no Android e escolher SAP6/DiscoX no TopoDroid ou SexyTopo.
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
