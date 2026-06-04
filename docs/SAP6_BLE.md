# SAP6 BLE (CaveBLE) no MM1-BLACK

Firmware em `src/sap6_ble.cpp` — substitui emulação **DistoX A3** (SPP) por **SAP6** ([CircuitPython_CaveBLE](https://github.com/furbrain/CircuitPython_CaveBLE)), o mesmo protocolo do **DiscoX** / **SAP6**.

## TopoDroid X (6.4.x) — passo a passo

O MM1 **não é mais DistoX A3** (Bluetooth clássico). Se o TopoDroid ainda mostra **A3** no topo, é o dispositivo **antigo guardado** nas preferências — não o que o scan encontrou.

### 1. Limpar o passado

1. Android → Bluetooth → **esquecer** entradas antigas `DistoX`, `SAP6_*`, MM1, etc.
2. TopoDroid → botão **Device** (dispositivo) → menu (≡) → **Detach** / limpar o ativo se ainda for **A3**.
3. Na lista do meio, apague entradas antigas (toque longo / opções) se existirem.

### 2. Scan dentro do TopoDroid (não só “Conectar”)

O botão **Conectar** no desenho usa o dispositivo **já escolhido** no topo (ex. A3 antigo). Primeiro troque o dispositivo na janela **Device**.

1. MM1 ligado, **sem** portal Wi‑Fi ativo (SETUP → WiFi desligado).
2. TopoDroid → botão **Device** (ícone do aparelho) → menu **≡** → **Scan** (não “Pair” clássico).
3. O toast **“Found N devices”** (ex. 4) conta **todos** os BLE novos vistos no rádio; a **lista** só mostra os que têm nome reconhecido (`SAP6_…`, `discox_…`, `DistoX…`, etc.). Os outros 3 do toast são vizinhos (relógio, TV, etc.).
4. Na **lista horizontal** deve surgir algo como **`SAP6 0001 AA:BB:CC:DD:EE:FF`**.
5. **Toque nessa linha** — o texto no topo passa de **A3** / **DistoX** para **SAP6**.
6. Volte ao desenho → **Conectar** (agora liga por BLE GATT / `SapComm`, não SPP A3).

**A3 ainda no topo?** É cache: menu **Detach**, apague entradas antigas na lista, faça **Scan** de novo e selecione a linha **SAP6**.

### 3. “1 found” mas lista vazia / “device not selected”

Isto é o caso mais comum com **TopoDroid 6.4**:

- O toast **“1 found”** = o MM1 foi **visto** no rádio (MAC novo).
- A **lista** só enche se `BluetoothDevice.getName()` não for null **e** começar por `SAP6_` / `SAP` / `discox_` / …
- O **nRF Connect** lê o nome no pacote BLE; o **TopoDroid não** — usa só `getName()`, que no Android muitas vezes fica **null** → nada na lista.

**Sem firmware novo (contorno):**

1. Android → **Definições → Bluetooth** → emparelhar **`SAP6_MM1`** ou **`SAP6_0001`** (PIN `0000` se pedir).
2. TopoDroid → **Device** — o aparelho pode aparecer na lista **só por estar emparelhado** (sem Scan).
3. Toque na linha **SAP6 …** → topo deixa de ser A3.

**Com firmware recente** (advertising via biblioteca BLE — nome no pacote ADV principal):

> Versões intermédias que chamavam `esp_ble_gap_config_adv_data` duas vezes seguidas podiam **silenciar o BLE** (0 devices no scan). Regrave o firmware atual.

```bash
pio run -t upload -e denky32
```

Depois **Scan** de novo. Se aparecer toast **“device … without name”**, o telefone ainda não leu o nome — use o emparelhamento Android acima.

### 4. Outras dicas

- Confirme no **nRF Connect** o nome (`SAP6_0001` ou `SAP6_MM1` — ambos aceites pelo TopoDroid se `getName()` funcionar).
- **Disconnect** no TopoDroid só limpa o **ativo** (A3); não apaga o MM1 da lista se já estiver na base de dados.
- Menu **Scan** com MM1 a <2 m, Wi‑Fi do MM1 desligado.

### 5. Nome BLE e modelo na app

| Requisito TopoDroid | MM1 |
|---------------------|-----|
| Prefixo aceite no scan | `SAP6_` (ex.: `SAP6_0001`) |
| Alternativa | `discox_0001` (tipo DiscoX na app) |
| **Não** usar | DistoX A3 / SPP — firmware não suporta |

Modo contínuo na app; cada medição no MM1 envia um **Leg** de 17 bytes; a app responde **ACK** `0x55` ou `0x56`.

### Botão TX (STREAM) na aba POINTS

- Com **SD**: lê o **CSV activo** no cartão em **lotes** (até **5000** legs) e envia por BLE — **não** precisa caber na tabela.
- **Popup** compacto (painel no centro, barra `RAM n/N` / `SD n/N`, %) e **Cancelar** — sem escurecer o ecrã inteiro (evita reset do ESP32).
- Tabela em RAM: até **100** pontos (só para ver/editar); o TX envia o **ficheiro completo** no SD se existir.
- **TopoDroid ligado** (GATT) antes de TX.
- Um leg de cada vez, à velocidade dos ACKs; fila interna **32** — se o TopoDroid for lento, demora mas o ecrã não bloqueia.
- Sem SD: TX envia só os pontos em RAM (`pt_count`).

No MM1, SETUP → BT: MAC e estado; ícone BT verde quando a app está ligada (GATT).

### SETUP → BT (botões)

| Botão | Função |
|-------|--------|
| **Medir** | Uma medição (laser + IMU), grava na tabela e envia **um leg** SAP6 se o TopoDroid estiver ligado. Equivalente ao shot na app ou ao botão físico. |
| **TX** | Igual ao **TX** na aba POINTS: envia o CSV activo no SD (até 5000 legs) ou os pontos em RAM, com popup de progresso. |
| **Reiniciar BLE** | Reinicia advertising (`SAP6_0001`). |
| **Desemparelhar** | Apaga vínculos no MM1. |

**Não** há exportação de ficheiro CSV por BLE (antigo `LIST` / SPP). Use **SD**, aba **FILES** ou **SETUP → WiFi**.

## GATT

| UUID | Função |
|------|--------|
| `137c4435-8a64-4bcb-93f1-3792c6bdc965` | Serviço SurveyProtocol |
| `…c966` | Name (read) = `SAP6` |
| `…c967` | Command (write) — laser, shot, ACK |
| `…c968` | Leg (notify) — `<Bffff` LE: seq, az°, inc°, roll°, dist m |

## Robustez

- Fila de até **32** legs.
- Reenvio automático a cada **5 s** até ACK (como referência CaveBLE).
- Contadores em SETUP → BT: legs, ACK ok, erros, resends, fila.

## Ficheiros CSV

Transferência de ficheiros: **SD**, **SETUP → WiFi** (portal). Comandos texto SPP (`LIST`, `FILE_SEND`) foram removidos.

## Build

```bash
pio run -e denky32
# Nome BLE (tem de começar por SAP6_ para o TopoDroid):
# build_flags += -D BT_DEVICE_NAME='"SAP6_0001"'
```
