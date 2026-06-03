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

### 3. Se o scan não listar o MM1

- Confirme no **nRF Connect** que aparece **`SAP6_0001`** (nome padrão; pode mudar com `-DBT_DEVICE_NAME='"SAP6_1234"'` no `platformio.ini`, sempre prefixo `SAP6_` + 4 dígitos).
- Se o toast disser **“Device … without name”**, o Android não leu o nome no anúncio — atualize o firmware (nome no ADV + scan response) e tente de novo.
- Menu **Scan** outra vez com o MM1 a <2 m.
- Opcional: em Device, se aparecer diálogo **sem nome**, escolha modelo **`SAP6_`** e número **`0001`**.

### 4. Nome BLE e modelo na app

| Requisito TopoDroid | MM1 |
|---------------------|-----|
| Prefixo aceite no scan | `SAP6_` (ex.: `SAP6_0001`) |
| Alternativa | `discox_0001` (tipo DiscoX na app) |
| **Não** usar | DistoX A3 / SPP — firmware não suporta |

Modo contínuo na app; cada medição no MM1 envia um **Leg** de 17 bytes; a app responde **ACK** `0x55` ou `0x56`.

No MM1, SETUP → BT: MAC e estado; ícone BT verde quando a app está ligada (GATT).

## GATT

| UUID | Função |
|------|--------|
| `137c4435-8a64-4bcb-93f1-3792c6bdc965` | Serviço SurveyProtocol |
| `…c966` | Name (read) = `SAP6` |
| `…c967` | Command (write) — laser, shot, ACK |
| `…c968` | Leg (notify) — `<Bffff` LE: seq, az°, inc°, roll°, dist m |

## Robustez

- Fila de até 20 legs.
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
