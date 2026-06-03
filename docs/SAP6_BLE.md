# SAP6 BLE (CaveBLE) no MM1-BLACK

Firmware em `src/sap6_ble.cpp` — substitui emulação **DistoX A3** (SPP) por **SAP6** ([CircuitPython_CaveBLE](https://github.com/furbrain/CircuitPython_CaveBLE)), o mesmo protocolo do **DiscoX** / **SAP6**.

## TopoDroid / SexyTopo

O MM1 expõe **Bluetooth Low Energy (BLE)**, não Bluetooth clássico (SPP). **Não** procure `SAP6_MM1` em *Definições → Bluetooth* do Android — esse ecrã só lista áudio/SPP; o SAP6 quase nunca aparece aí.

1. Instale **nRF Connect** (ou similar) e confirme que vê **`SAP6_MM1`** na lista BLE (scan ativo).
2. Na **TopoDroid** / **SexyTopo**: tipo de dispositivo **SAP6** / **DiscoX** / **Shetland Attack Pony 6** (não DistoX) e ligue pelo scan **dentro da app**.
3. Modo contínuo; cada medição no MM1 envia um **Leg** de 17 bytes; a app responde **ACK** `0x55` ou `0x56`.

No MM1, SETUP → BT deve mostrar MAC e estado; o ícone BT no topo fica verde quando a app está ligada.

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
# Nome BLE opcional:
# build_flags += -D BT_DEVICE_NAME=\"SAP6_MM1\"
```
