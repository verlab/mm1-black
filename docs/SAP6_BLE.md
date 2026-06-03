# SAP6 BLE (CaveBLE) no MM1-BLACK

Firmware em `src/sap6_ble.cpp` — substitui emulação **DistoX A3** (SPP) por **SAP6** ([CircuitPython_CaveBLE](https://github.com/furbrain/CircuitPython_CaveBLE)), o mesmo protocolo do **DiscoX** / **SAP6**.

## TopoDroid / SexyTopo

1. Android → Bluetooth → emparelhar **`SAP6_MM1`** (PIN `0000` se pedido).
2. Na app: dispositivo tipo **SAP6** / **DiscoX** / **Shetland Attack Pony 6** (não DistoX).
3. Modo contínuo; cada medição no MM1 envia um **Leg** de 17 bytes; a app responde **ACK** `0x55` ou `0x56`.

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
