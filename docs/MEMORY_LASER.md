# Memória — laser no MM1-BLACK

Dois mundos de hardware/protocolo aparecem em projetos “trena laser barata”. O firmware atual (`src/main.cpp`) implementa **apenas o protocolo M01 / BeneWake-style**.

Referência do módulo antigo X-40 (família diferente):  
[Laser_tape_reverse_engineering](https://github.com/iliasam/Laser_tape_reverse_engineering) · [Hackster / Arduino](https://www.hackster.io/iliasam/making-a-cheap-laser-rangefinder-for-arduino-4dd849) · [Habr (RU)](https://habr.com/post/327642/)

---

## 1. O que o MM1 usa hoje (M01 / UART binário 9600)

| Item | Valor |
|------|--------|
| Baud | **9600** 8N1 (`LZR_BAUD`) |
| Fio | Módulo **TX → ESP RX**, módulo **RX → ESP TX**, **3,3 V** |
| UART no CYD | Preferir UART dedicada: `LZR_SHARE_USB_UART=0`, RX=22 TX=21 |
| Frames | Cabeçalho **`0xAA`**, 13 bytes, checksum = soma bytes [1..n-2] |

### Comandos enviados (ESP → módulo)

| Comando | Bytes (resumo) | Função |
|---------|----------------|--------|
| `LASER_ON` | `AA 00 01 BE 00 01 00 01 C1` | Feixe vermelho (mira) |
| `LASER_OFF` | `AA 00 01 BE 00 01 00 00 C0` | Apaga feixe |
| `SINGLE` | `AA 00 00 20 …` | Uma medição (distância) |
| `CONTINUOUS` | func `0x21` | Stream de medições |
| `QUICK` + `READ_RES` | `0x22` + `0x80…` | Fallback se timeout |

### Resposta de distância (módulo → ESP)

- `f[0] == 0xAA`, checksum OK  
- `f[3]` ∈ {`0x20`, `0x21`, `0x22`} (single / continuous / quick)  
- `f[4]==0x00`, `f[5]==0x04` → distância em **BCD** 4 bytes em `f[6..9]`, escala **/1000 → metros**  
- Variável global: `lzr_last_m` (float m), `tof_dist_mm`, `tof_ok`

### Botão físico (BRIC5 / issue #2)

1. **1.ª pressão** — só `LASER_ON` (+ keepalive a cada ~2,5 s). **Sem** `SINGLE`, **sem** som.  
2. **2.ª pressão** — `play_capture_sound()` → `lzr_poll_send_measure()` (ON + SINGLE + poll) → grava ponto → `LASER_OFF` duas vezes.  
3. Se não houver frame válido (dist &lt; 0,01 m), **não** grava linha `0.000` na tabela.

### O que o MM1 **não** lê do laser

- Amplitude APD, temperatura, tensão APD (existem noutros produtos, não no parser M01 atual).  
- Texto ASCII tipo `DIST;…` (isso é outro módulo, ver §2).

---

## 2. Módulo X-40 “antigo” ([iliasam](https://github.com/iliasam/Laser_tape_reverse_engineering))

Placa interna **512A / 701A** (STM32F1 + Si5351 + APD + laser). O MCU **já calcula** a distância; o host só fala UART.

| Item | Valor |
|------|--------|
| Baud | **256000** (firmware `Firmware_dist_calculation_fast`) |
| Formato | **ASCII** linha fixa |
| Exemplo RX | `DIST;01574;AMP;0993;TEMP;1343;VOLT;082\r\n` |
| Campos | **DIST** mm, **AMP** amplitude, **TEMP** ADC temperatura APD, **VOLT** tensão APD |
| Comandos TX | **`E`** = liga laser + medição contínua · **`D`** = desliga · **`C`** = calibração zero (objeto branco &gt;10 cm) |
| Taxa | ~60 Hz distância contínua após `E` |

**Não é compatível** com o parser `0xAA` do MM1. Se o hardware for 701A/X-40 e o firmware M01, o sintoma típico é **distância sempre 0**, **sem `tof_ok`**, ou lixo UART.

Módulos mais novos (**M88B**, **U85**, **B2A**) usam outro MCU (STM32F0/G0) — também fora do firmware iliasam antigo e fora do MM1 atual.

---

## 3. Diagnóstico rápido “medidas zeradas”

| Sintoma | Causa provável |
|---------|----------------|
| `0.000` na tabela, coluna **E** | UART sem frames `0xAA` (cabo, baud, TX/RX trocados, USB partilhado com laser) |
| Laser acende mas distância 0 | Medição só com `SINGLE` sem sequência ON+poll; ou alvo inválido / timeout |
| Nada acende | Alimentação 3V3, `LZR_ENA_PIN`, ou módulo é X-40 (precisa `E` a 256000) |

### Checklist MM1 (M01)

1. `platformio.ini`: `LZR_SHARE_USB_UART=0` em campo; USB só para flash/log.  
2. Confirmar **9600** no módulo (não 115200 nem 256000).  
3. Aba **SENSOR**: deve atualizar distância ~1 Hz se UART OK.  
4. `g_bt` / debug: `lzr_rx_bytes_total` sobe ao medir?

---

## 4. Ficheiros no repositório

| Ficheiro | Papel |
|----------|--------|
| `src/main.cpp` | `lzr_*`, botão, `add_point`, `tof_*` |
| `platformio.ini` | `LZR_SHARE_USB_UART`, pinos via `-D` |
| `docs/MEMORY_LASER.md` | Este documento (memória de protocolo) |

## 5. Baud rate (9600 vs mais rápido)

O firmware usa **9600** (`LZR_BAUD`), padrão do módulo M01/BeneWake-style.

| Subir baud? | Quando faz sentido |
|-------------|-------------------|
| **Não** (recomendado) | Módulo veio em 9600; medição já limitada pelo tempo óptico do laser (~1 Hz na aba SENSOR), não pelo UART. |
| **Sim** | Só se o módulo foi **reconfigurado** no fabricante (ex. 115200 via `platformio.ini` `-D LZR_BAUD=115200`) **e** a UART dedicada não partilha USB (`LZR_SHARE_USB_UART=0`). |

Aumentar baud **não** corrige conflito USB+CH340 vs laser na mesma UART0. O gargalo típico é cabo/partilha de porta, não velocidade serial.

**Temperatura:** o protocolo M01 atual **não** expõe temp do laser no frame `0xAA`. O MM1 grava/mostra **temperatura do MCU ESP32** (`temperatureRead()`), já exportada na coluna Temperature do CSV TopoDroid.

Última revisão: captura botão BRIC5 + temp MCU na UI.
