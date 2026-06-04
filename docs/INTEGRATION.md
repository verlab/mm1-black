# Reintegração de features (base v0.6.1)

## O que funciona

- **`main.cpp` da tag v0.6.1** + `lv_conf.h` com layer **24 KB** → TX/SAVE/UI estáveis.

## O que travou (fase 1 — não repetir)

Juntar num só build:

- Tema escuro + `ui_apply_theme` após `build_ui`
- UI extra no Cal (geom) + Bright (theme)
- Save assíncrono + globals `File`
- `ucol_*` em todo o draw
- Correção IMU no `poll_imu`

Resultado: **qualquer toque** congela o ecrã (heap LVGL ou reentrância), não só TX.

**Causa provável:** `LV_MEM_SIZE` 40 KB já no limite na UI v0.6.1; widgets extra + refresh de tema esgotam heap no primeiro clique.

**Outra causa confirmada antes:** `LV_LAYER_SIMPLE_BUF_SIZE` 8 KB → freeze no popup STREAM (manter **24 KB**).

## Regra de ouro

1. Um flash = **uma** alteração.
2. Testar: toque em tabs → SAVE 50 pts → TX 50 pts.
3. Se falhar, `git show v0.6.1:src/main.cpp > src/main.cpp` e recomeçar.

## Ordem sugerida (próximos builds)

| Passo | Alteração | Risco |
|-------|-----------|-------|
| R0 | Só v0.6.1 `main.cpp` (recovery) | ✅ |
| R1 | `LV_MEM_SIZE` → 48 KB (só `lv_conf.h`) | ✅ |
| R2 | Só `mm1_distance_at_ref_m` em `add_point` (Bottom, trim 0) | ✅ |
| R3 | Prefs geom + SETUP Cal (Bottom/Top, trim, Default) | ✅ |
| R4 | Save em fatias no fim do `loop()` | ✅ |
| R5 | Tema escuro (sem refresh pesado no clique) | ✅ |
| R6 | Retrato / tab bar custom (HEAD) | alto — só se precisares |

## Save (quando chegar em R4)

- 4 pontos por `loop()`, **depois** de `lv_timer_handler()`.
- Nunca `lv_timer_handler()` dentro do write SD.
- Não correr save se `ble_csv_tx_busy()`.
