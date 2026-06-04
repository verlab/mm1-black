# CI/CD (GitHub Actions)

## CI (`ci.yml`)

Em cada push/PR para `main` ou `dev`:

- Ubuntu + `pip install platformio` (cache em `~/.platformio`)
- `pio run -e denky32`
- Artefacto `firmware-denky32` (`.pio/build/denky32/firmware.bin`)

## Release (`release.yml`)

Ao criar tag `v*` (ex. `v0.6.0`):

1. Build com `VERSION` = tag (macro `FW_VERSION` via `scripts/pio_firmware_version.py`)
2. Anexa ao GitHub Release:
   - `MM1-BLACK-denky32-vX.Y.Z.bin`
   - ficheiro `.sha256`
3. O badge **release** no README usa a API `releases/latest` (atualiza sozinho após cada tag).

## Versão no firmware

- Script `pre:scripts/pio_firmware_version.py`
- Header `include/firmware_version.h`
- UI: **SETUP → About**

## Retag de release

Para mover `v0.6.0` para um commit novo (após merge):

```bash
git tag -d v0.6.0
git push origin :refs/tags/v0.6.0
git tag -a v0.6.0 -m "v0.6.0"
git push origin v0.6.0
```

O workflow `release.yml` republica o binário no release.
