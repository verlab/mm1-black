#!/usr/bin/env python3
"""Regenerate src/mira_splash_img.c from assets/MIRA_principal_R.png (requires Pillow)."""

import os
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PNG = os.path.join(ROOT, "assets", "MIRA_principal_R.png")
OUT = os.path.join(ROOT, "src", "mira_splash_img.c")

SW, SH = 480, 320
BG = (0, 0, 0)  # black splash letterbox (matches LVGL backdrop)


def rgb565(r: int, g: int, b: int) -> int:
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def main() -> None:
    im = Image.open(PNG).convert("RGBA")
    iw, ih = im.size
    scale = min(SW / iw, SH / ih)
    nw = max(1, int(round(iw * scale)))
    nh = max(1, int(round(ih * scale)))
    im_r = im.resize((nw, nh), Image.Resampling.LANCZOS)

    canvas = Image.new("RGB", (SW, SH), BG)
    ox = (SW - nw) // 2
    oy = (SH - nh) // 2
    canvas.paste(im_r, (ox, oy), im_r.split()[3])

    blob = bytearray()
    px = canvas.load()
    for y in range(SH):
        for x in range(SW):
            r, g, b = px[x, y]
            v = rgb565(r, g, b)
            blob.append(v & 0xFF)
            blob.append((v >> 8) & 0xFF)

    lines = []
    row = []
    for i, b in enumerate(blob):
        row.append(f"0x{b:02x}")
        if len(row) >= 24:
            lines.append("  " + ", ".join(row) + ",")
            row = []
    if row:
        lines.append("  " + ", ".join(row) + ",")

    header = """/**
 * MIRA boot splash — RGB565 for LVGL (generated from assets/MIRA_principal_R.png).
 * Regenerate: python3 tools/gen_mira_splash.py
 */
#include \"lvgl.h\"

#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

const LV_ATTRIBUTE_MEM_ALIGN uint8_t mira_splash_map[] = {
"""

    footer = f"""
}};

const lv_img_dsc_t mira_splash = {{
  .header.always_zero = 0,
  .header.w = {SW},
  .header.h = {SH},
  .data_size = sizeof(mira_splash_map),
  .header.cf = LV_IMG_CF_TRUE_COLOR,
  .data = mira_splash_map,
}};
"""

    with open(OUT, "w", encoding="utf-8") as f:
        f.write(header)
        f.write("\n".join(lines))
        f.write("\n")
        f.write(footer)

    print(f"Wrote {OUT} ({len(blob)} bytes bitmap)")


if __name__ == "__main__":
    main()
