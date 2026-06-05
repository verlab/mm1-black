#!/usr/bin/env python3
"""Regenerate src/mira_splash_img.c from assets/MIRA_principal_R.png (requires Pillow).

Usage:
  python3 tools/gen_mira_splash.py            # landscape 480×320 (TFT_ROTATION 1/3)
  python3 tools/gen_mira_splash.py --portrait # portrait 320×480 (TFT_ROTATION 0/2)
"""

import argparse
import os
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PNG = os.path.join(ROOT, "assets", "MIRA_principal_R.png")
OUT = os.path.join(ROOT, "src", "mira_splash_img.c")
BG = (0, 0, 0)


def rgb565(r: int, g: int, b: int) -> int:
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def build_canvas(portrait: bool) -> tuple[Image.Image, int, int]:
    if portrait:
        sw, sh = 320, 480
        im = Image.open(PNG).convert("RGBA")
    else:
        sw, sh = 480, 320
        im = Image.open(PNG).convert("RGBA")
        im = im.transpose(Image.Transpose.ROTATE_90)

    iw, ih = im.size
    scale = min(sw / iw, sh / ih)
    nw = max(1, int(round(iw * scale)))
    nh = max(1, int(round(ih * scale)))
    im_r = im.resize((nw, nh), Image.Resampling.LANCZOS)

    canvas = Image.new("RGB", (sw, sh), BG)
    ox = (sw - nw) // 2
    oy = (sh - nh) // 2
    canvas.paste(im_r, (ox, oy), im_r.split()[3])
    return canvas, sw, sh


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--portrait",
        action="store_true",
        help="320×480 for TFT_ROTATION=0/2 (default firmware)",
    )
    args = parser.parse_args()

    canvas, sw, sh = build_canvas(args.portrait)
    orient = "portrait" if args.portrait else "landscape"

    blob = bytearray()
    px = canvas.load()
    for y in range(sh):
        for x in range(sw):
            r, g, b = px[x, y]
            v = rgb565(r, g, b)
            blob.append(v & 0xFF)
            blob.append((v >> 8) & 0xFF)

    lines = []
    row = []
    for b in blob:
        row.append(f"0x{b:02x}")
        if len(row) >= 24:
            lines.append("  " + ", ".join(row) + ",")
            row = []
    if row:
        lines.append("  " + ", ".join(row) + ",")

    header = f"""/**
 * MIRA boot splash — RGB565 raw {sw}×{sh} ({orient}, gen_mira_splash.py).
 * Regenerate: python3 tools/gen_mira_splash.py{" --portrait" if args.portrait else ""}
 */
#include <stdint.h>

#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

const LV_ATTRIBUTE_MEM_ALIGN uint8_t mira_splash_map[] = {{
"""

    footer = """
};
"""

    with open(OUT, "w", encoding="utf-8") as f:
        f.write(header)
        f.write("\n".join(lines))
        f.write("\n")
        f.write(footer)

    print(f"Wrote {OUT} ({len(blob)} bytes, {sw}×{sh} {orient})")


if __name__ == "__main__":
    main()
