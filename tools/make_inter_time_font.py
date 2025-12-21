#!/usr/bin/env python3
"""
Generate a small "InterTime" TTF for Divinus OSD:
- Inter UI look (proportional)
- tabular digits WITHOUT a shaping engine (forces equal advance-width for 0..9)
- optional subsetting to just the time characters to reduce size

Why:
Divinus text renderer (schrift) does not apply OpenType features like tnum.
This tool bakes tabular digit metrics into the font so time strings don't "dance".

Usage:
  python3 tools/make_inter_time_font.py \
    --in misc/Inter-Regular.ttf \
    --out build/fonts/InterTime.ttf \
    --subset-time

Dependencies:
  pip install fonttools
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path


# Default charset for "time OSD + basic labels":
# - time: digits + separators
# - labels: Latin alphabet (both cases)
TIME_CHARS = "0123456789- :./ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"


def _make_tab_digits(font_path: str, out_path: str) -> None:
    from fontTools.ttLib import TTFont

    f = TTFont(font_path)
    cmap = f.getBestCmap()
    digit_glyphs = [cmap.get(ord(c)) for c in "0123456789"]
    digit_glyphs = [g for g in digit_glyphs if g]
    if not digit_glyphs:
        raise RuntimeError("No digit glyphs found in cmap; unexpected Inter build?")

    hmtx = f["hmtx"].metrics
    max_adv = max(hmtx[g][0] for g in digit_glyphs)
    for g in digit_glyphs:
        adv, lsb = hmtx[g]
        hmtx[g] = (max_adv, lsb)

    # Keep name tables etc. intact; this is a metric tweak only.
    f.save(out_path)


def _subset_ttf(font_path: str, out_path: str, text: str) -> None:
    from fontTools.ttLib import TTFont
    from fontTools import subset

    f = TTFont(font_path)
    opts = subset.Options()
    opts.name_IDs = ["*"]
    opts.name_legacy = True
    opts.name_languages = ["*"]
    opts.recalc_bounds = True
    opts.recalc_timestamp = False
    opts.flavor = "ttf"
    opts.with_zopfli = False

    sub = subset.Subsetter(options=opts)
    sub.populate(text=text)
    sub.subset(f)
    f.save(out_path)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True, help="Input TTF (e.g. misc/Inter-Regular.ttf)")
    ap.add_argument("--out", dest="out", required=True, help="Output TTF path (e.g. build/fonts/InterTime.ttf)")
    ap.add_argument("--subset-time", action="store_true", help="Subset to time characters only")
    ap.add_argument("--text", default=TIME_CHARS, help="Characters to keep when subsetting")
    args = ap.parse_args()

    inp = Path(args.inp)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)

    # Stage 1: bake tabular digits into a temp file.
    tmp = out.with_suffix(".tabdigits.ttf")
    _make_tab_digits(str(inp), str(tmp))

    # Stage 2: optionally subset.
    if args.subset_time:
        _subset_ttf(str(tmp), str(out), args.text)
        try:
            tmp.unlink()
        except OSError:
            pass
    else:
        os.replace(tmp, out)

    print(f"OK: wrote {out}")
    if args.subset_time:
        print(f"Subset text: {args.text!r}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


