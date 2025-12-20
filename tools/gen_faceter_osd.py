#!/usr/bin/env python3
"""
Generate a Faceter logo OSD mask PNG from the upstream SVG path.

Why a generator:
- The project OSD pipeline accepts PNG/BMP, not SVG.
- Alpha in OSD is effectively 1-bit (see src/region.c: alpha & 0x80), so we emit
  a crisp mask (A=255 inside the logo, A=0 outside) and keep the gradient in RGB.

Outputs (by default):
- res/faceter.svg        : SVG with a gradient fill (reference / source of truth)
- res/faceter_osd.png    : RGBA PNG (transparent bg, opaque logo with gradient)

No third-party dependencies; uses only the Python standard library.
"""

from __future__ import annotations

import argparse
import math
import re
import struct
import zlib
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple


Point = Tuple[float, float]


FACETER_VIEWBOX = (0.0, 0.0, 560.0, 400.0)

# Path from https://static.cdnlogo.com/logos/f/87/faceter.svg (single <path>).
# We keep it verbatim to be able to re-emit as an SVG reference.
FACETER_PATH_D = (
    "m45.7 0h-21c-1.4 0-2.5 1.1-2.5 2.5v21.6h-7.6c-1.4 0-2.6 1-2.6 2.3s1.2 2.3 2.6 2.3h10.2"
    "c1.4 0 2.5-1.1 2.5-2.5v-11.1h12.4c1.3 0 2.3-1 2.3-2.3s-1-2.3-2.3-2.3h-12.5v-5.9h18.5"
    "c1.3 0 2.3-1 2.3-2.3s-1-2.3-2.3-2.3zm-20.8 36.4h-10.7c-1.3 0-2.3 1-2.3 2.3s1 2.3 2.3 2.3"
    "h10.7c1.3 0 2.3-1 2.3-2.3s-1-2.3-2.3-2.3zm-11.9-25.9h-10.7c-1.3 0-2.3 1-2.3 2.3s1 2.3 2.3 2.3"
    "h10.7c1.3 0 2.3-1 2.3-2.3 0-1.2-1-2.3-2.3-2.3z"
)

# Transform matrix from the upstream SVG: matrix(5.625 0 0 5.625 145 84.6875)
FACETER_TRANSFORM = (5.625, 0.0, 0.0, 5.625, 145.0, 84.6875)


def write_faceter_svg(path: str, colors: Sequence[str]) -> None:
    c0, c1, c2 = colors
    svg = f"""<svg clip-rule="evenodd" fill-rule="evenodd" stroke-linejoin="round" stroke-miterlimit="2" viewBox="0 0 560 400" xmlns="http://www.w3.org/2000/svg">
  <defs>
    <linearGradient id="faceterGrad" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0" stop-color="{c0}"/>
      <stop offset="0.55" stop-color="{c1}"/>
      <stop offset="1" stop-color="{c2}"/>
    </linearGradient>
  </defs>
  <path d="{FACETER_PATH_D}" fill="url(#faceterGrad)" fill-rule="nonzero"
        transform="matrix(5.625 0 0 5.625 145 84.6875)"/>
</svg>
"""
    with open(path, "w", encoding="utf-8") as f:
        f.write(svg)


def png_write_rgba(path: str, width: int, height: int, rgba: bytes) -> None:
    if len(rgba) != width * height * 4:
        raise ValueError("RGBA buffer size mismatch")

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)  # 8-bit RGBA

    # Filter type 0 per row
    stride = width * 4
    raw = bytearray()
    for y in range(height):
        raw.append(0)
        raw.extend(rgba[y * stride : (y + 1) * stride])
    comp = zlib.compress(bytes(raw), level=9)

    out = bytearray()
    out += sig
    out += chunk(b"IHDR", ihdr)
    out += chunk(b"IDAT", comp)
    out += chunk(b"IEND", b"")

    with open(path, "wb") as f:
        f.write(out)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def hex_to_rgb(h: str) -> Tuple[int, int, int]:
    h = h.strip()
    if h.startswith("#"):
        h = h[1:]
    if len(h) != 6:
        raise ValueError(f"Expected #RRGGBB, got {h!r}")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


def gradient_color(
    x: float,
    y: float,
    bbox: Tuple[float, float, float, float],
    colors: Sequence[Tuple[int, int, int]],
) -> Tuple[int, int, int]:
    """
    Diagonal gradient from top-left to bottom-right across bbox.
    colors: 3 stops.
    """
    minx, miny, maxx, maxy = bbox
    dx = maxx - minx
    dy = maxy - miny
    if dx <= 0 or dy <= 0:
        return colors[-1]
    u = (x - minx) / dx
    v = (y - miny) / dy
    t = clamp((u + v) * 0.5, 0.0, 1.0)

    # 3-stop: [0..0.55] c0->c1, [0.55..1] c1->c2
    if t <= 0.55:
        tt = t / 0.55
        c0, c1 = colors[0], colors[1]
        return (
            int(round(lerp(c0[0], c1[0], tt))),
            int(round(lerp(c0[1], c1[1], tt))),
            int(round(lerp(c0[2], c1[2], tt))),
        )
    else:
        tt = (t - 0.55) / 0.45
        c1, c2 = colors[1], colors[2]
        return (
            int(round(lerp(c1[0], c2[0], tt))),
            int(round(lerp(c1[1], c2[1], tt))),
            int(round(lerp(c1[2], c2[2], tt))),
        )


@dataclass
class Cubic:
    p0: Point
    p1: Point
    p2: Point
    p3: Point


def cubic_point(c: Cubic, t: float) -> Point:
    x0, y0 = c.p0
    x1, y1 = c.p1
    x2, y2 = c.p2
    x3, y3 = c.p3
    mt = 1.0 - t
    a = mt * mt * mt
    b = 3.0 * mt * mt * t
    d = 3.0 * mt * t * t
    e = t * t * t
    return (a * x0 + b * x1 + d * x2 + e * x3, a * y0 + b * y1 + d * y2 + e * y3)


def cubic_flatness(c: Cubic) -> float:
    """
    A simple flatness metric: distance of control points from the line p0->p3.
    """
    x0, y0 = c.p0
    x3, y3 = c.p3
    dx = x3 - x0
    dy = y3 - y0
    denom = math.hypot(dx, dy)
    if denom == 0:
        return 0.0

    def dist(p: Point) -> float:
        x, y = p
        # Distance from point to line through p0,p3
        return abs(dy * x - dx * y + x3 * y0 - y3 * x0) / denom

    return max(dist(c.p1), dist(c.p2))


def cubic_split(c: Cubic) -> Tuple[Cubic, Cubic]:
    """
    De Casteljau split at t=0.5.
    """
    def mid(a: Point, b: Point) -> Point:
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)

    p01 = mid(c.p0, c.p1)
    p12 = mid(c.p1, c.p2)
    p23 = mid(c.p2, c.p3)
    p012 = mid(p01, p12)
    p123 = mid(p12, p23)
    p0123 = mid(p012, p123)
    return (
        Cubic(c.p0, p01, p012, p0123),
        Cubic(p0123, p123, p23, c.p3),
    )


def cubic_flatten(c: Cubic, flatness: float) -> List[Point]:
    """
    Returns points INCLUDING c.p3, excluding c.p0 (caller should add start).
    """
    if cubic_flatness(c) <= flatness:
        return [c.p3]
    left, right = cubic_split(c)
    return cubic_flatten(left, flatness)[:-1] + cubic_flatten(right, flatness)


def apply_transform(p: Point, m: Tuple[float, float, float, float, float, float]) -> Point:
    a, b, c, d, e, f = m
    x, y = p
    return (a * x + c * y + e, b * x + d * y + f)


def tokenize_path(d: str) -> List[str]:
    # Commands or numbers, SVG allows separators as spaces/commas or implicit.
    return re.findall(r"[a-zA-Z]|[-+]?(?:\d*\.\d+|\d+)(?:[eE][-+]?\d+)?", d)


def parse_svg_path_to_polylines(d: str) -> List[List[Point]]:
    """
    Parses a subset of SVG path syntax sufficient for FACETER_PATH_D.
    Returns a list of closed polylines (each polyline ends at the start point).
    """
    toks = tokenize_path(d)
    i = 0
    cmd = None
    cur: Point = (0.0, 0.0)
    start: Optional[Point] = None
    sub: List[Point] = []
    subs: List[List[Point]] = []
    last_c2: Optional[Point] = None  # last cubic's second control point (for S/s)

    def read_num() -> float:
        nonlocal i
        v = float(toks[i])
        i += 1
        return v

    def close_subpath() -> None:
        nonlocal sub, start, cur, last_c2
        if start is not None and sub:
            if sub[-1] != start:
                sub.append(start)
            subs.append(sub)
        sub = []
        start = None
        last_c2 = None

    while i < len(toks):
        t = toks[i]
        if re.match(r"[a-zA-Z]", t):
            cmd = t
            i += 1
        if cmd is None:
            raise ValueError("Path missing initial command")

        if cmd in ("M", "m"):
            is_rel = cmd == "m"
            x = read_num()
            y = read_num()
            cur = (cur[0] + x, cur[1] + y) if is_rel else (x, y)
            close_subpath()
            start = cur
            sub = [cur]
            last_c2 = None
            # Subsequent pairs are implicit lineto
            cmd = "l" if is_rel else "L"
        elif cmd in ("L", "l"):
            is_rel = cmd == "l"
            while i < len(toks) and not re.match(r"[a-zA-Z]", toks[i]):
                x = read_num()
                y = read_num()
                cur = (cur[0] + x, cur[1] + y) if is_rel else (x, y)
                sub.append(cur)
            last_c2 = None
        elif cmd in ("H", "h"):
            is_rel = cmd == "h"
            while i < len(toks) and not re.match(r"[a-zA-Z]", toks[i]):
                x = read_num()
                nx = cur[0] + x if is_rel else x
                cur = (nx, cur[1])
                sub.append(cur)
            last_c2 = None
        elif cmd in ("V", "v"):
            is_rel = cmd == "v"
            while i < len(toks) and not re.match(r"[a-zA-Z]", toks[i]):
                y = read_num()
                ny = cur[1] + y if is_rel else y
                cur = (cur[0], ny)
                sub.append(cur)
            last_c2 = None
        elif cmd in ("C", "c"):
            is_rel = cmd == "c"
            while i < len(toks) and not re.match(r"[a-zA-Z]", toks[i]):
                x1, y1 = read_num(), read_num()
                x2, y2 = read_num(), read_num()
                x3, y3 = read_num(), read_num()
                p1 = (cur[0] + x1, cur[1] + y1) if is_rel else (x1, y1)
                p2 = (cur[0] + x2, cur[1] + y2) if is_rel else (x2, y2)
                p3 = (cur[0] + x3, cur[1] + y3) if is_rel else (x3, y3)
                c = Cubic(cur, p1, p2, p3)
                pts = cubic_flatten(c, flatness=0.35)
                sub.extend(pts)
                cur = p3
                last_c2 = p2
        elif cmd in ("S", "s"):
            is_rel = cmd == "s"
            while i < len(toks) and not re.match(r"[a-zA-Z]", toks[i]):
                x2, y2 = read_num(), read_num()
                x3, y3 = read_num(), read_num()
                if last_c2 is None:
                    p1 = cur
                else:
                    # Reflect last control point about current point
                    p1 = (2.0 * cur[0] - last_c2[0], 2.0 * cur[1] - last_c2[1])
                p2 = (cur[0] + x2, cur[1] + y2) if is_rel else (x2, y2)
                p3 = (cur[0] + x3, cur[1] + y3) if is_rel else (x3, y3)
                c = Cubic(cur, p1, p2, p3)
                pts = cubic_flatten(c, flatness=0.35)
                sub.extend(pts)
                cur = p3
                last_c2 = p2
        elif cmd in ("Z", "z"):
            # Close current subpath. Note: the command token is already consumed
            # by the generic "if token is a command" branch above.
            close_subpath()
            cmd = None
        else:
            raise ValueError(f"Unsupported SVG path command: {cmd}")

    close_subpath()
    return subs


def poly_bbox(polys: Sequence[Sequence[Point]]) -> Tuple[float, float, float, float]:
    xs: List[float] = []
    ys: List[float] = []
    for poly in polys:
        for x, y in poly:
            xs.append(x)
            ys.append(y)
    return (min(xs), min(ys), max(xs), max(ys))


def winding_number(point: Point, poly: Sequence[Point]) -> int:
    """
    Non-zero winding number for a point vs a closed polygon poly (last == first).
    """
    x, y = point
    wn = 0
    for i in range(len(poly) - 1):
        x0, y0 = poly[i]
        x1, y1 = poly[i + 1]
        if y0 <= y:
            if y1 > y and (x1 - x0) * (y - y0) - (x - x0) * (y1 - y0) > 0:
                wn += 1
        else:
            if y1 <= y and (x1 - x0) * (y - y0) - (x - x0) * (y1 - y0) < 0:
                wn -= 1
    return wn


def point_in_path(point: Point, polys: Sequence[Sequence[Point]]) -> bool:
    wn = 0
    for poly in polys:
        wn += winding_number(point, poly)
    return wn != 0


def render_faceter_png(
    out_path: str,
    width: int,
    pad: int,
    colors_hex: Sequence[str],
) -> None:
    # Parse + transform into viewBox coordinates
    polys = parse_svg_path_to_polylines(FACETER_PATH_D)
    polys = [[apply_transform(p, FACETER_TRANSFORM) for p in poly] for poly in polys]

    minx, miny, maxx, maxy = poly_bbox(polys)
    # Add padding in viewBox units (relative to desired pixel padding)
    bbox_w = maxx - minx
    bbox_h = maxy - miny

    # Target aspect ratio with padding in pixels
    scale = (width - 2 * pad) / bbox_w
    height = int(round(bbox_h * scale + 2 * pad))
    height = max(height, 1)

    # Build mapping from pixel->viewBox coords
    def pix_to_vb(px: float, py: float) -> Point:
        vx = minx + (px - pad + 0.5) / scale
        vy = miny + (py - pad + 0.5) / scale
        return (vx, vy)

    # Gradient in pixel space across the tight bbox (so it looks consistent when scaled)
    grad_bbox = (0.0, 0.0, float(width - 1), float(height - 1))
    grad_colors = [hex_to_rgb(c) for c in colors_hex]

    rgba = bytearray(width * height * 4)
    for py in range(height):
        for px in range(width):
            vx, vy = pix_to_vb(px, py)
            inside = point_in_path((vx, vy), polys)
            idx = (py * width + px) * 4
            if inside:
                r, g, b = gradient_color(float(px), float(py), grad_bbox, grad_colors)
                rgba[idx + 0] = r
                rgba[idx + 1] = g
                rgba[idx + 2] = b
                rgba[idx + 3] = 255
            else:
                rgba[idx + 0] = 0
                rgba[idx + 1] = 0
                rgba[idx + 2] = 0
                rgba[idx + 3] = 0

    png_write_rgba(out_path, width, height, bytes(rgba))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-svg", default="res/faceter.svg")
    ap.add_argument("--out-png", default="res/faceter_mask.png")
    ap.add_argument("--width", type=int, default=256)
    ap.add_argument("--pad", type=int, default=10)
    ap.add_argument(
        "--colors",
        default="#00C2FF,#2D7CFF,#0047FF",
        help="3 comma-separated hex colors for the gradient (e.g. #00C2FF,#2D7CFF,#0047FF)",
    )
    args = ap.parse_args()

    colors = [c.strip() for c in args.colors.split(",") if c.strip()]
    if len(colors) != 3:
        raise SystemExit("Expected exactly 3 colors")

    write_faceter_svg(args.out_svg, colors)
    render_faceter_png(args.out_png, args.width, args.pad, colors)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


