## Overlays Reference

### Usage Example

Parameters for each region can be passed in one or many calls:
```
curl "http://192.168.1.17/api/osd/0?font=comic&size=32.0&text=Frontdoor"
curl http://192.168.1.17/api/osd/0?posy=72
```
N.B. Ampersands require the address to be enclosed with double quotes under Windows or to be escaped with a backslash under Unix OSes

Supported fonts (sourced from /usr/share/fonts/truetype/) can render Unicode characters:
```
curl http://192.168.1.17/api/osd/0?text=Entrée
```

The text color is configurable (using a hexadecimal RGB555 representation) and can be made easier to read by applying an outline this way:
```
curl http://192.168.1.17/api/osd/1?color=%23FFFF&outl=%238000&thick=1.0
```
N.B. Hashtags have to be espaced with %23 in curl URL syntaxes

Empty strings are used to clear the regions:
```
curl http://192.168.1.17/api/osd/1?text=
```

Specifiers starting with a dollar sign are used to represent real-time statistics:
```
curl http://192.168.1.17/api/osd/1?text=$B%20C:$C%20M:$M
```
N.B. Spaces have to be escaped with %20 in curl URL syntaxes, $B can take an optional interface name preceded by a colon

Showing the time and customizing the time format is done this way:
```
curl http://192.168.1.17/api/time?fmt=%25Y/%25m/%25d%20%25H:%25M:%25S
curl http://192.168.1.17/api/osd/2?text=$t&posy=120
```
N.B. Percent signs have to be escaped with %25 in curl URL syntaxes

### Proportional UI font for time (tabular digits, no shaping engine)

Divinus' current text renderer does not apply OpenType features like `tnum` (tabular numbers).
If you want a modern proportional UI font (Inter-style) **without the time string width "dancing"**,
generate a small dedicated TTF with baked tabular digit metrics:

```
python3 -m pip install fonttools
python3 tools/make_inter_time_font.py --in misc/Inter-Regular.ttf --out build/fonts/InterTime.ttf --subset-time
```

By default, `--subset-time` keeps:
- digits and common separators for time/date
- **Latin alphabet (A–Z, a–z)** for short labels (e.g. CAM, FRONTDOOR)

You can override the kept character set with `--text=...` if you need extra symbols.

Copy the resulting `InterTime.ttf` onto the device (e.g. `/usr/share/fonts/truetype/InterTime.ttf`)
and set `reg0_font: InterTime` in `divinus.yaml`.

UTC date and time can be set using Unix timestamps:
```
curl http://192.168.1.17/api/time?ts=1712320920
```

24- and 32-bit bitmap files (.bmp) can be uploaded to a region using this command:
```
curl -F data=@.\Desktop\myimage.bmp http://192.168.1.17/api/osd/3
```
N.B. curl already implies "-X POST" when passing a file with "-F"

### Faceter logo mask (PNG)

This repository includes a ready-to-upload Faceter logo mask:
- `res/faceter_mask.png` (transparent background, opaque logo; gradient is baked into RGB)

Generate it (and the reference SVG) locally:
```
python3 tools/gen_faceter_osd.py
```

Upload to OSD region 3 and place it (example):
```
curl -F data=@res/faceter_mask.png "http://192.168.1.17/api/osd/3?posx=16&posy=16&opal=255"
```

Tip: to show a local image on boot, set `osd.regN_img` to a filesystem path (e.g. `/tmp/osd3.png`)
and keep `osd.regN_text` empty.