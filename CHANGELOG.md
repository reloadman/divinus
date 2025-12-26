# Divinus fork — technical changelog (for upstream developers)

This document describes **what changed in this fork** compared to the original upstream codebase located at:

- **Upstream baseline (local path)**: `/Users/romannaumenko/Work/Faceter/OpenIPC/orig_divinus/divinus/`
- **This fork (current branch working tree)**: `/Users/romannaumenko/Work/Faceter/OpenIPC/divinus/`

Scope/constraints:

- Only **current working tree code** was inspected (no git history; no assumptions).
- Diff excludes `.git/`, `build/` and `.DS_Store`.

High-level inventory (files):

- **Added**: 1191
- **Removed**: 1 (`divinus.yaml`)
- **Modified**: 65 (mostly `src/*` + build + docs)

## Summary of major functional changes

- **RTSP reworked**: integrated **smolrtsp + libevent** RTSP server (`src/rtsp_smol.c`) and switched runtime startup to it (`src/main.c`).
- **Audio pipeline changed**: upstream MP3 (Shine) path replaced by **AAC-LC (FAAC)**; added **SpeexDSP preprocess** (denoise/AGC/VAD/dereverb) for the AAC PCM path (`src/media.c`, `src/app_config.*`).
- **Config system replaced**: upstream “print YAML by hand + loose parsing” replaced by **libfyaml-based YAML parsing/emitting**; config path policy changed (`src/app_config.c`, `src/app_config.h`).
- **Night mode reworked**: safer GPIO handling, optional grayscale on IR, persisted manual mode, ISO/luma based switching (hisi/v4), and **IQ reload on mode switch** (`src/night.c`, `src/media.c`, `src/hal/hisi/v4_hal.*`).
- **Build system redesigned**: vendored deps under `3dparty/`, per-toolchain/per-float-ABI object caching, static linking of libevent, LTO enabled by default (`src/Makefile`, `build.sh`).

## Repository layout changes

### New top-level directories/files

- **`3dparty/`**: vendored third-party dependencies (see below).
- **`misc/`**: runtime assets and example configs:
  - `misc/divinus.yaml` (fork default config example)
  - `misc/imx307_openipc.ini`, `misc/iq_template_v4_min.ini` (IQ/ISP related templates)
  - `misc/Inter-Regular.ttf` (font asset)
  - `misc/S95divinus` (init/service script)
- **Board-specific configs**: `divinus.yaml.gk7205v200`, `divinus.yaml.infinity6b0` (upstream had a single `divinus.yaml`).
- **New sources**:
  - `src/rtsp_smol.c`, `src/rtsp_smol.h` (smolrtsp integration)
  - `src/single_instance.c`, `src/single_instance.h` (pidfile+flock single-instance lock)

### Vendored dependencies (`3dparty/`)

The fork vendors and builds several libraries from source (static):

- **RTSP / networking**
  - `3dparty/smolrtsp/` + `3dparty/smolrtsp-libevent/`: RTSP protocol implementation + libevent adapter.
  - `3dparty/libevent-2.1.12/`: event loop & socket I/O backend.
- **Audio**
  - `3dparty/faac/`: AAC-LC encoder (FAAC).
  - `3dparty/speexdsp/`: SpeexDSP preprocess (denoise/AGC/VAD/dereverb/resampler pieces).
- **Config parsing**
  - `3dparty/libfyaml/`: YAML 1.2 parser/emitter.
- **Header-only utilities used by smolrtsp/lib integration**
  - `3dparty/datatype99.h`, `3dparty/interface99.h`, `3dparty/slice99.h`
  - `3dparty/metalang99/` (macro meta-programming used by datatype99/interface99 ecosystem)

Notes:

- Some **`.o` files are present in the repo** (e.g. under `src/` and `3dparty/`). They appear to be build artifacts.

## Build system changes

### `build.sh`: toolchain management policy changed

Upstream behavior:

- Downloads toolchains on demand, keeps a `.previous` selector, builds single target.

Fork behavior (`build.sh`):

- **Offline** build helper: assumes toolchains already exist under `$TOOLCHAIN_ROOT` (default `$HOME/openipc/toolchain`).
- Supports building **all** targets from a fixed list (or one selected platform).
- Produces artifacts into `builds/divinus.<platform>` (e.g. `builds/divinus.infinity6b0`).
- Adds explicit **debug** mode toggling `OPT` and `LTO`.

### `src/Makefile`: from “compile everything” to staged + cached static deps

Upstream `src/Makefile`:

- Compiles all `*.c` into `*.o`, links with `-rdynamic`.

Fork `src/Makefile` highlights:

- **MP3(Shine) removed from build**:
  - Source list excludes `./lib/shine/*` and the Makefile explicitly states “MP3 (Shine) support removed”.
- Builds and caches (per toolchain + float ABI + LTO tag):
  - **FAAC** into `../build/faac/<toolchain>-<floatabi>-<ltoTag>/libfaac.a`
  - **libfyaml** into `../build/libfyaml/.../libfyaml.a`
  - **smolrtsp** and **smolrtsp-libevent** into cached `libsmolrtsp*.a`
  - **SpeexDSP** into `../build/speexdsp/.../libspeexdsp.a`
  - **libevent** built into `../build/libevent/...` and linked statically.
- **Default flags**:
  - `OPT` defaults to `-Os -pipe -s`
  - `LTO=1` by default (`-flto`), with guardrails about GCC LTO bytecode compatibility.
  - `-ffunction-sections -fdata-sections` + linker GC (`--gc-sections` on ELF; `-dead_strip` on Darwin)
- Replaces `-rdynamic` with a **whitelist of exported ISP symbols** (see `ISP_EXPORT_SYMS` block).
- Adds `CPPFLAGS += -DDIVINUS_WITH_SPEEXDSP` (SpeexDSP preprocessing is compiled in by default).

## Configuration system changes (YAML)

### Parser/emitter replacement

Upstream `src/app_config.c`:

- Searches for config in multiple locations (including adjacent to the executable via `/proc/self/exe`).
- Writes YAML text manually via `fprintf`.
- Supports restoring `.bak` configs via `restore_app_config()`.

Fork `src/app_config.c` + `src/app_config.h`:

- Uses **libfyaml** (`#include <libfyaml.h>`) for both parsing and emitting.
- Implements a minimal YAML path accessor (mapping-only `/a/b/c`), intentionally avoiding libfyaml ypath.
- **Config path policy changed**:
  - `DIVINUS_CONFIG_PATH` defaults to `"/etc/divinus.yaml"` (single fixed path; compile-time override).
  - The upstream fallback search logic was removed.
- `restore_app_config()` API is removed from the header and main shutdown path no longer calls it.
- Saving now builds a YAML document tree and emits it; special strings (e.g. `time_format` starting with `%`)
  are handled via **quoted scalar emission** to stay YAML-safe.

### Schema expansions (examples)

The fork expanded the YAML schema significantly. A curated list:

- **`system`**
  - `iq_config` (platform-specific IQ profile)
  - `web_bind`
  - `night_thread_stack_size`
- **`night_mode`**
  - `manual`, `grayscale`
  - `white_led_pin`
  - hisi/v4 ISP-driven triggers: `isp_lum_*`, `isp_iso_*`, `isp_exptime_low`, `isp_switch_lockout_s`
- **`isp`**
  - `sensor_mirror`, `sensor_flip` (factory orientation pre-transform)
- **`rtsp`**
  - `bind` (IP bind address)
- **`audio`**
  - `mute`, `codec`, `channels`
  - FAAC advanced knobs: `aac_quantqual`, `aac_bandwidth`, `aac_tns`
  - SpeexDSP preprocess knobs: `speex_*` (enable/denoise/agc/vad/dereverb + tuning)
- **`mp4`**
  - `mp4_h264_plus` (H.264+ mode)
- **`jpeg`**
  - schema and semantics changed: JPEG/MJPEG are handled under one section (see below).

For the current fork’s public reference, see `doc/config.md` (it was updated accordingly).

## RTSP changes

### Runtime RTSP server backend switched to smolrtsp

Upstream:

- Starts the built-in RTSP server via `rtsp_create()` from `src/rtsp/*` (`src/main.c`).

Fork:

- Starts a **smolrtsp + libevent** based server via `smolrtsp_server_start()` (`src/main.c`, `src/rtsp_smol.c`).
- New server characteristics (as implemented in `src/rtsp_smol.c`):
  - `MAX_CLIENTS` is set to **4** (separate from upstream’s RTSP module limits).
  - Supports H.264/H.265 NAL streaming; collects SPS/PPS/VPS from live bitstream to populate SDP.
  - Supports an AAC track (dynamic PT **97**).
  - Uses a bounded TCP output buffer (`RTSP_TCP_MAX_BUFFER`) and trims oldest interleaved frames when over limit.
  - Tracks audio timestamps as both microsecond and raw clock-domain counters to generate consistent RTP timestamps.

### RTSP authentication status

The fork’s config still contains `rtsp.enable_auth`, `rtsp.auth_user`, `rtsp.auth_pass` and ONVIF uses them
to embed credentials in the RTSP URL (`src/onvif.c`). However:

- In the **smolrtsp integration** (`src/rtsp_smol.c`) there is **no code path that enforces RTSP auth**
  (no parsing of `Authorization`, no `401`/`WWW-Authenticate` generation).
- The legacy RTSP implementation (`src/rtsp/*`) was extended with AAC helpers (see below), but it is no longer
  the server started from `main.c`.

If upstream expects RTSP auth parity, this is a **behavioral regression** relative to upstream’s legacy RTSP auth feature.

### Legacy RTSP module updates (still present in tree)

Even though the runtime server is now `smolrtsp`, the legacy RTSP headers/sources were modified:

- `src/rtsp/rtsp_server.h`: new `rtp_send_aac()` declaration.
- `src/rtsp/rtp.c`: implemented AAC payloadization (`pt=97`) with AU headers.

## Audio subsystem changes

### MP3 (Shine) removed, AAC (FAAC) added

Upstream audio path (`src/media.c`):

- Captures PCM, encodes MP3 via Shine (`lib/shine`), sends via HTTP and RTSP (`rtp_send_mp3()`).

Fork audio path (`src/media.c`, `src/Makefile`):

- Shine sources are excluded from compilation; FAAC is built and linked.
- Adds AAC encoder integration (`<faac.h>`) and RTSP push API for AAC (`smolrtsp_push_aac()`).

### SpeexDSP preprocessing for AAC PCM path

When compiled with `DIVINUS_WITH_SPEEXDSP` (default in fork Makefile):

- The PCM path into FAAC can be preprocessed by SpeexDSP:
  - denoise / AGC / VAD / optional dereverb
  - configurable frame size and tuning parameters exposed in YAML (`audio.speex_*`)
- Preprocess is explicitly scoped to **AAC + mono** (`channels == 1`) in code.

### Runtime audio “mute without removing track”

The fork implements **persistent mute** semantics:

- `audio.mute` keeps the audio pipeline alive and RTSP track present, but feeds **digital silence**
  to the encoder to avoid “comfort noise” (see `media_set_audio_mute()` and `g_audio_mute`).
- When muted (AAC bitrate mode), code optionally reduces bitrate to save bandwidth.

## Night mode / IR / IQ changes

Upstream night mode:

- Direct GPIO toggles; grayscale implied night mode; simple ADC/GPIO switching loop.

Fork night mode (`src/night.c`, `src/night.h`):

- Separates “IR mode” from “grayscale” (night mode now means IR-cut removed + IR LED on; grayscale is optional).
- Adds **white LED** pin support.
- Adds **startup IR-cut exercise** routine to unstick the filter (`night_ircut_exercise_startup()`).
- Adds interruptible sleeps to speed up shutdown/join.
- Adds safer config pin decoding rules: `999` disables; negative values treated as disabled; validates ranges.
- Adds ISP-driven auto switching (hisi/v4):
  - luminance-based with hysteresis (`isp_lum_low`, `isp_lum_hi`)
  - ISO-based with probe/lockout logic (`isp_iso_low`, `isp_iso_hi`, `isp_exptime_low`, `isp_switch_lockout_s`)
- Adds **IQ reload** on mode change via `media_reload_iq()` to immediately apply `static_*` vs `ir_static_*` sections.
- Adds persisted manual mode application early in startup (`main.c` calls `night_manual(app_config.night_mode_manual)`).

## ISP diagnostics & OSD

The fork introduced ISP readback helpers used for debugging overlays and API responses:

- `src/media.h` exports:
  - `get_isp_avelum()`, `get_isp_exposure_info()`, `get_isp_drc_strength()`, `get_isp_ae_auto_params()`
- Implementations are platform-dependent and exist in:
  - `src/hal/hisi/v4_hal.c` (`v4_get_isp_avelum()`, `v4_get_isp_exposure_info()`, etc.)
  - `src/hal/star/i6_hal.c`, `src/hal/star/i6c_hal.c`, `src/hal/star/m6_hal.c` (exposure info)
- OSD can optionally show ISP debug text (config: `osd.osd_isp_debug`; code: `src/region.c`).

### OSD text rendering changes (fonts, background box)

Compared to upstream, `src/text.c` was extended significantly:

- **Font loading behavior**:
  - Upstream loaded and freed the font file on each render.
  - Fork caches the last loaded font (`g_cached_font`) to avoid periodic stalls on embedded storage.
- **Background box support for text**:
  - Fork’s text renderer can crop to the tight bounding box of drawn pixels and apply a padded background fill
    (see config keys `osd.regX_bg`, `osd.regX_bgopal`, `osd.regX_pad` documented in `doc/config.md`).
- **API/signature changes**:
  - The `text_create_rendered(...)` signature was expanded to include background parameters.

### `time_format` sanitization / robustness

Time formatting for dynamic OSD (`timefmt` in `src/region.c`) is now treated as potentially tainted input:

- `src/app_config.c` introduces `timefmt_set()` and `timefmt_repair_runtime()`:
  - Sanitizes to printable ASCII, enforces null-termination, and falls back to `DEF_TIMEFMT` when empty/invalid.
  - Keeps a canonical copy to avoid persisting corrupted runtime buffer state back to YAML.
- `/api/time?fmt=...` updates both runtime and canonical copies (`src/server.c`).

## HTTP server & API changes

The HTTP server (`src/server.c`) diverged materially:

- Streaming types changed:
  - Upstream had `STREAM_MP3`; fork removed MP3 streaming type and introduced more granular counters:
    `server_pcm_clients`, `server_h26x_clients`, `server_mp4_clients`, `server_mjpeg_clients`.
  - Handlers now often short-circuit when there are **no subscribed clients**, to avoid blocking pipelines.
- Request buffer size changed: `REQSIZE` reduced from `512 * 1024` to `32 * 1024`.
- API surface expanded; see updated reference docs:
  - `doc/endpoints.md` (notably `/api/mjpeg`, `/api/night`, `/api/time`, `/api/audio` updates)
  - `doc/config.md` (schema)

Semantic changes worth calling out:

- **`/image.jpg`** in the fork returns the **last MJPEG frame** (cached in `src/media.c`), rather than
  requiring a separate JPEG encoder path.
- `jpeg` section now controls MJPEG/JPEG behavior; the standalone upstream `mjpeg:` config section is removed.

## Appendix: core modified files (non-vendored)

Modified (selected) core files include:

- **Build / docs**: `.gitignore`, `build.sh`, `doc/config.md`, `doc/endpoints.md`, `doc/overlays.md`, `res/index.html`, `src/Makefile`
- **Core runtime**: `src/main.c`, `src/server.c`, `src/media.c`, `src/night.c`, `src/onvif.c`
- **Config**: `src/app_config.c`, `src/app_config.h`
- **HAL**: `src/hal/config.*`, `src/hal/support.*`, multiple SoC HAL files (notably `src/hal/hisi/v4_*`)
- **Legacy RTSP module**: `src/rtsp/rtp.c`, `src/rtsp/rtcp.h`, `src/rtsp/rtsp.h`, `src/rtsp/rtsp_server.h`


