# Configuration Reference

This document describes the fields that can be found within a configuration file for Divinus.

## System section

- **sensor_config**: Path to the sensor calibration or configuration file, if applicable (e.g., `/etc/sensors/imx415.bin`).
- **iq_config**: Optional path to an ISP/IQ profile (INI on HiSilicon/Goke v4, BIN on Sigmastar i6/i6c/m6). When provided and the file exists, it is applied at startup instead of the SDK default `/etc/firmware/iqfile0.bin`.
- **web_port**: Port number for the web server (default: `80`).
- **web_bind**: IPv4 address to bind the web/API server to (default: all interfaces).
- **web_whitelist**: Array of up to 4 IP addresses or domains allowed to access the web server.
- **web_enable_auth**: Boolean to enable authentication on the API, live stream and WebUI endpoints (default: `false`).
- **web_auth_user**: Username for basic authentication (default: `admin`).
- **web_auth_pass**: Password for basic authentication (default: `12345`).
- **web_enable_static**: Boolean to enable serving static web content (default: `false`).
- **isp_thread_stack_size**: Stack size for ISP thread, if applicable (default: `16384`).
- **venc_stream_thread_stack_size**: Stack size for video encoding stream thread (default: `16384`).
- **web_server_thread_stack_size**: Stack size for web server thread (default: `65536`).
- **night_thread_stack_size**: Stack size for night mode worker thread (default: `65536`). Increase this if the device crashes on day/night switch (notably on some **hisi/v4** SDK builds where IQ reload is stack-hungry).
- **time_format**: Format for displaying time, refer to strftime() modifiers for exact parameters (e.g., `"%Y-%m-%d %H:%M:%S"`).
- **watchdog**: Watchdog timer in seconds, where 0 means disabled (default: `30`).

## Night mode section

- **enable**: Boolean to activate night mode support.
- **manual**: Boolean to persist manual mode (disables automatic switching when true).
- **grayscale**: Boolean to enable encoder grayscale when switching to night (IR) mode.
- **ir_cut_pin1**: GPIO number for IR cut filter control (normal state pin).
- **ir_cut_pin2**: GPIO number for IR cut filter control (inverted state pin).
- **ir_led_pin**: GPIO number for IR LED control.
- **white_led_pin**: GPIO number for white LED control (manual; optional).
- **ir_sensor_pin**: GPIO number for PIR motion sensor or similar digital toggle.
- **check_interval_s**: Interval in seconds to check night mode conditions.
- **pin_switch_delay_us**: Delay in microseconds before switching GPIO pins, must be used to protect cut filter coils from burning.
- **adc_device**: Path to the ADC device used for night mode.
- **adc_threshold**: Threshold raw value to trigger night mode, depends on the bitness of the given ADC device.
- **isp_lum_low**: Low threshold (0-255) for ISP-derived average luminance (hisi/v4 only). When `u8AveLum <= isp_lum_low`, switch to night mode.
- **isp_lum_hi**: High threshold (0-255) for ISP-derived average luminance (hisi/v4 only). When `u8AveLum >= isp_lum_hi`, switch back to day mode.
- **isp_iso_low**: Low threshold for ISP-derived ISO (hisi/v4 only). Used as the "exit night" threshold when ISO-based switching is enabled.
- **isp_iso_hi**: High threshold for ISP-derived ISO (hisi/v4 only). Used as the "enter night" threshold when ISO-based switching is enabled.
- **isp_exptime_low**: Low threshold for ISP-derived exposure time (hisi/v4 only). When enabled, the app will only attempt leaving IR mode (probe day) when `u32ExpTime <= isp_exptime_low`.
- **isp_switch_lockout_s**: Minimum time in seconds between automatic mode switches (hisi/v4 only). Helps prevent oscillations, especially when IR LEDs are very bright.

## ISP section

- **sensor_mirror**: Factory/default mirror applied at boot (before user mirror). Set per hardware variant (default: `false`).
- **sensor_flip**: Factory/default flip applied at boot (before user flip). Set per hardware variant (default: `false`).
- **mirror**: Boolean to turn on image mirroring (default: `false`).
- **flip**: Boolean to turn on image flipping (default: `false`).
- **antiflicker**: Antiflicker setting in Hz (default: `60`).

## mDNS section

- **enable**: Boolean to turn on mDNS announcer, lets LAN users access the device by a domain name, its configured hostname followed by .local (default: `false`).

## ONVIF section

- **enable**: Boolean to activate ONVIF services (default: `false`).
- **enable_auth**: Boolean to turn on ONVIF authentication (default: `false`).
- **auth_user**: Username for ONVIF authentication (default: `admin`).
- **auth_pass**: Password for ONVIF authentication (default: `12345`).

## RTSP section

- **enable**: Boolean to activate the integrated RTSP server (default: `true`).
- **enable_auth**: Boolean to turn on RTSP authentication (default: `false`).
- **auth_user**: Username for RTSP authentication (default: `admin`).
- **auth_pass**: Password for RTSP authentication (default: `12345`).
- **port**: Port number for RTSP server (default: `554`).
- **bind**: IPv4 address to bind the RTSP server to (default: all interfaces).

## Record section

- **enable**: Boolean to allow or block recording operations (default: `false`).
- **continuous**: Boolean to turn on continuous recording at launch (default: `false`).
- **path**: Path to save recordings (e.g., `/mnt/sdcard/recordings`).
- **filename**: String for a fixed destination file, leave empty to use incremental numbering
- **segment_duration**: Target duration for a recording in seconds
- **segment_size**: Target file size for a recording in bytes

## Stream section

- **enable**: Boolean to turn on special streaming methods (default: `false`).
- **udp_srcport**: Source port for UDP streaming (default: `5600`).
- **dest**: List of destination URLs for streaming (e.g., `udp://239.255.255.0:5600`).

## Audio section

- **enable**: Boolean to activate or deactivate audio functionality.
- **mute**: Boolean to mute audio while keeping the RTSP/audio track alive. When `true`, Divinus sends digital silence to the encoder (no track removal).
- **codec**: Audio codec, `MP3` (default) or `AAC` (AAC-LC).
- **bitrate**: Audio bitrate in kbps (e.g., `128`).
- **gain**: Audio input level.
  - On **hisi/v4**: `0..100` where `50` means unity gain, `<50` quieter, `>50` louder.
  - On other platforms: legacy **decibel** level (typically `-60..+30`).
- **srate**: Audio sampling rate in Hz (default `48000`).
- **channels**: Number of channels (1 mono, 2 stereo). Default `1`.
- **aac_quantqual**: FAAC quality setting (enables quality/VBR mode when `> 0`, range `10..5000` in this build). When set, FAAC ignores `bitrate` and uses `quantqual` instead.
- **aac_bandwidth**: FAAC encoder bandwidth in Hz (`0` = auto).
- **aac_tns**: Enable FAAC Temporal Noise Shaping (TNS) (`true/false`).
- **speex_enable**: Master switch for SpeexDSP preprocess on the **AAC** PCM path (`true/false`). When `false`, PCM goes directly to the encoder (bypass SpeexDSP to save CPU/memory).
- **speex_denoise**: Enable SpeexDSP denoiser (`true/false`).
- **speex_agc**: Enable SpeexDSP Automatic Gain Control (`true/false`).
- **speex_vad**: Enable SpeexDSP Voice Activity Detection (`true/false`).
- **speex_dereverb**: Enable SpeexDSP dereverb (`true/false`). This is expensive and upstream notes it as experimental; default is recommended `false`.
- **speex_frame_size**: PCM samples per channel per preprocess call. Typical values:
  - `160` @ `8000` Hz (20 ms)
  - `320` @ `16000` Hz (20 ms)
  - `960` @ `48000` Hz (20 ms)
  If set to `0` (or omitted), it is computed automatically as `srate / 50` (20 ms).
- **speex_noise_suppress_db**: Noise suppression level in dB (negative), typical `-15..-30`.
- **speex_agc_level**: AGC target level (SpeexDSP uses a float internally in floating-point builds), typical around `24000` (clamped to `1..32768`).
- **speex_agc_increment**: AGC maximum increase rate (SpeexDSP units).
- **speex_agc_decrement**: AGC maximum decrease rate (SpeexDSP units).
- **speex_agc_max_gain_db**: AGC maximum gain in dB.
- **speex_vad_prob_start**: VAD probability threshold to start speech (percent `0..100`).
- **speex_vad_prob_continue**: VAD probability threshold to continue speech (percent `0..100`).

## MP4 section

- **enable**: Boolean to activate MP4 output (required by live streams and RTSP).
- **codec**: Codec used for encoding (`H.264`, `H.265`, or `H.264+`).
- **codec = H.264+**: Enables "H.264+" style optimizations where supported by the current SoC/SDK
  (e.g. AVBR/EVBR, smart GOP, wider QP window, better entropy/preset, motion metadata).
- **mode**: Encoding mode.
- **width**: Video width in pixels.
- **height**: Video height in pixels.
- **fps**: Frames per second.
- **gop**: Interval between keyframes.
- **profile**: Encoding profile.
- **bitrate**: Bitrate in kbps.

## OSD section

- **enable**: Boolean to turn on On-Screen Display regions globally, used to reduce resource usage or let another app manage the functionality (default: `true`).
- **regX_img**: Path to the image for OSD region X.
- **regX_text**: Text displayed in OSD region X.
- **regX_font**: Font used for text in OSD region X. Prefer a **full path** to a font file (e.g. `/oem/usr/share/UbuntuMono-Regular.ttf`). For backward compatibility you can still use a font name (e.g. `UbuntuMono-Regular`) and Divinus will try to locate it in common font directories.
- **regX_opal**: Opacity of OSD region X.
- **regX_posx**: X position of OSD region X.
- **regX_posy**: Y position of OSD region X.
- **regX_size**: Size of the text or image in OSD region X.
- **regX_color**: Color of the text or image in OSD region X.
- **regX_outl**: Outline color of the text in OSD region X.
- **regX_thick**: Thickness of the text outline in OSD region X.
- **regX_bg**: Background box color behind the text (RGB555, `0..0x7FFF`). Used only when `regX_bgopal > 0`.
- **regX_bgopal**: Background box opacity (`0..255`). When `0` (default), background box is disabled.
- **regX_pad**: Padding in pixels around the text when `regX_bgopal > 0` (default: `6`).

Notes:
- If you only set `regX_text` (and omit styling fields), Divinus uses built-in defaults for
  **color/position/font/outline** so the OSD stays readable on bright backgrounds.

## JPEG section

This section controls the **MJPEG stream** (`/mjpeg`, `multipart/x-mixed-replace`).
The snapshot endpoint (`/image.jpg`) returns the **last MJPEG frame** (no separate JPEG encoder).

- **enable**: Boolean to activate MJPEG output.
- **osd_enable**: Boolean to enable/disable OSD for the JPEG/MJPEG stream only (default: `true`).
- **grayscale_night**: If `true`, the MJPEG/JPEG encoder channel follows `night_mode.grayscale` toggles (default: `true`).
- **mode**: Encoding mode (kept for compatibility; MJPEG uses `QP` internally).
- **width**: Video width in pixels.
- **height**: Video height in pixels.
- **fps**: Frames per second.
- **qfactor**: JPEG compression quality factor for the MJPEG stream.

## HTTP POST section

- **enable**: Boolean to activate HTTP POST snapshots.
- **host**: Hostname for HTTP POST requests.
- **url**: URL for HTTP POST requests.
- **login**: Login for HTTP POST authentication.
- **password**: Password for HTTP POST authentication.
- **width**: Image width sent in pixels.
- **height**: Image height sent in pixels.
- **interval**: Interval between requests in seconds.
- **qfactor**: JPEG compression quality factor for HTTP POST.