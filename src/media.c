#include "media.h"
#include "hal/config.h"
#include <stdint.h>
#include <faac.h>
#if defined(DIVINUS_WITH_SPEEXDSP)
#include <speex/speex_preprocess.h>
#endif

char audioOn = 0, udpOn = 0;
pthread_mutex_t aencMtx, chnMtx, mp4Mtx;
pthread_t aencPid = 0, audPid = 0, ispPid = 0, vidPid = 0;

static hal_audcodec active_audio_codec;
// Global AAC encoder state is declared later in this file; forward-declare for helpers below.
extern faacEncHandle aacEnc;
extern unsigned int aacChannels;
// Runtime mute (keep track alive, send silence).
static volatile int g_audio_mute = 0;

// Protect FAAC encoder instance/config from concurrent use.
static pthread_mutex_t g_aac_enc_mtx = PTHREAD_MUTEX_INITIALIZER;
// Protect PCM preprocess + stash against concurrent reset (mute toggles).
static pthread_mutex_t g_aac_pcm_mtx = PTHREAD_MUTEX_INITIALIZER;

#if defined(DIVINUS_WITH_SPEEXDSP)
static void speex_aac_flush(void);
#endif

// When muted and using AAC bitrate mode, temporarily reduce bitrate to save network.
// 8 kbps mono is typically enough for "silence keepalive" and remains decodable.
#define AUDIO_MUTE_AAC_BITRATE_KBPS 32u

static inline int audio_is_muted(void) {
    return g_audio_mute != 0;
}

static inline bool isp_mirror_effective(void) {
    return app_config.sensor_mirror ^ app_config.mirror;
}

static inline bool isp_flip_effective(void) {
    return app_config.sensor_flip ^ app_config.flip;
}

int media_get_audio_mute(void) {
    return audio_is_muted();
}

// Cache the last MJPEG (JPEG) frame so snapshots (/image.jpg, http_post, etc)
// can be served without creating a dedicated JPEG encoder channel.
static pthread_mutex_t mjpeg_last_mtx = PTHREAD_MUTEX_INITIALIZER;
static unsigned char *mjpeg_last_buf = NULL;
static size_t mjpeg_last_cap = 0;
static size_t mjpeg_last_len = 0;
static uint64_t mjpeg_last_ts = 0;
static int mjpeg_last_valid = 0;

static void mjpeg_last_clear(void) {
    pthread_mutex_lock(&mjpeg_last_mtx);
    mjpeg_last_valid = 0;
    mjpeg_last_len = 0;
    mjpeg_last_ts = 0;
    pthread_mutex_unlock(&mjpeg_last_mtx);
}

static void mjpeg_last_update(const unsigned char *buf, size_t len, uint64_t ts) {
    if (!buf || !len)
        return;
    pthread_mutex_lock(&mjpeg_last_mtx);
    if (len > mjpeg_last_cap) {
        unsigned char *tmp = realloc(mjpeg_last_buf, len);
        if (!tmp) {
            pthread_mutex_unlock(&mjpeg_last_mtx);
            return;
        }
        mjpeg_last_buf = tmp;
        mjpeg_last_cap = len;
    }
    memcpy(mjpeg_last_buf, buf, len);
    mjpeg_last_len = len;
    mjpeg_last_ts = ts;
    mjpeg_last_valid = 1;
    pthread_mutex_unlock(&mjpeg_last_mtx);
}

int media_get_last_mjpeg_frame(hal_jpegdata *jpeg, unsigned int timeout_ms) {
    if (!jpeg)
        return EXIT_FAILURE;

    // Wait a bit for the first frame after startup/reconfigure.
    const unsigned int step_ms = 10;
    unsigned int waited = 0;
    while (waited <= timeout_ms) {
        pthread_mutex_lock(&mjpeg_last_mtx);
        const int valid = mjpeg_last_valid;
        const size_t len = mjpeg_last_len;
        // Snapshot state quickly, then copy outside? We need the buffer stable.
        // Keep the mutex while copying to guarantee coherent data.
        if (valid && len > 0 && mjpeg_last_buf) {
            if (len > jpeg->length) {
                unsigned char *tmp = realloc(jpeg->data, len);
                if (!tmp) {
                    pthread_mutex_unlock(&mjpeg_last_mtx);
                    return EXIT_FAILURE;
                }
                jpeg->data = tmp;
                jpeg->length = (unsigned int)len;
            }
            memcpy(jpeg->data, mjpeg_last_buf, len);
            jpeg->jpegSize = (unsigned int)len;
            pthread_mutex_unlock(&mjpeg_last_mtx);
            return EXIT_SUCCESS;
        }
        pthread_mutex_unlock(&mjpeg_last_mtx);

        if (waited == timeout_ms)
            break;
        usleep(step_ms * 1000);
        waited += step_ms;
        if (waited > timeout_ms)
            waited = timeout_ms;
    }

    return EXIT_FAILURE;
}

// Prevent unbounded growth if downstream (network/client) stalls.
// When exceeded, we drop queued audio to keep capture threads healthy.
#define AUDIO_ENC_BUF_MAX (32 * 1024)

// Simple ring buffers to avoid memmove() in hot audio paths.
typedef struct {
    uint8_t *buf;
    uint32_t size;   // capacity in bytes
    uint32_t rpos;   // read index
    uint32_t wpos;   // write index
    uint32_t offset; // used bytes (kept for compatibility with old code patterns)
} AudioByteRing;

typedef struct {
    int16_t *buf;
    uint32_t cap;    // capacity in samples (int16_t)
    uint32_t rpos;
    uint32_t wpos;
    uint32_t len;    // used samples
} PcmRing;

static inline void audio_ring_reset(AudioByteRing *q) {
    if (!q) return;
    q->rpos = 0;
    q->wpos = 0;
    q->offset = 0;
}

static inline int audio_ring_init(AudioByteRing *q, uint32_t cap) {
    if (!q) return 0;
    if (cap == 0) cap = AUDIO_ENC_BUF_MAX;
    if (q->buf && q->size == cap) {
        audio_ring_reset(q);
        return 1;
    }
    free(q->buf);
    q->buf = (uint8_t *)malloc(cap);
    q->size = q->buf ? cap : 0;
    audio_ring_reset(q);
    return q->buf != NULL;
}

static inline void audio_ring_free(AudioByteRing *q) {
    if (!q) return;
    free(q->buf);
    q->buf = NULL;
    q->size = 0;
    q->rpos = q->wpos = q->offset = 0;
}

static inline uint32_t audio_ring_space(const AudioByteRing *q) {
    return (!q || q->size < q->offset) ? 0 : (q->size - q->offset);
}

static inline int audio_ring_write(AudioByteRing *q, const void *data, uint32_t n) {
    if (!q || !q->buf || q->size == 0 || !data || n == 0) return 0;
    if (n > q->size) {
        // Single frame doesn't fit even in empty queue.
        audio_ring_reset(q);
        return 0;
    }
    if (audio_ring_space(q) < n) {
        // Drop queued audio and keep newest (matches old behavior).
        audio_ring_reset(q);
    }
    // Write in up to 2 segments (wrap).
    uint32_t first = q->size - q->wpos;
    if (first > n) first = n;
    memcpy(q->buf + q->wpos, data, first);
    if (n > first)
        memcpy(q->buf, (const uint8_t *)data + first, n - first);
    q->wpos = (q->wpos + n) % q->size;
    q->offset += n;
    return 1;
}

static inline int audio_ring_peek(const AudioByteRing *q, uint32_t off, void *out, uint32_t n) {
    if (!q || !q->buf || !out || n == 0) return 0;
    if (off + n > q->offset) return 0;
    uint32_t pos = (q->rpos + off) % q->size;
    uint32_t first = q->size - pos;
    if (first > n) first = n;
    memcpy(out, q->buf + pos, first);
    if (n > first)
        memcpy((uint8_t *)out + first, q->buf, n - first);
    return 1;
}

static inline int audio_ring_drop(AudioByteRing *q, uint32_t n) {
    if (!q || !q->buf || n == 0) return 1;
    if (n > q->offset) return 0;
    q->rpos = (q->rpos + n) % q->size;
    q->offset -= n;
    return 1;
}

static inline int audio_ring_read(AudioByteRing *q, void *out, uint32_t n) {
    if (!audio_ring_peek(q, 0, out, n)) return 0;
    return audio_ring_drop(q, n);
}

static inline uint16_t read_u16_le_bytes(const uint8_t b[2]) {
    return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

static inline uint64_t read_u64_le_bytes(const uint8_t b[8]) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++)
        v |= ((uint64_t)b[i]) << (8 * i);
    return v;
}

static inline void write_u16_le_bytes(uint8_t b[2], uint16_t v) {
    b[0] = (uint8_t)(v & 0xFF);
    b[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void write_u64_le_bytes(uint8_t b[8], uint64_t v) {
    for (int i = 0; i < 8; i++)
        b[i] = (uint8_t)((v >> (i * 8)) & 0xFF);
}

static inline void pcm_ring_reset(PcmRing *r) {
    if (!r) return;
    r->rpos = 0;
    r->wpos = 0;
    r->len = 0;
}

static inline int pcm_ring_init(PcmRing *r, uint32_t cap_samples) {
    if (!r) return 0;
    if (cap_samples == 0) return 0;
    if (r->buf && r->cap == cap_samples) {
        pcm_ring_reset(r);
        return 1;
    }
    free(r->buf);
    r->buf = (int16_t *)calloc(cap_samples, sizeof(int16_t));
    r->cap = r->buf ? cap_samples : 0;
    pcm_ring_reset(r);
    return r->buf != NULL;
}

static inline void pcm_ring_free(PcmRing *r) {
    if (!r) return;
    free(r->buf);
    r->buf = NULL;
    r->cap = 0;
    r->rpos = r->wpos = r->len = 0;
}

static inline int pcm_ring_write(PcmRing *r, const int16_t *samples, uint32_t n) {
    if (!r || !r->buf || r->cap == 0 || !samples || n == 0) return 0;
    if (n > r->cap) {
        // Keep the newest tail that fits.
        samples = samples + (n - r->cap);
        n = r->cap;
        pcm_ring_reset(r);
    } else if (r->cap - r->len < n) {
        // Drop queued samples; keep newest (matches old behavior).
        pcm_ring_reset(r);
    }
    uint32_t first = r->cap - r->wpos;
    if (first > n) first = n;
    memcpy(r->buf + r->wpos, samples, first * sizeof(int16_t));
    if (n > first)
        memcpy(r->buf, samples + first, (n - first) * sizeof(int16_t));
    r->wpos = (r->wpos + n) % r->cap;
    r->len += n;
    return 1;
}

static inline int pcm_ring_read(PcmRing *r, int16_t *out, uint32_t n) {
    if (!r || !r->buf || !out || n == 0) return 0;
    if (n > r->len) return 0;
    uint32_t first = r->cap - r->rpos;
    if (first > n) first = n;
    memcpy(out, r->buf + r->rpos, first * sizeof(int16_t));
    if (n > first)
        memcpy(out + first, r->buf, (n - first) * sizeof(int16_t));
    r->rpos = (r->rpos + n) % r->cap;
    r->len -= n;
    return 1;
}

static AudioByteRing aacBuf;
faacEncHandle aacEnc = NULL;
unsigned long aacInputSamples = 0;
unsigned long aacMaxOutputBytes = 0;
int16_t *aacPcm = NULL;      // encoder input buffer (16-bit LE PCM)
unsigned char *aacOut = NULL;
unsigned int aacPcmPos = 0;
unsigned int aacChannels = 1;
static PcmRing aacPcmStash;  // stash of incoming PCM (16-bit LE samples)

#if defined(DIVINUS_WITH_SPEEXDSP)
// SpeexDSP preprocess state for AAC PCM path (optional).
static struct {
    SpeexPreprocessState *st;
    unsigned int srate;
    unsigned int channels;
    unsigned int frame_size;      // samples per channel per preprocess run
    int active;
    PcmRing in;                   // input FIFO (mono)
    int16_t *frame;               // frame_size samples scratch for preprocess
} speex_aac = {0};
#endif

// --- Runtime mute / bitrate controls (AAC) ---
static void aac_apply_bitrate_kbps_locked(unsigned int kbps_total) {
    if (!aacEnc || aacChannels == 0)
        return;
    // Do not override VBR/quality mode.
    if (app_config.audio_aac_quantqual > 0)
        return;
    faacEncConfigurationPtr cfg = faacEncGetCurrentConfiguration(aacEnc);
    if (!cfg)
        return;
    cfg->quantqual = 0;
    cfg->bitRate = (unsigned long)((kbps_total * 1000u) / (unsigned int)aacChannels);
    // Keep user-configured bandwidth and TNS; bitrate mode only changes bitRate.
    cfg->bandWidth = app_config.audio_aac_bandwidth;
    // During mute, force-disable TNS to avoid audible artifacts on "silence".
    cfg->useTns = audio_is_muted() ? 0 : (app_config.audio_aac_tns ? 1 : 0);
    (void)faacEncSetConfiguration(aacEnc, cfg);
}

void media_set_audio_bitrate_kbps(unsigned int kbps) {
    // Update configured bitrate (used for encoder init / unmute restore).
    if (kbps > 0)
        app_config.audio_bitrate = kbps;

    pthread_mutex_lock(&g_aac_enc_mtx);
    if (active_audio_codec == HAL_AUDCODEC_AAC && aacEnc) {
        if (!audio_is_muted())
            aac_apply_bitrate_kbps_locked(app_config.audio_bitrate);
        else
            aac_apply_bitrate_kbps_locked(AUDIO_MUTE_AAC_BITRATE_KBPS);
    }
    pthread_mutex_unlock(&g_aac_enc_mtx);
}

void media_set_audio_mute(int muted) {
    const int was_muted = audio_is_muted();
    g_audio_mute = muted ? 1 : 0;

    // Flush any buffered pre-mute audio so the effect is immediate.
    if (!was_muted && audio_is_muted()) {
        pthread_mutex_lock(&g_aac_pcm_mtx);
        pcm_ring_reset(&aacPcmStash);
#if defined(DIVINUS_WITH_SPEEXDSP)
        speex_aac_flush();
#endif
        // Drop any already-encoded (pre-mute) AAC frames queued for network send.
        pthread_mutex_lock(&aencMtx);
        audio_ring_reset(&aacBuf);
        pthread_mutex_unlock(&aencMtx);
        pthread_mutex_unlock(&g_aac_pcm_mtx);
    }

    // Best-effort: reduce bitrate while muted (AAC bitrate mode only).
    pthread_mutex_lock(&g_aac_enc_mtx);
    if (active_audio_codec == HAL_AUDCODEC_AAC && aacEnc) {
        if (audio_is_muted())
            aac_apply_bitrate_kbps_locked(AUDIO_MUTE_AAC_BITRATE_KBPS);
        else
            aac_apply_bitrate_kbps_locked(app_config.audio_bitrate);
    }
    pthread_mutex_unlock(&g_aac_enc_mtx);
}

// Temporary buffers to send an extracted encoded frame without holding aencMtx.
static unsigned char *aac_send_buf = NULL;
static size_t aac_send_cap = 0;

static inline uint8_t aac_samplerate_index(uint32_t srate) {
    switch (srate) {
        case 96000: return 0;
        case 88200: return 1;
        case 64000: return 2;
        case 48000: return 3;
        case 44100: return 4;
        case 32000: return 5;
        case 24000: return 6;
        case 22050: return 7;
        case 16000: return 8;
        case 12000: return 9;
        case 11025: return 10;
        case 8000:  return 11;
        case 7350:  return 12;
        default:    return 3; // fallback to 48 kHz index
    }
}

static void *aenc_thread_aac(void);
static int save_audio_stream_aac(hal_audframe *frame);

static inline void aac_stash_append(const int16_t *pcm16, unsigned int total_samples) {
    if (total_samples == 0)
        return;
    (void)pcm_ring_write(&aacPcmStash, pcm16, (uint32_t)total_samples);
}

#if defined(DIVINUS_WITH_SPEEXDSP)
static void speex_aac_reset(void) {
    if (speex_aac.st) {
        speex_preprocess_state_destroy(speex_aac.st);
        speex_aac.st = NULL;
    }
    pcm_ring_free(&speex_aac.in);
    free(speex_aac.frame);
    speex_aac.frame = NULL;
    speex_aac.srate = 0;
    speex_aac.channels = 0;
    speex_aac.frame_size = 0;
    speex_aac.active = 0;
}

static void speex_aac_flush(void) {
    if (!speex_aac.active)
        return;
    pcm_ring_reset(&speex_aac.in);
}

static void speex_aac_init_from_config(unsigned int srate, unsigned int channels) {
    speex_aac_reset();

    // Use SpeexDSP only for AAC and mono input (simple, predictable CPU).
    if (!app_config.audio_speex_enable)
        return;
    if (channels != 1) {
        HAL_WARNING("media", "SpeexDSP preprocess is enabled but channels=%u; only mono is supported, bypassing.\n",
            channels);
        return;
        }

    unsigned int frame_size = app_config.audio_speex_frame_size;
    if (frame_size == 0)
        frame_size = srate / 50U; // 20 ms fallback
    if (frame_size < 80U)
        frame_size = 80U;
    if (frame_size > 8192U)
        frame_size = 8192U;

    SpeexPreprocessState *st = speex_preprocess_state_init((int)frame_size, (int)srate);
    if (!st) {
        HAL_ERROR("media", "SpeexDSP preprocess init failed (frame_size=%u srate=%u); bypassing.\n",
            frame_size, srate);
        return;
    }

    // Apply config. Most ctl() take spx_int32_t* in our fixed-point build.
    spx_int32_t v = 0;
    v = app_config.audio_speex_denoise ? 1 : 0;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_DENOISE, &v);
    v = app_config.audio_speex_agc ? 1 : 0;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC, &v);
    v = app_config.audio_speex_vad ? 1 : 0;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_VAD, &v);
    v = app_config.audio_speex_dereverb ? 1 : 0;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_DEREVERB, &v);

    v = (spx_int32_t)app_config.audio_speex_noise_suppress_db;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_NOISE_SUPPRESS, &v);
    {
        // SpeexDSP expects float* for AGC_LEVEL in floating-point builds.
        // Keep config type numeric and cast here.
        float agc_level = (float)app_config.audio_speex_agc_level;
        speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_LEVEL, &agc_level);
    }
    v = (spx_int32_t)app_config.audio_speex_agc_increment;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_INCREMENT, &v);
    v = (spx_int32_t)app_config.audio_speex_agc_decrement;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_DECREMENT, &v);
    v = (spx_int32_t)app_config.audio_speex_agc_max_gain_db;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_AGC_MAX_GAIN, &v);
    v = (spx_int32_t)app_config.audio_speex_vad_prob_start;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_PROB_START, &v);
    v = (spx_int32_t)app_config.audio_speex_vad_prob_continue;
    speex_preprocess_ctl(st, SPEEX_PREPROCESS_SET_PROB_CONTINUE, &v);

    // Allocate stash for incoming PCM. Keep a small multiple of frame size.
    unsigned int cap = frame_size * 8U;
    if (!pcm_ring_init(&speex_aac.in, cap)) {
        HAL_ERROR("media", "SpeexDSP ring allocation failed (%u samples); bypassing.\n", cap);
        speex_preprocess_state_destroy(st);
        return;
    }
    int16_t *frame = (int16_t *)malloc(frame_size * sizeof(int16_t));
    if (!frame) {
        HAL_ERROR("media", "SpeexDSP frame allocation failed (%u samples); bypassing.\n", frame_size);
        speex_preprocess_state_destroy(st);
        pcm_ring_free(&speex_aac.in);
        return;
            }

    speex_aac.st = st;
    speex_aac.srate = srate;
    speex_aac.channels = channels;
    speex_aac.frame_size = frame_size;
    speex_aac.frame = frame;
    speex_aac.active = 1;

    HAL_INFO("media", "SpeexDSP preprocess enabled for AAC: frame_size=%u srate=%u (denoise=%d agc=%d vad=%d dereverb=%d)\n",
        frame_size, srate,
        app_config.audio_speex_denoise ? 1 : 0,
        app_config.audio_speex_agc ? 1 : 0,
        app_config.audio_speex_vad ? 1 : 0,
        app_config.audio_speex_dereverb ? 1 : 0);
}

static inline void speex_aac_push_pcm(const int16_t *pcm16, unsigned int total_samples) {
    // During mute we want *true digital silence*. Some preprocess configurations
    // (especially AGC/VAD) can produce audible "comfort noise"/hiss even on a
    // near-silent input. Bypass SpeexDSP entirely when muted.
    if (audio_is_muted()) {
        aac_stash_append(pcm16, total_samples);
        return;
    }

    if (!speex_aac.active || !speex_aac.st || !speex_aac.in.buf || !speex_aac.frame || total_samples == 0) {
        aac_stash_append(pcm16, total_samples);
        return;
    }

    // Mono only in this integration.
    const unsigned int frame_samples = speex_aac.frame_size;
    if (frame_samples == 0) {
        aac_stash_append(pcm16, total_samples);
        return;
    }

    // Push into FIFO and process full frames.
    (void)pcm_ring_write(&speex_aac.in, pcm16, total_samples);
    while (speex_aac.in.len >= frame_samples) {
        if (!pcm_ring_read(&speex_aac.in, speex_aac.frame, frame_samples))
            break;
        speex_preprocess_run(speex_aac.st, (spx_int16_t *)speex_aac.frame);
        aac_stash_append(speex_aac.frame, frame_samples);
    }
}
#endif

void *aenc_thread(void) {
    // MP3 support removed; AAC-only.
    return aenc_thread_aac();
}

static void *aenc_thread_aac(void) {
    HAL_INFO("media", "AAC encode thread loop start\n");
    while (keepRunning && audioOn) {
        uint16_t frame_len = 0;
        uint64_t ts_us = 0;

        pthread_mutex_lock(&aencMtx);
        if (aacBuf.offset < sizeof(uint16_t) + sizeof(uint64_t)) {
            pthread_mutex_unlock(&aencMtx);
            usleep(10000);
            continue;
        }

        uint8_t hdr[2 + 8];
        if (!audio_ring_peek(&aacBuf, 0, hdr, sizeof(hdr))) {
            pthread_mutex_unlock(&aencMtx);
            usleep(10000);
            continue;
        }
        frame_len = read_u16_le_bytes(hdr);
        ts_us = read_u64_le_bytes(hdr + 2);

        if (frame_len == 0 || frame_len > aacMaxOutputBytes) {
            HAL_WARNING("media", "AAC frame_len invalid: %u queued=%u\n",
                frame_len, aacBuf.offset);
            audio_ring_reset(&aacBuf);
            pthread_mutex_unlock(&aencMtx);
            continue;
        }

        if (aacBuf.offset < frame_len + sizeof(uint16_t) + sizeof(uint64_t)) {
            pthread_mutex_unlock(&aencMtx);
            usleep(5000);
            continue;
        }

        if (frame_len > aac_send_cap) {
            unsigned char *nb = realloc(aac_send_buf, frame_len);
            if (!nb) {
                HAL_ERROR("media", "AAC send buffer realloc failed (%u)\n", frame_len);
                audio_ring_reset(&aacBuf);
                pthread_mutex_unlock(&aencMtx);
                continue;
            }
            aac_send_buf = nb;
            aac_send_cap = frame_len;
        }

        // Consume header + payload without memmove().
        (void)audio_ring_drop(&aacBuf, sizeof(uint16_t) + sizeof(uint64_t));
        if (!audio_ring_read(&aacBuf, aac_send_buf, frame_len)) {
            audio_ring_reset(&aacBuf);
            pthread_mutex_unlock(&aencMtx);
            continue;
        }
        pthread_mutex_unlock(&aencMtx);

        if (app_config.mp4_enable && (server_mp4_clients > 0 || recordOn)) {
        pthread_mutex_lock(&mp4Mtx);
        mp4_ingest_audio((char *)aac_send_buf, frame_len);
        pthread_mutex_unlock(&mp4Mtx);
        }

        if (app_config.rtsp_enable)
            smolrtsp_push_aac(aac_send_buf, frame_len, ts_us);
    }
    HAL_INFO("media", "Shutting down AAC encoding thread...\n");
    return NULL;
}

int save_audio_stream(hal_audframe *frame) {
    int ret = EXIT_SUCCESS;

#ifdef DEBUG_AUDIO
    printf("[audio] data:%p - %02x %02x %02x %02x %02x %02x %02x %02x\n", 
        frame->data, frame->data[0][0], frame->data[0][1], frame->data[0][2], frame->data[0][3],
        frame->data[0][4], frame->data[0][5], frame->data[0][6], frame->data[0][7]);
    printf("        len:%d\n", frame->length[0]);
    printf("        seq:%d\n", frame->seq);
    printf("        ts:%d\n", frame->timestamp);
#endif

    // If muted, zero PCM in-place before any downstream consumers.
    if (audio_is_muted() && frame && frame->data[0] && frame->length[0] > 0) {
        memset(frame->data[0], 0, (size_t)frame->length[0]);
    }

    // Avoid taking server mutex on every frame when nobody is subscribed to raw PCM.
    if (server_pcm_clients > 0)
        send_pcm_to_client(frame);

    // MP3 support removed; AAC-only.
        return save_audio_stream_aac(frame);
}

static int save_audio_stream_aac(hal_audframe *frame) {
    if (!aacEnc || !aacPcm || !aacOut) {
        HAL_ERROR("media", "AAC path not initialized (enc=%p pcm=%p out=%p)\n",
            (void*)aacEnc, (void*)aacPcm, (void*)aacOut);
        return EXIT_FAILURE;
    }

    unsigned int channels = frame->channelCnt ? (unsigned int)frame->channelCnt : aacChannels;
    if (channels == 0) channels = 1;

    // Ignore HAL channel changes; force configured channel count (mono) to keep timing correct.
    channels = aacChannels;

    // HAL PCM is 16-bit interleaved
    unsigned int samples_per_ch = frame->length[0] / (2 * channels);
    short *pcm16 = (short *)frame->data[0];
    static uint32_t last_ts = 0;
    static int log_cnt = 0;
    uint32_t delta_ts = frame->timestamp - last_ts;
        if (log_cnt < 3) {
        HAL_INFO("media", "AAC in frame len=%u samples/ch=%u ch=%u ts=%u dt=%u\n",
            frame->length[0], samples_per_ch, channels, frame->timestamp, delta_ts);
            log_cnt++;
        }
    last_ts = frame->timestamp;

    // Append to stash (16-bit LE samples, interleaved).
    unsigned int total_samples = samples_per_ch * channels;
    pthread_mutex_lock(&g_aac_pcm_mtx);
#if defined(DIVINUS_WITH_SPEEXDSP)
    speex_aac_push_pcm(pcm16, total_samples);
#else
    aac_stash_append(pcm16, total_samples);
#endif

    // Consume stash in blocks of aacInputSamples * channels (16-bit LE).
    const uint32_t need_samples = (uint32_t)aacInputSamples * channels;
    while (aacPcmStash.buf && aacPcmStash.len >= need_samples) {
        if (!pcm_ring_read(&aacPcmStash, aacPcm, need_samples))
            break;
        pthread_mutex_unlock(&g_aac_pcm_mtx);

        int bytes;
        pthread_mutex_lock(&g_aac_enc_mtx);
        bytes = faacEncEncode(aacEnc, (int32_t *)aacPcm, (unsigned int)aacInputSamples,
            aacOut, aacMaxOutputBytes);
        pthread_mutex_unlock(&g_aac_enc_mtx);
        static int log_fail = 0;
        static int log_ok = 0;
        if (bytes <= 0) {
            if (log_fail < 5) {
                HAL_WARNING("media", "faacEncEncode returned %d (ts=%u)\n",
                    bytes, frame->timestamp);
                log_fail++;
            }
            continue;
        }
        if ((unsigned int)bytes > aacMaxOutputBytes) {
            HAL_ERROR("media", "AAC frame %d exceeds buffer %lu\n",
                bytes, aacMaxOutputBytes);
            bytes = (int)aacMaxOutputBytes;
        }
        if (bytes > UINT16_MAX)
            bytes = UINT16_MAX;

        // RTP timestamp from stash base
        // Let RTSP layer derive timestamps from monotonic clock to avoid drift.
        uint64_t ts_us = 0;

        pthread_mutex_lock(&aencMtx);
        if (!aacBuf.buf || aacBuf.size != AUDIO_ENC_BUF_MAX)
            (void)audio_ring_init(&aacBuf, AUDIO_ENC_BUF_MAX);

        const uint32_t need = (uint32_t)bytes + sizeof(uint16_t) + sizeof(uint64_t);
        if (need > aacBuf.size) {
            audio_ring_reset(&aacBuf);
        } else {
            if (audio_ring_space(&aacBuf) < need)
                audio_ring_reset(&aacBuf);
            uint8_t hdr[2 + 8];
            write_u16_le_bytes(hdr, (uint16_t)bytes);
            write_u64_le_bytes(hdr + 2, ts_us);
            (void)audio_ring_write(&aacBuf, hdr, sizeof(hdr));
            (void)audio_ring_write(&aacBuf, aacOut, (uint32_t)bytes);
        }
        if (log_ok < 3) {
            HAL_INFO("media", "AAC encoded bytes=%d ts_calc=%llu\n",
                bytes, (unsigned long long)ts_us);
            log_ok++;
        }
        pthread_mutex_unlock(&aencMtx);

        pthread_mutex_lock(&g_aac_pcm_mtx);
    }
    pthread_mutex_unlock(&g_aac_pcm_mtx);

    return EXIT_SUCCESS;
}

int save_video_stream(char index, hal_vidstream *stream) {
    int ret;

    switch (chnState[index].payload) {
        case HAL_VIDCODEC_H264:
        case HAL_VIDCODEC_H265:
        {
            char isH265 = chnState[index].payload == HAL_VIDCODEC_H265 ? 1 : 0;

            const int have_h26x_clients = (server_h26x_clients > 0);
            const int have_mp4_clients = (server_mp4_clients > 0);
            const int do_mp4 = (app_config.mp4_enable && (have_mp4_clients || recordOn));

            if (do_mp4) {
                pthread_mutex_lock(&mp4Mtx);
                if (have_mp4_clients)
                send_mp4_to_client(index, stream, isH265);
                if (recordOn)
                    send_mp4_to_record(stream, isH265);
                pthread_mutex_unlock(&mp4Mtx);
                
                if (have_h26x_clients)
                    send_h26x_to_client(index, stream);
            } else if (app_config.mp4_enable && have_h26x_clients) {
                // Raw H.26x over HTTP (video.264/video.265) does not require MP4 muxing.
                send_h26x_to_client(index, stream);
            }
            if (app_config.rtsp_enable)
                for (int i = 0; i < stream->count; i++)
                    smolrtsp_push_video(
                        stream->pack[i].data + stream->pack[i].offset,
                        stream->pack[i].length - stream->pack[i].offset,
                        isH265,
                        stream->pack[i].timestamp);

            if (app_config.stream_enable && udp_stream_has_clients())
                for (int i = 0; i < stream->count; i++)
                    udp_stream_send_nal(stream->pack[i].data + stream->pack[i].offset, 
                        stream->pack[i].length - stream->pack[i].offset, 
                        stream->pack[i].nalu[0].type == NalUnitType_CodedSliceIdr, isH265);
            
            break;
        }
        case HAL_VIDCODEC_MJPG:
            if (app_config.jpeg_enable) {
                static char *mjpeg_buf;
                static ssize_t mjpeg_buf_size = 0;
                ssize_t buf_size = 0;
                const int want_mjpeg_stream = (server_mjpeg_clients > 0);
                for (unsigned int i = 0; i < stream->count; i++) {
                    hal_vidpack *data = &stream->pack[i];
                    // Keep a raw JPEG bitstream for snapshot cache; only reserve extra bytes
                    // for HTTP MJPEG streaming when there are active MJPEG clients.
                    ssize_t need_size = buf_size + data->length - data->offset;
                    if (need_size > mjpeg_buf_size)
                        mjpeg_buf = realloc(mjpeg_buf, mjpeg_buf_size = need_size);
                    memcpy(mjpeg_buf + buf_size, data->data + data->offset,
                        data->length - data->offset);
                    buf_size += data->length - data->offset;
                }
                // Keep a copy for snapshot users (jpeg_get when MJPEG is enabled).
                uint64_t ts = 0;
                if (stream->count)
                    ts = stream->pack[stream->count - 1].timestamp;
                mjpeg_last_update((unsigned char *)mjpeg_buf, (size_t)buf_size, ts);
                if (want_mjpeg_stream) {
                    // send_mjpeg_to_client appends "\r\n" in-place; ensure capacity for 2 bytes once.
                    if (buf_size + 2 > mjpeg_buf_size)
                        mjpeg_buf = realloc(mjpeg_buf, mjpeg_buf_size = buf_size + 2);
                send_mjpeg_to_client(index, mjpeg_buf, buf_size);
                }
            }
            break;
        case HAL_VIDCODEC_JPG:
        {
            static char *jpeg_buf;
            static ssize_t jpeg_buf_size = 0;
            ssize_t buf_size = 0;
            for (unsigned int i = 0; i < stream->count; i++) {
                hal_vidpack *data = &stream->pack[i];
                // send_jpeg_to_client appends "\r\n" in-place; reserve extra bytes once below.
                ssize_t need_size = buf_size + data->length - data->offset;
                if (need_size > jpeg_buf_size)
                    jpeg_buf = realloc(jpeg_buf, jpeg_buf_size = need_size);
                memcpy(jpeg_buf + buf_size, data->data + data->offset,
                    data->length - data->offset);
                buf_size += data->length - data->offset;
            }
            if (app_config.jpeg_enable) {
                if (buf_size + 2 > jpeg_buf_size)
                    jpeg_buf = realloc(jpeg_buf, jpeg_buf_size = buf_size + 2);
                send_jpeg_to_client(index, jpeg_buf, buf_size);
            }
            break;
        }
        default:
            return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

int start_streaming(void) {
    int ret = EXIT_SUCCESS;

    for (int i = 0; app_config.stream_dests[i] && *app_config.stream_dests[i]; i++) {
        if (STARTS_WITH(app_config.stream_dests[i], "udp://")) {
            char *endptr, *hostptr, *portptr, dst[16];
            unsigned short port = 0;
            long val;

            if (portptr = strrchr(app_config.stream_dests[i], ':')) {
                val = strtol(portptr + 1, &endptr, 10);
                if (endptr != portptr + 1)
                    port = (unsigned short)val;
                else {
                    if (portptr[2] != '/')
                        HAL_DANGER("media", "Invalid UDP port: %s, going with defaults!\n",
                            app_config.stream_dests[i]);
                }
            }

            hostptr = &app_config.stream_dests[i][6];
            if (portptr) {
                size_t hostlen = portptr - hostptr;
                if (hostlen > sizeof(dst) - 1) hostlen = sizeof(dst) - 1;
                strncpy(dst, hostptr, hostlen);
                dst[hostlen] = '\0';
            } else {
                strncpy(dst, hostptr, sizeof(dst) - 1);
                dst[sizeof(dst) - 1] = '\0';
            }

            if (!udpOn) {
                val = strtol(hostptr, &endptr, 10);
                if (endptr != hostptr && val >= 224 && val <= 239) {
                    if (udp_stream_init(app_config.stream_udp_srcport, dst))
                        udpOn = 1;
                    else return EXIT_FAILURE;
                } else {
                    if (udp_stream_init(app_config.stream_udp_srcport, NULL))
                        udpOn = 1;
                    else return EXIT_FAILURE;
                }
            }
            
            if (udp_stream_add_client(dst, port) != -1)
                HAL_INFO("media", "Starting streaming to %s...\n", app_config.stream_dests[i]);
        }
    }
}

void stop_streaming(void) {
    if (udpOn) {
        udp_stream_close();
        udpOn = 0;
    }
}

void request_idr(void) {
    signed char index = -1;
    pthread_mutex_lock(&chnMtx);
    for (int i = 0; i < chnCount; i++) {
        if (!chnState[i].enable) continue;
        if (chnState[i].payload != HAL_VIDCODEC_H264 &&
            chnState[i].payload != HAL_VIDCODEC_H265) continue;
        index = i;
        break;
    }
    if (index != -1) switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_video_request_idr(index); break;
        case HAL_PLATFORM_I6C: i6c_video_request_idr(index); break;
        case HAL_PLATFORM_M6:  m6_video_request_idr(index); break;
        case HAL_PLATFORM_RK:  rk_video_request_idr(index); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ak_video_request_idr(index); break;
        case HAL_PLATFORM_GM:  gm_video_request_idr(index); break;
        case HAL_PLATFORM_V1:  v1_video_request_idr(index); break;
        case HAL_PLATFORM_V2:  v2_video_request_idr(index); break;
        case HAL_PLATFORM_V3:  v3_video_request_idr(index); break;
        case HAL_PLATFORM_V4:  v4_video_request_idr(index); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_video_request_idr(index); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_video_request_idr(index); break;
#endif
    }  
    pthread_mutex_unlock(&chnMtx);
}

void set_grayscale(bool active) {
    pthread_mutex_lock(&chnMtx);
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_channel_grayscale(active); break;
        case HAL_PLATFORM_I6C: i6c_channel_grayscale(active); break;
        case HAL_PLATFORM_M6:  m6_channel_grayscale(active); break;
        case HAL_PLATFORM_RK:  rk_channel_grayscale(active); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ak_channel_grayscale(active); break;
        case HAL_PLATFORM_V1:  v1_channel_grayscale(active); break;
        case HAL_PLATFORM_V2:  v2_channel_grayscale(active); break;
        case HAL_PLATFORM_V3:  v3_channel_grayscale(active); break;
        case HAL_PLATFORM_V4:  v4_channel_grayscale(active); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_channel_grayscale(active); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_channel_grayscale(active); break;
#endif
    }
    pthread_mutex_unlock(&chnMtx);
}

int media_set_isp_orientation(bool mirror, bool flip) {
    int ret = EXIT_FAILURE;
    bool eff_mirror = app_config.sensor_mirror ^ mirror;
    bool eff_flip = app_config.sensor_flip ^ flip;
    pthread_mutex_lock(&chnMtx);
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  ret = i6_set_orientation(eff_mirror, eff_flip); break;
        case HAL_PLATFORM_I6C: ret = i6c_set_orientation(eff_mirror, eff_flip); break;
        case HAL_PLATFORM_M6:  ret = m6_set_orientation(eff_mirror, eff_flip); break;
        default: ret = EXIT_FAILURE; break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_V4:
            ret = v4_channel_set_orientation(eff_mirror, eff_flip, (char)app_config.mp4_fps, (char)app_config.jpeg_fps);
            break;
        default: ret = EXIT_FAILURE; break;
#else
        default: ret = EXIT_FAILURE; break;
#endif
    }
    pthread_mutex_unlock(&chnMtx);

    if (ret == 0)
        request_idr();

    return ret;
}

int media_reload_iq(void) {
    int ret = EXIT_FAILURE;
    pthread_mutex_lock(&chnMtx);
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        default: ret = EXIT_FAILURE; break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_V4:
            ret = v4_iq_reload();
            break;
        default: ret = EXIT_FAILURE; break;
#else
        default: ret = EXIT_FAILURE; break;
#endif
    }
    pthread_mutex_unlock(&chnMtx);

    if (ret == 0)
        request_idr();

    return ret;
}

int get_isp_avelum(unsigned char *lum) {
    if (!lum) return EXIT_FAILURE;
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:
        case HAL_PLATFORM_I6C:
        case HAL_PLATFORM_M6: {
            // Sigmastar: no direct "average luma" API is wired in our HAL today.
            // Provide a best-effort proxy derived from exposure time and gains, so
            // OSD ISP debug can show a stable 0..255 "brightness" indicator.
            //
            // darkness ~= exp_time * iso (bigger means darker). Map log2(darkness)
            // into 0..255, where higher is brighter.
            unsigned int iso = 0, exptime = 0, again = 0, dgain = 0, ispdgain = 0;
            int ismax = 0;
            if (get_isp_exposure_info(&iso, &exptime, &again, &dgain, &ispdgain, &ismax) != EXIT_SUCCESS)
                return EXIT_FAILURE;
            unsigned long long darkness = (unsigned long long)iso * (unsigned long long)exptime;
            if (darkness == 0) darkness = 1;

            // Integer log2 for 64-bit.
            unsigned int l2 = 0;
            unsigned long long v = darkness;
            while (v >>= 1) l2++;

            // Calibrated loosely so that typical bright scenes land high and
            // low-light scenes land near zero.
            // - l2 ~ 21 for (iso~2000, exptime~1000us)
            // - l2 ~ 31 for (iso~100000, exptime~20000us)
            int score = 255;
            int delta = (int)l2 - 20;
            if (delta < 0) delta = 0;
            score -= delta * 23; // ~11 steps to reach ~0
            if (score < 0) score = 0;
            if (score > 255) score = 255;
            *lum = (unsigned char)score;
            return EXIT_SUCCESS;
        }
#endif
#if defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_V4:
            return v4_get_isp_avelum(lum);
#endif
        default:
            return EXIT_FAILURE;
    }
}

int get_isp_exposure_info(unsigned int *iso, unsigned int *exp_time,
    unsigned int *again, unsigned int *dgain, unsigned int *ispdgain,
    int *exposure_is_max) {
    if (!iso || !exp_time || !again || !dgain || !ispdgain || !exposure_is_max)
        return EXIT_FAILURE;
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:
            return i6_get_isp_exposure_info(iso, exp_time, again, dgain, ispdgain, exposure_is_max);
        case HAL_PLATFORM_I6C:
            return i6c_get_isp_exposure_info(iso, exp_time, again, dgain, ispdgain, exposure_is_max);
        case HAL_PLATFORM_M6:
            return m6_get_isp_exposure_info(iso, exp_time, again, dgain, ispdgain, exposure_is_max);
#endif
#if defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_V4:
            return v4_get_isp_exposure_info(iso, exp_time, again, dgain, ispdgain, exposure_is_max);
#endif
        default:
            return EXIT_FAILURE;
    }
}

int get_isp_drc_strength(unsigned int *strength) {
    if (!strength) return EXIT_FAILURE;
    switch (plat) {
#if defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_V4:
            return v4_get_drc_strength(strength);
#endif
        default:
            return EXIT_FAILURE;
    }
}

int get_iq_lowlight_state(unsigned int iso, unsigned int exp_time, int *active) {
    if (!active) return EXIT_FAILURE;
    switch (plat) {
#if defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_V4:
            return v4_get_iq_lowlight_state(iso, exp_time, active);
#endif
        default:
            return EXIT_FAILURE;
    }
}

int get_isp_ae_auto_params(unsigned int *comp, unsigned int *expmax, unsigned int *sysgainmax) {
    if (!comp || !expmax || !sysgainmax) return EXIT_FAILURE;
    switch (plat) {
#if defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_V4:
            return v4_get_ae_auto_params(comp, expmax, sysgainmax);
#endif
        default:
            return EXIT_FAILURE;
    }
}

int take_next_free_channel(bool mainLoop) {
    pthread_mutex_lock(&chnMtx);
    for (int i = 0; i < chnCount; i++) {
        if (chnState[i].enable) continue;
        chnState[i].enable = true;
        chnState[i].mainLoop = mainLoop;
        pthread_mutex_unlock(&chnMtx);
        return i;
    }
    pthread_mutex_unlock(&chnMtx);
    return -1;
}

int create_channel(char index, short width, short height, char framerate, char jpeg) {
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  return i6_channel_create(index, width, height, jpeg);
        case HAL_PLATFORM_I6C: return i6c_channel_create(index, width, height, jpeg);
        case HAL_PLATFORM_M6:  return m6_channel_create(index, width, height, jpeg);
        case HAL_PLATFORM_RK:  return rk_channel_create(index, width, height,
            isp_mirror_effective(), isp_flip_effective());
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  return EXIT_SUCCESS;
        case HAL_PLATFORM_GM:  return EXIT_SUCCESS;
        case HAL_PLATFORM_V1:  return v1_channel_create(index, width, height,
            isp_mirror_effective(), isp_flip_effective(), framerate);
        case HAL_PLATFORM_V2:  return v2_channel_create(index, width, height,
            isp_mirror_effective(), isp_flip_effective(), framerate);
        case HAL_PLATFORM_V3:  return v3_channel_create(index, width, height,
            isp_mirror_effective(), isp_flip_effective(), framerate);
        case HAL_PLATFORM_V4:  return v4_channel_create(index, isp_mirror_effective(),
            isp_flip_effective(), framerate);
#elif defined(__mips__)
        case HAL_PLATFORM_T31: return t31_channel_create(index, width, height,
            framerate, jpeg);
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: return cvi_channel_create(index, width, height,
            isp_mirror_effective(), isp_flip_effective());
#endif
    }
}

int bind_channel(char index, char framerate, char jpeg) {
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  return i6_channel_bind(index, framerate);
        case HAL_PLATFORM_I6C: return i6c_channel_bind(index, framerate);
        case HAL_PLATFORM_M6:  return m6_channel_bind(index, framerate);
        case HAL_PLATFORM_RK:  return rk_channel_bind(index);
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  return ak_channel_bind(index);
        case HAL_PLATFORM_GM:  return gm_channel_bind(index);
        case HAL_PLATFORM_V1:  return v1_channel_bind(index);
        case HAL_PLATFORM_V2:  return v2_channel_bind(index);
        case HAL_PLATFORM_V3:  return v3_channel_bind(index);
        case HAL_PLATFORM_V4:  return v4_channel_bind(index);
#elif defined(__mips__)
        case HAL_PLATFORM_T31: return t31_channel_bind(index);
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: return cvi_channel_bind(index);
#endif
    }
}

int unbind_channel(char index, char jpeg) {
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  return i6_channel_unbind(index);
        case HAL_PLATFORM_I6C: return i6c_channel_unbind(index);
        case HAL_PLATFORM_M6:  return m6_channel_unbind(index);
        case HAL_PLATFORM_RK:  return rk_channel_unbind(index);
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  return ak_channel_unbind(index);
        case HAL_PLATFORM_GM:  return gm_channel_unbind(index);
        case HAL_PLATFORM_V1:  return v1_channel_unbind(index);
        case HAL_PLATFORM_V2:  return v2_channel_unbind(index);
        case HAL_PLATFORM_V3:  return v3_channel_unbind(index);
        case HAL_PLATFORM_V4:  return v4_channel_unbind(index);
#elif defined(__mips__)
        case HAL_PLATFORM_T31: return t31_channel_unbind(index);
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: return cvi_channel_unbind(index);
#endif
    }
}

int disable_video(char index, char jpeg) {
    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  return i6_video_destroy(index);
        case HAL_PLATFORM_I6C: return i6c_video_destroy(index);
        case HAL_PLATFORM_M6:  return m6_video_destroy(index);
        case HAL_PLATFORM_RK:  return rk_video_destroy(index);
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  return ak_video_destroy(index);
        case HAL_PLATFORM_GM:  return gm_video_destroy(index);
        case HAL_PLATFORM_V1:  return v1_video_destroy(index);
        case HAL_PLATFORM_V2:  return v2_video_destroy(index);
        case HAL_PLATFORM_V3:  return v3_video_destroy(index);
        case HAL_PLATFORM_V4:  return v4_video_destroy(index);
#elif defined(__mips__)
        case HAL_PLATFORM_T31: return t31_video_destroy(index);
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: return cvi_video_destroy(index);
#endif
    }    
    return 0;
}

void disable_audio(void) {
    if (!audioOn) return;

    audioOn = 0;

    pthread_join(aencPid, NULL);
    pthread_join(audPid, NULL);
    {
#if defined(DIVINUS_WITH_SPEEXDSP)
        speex_aac_reset();
#endif
        pthread_mutex_lock(&g_aac_enc_mtx);
        if (aacEnc)
            faacEncClose(aacEnc);
        aacEnc = NULL;
        pthread_mutex_unlock(&g_aac_enc_mtx);
        free(aacPcm);
        free(aacOut);
        pcm_ring_free(&aacPcmStash);
        audio_ring_free(&aacBuf);
        free(aac_send_buf);
        aac_send_buf = NULL;
        aac_send_cap = 0;
        aacPcm = NULL;
        aacOut = NULL;
        aacInputSamples = 0;
        aacMaxOutputBytes = 0;
        aacPcmPos = 0;
    }
    active_audio_codec = HAL_AUDCODEC_UNSPEC;

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_audio_deinit(); break;
        case HAL_PLATFORM_I6C: i6c_audio_deinit(); break;
        case HAL_PLATFORM_M6:  m6_audio_deinit(); break;
        case HAL_PLATFORM_RK:  rk_audio_deinit(); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_GM:  gm_audio_deinit(); break;
        case HAL_PLATFORM_V1:  v1_audio_deinit(); break;
        case HAL_PLATFORM_V2:  v2_audio_deinit(); break;
        case HAL_PLATFORM_V3:  v3_audio_deinit(); break;
        case HAL_PLATFORM_V4:  v4_audio_deinit(); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_audio_deinit(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_audio_deinit(); break;
#endif
    }
}

int enable_audio(void) {
    int ret = EXIT_SUCCESS;

    if (audioOn) return ret;

    // MP3 support removed; AAC-only.
    active_audio_codec = HAL_AUDCODEC_AAC;
    aacChannels = app_config.audio_channels ? app_config.audio_channels : 1;
    if (aacChannels > 2) aacChannels = 2;
    HAL_INFO("media", "Audio init: codec=%s srate=%u bitrate=%u channels=%u gain=%d\n",
        "AAC",
        app_config.audio_srate, app_config.audio_bitrate,
        aacChannels,
        app_config.audio_gain);

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  ret = i6_audio_init(app_config.audio_srate, app_config.audio_gain); break;
        case HAL_PLATFORM_I6C: ret = i6c_audio_init(app_config.audio_srate, app_config.audio_gain); break;
        case HAL_PLATFORM_M6:  ret = m6_audio_init(app_config.audio_srate, app_config.audio_gain); break;
        case HAL_PLATFORM_RK:  ret = rk_audio_init(app_config.audio_srate); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_GM:  ret = gm_audio_init(app_config.audio_srate); break;
        case HAL_PLATFORM_V1:  ret = v1_audio_init(app_config.audio_srate); break;
        case HAL_PLATFORM_V2:  ret = v2_audio_init(app_config.audio_srate); break;
        case HAL_PLATFORM_V3:  ret = v3_audio_init(app_config.audio_srate); break;
        case HAL_PLATFORM_V4:  ret = v4_audio_init(app_config.audio_srate, app_config.audio_gain); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: ret = t31_audio_init(app_config.audio_srate); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: ret = cvi_audio_init(app_config.audio_srate); break;
#endif
    }
    if (ret)
        HAL_ERROR("media", "Audio initialization failed with %#x!\n%s\n",
            ret, errstr(ret));

    (void)audio_ring_init(&aacBuf, AUDIO_ENC_BUF_MAX);
    aacEnc = faacEncOpen(app_config.audio_srate, aacChannels,
            &aacInputSamples, &aacMaxOutputBytes);
        if (!aacEnc) {
            HAL_ERROR("media", "AAC encoder initialization failed!\n");
            return EXIT_FAILURE;
        }
        HAL_INFO("media", "faacEncOpen ok: inputSamples=%lu maxOut=%lu\n",
            aacInputSamples, aacMaxOutputBytes);

        faacEncConfigurationPtr cfg = faacEncGetCurrentConfiguration(aacEnc);
        cfg->aacObjectType = LOW;
        cfg->mpegVersion = MPEG4;
    cfg->useTns = app_config.audio_aac_tns ? 1 : 0;
        cfg->allowMidside = aacChannels > 1;
        cfg->outputFormat = 0; // raw AAC-LC frames
    // FAAC supports two modes:
    // - bitrate mode: bitRate > 0 (per-channel), quantqual == 0
    // - quality/VBR mode: quantqual > 0, bitRate == 0
    cfg->bandWidth = app_config.audio_aac_bandwidth; // 0 = auto
    if (app_config.audio_aac_quantqual > 0) {
        cfg->quantqual = app_config.audio_aac_quantqual;
        cfg->bitRate = 0;
    } else {
        cfg->quantqual = 0;
        cfg->bitRate = app_config.audio_bitrate * 1000 / aacChannels;
    }
        cfg->inputFormat = FAAC_INPUT_16BIT;
        if (!faacEncSetConfiguration(aacEnc, cfg)) {
            HAL_ERROR("media", "AAC encoder configuration failed!\n");
            return EXIT_FAILURE;
        }

    aacPcm = calloc(aacInputSamples * aacChannels, sizeof(int16_t));
        aacOut = malloc(aacMaxOutputBytes);
    if (!pcm_ring_init(&aacPcmStash, (uint32_t)aacInputSamples * aacChannels * 4U)) {
        HAL_ERROR("media", "AAC PCM stash ring allocation failed!\n");
        return EXIT_FAILURE;
    }
    if (!aacPcm || !aacOut) {
            HAL_ERROR("media", "AAC encoder buffer allocation failed!\n");
            return EXIT_FAILURE;
        }
    pcm_ring_reset(&aacPcmStash);
        HAL_INFO("media", "AAC buffers allocated: pcm=%p out=%p\n", (void*)aacPcm, (void*)aacOut);

#if defined(DIVINUS_WITH_SPEEXDSP)
    // Optional SpeexDSP preprocess for AAC PCM path (denoise/AGC/VAD).
    speex_aac_init_from_config(app_config.audio_srate, aacChannels);
#endif

    // IMPORTANT: audio capture/encode threads spin on `while (keepRunning && audioOn)`.
    // Set audioOn BEFORE starting threads, otherwise they may exit immediately.
    audioOn = 1;

    {
        pthread_attr_t thread_attr;
        pthread_attr_init(&thread_attr);
        size_t stacksize;
        pthread_attr_getstacksize(&thread_attr, &stacksize);
        size_t new_stacksize = 64 * 1024;
        if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
            HAL_DANGER("media", "Can't set stack size %zu\n", new_stacksize);
        if (!aud_thread) {
            HAL_ERROR("media", "Audio capture thread pointer is NULL!\n");
        } else if (pthread_create(
                        &audPid, &thread_attr, (void *(*)(void *))aud_thread, NULL)) {
            HAL_ERROR("media", "Starting the audio capture thread failed!\n");
        } else {
            HAL_INFO("media", "Audio capture thread started (aud_thread=%p)\n",
                aud_thread);
        }
        if (pthread_attr_setstacksize(&thread_attr, stacksize))
            HAL_DANGER("media", "Can't set stack size %zu\n", stacksize);
        pthread_attr_destroy(&thread_attr);
    }

    {
        pthread_attr_t thread_attr;
        pthread_attr_init(&thread_attr);
        size_t stacksize;
        pthread_attr_getstacksize(&thread_attr, &stacksize);
        size_t new_stacksize = 64 * 1024;
        if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
            HAL_DANGER("media", "Can't set stack size %zu\n", new_stacksize);
        if (pthread_create(
                        &aencPid, &thread_attr, (void *(*)(void *))aenc_thread, NULL))
            HAL_ERROR("media", "Starting the audio encoding thread failed!\n");
        else
            HAL_INFO("media", "Audio encoding thread started (codec=AAC)\n");
        if (pthread_attr_setstacksize(&thread_attr, stacksize))
            HAL_DANGER("media", "Can't set stack size %zu\n", stacksize);
        pthread_attr_destroy(&thread_attr);
    }

    return ret;
}

int disable_mjpeg(void) {
    int ret;

    // If MJPEG is being stopped/reconfigured, drop the cached snapshot.
    mjpeg_last_clear();

    for (char i = 0; i < chnCount; i++) {
        if (!chnState[i].enable) continue;
        if (chnState[i].payload != HAL_VIDCODEC_MJPG) continue;

        if (ret = unbind_channel(i, 1))
            HAL_ERROR("media", "Unbinding channel %d failed with %#x!\n%s\n", 
                i, ret, errstr(ret));

        if (ret = disable_video(i, 1))
            HAL_ERROR("media", "Disabling encoder %d failed with %#x!\n%s\n", 
                i, ret, errstr(ret));
    }

    return EXIT_SUCCESS;
}

int enable_mjpeg(void) {
    int ret;

    int index = take_next_free_channel(true);

    if (ret = create_channel(index, app_config.jpeg_width,
        app_config.jpeg_height, app_config.jpeg_fps, 1))
        HAL_ERROR("media", "Creating channel %d failed with %#x!\n%s\n", 
            index, ret, errstr(ret));

    {
        hal_vidconfig config = {0};
        config.width = app_config.jpeg_width;
        config.height = app_config.jpeg_height;
        config.codec = HAL_VIDCODEC_MJPG;
        // MJPEG is controlled via JPEG quality factor (qfactor) now.
        // We force QP mode, because bitrate-based modes are not exposed/used.
        config.mode = HAL_VIDMODE_QP;
        config.framerate = app_config.jpeg_fps;
        // Some vendor HALs still read bitrate fields even in QP; keep safe defaults.
        config.bitrate = 1024;
        config.maxBitrate = 1024 * 5 / 4;
        unsigned int q = app_config.jpeg_qfactor;
        if (q < 1) q = 1;
        if (q > 99) q = 99;
        config.minQual = (unsigned char)q;
        config.maxQual = (unsigned char)q;

        switch (plat) {
#if defined(__ARM_PCS_VFP)
            case HAL_PLATFORM_I6:  ret = i6_video_create(index, &config); break;
            case HAL_PLATFORM_I6C: ret = i6c_video_create(index, &config); break;
            case HAL_PLATFORM_M6:  ret = m6_video_create(index, &config); break;
            case HAL_PLATFORM_RK:  ret = rk_video_create(index, &config); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
            case HAL_PLATFORM_AK:  ret = ak_video_create(index, &config); break;
            case HAL_PLATFORM_GM:  ret = gm_video_create(index, &config); break;
            case HAL_PLATFORM_V1:  ret = v1_video_create(index, &config); break;
            case HAL_PLATFORM_V2:  ret = v2_video_create(index, &config); break;
            case HAL_PLATFORM_V3:  ret = v3_video_create(index, &config); break;
            case HAL_PLATFORM_V4:  ret = v4_video_create(index, &config); break;
#elif defined(__mips__)
            case HAL_PLATFORM_T31: ret = t31_video_create(index, &config); break;
#elif defined(__riscv) || defined(__riscv__)
            case HAL_PLATFORM_CVI: ret = cvi_video_create(index, &config); break;
#endif
        }

        if (ret)
            HAL_ERROR("media", "Creating encoder %d failed with %#x!\n%s\n", 
                index, ret, errstr(ret));
    }

    if (ret = bind_channel(index, app_config.jpeg_fps, 1))
        HAL_ERROR("media", "Binding channel %d failed with %#x!\n%s\n",
            index, ret, errstr(ret));

    return EXIT_SUCCESS;
}

int disable_mp4(void) {
    int ret;

    for (char i = 0; i < chnCount; i++) {
        if (!chnState[i].enable) continue;
        if (chnState[i].payload != HAL_VIDCODEC_H264 ||
            chnState[i].payload != HAL_VIDCODEC_H265) continue;

        if (ret = unbind_channel(i, 1))
            HAL_ERROR("media", "Unbinding channel %d failed with %#x!\n%s\n", 
                i, ret, errstr(ret));

        if (ret = disable_video(i, 1))
            HAL_ERROR("media", "Disabling encoder %d failed with %#x!\n%s\n", 
                i, ret, errstr(ret));
    }

    return EXIT_SUCCESS;
}

int enable_mp4(void) {
    int ret;

    int index = take_next_free_channel(true);

    if (ret = create_channel(index, app_config.mp4_width, 
        app_config.mp4_height, app_config.mp4_fps, 0))
        HAL_ERROR("media", "Creating channel %d failed with %#x!\n%s\n", 
            index, ret, errstr(ret));

    {
        hal_vidconfig config = {0};
        config.width = app_config.mp4_width;
        config.height = app_config.mp4_height;
        config.codec = app_config.mp4_codecH265 ?
            HAL_VIDCODEC_H265 : HAL_VIDCODEC_H264;
        config.mode = app_config.mp4_mode;
        config.profile = app_config.mp4_profile;
        config.gop = app_config.mp4_gop;
        config.framerate = app_config.mp4_fps;
        config.bitrate = app_config.mp4_bitrate;
        config.maxBitrate = app_config.mp4_bitrate * 5 / 4;
        // Deterministic defaults + baseline for H.264+ logic in HAL.
        config.minQual = 34;
        config.maxQual = 48;
        if (!app_config.mp4_codecH265 && app_config.mp4_h264_plus) {
            // GM/Goke (libgm.so) firmwares can reject extended H.264 settings with NOT_SUPPORT.
            // Keep "H.264+" as a best-effort feature: enable where supported, otherwise fall back.
            if (plat == HAL_PLATFORM_GM) {
                HAL_WARNING("media", "H.264+ requested, but GM platform may not support it; falling back to H.264.\n");
            } else {
                config.flags |= HAL_VIDOPT_H264_PLUS;
            }
        }

        switch (plat) {
#if defined(__ARM_PCS_VFP)
            case HAL_PLATFORM_I6:  ret = i6_video_create(index, &config); break;
            case HAL_PLATFORM_I6C: ret = i6c_video_create(index, &config); break;
            case HAL_PLATFORM_M6:  ret = m6_video_create(index, &config); break;
            case HAL_PLATFORM_RK:  ret = rk_video_create(index, &config); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
            case HAL_PLATFORM_AK:  ret = ak_video_create(index, &config); break;
            case HAL_PLATFORM_GM:  ret = gm_video_create(index, &config); break;
            case HAL_PLATFORM_V1:  ret = v1_video_create(index, &config); break;
            case HAL_PLATFORM_V2:  ret = v2_video_create(index, &config); break;
            case HAL_PLATFORM_V3:  ret = v3_video_create(index, &config); break;
            case HAL_PLATFORM_V4:  ret = v4_video_create(index, &config); break;
#elif defined(__mips__)
            case HAL_PLATFORM_T31: ret = t31_video_create(index, &config); break;
#elif defined(__riscv) || defined(__riscv__)
            case HAL_PLATFORM_CVI: ret = cvi_video_create(index, &config); break;
#endif
        }

        if (ret)
            HAL_ERROR("media", "Creating encoder %d failed with %#x!\n%s\n", 
                index, ret, errstr(ret));

        mp4_set_config(app_config.mp4_width, app_config.mp4_height, app_config.mp4_fps,
            app_config.audio_enable ? active_audio_codec : HAL_AUDCODEC_UNSPEC,
            app_config.audio_bitrate,
            app_config.audio_channels ? app_config.audio_channels : 1,
            app_config.audio_srate);
    }

    if (ret = bind_channel(index, app_config.mp4_fps, 0))
        HAL_ERROR("media", "Binding channel %d failed with %#x!\n%s\n",
            index, ret, errstr(ret));

    return EXIT_SUCCESS;
}

int start_sdk(void) {
    int ret;

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I3:  ret = i3_hal_init(); break;
        case HAL_PLATFORM_I6:  ret = i6_hal_init(); break;
        case HAL_PLATFORM_I6C: ret = i6c_hal_init(); break;
        case HAL_PLATFORM_M6:  ret = m6_hal_init(); break;
        case HAL_PLATFORM_RK:  ret = rk_hal_init(); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ret = ak_hal_init(); break;
        case HAL_PLATFORM_GM:  ret = gm_hal_init(); break;
        case HAL_PLATFORM_V1:  ret = v1_hal_init(); break;
        case HAL_PLATFORM_V2:  ret = v2_hal_init(); break;
        case HAL_PLATFORM_V3:  ret = v3_hal_init(); break;
        case HAL_PLATFORM_V4:  ret = v4_hal_init(); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: ret = t31_hal_init(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: ret = cvi_hal_init(); break;
#endif
    }
    if (ret)
        HAL_ERROR("media", "HAL initialization failed with %#x!\n%s\n",
            ret, errstr(ret));

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:
            i6_aud_cb = save_audio_stream;
            i6_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_I6C:
            i6c_aud_cb = save_audio_stream;
            i6c_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_M6:
            m6_aud_cb = save_audio_stream;
            m6_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_RK:
            rk_aud_cb = save_audio_stream;
            rk_vid_cb = save_video_stream;
            break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_GM:
            gm_aud_cb = save_audio_stream;
            gm_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_V1:
            v1_aud_cb = save_audio_stream;
            v1_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_V2:
            v2_aud_cb = save_audio_stream;
            v2_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_V3:
            v3_aud_cb = save_audio_stream;
            v3_vid_cb = save_video_stream;
            break;
        case HAL_PLATFORM_V4:
            v4_aud_cb = save_audio_stream;
            v4_vid_cb = save_video_stream;
            break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31:
            t31_aud_cb = save_audio_stream;
            t31_vid_cb = save_video_stream;
            break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI:
            cvi_aud_cb = save_audio_stream;
            cvi_vid_cb = save_video_stream;
            break;
#endif
    }

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I3:  ret = i3_system_init(); break;
        case HAL_PLATFORM_I6:  ret = i6_system_init(); break;
        case HAL_PLATFORM_I6C: ret = i6c_system_init(); break;
        case HAL_PLATFORM_M6:  ret = m6_system_init(); break;
        case HAL_PLATFORM_RK:  ret = rk_system_init(0); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ret = ak_system_init(app_config.sensor_config); break;
        case HAL_PLATFORM_GM:  ret = gm_system_init(); break;
        case HAL_PLATFORM_V1:  ret = v1_system_init(app_config.sensor_config); break;
        case HAL_PLATFORM_V2:  ret = v2_system_init(app_config.sensor_config); break;
        case HAL_PLATFORM_V3:  ret = v3_system_init(app_config.sensor_config); break;
        case HAL_PLATFORM_V4:  ret = v4_system_init(app_config.sensor_config); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: ret = t31_system_init(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: ret = cvi_system_init(app_config.sensor_config); break;
#endif
    }
    if (ret)
        HAL_ERROR("media", "System initialization failed with %#x!\n%s\n",
            ret, errstr(ret));

    if (app_config.audio_enable) {
        ret = enable_audio();
        if (ret)
            HAL_ERROR("media", "Audio initialization failed with %#x!\n%s\n",
                ret, errstr(ret));
        // Apply persisted mute state immediately after starting audio.
        if (app_config.audio_mute)
            media_set_audio_mute(1);
    }

    short width = MAX(app_config.mp4_width, app_config.jpeg_width);
    short height = MAX(app_config.mp4_height, app_config.jpeg_height);
    short framerate = MAX(app_config.mp4_fps, app_config.jpeg_fps);

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  ret = i6_pipeline_create(0, width,
            height, isp_mirror_effective(), isp_flip_effective(), framerate); break;
        case HAL_PLATFORM_I6C: ret = i6c_pipeline_create(0, width,
            height, isp_mirror_effective(), isp_flip_effective(), framerate); break;
        case HAL_PLATFORM_M6:  ret = m6_pipeline_create(0, width,
            height, isp_mirror_effective(), isp_flip_effective(), framerate); break;
        case HAL_PLATFORM_RK:  ret = rk_pipeline_create(width, height); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ret = ak_pipeline_create(isp_mirror_effective(),
            isp_flip_effective()); break;
        case HAL_PLATFORM_GM:  ret = gm_pipeline_create(isp_mirror_effective(),
            isp_flip_effective()); break;
        case HAL_PLATFORM_V1:  ret = v1_pipeline_create(); break;
        case HAL_PLATFORM_V2:  ret = v2_pipeline_create(); break;
        case HAL_PLATFORM_V3:  ret = v3_pipeline_create(); break;
        case HAL_PLATFORM_V4:  ret = v4_pipeline_create(app_config.iq_config); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: ret = t31_pipeline_create(isp_mirror_effective(),
            isp_flip_effective(), app_config.antiflicker, framerate); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: ret = cvi_pipeline_create(); break;
#endif
    }
    if (ret)
        HAL_ERROR("media", "Pipeline creation failed with %#x!\n%s\n",
            ret, errstr(ret));

    if (isp_thread) {
        pthread_attr_t thread_attr;
        pthread_attr_init(&thread_attr);
        size_t stacksize;
        pthread_attr_getstacksize(&thread_attr, &stacksize);
        size_t new_stacksize = app_config.isp_thread_stack_size;
        if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
            HAL_DANGER("media", "Can't set stack size %zu!\n", new_stacksize);
        if (pthread_create(
                     &ispPid, &thread_attr, (void *(*)(void *))isp_thread, NULL))
            HAL_ERROR("media", "Starting the imaging thread failed!\n");
        if (pthread_attr_setstacksize(&thread_attr, stacksize))
            HAL_DANGER("media", "Can't set stack size %zu!\n", stacksize);
        pthread_attr_destroy(&thread_attr);
    }

    if (app_config.mp4_enable && (ret = enable_mp4()))
        HAL_ERROR("media", "MP4 initialization failed with %#x!\n", ret);

    if (app_config.jpeg_enable && (ret = enable_mjpeg()))
        HAL_ERROR("media", "MJPEG initialization failed with %#x!\n", ret);

    if (app_config.jpeg_enable && (ret = jpeg_init()))
        HAL_ERROR("media", "JPEG initialization failed with %#x!\n", ret);

    {
        pthread_attr_t thread_attr;
        pthread_attr_init(&thread_attr);
        size_t stacksize;
        pthread_attr_getstacksize(&thread_attr, &stacksize);
        size_t new_stacksize = app_config.venc_stream_thread_stack_size;
        if (pthread_attr_setstacksize(&thread_attr, new_stacksize))
            HAL_DANGER("media", "Can't set stack size %zu\n", new_stacksize);
        if (pthread_create(
                     &vidPid, &thread_attr, (void *(*)(void *))vid_thread, NULL))
            HAL_ERROR("media", "Starting the video encoding thread failed!\n");
        if (pthread_attr_setstacksize(&thread_attr, stacksize))
            HAL_DANGER("media", "Can't set stack size %zu\n", stacksize);
        pthread_attr_destroy(&thread_attr);
    }

    if (!access(app_config.sensor_config, F_OK) && !sleep(1))
        switch (plat) {
#if defined(__ARM_PCS_VFP)
            case HAL_PLATFORM_I3:  i3_config_load(app_config.sensor_config); break;
            case HAL_PLATFORM_I6:  i6_config_load(app_config.sensor_config); break;
            case HAL_PLATFORM_I6C: i6c_config_load(app_config.sensor_config); break;
            case HAL_PLATFORM_M6:  m6_config_load(app_config.sensor_config); break;
#elif defined(__mips__)
            case HAL_PLATFORM_T31: t31_config_load(app_config.sensor_config); break;
#endif
        }

#if defined(__ARM_PCS_VFP)
    if (!EMPTY(app_config.iq_config) &&
        (plat == HAL_PLATFORM_I6 || plat == HAL_PLATFORM_I6C || plat == HAL_PLATFORM_M6)) {
        if (!access(app_config.iq_config, F_OK)) {
            int iq_ret = 0;
            switch (plat) {
                case HAL_PLATFORM_I6:  iq_ret = i6_config_load(app_config.iq_config); break;
                case HAL_PLATFORM_I6C: iq_ret = i6c_config_load(app_config.iq_config); break;
                case HAL_PLATFORM_M6:  iq_ret = m6_config_load(app_config.iq_config); break;
                default: break;
            }
            if (iq_ret)
                HAL_WARNING("media", "IQ config '%s' apply failed with %#x\n",
                    app_config.iq_config, iq_ret);
            else
                HAL_INFO("media", "Applied IQ config '%s'\n", app_config.iq_config);
        } else {
            HAL_WARNING("media", "IQ config '%s' not found, skipping\n", app_config.iq_config);
        }
    }
#endif

    HAL_INFO("media", "SDK has started successfully!\n");

    return EXIT_SUCCESS;
}

int stop_sdk(void) {
    pthread_join(vidPid, NULL);

    if (app_config.jpeg_enable)
        jpeg_deinit();

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_video_destroy_all(); break;
        case HAL_PLATFORM_I6C: i6c_video_destroy_all(); break;
        case HAL_PLATFORM_M6:  m6_video_destroy_all(); break;
        case HAL_PLATFORM_RK:  rk_video_destroy_all(); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ak_video_destroy_all(); break;
        case HAL_PLATFORM_GM:  gm_video_destroy_all(); break;
        case HAL_PLATFORM_V1:  v1_video_destroy_all(); break;
        case HAL_PLATFORM_V2:  v2_video_destroy_all(); break;
        case HAL_PLATFORM_V3:  v3_video_destroy_all(); break;
        case HAL_PLATFORM_V4:  v4_video_destroy_all(); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_video_destroy_all(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_video_destroy_all(); break;
#endif
    }

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_pipeline_destroy(); break;
        case HAL_PLATFORM_I6C: i6c_pipeline_destroy(); break;
        case HAL_PLATFORM_M6:  m6_pipeline_destroy(); break;
        case HAL_PLATFORM_RK:  rk_pipeline_destroy(); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ak_pipeline_destroy(); break;
        case HAL_PLATFORM_GM:  gm_pipeline_destroy(); break;
        case HAL_PLATFORM_V1:  v1_pipeline_destroy(); break;
        case HAL_PLATFORM_V2:  v2_pipeline_destroy(); break;
        case HAL_PLATFORM_V3:  v3_pipeline_destroy(); break;
        case HAL_PLATFORM_V4:  v4_pipeline_destroy(); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_pipeline_destroy(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_pipeline_destroy(); break;
#endif
    }

    if (app_config.audio_enable)
        disable_audio();

    if (isp_thread)
        pthread_join(ispPid, NULL);

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I3:  i3_system_deinit(); break;
        case HAL_PLATFORM_I6:  i6_system_deinit(); break;
        case HAL_PLATFORM_I6C: i6c_system_deinit(); break;
        case HAL_PLATFORM_M6:  m6_system_deinit(); break;
        case HAL_PLATFORM_RK:  rk_system_deinit(); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ak_system_deinit(); break;
        case HAL_PLATFORM_GM:  gm_system_deinit(); break;
        case HAL_PLATFORM_V1:  v1_system_deinit(); break;
        case HAL_PLATFORM_V2:  v2_system_deinit(); break;
        case HAL_PLATFORM_V3:  v3_system_deinit(); break;
        case HAL_PLATFORM_V4:  v4_system_deinit(); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_system_deinit(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_system_deinit(); break;
#endif
    }

    switch (plat) {
#if defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_V1: v1_sensor_deinit(); break;
        case HAL_PLATFORM_V2: v2_sensor_deinit(); break;
        case HAL_PLATFORM_V3: v3_sensor_deinit(); break;
        case HAL_PLATFORM_V4: v4_sensor_deinit(); break;
#endif
    }

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I3:  i3_hal_deinit(); break;
        case HAL_PLATFORM_I6:  i6_hal_deinit(); break;
        case HAL_PLATFORM_I6C: i6c_hal_deinit(); break;
        case HAL_PLATFORM_M6:  m6_hal_deinit(); break;
        case HAL_PLATFORM_RK:  rk_hal_deinit(); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_AK:  ak_hal_deinit(); break;
        case HAL_PLATFORM_GM:  gm_hal_deinit(); break;
        case HAL_PLATFORM_V1:  v1_hal_deinit(); break;
        case HAL_PLATFORM_V2:  v2_hal_deinit(); break;
        case HAL_PLATFORM_V3:  v3_hal_deinit(); break;
        case HAL_PLATFORM_V4:  v4_hal_deinit(); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: t31_hal_deinit(); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_hal_deinit(); break;
#endif
    }

    HAL_INFO("media", "SDK had stopped successfully!\n");
    return EXIT_SUCCESS;
}
