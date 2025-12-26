#if defined(__arm__) && !defined(__ARM_PCS_VFP)

#include "v4_hal.h"
#include "../../app_config.h"
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <strings.h>
#include <pthread.h>
#include <time.h>

// Night mode state is implemented in src/night.c. We only need this one symbol here.
extern bool night_mode_on(void);

// For debug prints; avoids adding include dependency here.
char *errstr(int error);

v4_isp_alg      v4_ae_lib = { .id = 0, .libName = "ae_lib" };
v4_aud_impl     v4_aud;
v4_isp_alg      v4_awb_lib = { .id = 0, .libName = "awb_lib" };
v4_config_impl  v4_config;
v4_isp_impl     v4_isp;
v4_snr_drv_impl v4_snr_drv;
v4_rgn_impl     v4_rgn;
v4_sys_impl     v4_sys;
v4_vb_impl      v4_vb;
v4_venc_impl    v4_venc;
v4_vi_impl      v4_vi;
v4_vpss_impl    v4_vpss;

hal_chnstate v4_state[V4_VENC_CHN_NUM] = {0};
int (*v4_aud_cb)(hal_audframe*);
int (*v4_vid_cb)(char, hal_vidstream*);

char _v4_aud_chn = 0;
char _v4_aud_dev = 0;
char _v4_isp_chn = 0;
char _v4_isp_dev = 0;
char _v4_venc_dev = 0;
char _v4_vi_chn = 0;
char _v4_vi_dev = 0;
char _v4_vi_pipe = 0;
char _v4_vpss_chn = 0;
char _v4_vpss_grp = 0;

static char _v4_iq_cfg_path[256] = {0};

// Audio gain handling:
// - Prefer hardware AI volume control when SDK exposes it
// - Otherwise apply software scaling on captured PCM samples (16-bit)
static float _v4_audio_sw_mul = 1.0f;
static int _v4_audio_sw_on = 0;

static inline int v4_clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Map user gain percent 0..100 to "dB" value used for AI volume APIs / scaling.
// Semantics for hisi/v4:
// - 50 => 0 dB (unity)
// - 0  => -60 dB
// - 100 => +30 dB
static int v4_gain_percent_to_db(int percent) {
    int p = v4_clampi(percent, 0, 100);
    if (p <= 50) {
        // -60 .. 0
        return -60 + (p * 60) / 50;
    }
    // 0 .. +30
    return ((p - 50) * 30) / 50;
}

// Forward decl (used by delayed apply thread)
static int v4_iq_apply(const char *path, int pipe);
static void v4_iq_dyn_update_from_ini(struct IniConfig *ini, int pipe, bool enableDynDehaze, bool enableDynLinearDRC);
static void v4_iq_dyn_maybe_start(int pipe);

int v4_iq_reload(void) {
    if (!_v4_iq_cfg_path[0])
        return EXIT_SUCCESS;

    // Apply immediately using the currently active mode (DAY vs IR).
    // This ensures ir_* sections take effect right after night_mode() toggles IR.
    int ret = v4_iq_apply(_v4_iq_cfg_path, _v4_vi_pipe);

    // If dynamic IQ sections exist, ensure the dynamic thread is started.
    v4_iq_dyn_maybe_start(_v4_vi_pipe);
    return ret;
}

typedef struct {
    char path[256];
    int pipe;
} v4_iq_delayed_apply_ctx;

static void *v4_iq_delayed_apply_thread(void *arg) {
    v4_iq_delayed_apply_ctx *ctx = (v4_iq_delayed_apply_ctx *)arg;
    if (!ctx) return NULL;
    usleep(300 * 1000);
    v4_iq_apply(ctx->path, ctx->pipe);
    free(ctx);
    return NULL;
}

#if 0
// ---- Dynamic IQ (by ISO) ----
#define V4_IQ_DYN_MAX_POINTS 32
typedef struct {
    bool enabled;
    int n;                  // number of points
    HI_U32 iso_thr[V4_IQ_DYN_MAX_POINTS];
    HI_U8  str[V4_IQ_DYN_MAX_POINTS];
} v4_iq_dyn_dehaze_cfg;

typedef struct {
    bool enabled;
    int n;
    HI_U32 iso[V4_IQ_DYN_MAX_POINTS];

    HI_U8  localMixBrightMax[V4_IQ_DYN_MAX_POINTS];
    HI_U8  localMixBrightMin[V4_IQ_DYN_MAX_POINTS];
    HI_U8  localMixDarkMax[V4_IQ_DYN_MAX_POINTS];
    HI_U8  localMixDarkMin[V4_IQ_DYN_MAX_POINTS];
    HI_U8  brightGainLmt[V4_IQ_DYN_MAX_POINTS];
    HI_U8  brightGainLmtStep[V4_IQ_DYN_MAX_POINTS];
    HI_U8  darkGainLmtY[V4_IQ_DYN_MAX_POINTS];
    HI_U8  darkGainLmtC[V4_IQ_DYN_MAX_POINTS];
    HI_U8  fltScaleCoarse[V4_IQ_DYN_MAX_POINTS];
    HI_U8  fltScaleFine[V4_IQ_DYN_MAX_POINTS];
    HI_U8  contrastControl[V4_IQ_DYN_MAX_POINTS];
    HI_S8  detailAdjustFactor[V4_IQ_DYN_MAX_POINTS];

    HI_U8  asymmetry[V4_IQ_DYN_MAX_POINTS];
    HI_U8  secondPole[V4_IQ_DYN_MAX_POINTS];
    HI_U8  compress[V4_IQ_DYN_MAX_POINTS];
    HI_U8  stretch[V4_IQ_DYN_MAX_POINTS];

    HI_U16 strength[V4_IQ_DYN_MAX_POINTS];
} v4_iq_dyn_linear_drc_cfg;

typedef struct {
    pthread_mutex_t lock;
    int pipe;
    bool thread_started;
    v4_iq_dyn_dehaze_cfg dehaze;
    v4_iq_dyn_linear_drc_cfg linear_drc;

    // last applied (for log/avoid redundant Set*)
    HI_U32 last_iso;
    HI_BOOL has_last;
    HI_U16 last_drc_strength;
    HI_U8  last_dehaze_strength;
} v4_iq_dyn_state;

static v4_iq_dyn_state _v4_iq_dyn = { .lock = PTHREAD_MUTEX_INITIALIZER };

static inline HI_U32 v4_iq_interp_u32(HI_U32 x, const HI_U32 *xp, const HI_U32 *yp, int n) {
    if (n <= 1) return (n == 1) ? yp[0] : 0;
    if (x <= xp[0]) return yp[0];
    if (x >= xp[n - 1]) return yp[n - 1];
    for (int i = 0; i < n - 1; i++) {
        HI_U32 x0 = xp[i], x1 = xp[i + 1];
        if (x >= x0 && x <= x1) {
            HI_U32 y0 = yp[i], y1 = yp[i + 1];
            if (x1 == x0) return y1;
            // linear interpolation (integer)
            return (HI_U32)(y0 + (HI_U64)(y1 - y0) * (x - x0) / (x1 - x0));
        }
    }
    return yp[n - 1];
}

static inline HI_S32 v4_iq_interp_s32(HI_U32 x, const HI_U32 *xp, const HI_S32 *yp, int n) {
    if (n <= 1) return (n == 1) ? yp[0] : 0;
    if (x <= xp[0]) return yp[0];
    if (x >= xp[n - 1]) return yp[n - 1];
    for (int i = 0; i < n - 1; i++) {
        HI_U32 x0 = xp[i], x1 = xp[i + 1];
        if (x >= x0 && x <= x1) {
            HI_S32 y0 = yp[i], y1 = yp[i + 1];
            if (x1 == x0) return y1;
            return (HI_S32)(y0 + (HI_S64)(y1 - y0) * (HI_S64)(x - x0) / (HI_S64)(x1 - x0));
        }
    }
    return yp[n - 1];
}

static void v4_iq_dyn_load_dynamic_dehaze(struct IniConfig *ini) {
    v4_iq_dyn_dehaze_cfg cfg;
    memset(&cfg, 0, sizeof(cfg));

    int sec_s = 0, sec_e = 0;
    if (section_pos(ini, "dynamic_dehaze", &sec_s, &sec_e) != CONFIG_OK) {
        // no section -> disable
        pthread_mutex_lock(&_v4_iq_dyn.lock);
        _v4_iq_dyn.dehaze.enabled = false;
        pthread_mutex_unlock(&_v4_iq_dyn.lock);
        return;
    }

    int en = 1;
    parse_int(ini, "dynamic_dehaze", "Enable", 0, 1, &en); // optional
    cfg.enabled = (en != 0);

    char buf[2048];
    HI_U32 iso[V4_IQ_DYN_MAX_POINTS] = {0};
    HI_U32 str[V4_IQ_DYN_MAX_POINTS] = {0};
    int nIso = 0, nStr = 0;

    if (parse_param_value(ini, "dynamic_dehaze", "IsoThresh", buf) == CONFIG_OK)
        nIso = v4_iq_parse_csv_u32(buf, iso, V4_IQ_DYN_MAX_POINTS);
    if (parse_param_value(ini, "dynamic_dehaze", "AutoDehazeStr", buf) == CONFIG_OK)
        nStr = v4_iq_parse_csv_u32(buf, str, V4_IQ_DYN_MAX_POINTS);

    cfg.n = (nIso < nStr) ? nIso : nStr;
    if (cfg.n > V4_IQ_DYN_MAX_POINTS) cfg.n = V4_IQ_DYN_MAX_POINTS;
    for (int i = 0; i < cfg.n; i++) {
        cfg.iso_thr[i] = iso[i];
        HI_U32 v = str[i];
        if (v > 255) v = 255;
        cfg.str[i] = (HI_U8)v;
    }

    pthread_mutex_lock(&_v4_iq_dyn.lock);
    _v4_iq_dyn.dehaze = cfg;
    pthread_mutex_unlock(&_v4_iq_dyn.lock);
}

static void v4_iq_dyn_load_dynamic_linear_drc(struct IniConfig *ini) {
    v4_iq_dyn_linear_drc_cfg cfg;
    memset(&cfg, 0, sizeof(cfg));

    int sec_s = 0, sec_e = 0;
    if (section_pos(ini, "dynamic_linear_drc", &sec_s, &sec_e) != CONFIG_OK) {
        pthread_mutex_lock(&_v4_iq_dyn.lock);
        _v4_iq_dyn.linear_drc.enabled = false;
        pthread_mutex_unlock(&_v4_iq_dyn.lock);
        return;
    }

    int en = 1;
    if (parse_int(ini, "dynamic_linear_drc", "Enable", 0, 1, &en) == CONFIG_OK)
        cfg.enabled = (en != 0);
    else
        cfg.enabled = true;

    int isoCnt = 0;
    if (parse_int(ini, "dynamic_linear_drc", "IsoCnt", 1, V4_IQ_DYN_MAX_POINTS, &isoCnt) != CONFIG_OK)
        isoCnt = 0;

    char buf[4096];
    HI_U32 iso[V4_IQ_DYN_MAX_POINTS] = {0};
    int nIso = 0;
    if (parse_param_value(ini, "dynamic_linear_drc", "IsoLevel", buf) == CONFIG_OK)
        nIso = v4_iq_parse_csv_u32(buf, iso, V4_IQ_DYN_MAX_POINTS);
    cfg.n = isoCnt > 0 ? isoCnt : nIso;
    if (cfg.n > nIso) cfg.n = nIso;
    if (cfg.n > V4_IQ_DYN_MAX_POINTS) cfg.n = V4_IQ_DYN_MAX_POINTS;
    for (int i = 0; i < cfg.n; i++) cfg.iso[i] = iso[i];

    // Helper to parse per-iso u8 arrays
    HI_U32 tmp[V4_IQ_DYN_MAX_POINTS];
    #define PARSE_U8(key, dst) \
        do { \
            if (parse_param_value(ini, "dynamic_linear_drc", (key), buf) == CONFIG_OK) { \
                memset(tmp, 0, sizeof(tmp)); \
                int nn = v4_iq_parse_csv_u32(buf, tmp, V4_IQ_DYN_MAX_POINTS); \
                if (nn < cfg.n) cfg.n = nn; \
                for (int i = 0; i < cfg.n; i++) { \
                    HI_U32 v = tmp[i]; if (v > 255) v = 255; \
                    (dst)[i] = (HI_U8)v; \
                } \
            } \
        } while (0)

    PARSE_U8("LocalMixingBrightMax", cfg.localMixBrightMax);
    PARSE_U8("LocalMixingBrightMin", cfg.localMixBrightMin);
    PARSE_U8("LocalMixingDarkMax", cfg.localMixDarkMax);
    PARSE_U8("LocalMixingDarkMin", cfg.localMixDarkMin);
    PARSE_U8("BrightGainLmt", cfg.brightGainLmt);
    PARSE_U8("BrightGainLmtStep", cfg.brightGainLmtStep);
    PARSE_U8("DarkGainLmtY", cfg.darkGainLmtY);
    PARSE_U8("DarkGainLmtC", cfg.darkGainLmtC);
    PARSE_U8("FltScaleCoarse", cfg.fltScaleCoarse);
    PARSE_U8("FltScaleFine", cfg.fltScaleFine);
    PARSE_U8("ContrastControl", cfg.contrastControl);
    PARSE_U8("Asymmetry", cfg.asymmetry);
    PARSE_U8("SecondPole", cfg.secondPole);
    PARSE_U8("Compress", cfg.compress);
    PARSE_U8("Stretch", cfg.stretch);

    // Signed (DetailAdjustFactor)
    if (parse_param_value(ini, "dynamic_linear_drc", "DetailAdjustFactor", buf) == CONFIG_OK) {
        memset(tmp, 0, sizeof(tmp));
        int nn = v4_iq_parse_csv_u32(buf, tmp, V4_IQ_DYN_MAX_POINTS);
        if (nn < cfg.n) cfg.n = nn;
        for (int i = 0; i < cfg.n; i++) {
            HI_S32 v = (HI_S32)tmp[i];
            if (v > 127) v = 127;
            if (v < -128) v = -128;
            cfg.detailAdjustFactor[i] = (HI_S8)v;
        }
    }

    // Strength (u16)
    if (parse_param_value(ini, "dynamic_linear_drc", "Strength", buf) == CONFIG_OK) {
        memset(tmp, 0, sizeof(tmp));
        int nn = v4_iq_parse_csv_u32(buf, tmp, V4_IQ_DYN_MAX_POINTS);
        if (nn < cfg.n) cfg.n = nn;
        for (int i = 0; i < cfg.n; i++) {
            HI_U32 v = tmp[i];
            if (v > 65535) v = 65535;
            cfg.strength[i] = (HI_U16)v;
        }
    }

    #undef PARSE_U8

    pthread_mutex_lock(&_v4_iq_dyn.lock);
    _v4_iq_dyn.linear_drc = cfg;
    pthread_mutex_unlock(&_v4_iq_dyn.lock);
}

static void v4_iq_dyn_unbypass_modules(int pipe) {
    if (!v4_isp.fnGetModuleControl || !v4_isp.fnSetModuleControl) return;
    ISP_MODULE_CTRL_U mc;
    memset(&mc, 0, sizeof(mc));
    if (v4_isp.fnGetModuleControl(pipe, &mc)) return;
    mc.u64Key &= ~(1ULL << 5);  // Dehaze
    mc.u64Key &= ~(1ULL << 8);  // DRC
    mc.u64Key &= ~(1ULL << 21); // LDCI (safe, if dynamic uses it later)
    v4_isp.fnSetModuleControl(pipe, &mc);
}

static void *v4_iq_dynamic_thread(void *arg) {
    int pipe = (int)(intptr_t)arg;
    // Wait for ISP_Run to initialize internal state.
    usleep(700 * 1000);

    if (!v4_isp.fnQueryExposureInfo) {
        HAL_INFO("v4_iq", "Dynamic IQ: QueryExposureInfo API not available, skipping dynamics\n");
        return NULL;
    }

    v4_iq_dyn_unbypass_modules(pipe);

    for (;;) {
        // Snapshot current configs
        v4_iq_dyn_dehaze_cfg deh;
        v4_iq_dyn_linear_drc_cfg drc;
        HI_BOOL has_last;
        HI_U16 last_drc;
        HI_U8 last_deh;
        pthread_mutex_lock(&_v4_iq_dyn.lock);
        deh = _v4_iq_dyn.dehaze;
        drc = _v4_iq_dyn.linear_drc;
        has_last = _v4_iq_dyn.has_last;
        last_drc = _v4_iq_dyn.last_drc_strength;
        last_deh = _v4_iq_dyn.last_dehaze_strength;
        pthread_mutex_unlock(&_v4_iq_dyn.lock);

        if (!deh.enabled && !drc.enabled) {
            // nothing to do
            usleep(1000 * 1000);
            continue;
        }

        ISP_EXP_INFO_S expi;
        memset(&expi, 0, sizeof(expi));
        int qr = v4_isp.fnQueryExposureInfo(pipe, &expi);
        if (qr) {
            usleep(500 * 1000);
            continue;
        }
        HI_U32 iso = expi.u32ISO;

        // ---- dynamic dehaze ----
        if (deh.enabled && deh.n >= 2 && v4_isp.fnGetDehazeAttr && v4_isp.fnSetDehazeAttr) {
            HI_U32 deh_u32[V4_IQ_DYN_MAX_POINTS];
            for (int i = 0; i < deh.n; i++) deh_u32[i] = deh.str[i];
            HI_U8 target = (HI_U8)v4_iq_interp_u32(iso, deh.iso_thr, deh_u32, deh.n);
            if (!has_last || target != last_deh) {
                ISP_DEHAZE_ATTR_S dh;
                memset(&dh, 0, sizeof(dh));
                if (!v4_isp.fnGetDehazeAttr(pipe, &dh)) {
                    dh.bEnable = HI_TRUE;
                    dh.enOpType = OP_TYPE_AUTO;
                    dh.stAuto.u8strength = target;
                    if (!v4_isp.fnSetDehazeAttr(pipe, &dh)) {
                        pthread_mutex_lock(&_v4_iq_dyn.lock);
                        _v4_iq_dyn.last_dehaze_strength = target;
                        _v4_iq_dyn.last_iso = iso;
                        _v4_iq_dyn.has_last = HI_TRUE;
                        pthread_mutex_unlock(&_v4_iq_dyn.lock);
                        HAL_INFO("v4_iq", "Dynamic Dehaze: iso=%u strength=%u\n", (unsigned)iso, (unsigned)target);
                    }
                }
            }
        }

        // ---- dynamic linear drc ----
        if (drc.enabled && drc.n >= 2 && v4_isp.fnGetDRCAttr && v4_isp.fnSetDRCAttr) {
            // Interpolate all needed params
            HI_U32 tmp_u32[V4_IQ_DYN_MAX_POINTS];
            for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.strength[i];
            HI_U16 targetStrength = (HI_U16)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);

            if (!has_last || targetStrength != last_drc) {
                ISP_DRC_ATTR_S da;
                memset(&da, 0, sizeof(da));
                if (!v4_isp.fnGetDRCAttr(pipe, &da)) {
                    da.bEnable = HI_TRUE;
                    da.enOpType = OP_TYPE_AUTO;
                    da.enCurveSelect = DRC_CURVE_ASYMMETRY; // matches dynamic_linear_drc fields
                    da.stAuto.u16Strength = targetStrength;

                    // u8 fields
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.localMixBrightMax[i];
                    da.u8LocalMixingBrightMax = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.localMixBrightMin[i];
                    da.u8LocalMixingBrightMin = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.localMixDarkMax[i];
                    da.u8LocalMixingDarkMax = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.localMixDarkMin[i];
                    da.u8LocalMixingDarkMin = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.brightGainLmt[i];
                    da.u8BrightGainLmt = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.brightGainLmtStep[i];
                    da.u8BrightGainLmtStep = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.darkGainLmtY[i];
                    da.u8DarkGainLmtY = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.darkGainLmtC[i];
                    da.u8DarkGainLmtC = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.fltScaleCoarse[i];
                    da.u8FltScaleCoarse = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.fltScaleFine[i];
                    da.u8FltScaleFine = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.contrastControl[i];
                    da.u8ContrastControl = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);

                    // signed detail adjust
                    HI_S32 svals[V4_IQ_DYN_MAX_POINTS];
                    for (int i = 0; i < drc.n; i++) svals[i] = (HI_S32)drc.detailAdjustFactor[i];
                    da.s8DetailAdjustFactor = (HI_S8)v4_iq_interp_s32(iso, drc.iso, svals, drc.n);

                    // asymmetry curve params
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.asymmetry[i];
                    da.stAsymmetryCurve.u8Asymmetry = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.secondPole[i];
                    da.stAsymmetryCurve.u8SecondPole = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.stretch[i];
                    da.stAsymmetryCurve.u8Stretch = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);
                    for (int i = 0; i < drc.n; i++) tmp_u32[i] = drc.compress[i];
                    da.stAsymmetryCurve.u8Compress = (HI_U8)v4_iq_interp_u32(iso, drc.iso, tmp_u32, drc.n);

                    if (!v4_isp.fnSetDRCAttr(pipe, &da)) {
                        pthread_mutex_lock(&_v4_iq_dyn.lock);
                        _v4_iq_dyn.last_drc_strength = targetStrength;
                        _v4_iq_dyn.last_iso = iso;
                        _v4_iq_dyn.has_last = HI_TRUE;
                        pthread_mutex_unlock(&_v4_iq_dyn.lock);
                        HAL_INFO("v4_iq", "Dynamic Linear DRC: iso=%u strength=%u\n",
                            (unsigned)iso, (unsigned)targetStrength);
                    }
                }
            }
        }

        usleep(500 * 1000);
    }
    return NULL;
}

#endif

void v4_hal_deinit(void)
{
    v4_vpss_unload(&v4_vpss);
    v4_vi_unload(&v4_vi);
    v4_venc_unload(&v4_venc);
    v4_vb_unload(&v4_vb);
    v4_rgn_unload(&v4_rgn);
    v4_isp_unload(&v4_isp);
    v4_aud_unload(&v4_aud);
    v4_sys_unload(&v4_sys);
}

int v4_hal_init(void)
{
    int ret;

    if (ret = v4_sys_load(&v4_sys))
        return ret;
    if (ret = v4_aud_load(&v4_aud))
        return ret;
    if (ret = v4_isp_load(&v4_isp))
        return ret;
    if (ret = v4_rgn_load(&v4_rgn))
        return ret;
    if (ret = v4_vb_load(&v4_vb))
        return ret;
    if (ret = v4_venc_load(&v4_venc))
        return ret;
    if (ret = v4_vi_load(&v4_vi))
        return ret;
    if (ret = v4_vpss_load(&v4_vpss))
        return ret;

    return EXIT_SUCCESS;
}

void v4_audio_deinit(void)
{
    v4_aud.fnDisableChannel(_v4_aud_dev, _v4_aud_chn);

    v4_aud.fnDisableDevice(_v4_aud_dev);
}

int v4_audio_init(int samplerate, int gain_percent)
{
    int ret;

    {
        v4_aud_cnf config;
        config.rate = samplerate;
        config.bit = V4_AUD_BIT_16;
        config.intf = V4_AUD_INTF_I2S_MASTER;
        config.stereoOn = 0;
        config.expandOn = 0;
        config.frmNum = 30;
        config.packNumPerFrm = 320;
        config.chnNum = 1;
        config.syncRxClkOn = 0;
        config.i2sType = V4_AUD_I2ST_INNERCODEC;
        if (ret = v4_aud.fnSetDeviceConfig(_v4_aud_dev, &config))
            return ret;
    }
    if (ret = v4_aud.fnEnableDevice(_v4_aud_dev))
        return ret;
    
    if (ret = v4_aud.fnEnableChannel(_v4_aud_dev, _v4_aud_chn))
        return ret;

    // Apply gain/volume (optional in SDK).
    // If we can't set via MPI, fall back to software scaling in capture thread.
    _v4_audio_sw_mul = 1.0f;
    _v4_audio_sw_on = 0;

    const int db = v4_gain_percent_to_db(gain_percent);
    int vol_ret = -1;
    int applied_hw = 0;
    if (v4_aud.fnSetChnVolume) {
        vol_ret = v4_aud.fnSetChnVolume(_v4_aud_dev, _v4_aud_chn, db);
        applied_hw = (vol_ret == 0);
    } else if (v4_aud.fnSetDevVolume) {
        vol_ret = v4_aud.fnSetDevVolume(_v4_aud_dev, db);
        applied_hw = (vol_ret == 0);
    }

    if (!applied_hw) {
        if (vol_ret != -1) {
            HAL_WARNING("v4_aud", "AI SetVolume failed ret=%#x (db=%d, gain=%d%%); using software gain\n",
                vol_ret, db, v4_clampi(gain_percent, 0, 100));
        }
        _v4_audio_sw_mul = powf(10.0f, (float)db / 20.0f);
        // Enable only when meaningfully different from 1.0f.
        _v4_audio_sw_on = (fabsf(_v4_audio_sw_mul - 1.0f) > 0.0001f);
    } else {
        HAL_INFO("v4_aud", "AI gain applied in HW: gain=%d%% db=%d\n",
            v4_clampi(gain_percent, 0, 100), db);
    }

    return EXIT_SUCCESS;
}

void *v4_audio_thread(void)
{
    int ret;

    v4_aud_frm frame;
    v4_aud_efrm echoFrame;
    memset(&frame, 0, sizeof(frame));
    memset(&echoFrame, 0, sizeof(echoFrame));

    while (keepRunning && audioOn) {
        if (ret = v4_aud.fnGetFrame(_v4_aud_dev, _v4_aud_chn, 
            &frame, &echoFrame, 128)) {
            HAL_WARNING("v4_aud", "Getting the frame failed "
                "with %#x!\n", ret);
            continue;
        }

        // Software gain (only when hardware volume isn't available / failed).
        if (_v4_audio_sw_on && frame.addr[0] && frame.length >= 2) {
            const float mul = _v4_audio_sw_mul;
            int16_t *pcm = (int16_t *)frame.addr[0];
            const unsigned int samples = frame.length / 2;
            for (unsigned int i = 0; i < samples; i++) {
                const float fv = (float)pcm[i] * mul;
                int32_t v = (int32_t)lrintf(fv);
                if (v > 32767) v = 32767;
                if (v < -32768) v = -32768;
                pcm[i] = (int16_t)v;
            }
        }

        if (v4_aud_cb) {
            hal_audframe outFrame;
            outFrame.channelCnt = 1;
            outFrame.data[0] = frame.addr[0];
            outFrame.length[0] = frame.length;
            outFrame.seq = frame.sequence;
            outFrame.timestamp = frame.timestamp;
            (v4_aud_cb)(&outFrame);
        }

        if (ret = v4_aud.fnFreeFrame(_v4_aud_dev, _v4_aud_chn,
            &frame, &echoFrame)) {
            HAL_WARNING("v4_aud", "Releasing the frame failed"
                " with %#x!\n", ret);
        }
    }
    fprintf(stderr, "[v4_aud] Shutting down capture thread...\n");
}

int v4_channel_bind(char index)
{
    int ret;

    if (ret = v4_vpss.fnEnableChannel(_v4_vpss_grp, index))
        return ret;

    {
        v4_sys_bind source = { .module = V4_SYS_MOD_VPSS, 
            .device = _v4_vpss_grp, .channel = index };
        v4_sys_bind dest = { .module = V4_SYS_MOD_VENC,
            .device = _v4_venc_dev, .channel = index };
        if (ret = v4_sys.fnBind(&source, &dest))
            return ret;
    }

    return EXIT_SUCCESS;
}

int v4_channel_create(char index, char mirror, char flip, char framerate)
{
    int ret;

    {
        v4_vpss_chn channel;
        memset(&channel, 0, sizeof(channel));
        channel.dest.width = v4_config.isp.capt.width;
        channel.dest.height = v4_config.isp.capt.height;
        channel.pixFmt = V4_PIXFMT_YVU420SP;
        channel.hdr = V4_HDR_SDR8;
        channel.srcFps = v4_config.isp.framerate;
        channel.dstFps = framerate;
        channel.mirror = mirror;
        channel.flip = flip;
        if (ret = v4_vpss.fnSetChannelConfig(_v4_vpss_grp, index, &channel))
            return ret;
    }

    return EXIT_SUCCESS;
}

int v4_channel_grayscale(char enable)
{
    int last_ret = EXIT_SUCCESS;
    const int active = enable ? 1 : 0;

    // Preferred path: use the direct SDK toggle when available.
    // This is especially important for MJPEG/JPEG, where some SDK builds crash on GetChnParam.
    if (v4_venc.fnSetColorToGray) {
        for (char i = 0; i < V4_VENC_CHN_NUM; i++) {
            if (!v4_state[i].enable) continue;

            const bool is_h26x =
                (v4_state[i].payload == HAL_VIDCODEC_H264 ||
                 v4_state[i].payload == HAL_VIDCODEC_H265);
            const bool is_mjpeg =
                (v4_state[i].payload == HAL_VIDCODEC_MJPG ||
                 v4_state[i].payload == HAL_VIDCODEC_JPG);

            if (!is_h26x && !(app_config.jpeg_grayscale_night && is_mjpeg))
                continue;

            const int ret = v4_venc.fnSetColorToGray(i, (int *)&active);
            if (ret) last_ret = ret;
        }
        return last_ret;
    }

    // Fallback: Get/SetChnParam.
    for (char i = 0; i < V4_VENC_CHN_NUM; i++) {
        if (!v4_state[i].enable) continue;
        const bool is_h26x =
            (v4_state[i].payload == HAL_VIDCODEC_H264 ||
             v4_state[i].payload == HAL_VIDCODEC_H265);
        const bool is_mjpeg =
            (v4_state[i].payload == HAL_VIDCODEC_MJPG ||
             v4_state[i].payload == HAL_VIDCODEC_JPG);

        if (!is_h26x && !(app_config.jpeg_grayscale_night && is_mjpeg))
            continue;
        v4_venc_para param;
        int ret = v4_venc.fnGetChannelParam(i, &param);
        if (ret) return ret;
        param.grayscaleOn = enable;
        ret = v4_venc.fnSetChannelParam(i, &param);
        if (ret) return ret;
    }

    return EXIT_SUCCESS;
}

int v4_channel_set_orientation(char mirror, char flip, char h26x_fps, char mjpeg_fps)
{
    int last_ret = EXIT_SUCCESS;

    for (char i = 0; i < V4_VENC_CHN_NUM; i++) {
        if (!v4_state[i].enable) continue;

        char fps = h26x_fps;
        if (v4_state[i].payload == HAL_VIDCODEC_MJPG)
            fps = mjpeg_fps;
        else if (v4_state[i].payload == HAL_VIDCODEC_JPG)
            fps = 1;
        else if (v4_state[i].payload == HAL_VIDCODEC_UNSPEC)
            continue;

        v4_vpss_chn channel;
        memset(&channel, 0, sizeof(channel));
        channel.dest.width = v4_config.isp.capt.width;
        channel.dest.height = v4_config.isp.capt.height;
        channel.pixFmt = V4_PIXFMT_YVU420SP;
        channel.hdr = V4_HDR_SDR8;
        channel.srcFps = v4_config.isp.framerate;
        channel.dstFps = fps;
        channel.mirror = mirror;
        channel.flip = flip;

        int ret = v4_vpss.fnSetChannelConfig(_v4_vpss_grp, i, &channel);
        if (ret) {
            // Some SDKs require the channel to be disabled before updating attrs.
            v4_vpss.fnDisableChannel(_v4_vpss_grp, i);
            ret = v4_vpss.fnSetChannelConfig(_v4_vpss_grp, i, &channel);
            if (!ret)
                ret = v4_vpss.fnEnableChannel(_v4_vpss_grp, i);
        }

        if (ret) {
            last_ret = ret;
        }
    }

    return last_ret;
}

int v4_channel_unbind(char index)
{
    int ret;

    if (ret = v4_vpss.fnDisableChannel(_v4_vpss_grp, index))
        return ret;

    {
        v4_sys_bind source = { .module = V4_SYS_MOD_VPSS, 
            .device = _v4_vpss_grp, .channel = index };
        v4_sys_bind dest = { .module = V4_SYS_MOD_VENC,
            .device = _v4_venc_dev, .channel = index };
        if (ret = v4_sys.fnUnbind(&source, &dest))
            return ret;
    }

    return EXIT_SUCCESS;
}

void *v4_image_thread(void)
{
    int ret;

    // Some SDKs/firmwares overwrite ISP attributes when ISP_Run starts.
    // Re-apply IQ shortly after Run begins, in a helper thread.
    if (_v4_iq_cfg_path[0]) {
        pthread_t tid;
        v4_iq_delayed_apply_ctx *ctx = (v4_iq_delayed_apply_ctx *)calloc(1, sizeof(*ctx));
        if (ctx) {
            snprintf(ctx->path, sizeof(ctx->path), "%s", _v4_iq_cfg_path);
            ctx->pipe = _v4_vi_pipe;
            if (pthread_create(&tid, NULL, v4_iq_delayed_apply_thread, ctx) == 0)
                pthread_detach(tid);
            else
                free(ctx);
        }
    }

    // Start dynamic IQ thread (by ISO) if dynamic sections are present.
    if (_v4_iq_cfg_path[0])
        v4_iq_dyn_maybe_start(_v4_vi_pipe);

    if (ret = v4_isp.fnRun(_v4_isp_dev))
        HAL_DANGER("v4_isp", "Operation failed with %#x!\n", ret);
    HAL_INFO("v4_isp", "Shutting down ISP thread...\n");
}

// ---- IQ (scene_auto-style) INI loader (Goke/HiSilicon v4) ----
typedef int HI_BOOL;
typedef unsigned char HI_U8;
typedef unsigned short HI_U16;
typedef unsigned int HI_U32;
typedef unsigned long long HI_U64;
typedef long long HI_S64;
typedef signed char HI_S8;
typedef short HI_S16;
typedef int HI_S32;

#ifndef HI_TRUE
#define HI_TRUE 1
#endif
#ifndef HI_FALSE
#define HI_FALSE 0
#endif

typedef enum {
    OP_TYPE_AUTO = 0,
    OP_TYPE_MANUAL = 1
} ISP_OP_TYPE_E;

typedef enum {
    LONG_FRAME = 0,
    SHORT_FRAME = 1,
    PRIOR_FRAME_BUTT
} ISP_PRIOR_FRAME_E;

typedef enum {
    ISP_IRIS_F_NO_32_0 = 0,
    ISP_IRIS_F_NO_22_0,
    ISP_IRIS_F_NO_16_0,
    ISP_IRIS_F_NO_11_0,
    ISP_IRIS_F_NO_8_0,
    ISP_IRIS_F_NO_5_6,
    ISP_IRIS_F_NO_4_0,
    ISP_IRIS_F_NO_2_8,
    ISP_IRIS_F_NO_2_0,
    ISP_IRIS_F_NO_1_4,
    ISP_IRIS_F_NO_1_0,
    ISP_IRIS_F_NO_BUTT,
} ISP_IRIS_F_NO_E;

typedef struct {
    HI_U32 u32Max;
    HI_U32 u32Min;
} ISP_AE_RANGE_S;

typedef struct {
    HI_U16 u16BlackDelayFrame;
    HI_U16 u16WhiteDelayFrame;
} ISP_AE_DELAY_S;

typedef enum {
    AE_MODE_SLOW_SHUTTER = 0,
    AE_MODE_FIX_FRAME_RATE = 1,
    AE_MODE_BUTT
} ISP_AE_MODE_E;

typedef enum {
    AE_EXP_HIGHLIGHT_PRIOR = 0,
    AE_EXP_LOWLIGHT_PRIOR = 1,
    AE_STRATEGY_MODE_BUTT
} ISP_AE_STRATEGY_E;

typedef enum {
    ISP_ANTIFLICKER_NORMAL_MODE = 0x0,
    ISP_ANTIFLICKER_AUTO_MODE = 0x1,
    ISP_ANTIFLICKER_MODE_BUTT
} ISP_ANTIFLICKER_MODE_E;

typedef struct {
    HI_BOOL bEnable;
    HI_U8 u8Frequency;
    ISP_ANTIFLICKER_MODE_E enMode;
} ISP_ANTIFLICKER_S;

typedef struct {
    HI_BOOL bEnable;
    HI_U8 u8LumaDiff;
} ISP_SUBFLICKER_S;

typedef enum {
    ISP_FSWDR_NORMAL_MODE = 0x0,
    ISP_FSWDR_LONG_FRAME_MODE = 0x1,
    ISP_FSWDR_AUTO_LONG_FRAME_MODE = 0x2,
    ISP_FSWDR_MODE_BUTT
} ISP_FSWDR_MODE_E;

typedef struct {
    ISP_AE_RANGE_S stExpTimeRange;
    ISP_AE_RANGE_S stAGainRange;
    ISP_AE_RANGE_S stDGainRange;
    ISP_AE_RANGE_S stISPDGainRange;
    ISP_AE_RANGE_S stSysGainRange;
    HI_U32 u32GainThreshold;

    HI_U8 u8Speed;
    HI_U16 u16BlackSpeedBias;
    HI_U8 u8Tolerance;
    HI_U8 u8Compensation;
    HI_U16 u16EVBias;
    ISP_AE_STRATEGY_E enAEStrategyMode;
    HI_U16 u16HistRatioSlope;
    HI_U8 u8MaxHistOffset;

    ISP_AE_MODE_E enAEMode;
    ISP_ANTIFLICKER_S stAntiflicker;
    ISP_SUBFLICKER_S stSubflicker;
    ISP_AE_DELAY_S stAEDelayAttr;

    HI_BOOL bManualExpValue;
    HI_U32 u32ExpValue;

    ISP_FSWDR_MODE_E enFSWDRMode;
    HI_BOOL bWDRQuick;

    HI_U16 u16ISOCalCoef;
} ISP_AE_ATTR_S;

typedef struct {
    ISP_OP_TYPE_E enExpTimeOpType;
    ISP_OP_TYPE_E enAGainOpType;
    ISP_OP_TYPE_E enDGainOpType;
    ISP_OP_TYPE_E enISPDGainOpType;

    HI_U32 u32ExpTime;
    HI_U32 u32AGain;
    HI_U32 u32DGain;
    HI_U32 u32ISPDGain;
} ISP_ME_ATTR_S;

typedef struct {
    HI_BOOL bByPass;
    ISP_OP_TYPE_E enOpType;
    HI_U8 u8AERunInterval;
    HI_BOOL bHistStatAdjust;
    HI_BOOL bAERouteExValid;
    ISP_ME_ATTR_S stManual;
    ISP_AE_ATTR_S stAuto;
    ISP_PRIOR_FRAME_E enPriorFrame;
    HI_BOOL bAEGainSepCfg;
} ISP_EXPOSURE_ATTR_S;

#ifndef HIST_NUM
#define HIST_NUM 1024
#endif

#define ISP_AE_ROUTE_MAX_NODES 16
typedef struct {
    HI_U32 u32IntTime;
    HI_U32 u32SysGain;
    ISP_IRIS_F_NO_E enIrisFNO;
    HI_U32 u32IrisFNOLin;
} ISP_AE_ROUTE_NODE_S;

typedef struct {
    HI_U32 u32TotalNum;
    ISP_AE_ROUTE_NODE_S astRouteNode[ISP_AE_ROUTE_MAX_NODES];
} ISP_AE_ROUTE_S;

static HI_U32 v4_iq_sysgain_from_routeex(HI_U32 again, HI_U32 dgain, HI_U32 ispdgain) {
    // Gains are typically 22.10 fixed point (1.0 == 0x400).
    // sysgain should be 22.10 too, so multiply and shift back by 20.
    if (again == 0) again = 0x400;
    if (dgain == 0) dgain = 0x400;
    if (ispdgain == 0) ispdgain = 0x400;
    HI_U64 prod = (HI_U64)again * (HI_U64)dgain;
    prod = (prod * (HI_U64)ispdgain);
    HI_U64 sg = (prod >> 20);
    if (sg > 0xFFFFFFFFULL) sg = 0xFFFFFFFFULL;
    if (sg < 0x400ULL) sg = 0x400ULL;
    return (HI_U32)sg;
}

#define ISP_AE_ROUTE_EX_MAX_NODES 16
typedef struct {
    HI_U32 u32IntTime;
    HI_U32 u32Again;
    HI_U32 u32Dgain;
    HI_U32 u32IspDgain;
    ISP_IRIS_F_NO_E enIrisFNO;
    HI_U32 u32IrisFNOLin;
} ISP_AE_ROUTE_EX_NODE_S;

typedef struct {
    HI_U32 u32TotalNum;
    ISP_AE_ROUTE_EX_NODE_S astRouteExNode[ISP_AE_ROUTE_EX_MAX_NODES];
} ISP_AE_ROUTE_EX_S;

// Minimal-but-layout-safe exposure info used for dynamic IQ by ISO.
// Keep the full struct (incl. HIST_NUM array) to avoid SDK overwriting stack.
typedef struct {
    HI_U32 u32ExpTime;
    HI_U32 u32ShortExpTime;
    HI_U32 u32MedianExpTime;
    HI_U32 u32LongExpTime;
    HI_U32 u32AGain;
    HI_U32 u32DGain;
    HI_U32 u32AGainSF;
    HI_U32 u32DGainSF;
    HI_U32 u32ISPDGain;
    HI_U32 u32Exposure;
    HI_BOOL bExposureIsMAX;
    HI_S16 s16HistError;
    HI_U32 au32AE_Hist1024Value[HIST_NUM];
    HI_U8  u8AveLum;
    HI_U32 u32LinesPer500ms;
    HI_U32 u32PirisFNO;
    HI_U32 u32Fps;
    HI_U32 u32ISO;
    HI_U32 u32ISOSF;
    HI_U32 u32ISOCalibrate;
    HI_U32 u32RefExpRatio;
    HI_U32 u32FirstStableTime;
    ISP_AE_ROUTE_S stAERoute;
    ISP_AE_ROUTE_EX_S stAERouteEx;
    ISP_AE_ROUTE_S stAERouteSF;
    ISP_AE_ROUTE_EX_S stAERouteSFEx;
} ISP_EXP_INFO_S;

int v4_get_isp_avelum(unsigned char *lum) {
    if (!lum) return EXIT_FAILURE;
    if (!v4_isp.fnQueryExposureInfo) return EXIT_FAILURE;
    ISP_EXP_INFO_S expi;
    memset(&expi, 0, sizeof(expi));
    if (v4_isp.fnQueryExposureInfo(_v4_vi_pipe, &expi))
        return EXIT_FAILURE;
    *lum = (unsigned char)expi.u8AveLum;
    return EXIT_SUCCESS;
}

int v4_get_isp_exposure_info(unsigned int *iso, unsigned int *exp_time,
    unsigned int *again, unsigned int *dgain, unsigned int *ispdgain,
    int *exposure_is_max) {
    if (!iso || !exp_time || !again || !dgain || !ispdgain || !exposure_is_max)
        return EXIT_FAILURE;
    if (!v4_isp.fnQueryExposureInfo) return EXIT_FAILURE;
    ISP_EXP_INFO_S expi;
    memset(&expi, 0, sizeof(expi));
    if (v4_isp.fnQueryExposureInfo(_v4_vi_pipe, &expi))
        return EXIT_FAILURE;
    *iso = (unsigned int)expi.u32ISO;
    *exp_time = (unsigned int)expi.u32ExpTime;
    *again = (unsigned int)expi.u32AGain;
    *dgain = (unsigned int)expi.u32DGain;
    *ispdgain = (unsigned int)expi.u32ISPDGain;
    *exposure_is_max = (int)(expi.bExposureIsMAX ? 1 : 0);
    return EXIT_SUCCESS;
}

#define CCM_MATRIX_SIZE 9
#define CCM_MATRIX_NUM 7
typedef struct {
    HI_BOOL bSatEn;
    HI_U16 au16CCM[CCM_MATRIX_SIZE];
} ISP_COLORMATRIX_MANUAL_S;

typedef struct {
    HI_U16 u16ColorTemp;
    HI_U16 au16CCM[CCM_MATRIX_SIZE];
} ISP_COLORMATRIX_PARAM_S;

typedef struct {
    HI_BOOL bISOActEn;
    HI_BOOL bTempActEn;
    HI_U16 u16CCMTabNum;
    ISP_COLORMATRIX_PARAM_S astCCMTab[CCM_MATRIX_NUM];
} ISP_COLORMATRIX_AUTO_S;

typedef struct {
    ISP_OP_TYPE_E enOpType;
    ISP_COLORMATRIX_MANUAL_S stManual;
    ISP_COLORMATRIX_AUTO_S stAuto;
} ISP_COLORMATRIX_ATTR_S;

// Alternative CCM layout used by some MPP generations (e.g. hi3519-style):
// Auto has 3 fixed matrices (High/Mid/Low) with explicit color temperatures.
typedef struct {
    HI_BOOL bISOActEn;
    HI_BOOL bTempActEn;
    HI_U16  u16HighColorTemp;
    HI_U16  au16HighCCM[CCM_MATRIX_SIZE];
    HI_U16  u16MidColorTemp;
    HI_U16  au16MidCCM[CCM_MATRIX_SIZE];
    HI_U16  u16LowColorTemp;
    HI_U16  au16LowCCM[CCM_MATRIX_SIZE];
} ISP_COLORMATRIX_AUTO_HML_S;

typedef struct {
    ISP_OP_TYPE_E enOpType;
    ISP_COLORMATRIX_MANUAL_S stManual;
    ISP_COLORMATRIX_AUTO_HML_S stAuto;
} ISP_COLORMATRIX_ATTR_HML_S;

#define ISP_AUTO_ISO_STRENGTH_NUM 16
typedef struct {
    HI_U8 u8Saturation;
} ISP_SATURATION_MANUAL_S;

typedef struct {
    HI_U8 au8Sat[ISP_AUTO_ISO_STRENGTH_NUM];
} ISP_SATURATION_AUTO_S;

typedef struct {
    ISP_OP_TYPE_E enOpType;
    ISP_SATURATION_MANUAL_S stManual;
    ISP_SATURATION_AUTO_S stAuto;
} ISP_SATURATION_ATTR_S;

// ---- Statistics config (for AE weight table) ----
#ifndef AE_ZONE_ROW
#define AE_ZONE_ROW 15
#endif
#ifndef AE_ZONE_COLUMN
#define AE_ZONE_COLUMN 17
#endif

typedef int ISP_AE_SWITCH_E;
typedef int ISP_AE_FOUR_PLANE_MODE_E;
typedef int ISP_AE_HIST_SKIP_E;
typedef int ISP_AE_HIST_OFFSET_X_E;
typedef int ISP_AE_HIST_OFFSET_Y_E;
typedef int ISP_AE_STAT_MODE_E;

typedef struct {
    ISP_AE_HIST_SKIP_E enHistSkipX;
    ISP_AE_HIST_SKIP_E enHistSkipY;
    ISP_AE_HIST_OFFSET_X_E enHistOffsetX;
    ISP_AE_HIST_OFFSET_Y_E enHistOffsetY;
} ISP_AE_HIST_CONFIG_S;

typedef struct {
    HI_BOOL bEnable;
    HI_U16  u16X;
    HI_U16  u16Y;
    HI_U16  u16W;
    HI_U16  u16H;
} ISP_AE_CROP_S;

typedef struct {
    ISP_AE_SWITCH_E enAESwitch;
    ISP_AE_HIST_CONFIG_S stHistConfig;
    ISP_AE_FOUR_PLANE_MODE_E enFourPlaneMode;
    ISP_AE_STAT_MODE_E enHistMode;
    ISP_AE_STAT_MODE_E enAverMode;
    ISP_AE_STAT_MODE_E enMaxGainMode;
    ISP_AE_CROP_S stCrop;
    HI_U8 au8Weight[AE_ZONE_ROW][AE_ZONE_COLUMN];
} ISP_AE_STATISTICS_CFG_S;

typedef union {
    HI_U64 u64Key;
} ISP_STATISTICS_CTRL_U;

typedef struct {
    ISP_STATISTICS_CTRL_U unKey;
    ISP_AE_STATISTICS_CFG_S stAECfg;
    // Opaque tail: WB + Focus configs (we preserve bytes via Get/Set; size just needs to be >= real)
    HI_U8 _tail[4096];
} ISP_STATISTICS_CFG_S;

// Minimal ModuleControl union (we only need u64Key bitmask).
// In Hi/GK SDK, bit 8 is bitBypassDRC.
typedef union {
    HI_U64 u64Key;
} ISP_MODULE_CTRL_U;

// ---- LDCI ----
typedef struct {
    HI_U8 u8Wgt;
    HI_U8 u8Sigma;
    HI_U8 u8Mean;
} ISP_LDCI_GAUSS_COEF_ATTR_S;

typedef struct {
    ISP_LDCI_GAUSS_COEF_ATTR_S stHePosWgt;
    ISP_LDCI_GAUSS_COEF_ATTR_S stHeNegWgt;
} ISP_LDCI_HE_WGT_ATTR_S;

typedef struct {
    ISP_LDCI_HE_WGT_ATTR_S stHeWgt;
    HI_U16 u16BlcCtrl;
} ISP_LDCI_MANUAL_ATTR_S;

typedef struct {
    ISP_LDCI_HE_WGT_ATTR_S astHeWgt[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16 au16BlcCtrl[ISP_AUTO_ISO_STRENGTH_NUM];
} ISP_LDCI_AUTO_ATTR_S;

typedef struct {
    HI_BOOL bEnable;
    HI_U8 u8GaussLPFSigma;
    ISP_OP_TYPE_E enOpType;
    ISP_LDCI_MANUAL_ATTR_S stManual;
    ISP_LDCI_AUTO_ATTR_S stAuto;
    HI_U16 u16TprIncrCoef;
    HI_U16 u16TprDecrCoef;
} ISP_LDCI_ATTR_S;

// ---- Dehaze ----
typedef struct {
    HI_U8 u8strength;
} ISP_DEHAZE_MANUAL_ATTR_S;

typedef struct {
    HI_U8 u8strength;
} ISP_DEHAZE_AUTO_ATTR_S;

typedef struct {
    HI_BOOL bEnable;
    HI_BOOL bUserLutEnable;
    HI_U8 au8DehazeLut[256];
    ISP_OP_TYPE_E enOpType;
    ISP_DEHAZE_MANUAL_ATTR_S stManual;
    ISP_DEHAZE_AUTO_ATTR_S stAuto;
    HI_U16 u16TmprfltIncrCoef;
    HI_U16 u16TmprfltDecrCoef;
} ISP_DEHAZE_ATTR_S;

// ---- DRC ----
#ifndef HI_ISP_DRC_CC_NODE_NUM
#define HI_ISP_DRC_CC_NODE_NUM 33
#endif
#ifndef HI_ISP_DRC_TM_NODE_NUM
#define HI_ISP_DRC_TM_NODE_NUM 200
#endif
#ifndef HI_ISP_DRC_CUBIC_POINT_NUM
#define HI_ISP_DRC_CUBIC_POINT_NUM 5
#endif

typedef struct {
    HI_U16 u16X;
    HI_U16 u16Y;
    HI_U16 u16Slope;
} ISP_DRC_CUBIC_POINT_ATTR_S;

typedef struct {
    HI_U8 u8Asymmetry;
    HI_U8 u8SecondPole;
    HI_U8 u8Stretch;
    HI_U8 u8Compress;
} ISP_DRC_ASYMMETRY_CURVE_ATTR_S;

typedef struct {
    HI_U16 u16Strength;
} ISP_DRC_MANUAL_ATTR_S;

typedef struct {
    HI_U16 u16Strength;
    HI_U16 u16StrengthMax;
    HI_U16 u16StrengthMin;
} ISP_DRC_AUTO_ATTR_S;

typedef enum {
    DRC_CURVE_ASYMMETRY = 0x0,
    DRC_CURVE_CUBIC,
    DRC_CURVE_USER,
    DRC_CURVE_BUTT
} ISP_DRC_CURVE_SELECT_E;

typedef struct {
    HI_BOOL bEnable;
    ISP_DRC_CURVE_SELECT_E enCurveSelect;
    HI_U8  u8PDStrength;
    HI_U8  u8LocalMixingBrightMax;
    HI_U8  u8LocalMixingBrightMin;
    HI_U8  u8LocalMixingBrightThr;
    HI_S8  s8LocalMixingBrightSlo;
    HI_U8  u8LocalMixingDarkMax;
    HI_U8  u8LocalMixingDarkMin;
    HI_U8  u8LocalMixingDarkThr;
    HI_S8  s8LocalMixingDarkSlo;

    HI_U8  u8DetailBrightStr;
    HI_U8  u8DetailDarkStr;
    HI_U8  u8DetailBrightStep;
    HI_U8  u8DetailDarkStep;

    HI_U8  u8BrightGainLmt;
    HI_U8  u8BrightGainLmtStep;
    HI_U8  u8DarkGainLmtY;
    HI_U8  u8DarkGainLmtC;
    HI_U16 au16ColorCorrectionLut[HI_ISP_DRC_CC_NODE_NUM];
    HI_U16 au16ToneMappingValue[HI_ISP_DRC_TM_NODE_NUM];

    HI_U8  u8FltScaleCoarse;
    HI_U8  u8FltScaleFine;
    HI_U8  u8ContrastControl;
    HI_S8  s8DetailAdjustFactor;

    HI_U8  u8SpatialFltCoef;
    HI_U8  u8RangeFltCoef;
    HI_U8  u8RangeAdaMax;

    HI_U8  u8GradRevMax;
    HI_U8  u8GradRevThr;

    HI_U8  u8DpDetectRangeRatio;
    HI_U8  u8DpDetectThrSlo;
    HI_U16 u16DpDetectThrMin;

    ISP_OP_TYPE_E enOpType;
    ISP_DRC_MANUAL_ATTR_S stManual;
    ISP_DRC_AUTO_ATTR_S   stAuto;
    ISP_DRC_CUBIC_POINT_ATTR_S astCubicPoint[HI_ISP_DRC_CUBIC_POINT_NUM];
    ISP_DRC_ASYMMETRY_CURVE_ATTR_S stAsymmetryCurve;
} ISP_DRC_ATTR_S;

int v4_get_drc_strength(unsigned int *strength) {
    if (!strength) return EXIT_FAILURE;
    if (!v4_isp.fnGetDRCAttr) return EXIT_FAILURE;
    ISP_DRC_ATTR_S da;
    memset(&da, 0, sizeof(da));
    if (v4_isp.fnGetDRCAttr(_v4_vi_pipe, &da))
        return EXIT_FAILURE;
    // Prefer the currently-selected op type.
    if (da.enOpType == OP_TYPE_AUTO)
        *strength = (unsigned int)da.stAuto.u16Strength;
    else
        *strength = (unsigned int)da.stManual.u16Strength;
    return EXIT_SUCCESS;
}

// ---- NR ----
#ifndef ISP_BAYER_CHN_NUM
#define ISP_BAYER_CHN_NUM 4
#endif
#ifndef HI_ISP_BAYERNR_LUT_LENGTH
#define HI_ISP_BAYERNR_LUT_LENGTH 33
#endif
#ifndef WDR_MAX_FRAME_NUM
#define WDR_MAX_FRAME_NUM 4
#endif

typedef struct {
    HI_U8   au8ChromaStr[ISP_BAYER_CHN_NUM];
    HI_U8   u8FineStr;
    HI_U16  u16CoringWgt;
    HI_U16  au16CoarseStr[ISP_BAYER_CHN_NUM];
} ISP_NR_MANUAL_ATTR_S;

typedef struct {
    HI_U8   au8ChromaStr[ISP_BAYER_CHN_NUM][ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8   au8FineStr[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  au16CoringWgt[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16  au16CoarseStr[ISP_BAYER_CHN_NUM][ISP_AUTO_ISO_STRENGTH_NUM];
} ISP_NR_AUTO_ATTR_S;

typedef struct {
    HI_U8    au8WDRFrameStr[WDR_MAX_FRAME_NUM];
    HI_U8    au8FusionFrameStr[WDR_MAX_FRAME_NUM];
} ISP_NR_WDR_ATTR_S;

typedef struct {
    HI_BOOL  bEnable;
    HI_BOOL  bLowPowerEnable;
    HI_BOOL  bNrLscEnable;
    HI_U8    u8NrLscRatio;
    HI_U8    u8BnrLscMaxGain;
    HI_U16   u16BnrLscCmpStrength;
    HI_U16   au16CoringRatio[HI_ISP_BAYERNR_LUT_LENGTH];

    ISP_OP_TYPE_E enOpType;
    ISP_NR_AUTO_ATTR_S stAuto;
    ISP_NR_MANUAL_ATTR_S stManual;
    ISP_NR_WDR_ATTR_S  stWdr;
} ISP_NR_ATTR_S;

// ---- Gamma ----
#ifndef GAMMA_NODE_NUM
#define GAMMA_NODE_NUM 1025
#endif

typedef enum {
    ISP_GAMMA_CURVE_DEFAULT = 0x0,
    ISP_GAMMA_CURVE_SRGB,
    ISP_GAMMA_CURVE_HDR,
    ISP_GAMMA_CURVE_USER_DEFINE,
    ISP_GAMMA_CURVE_BUTT
} ISP_GAMMA_CURVE_TYPE_E;

typedef struct {
    HI_BOOL   bEnable;
    HI_U16    u16Table[GAMMA_NODE_NUM];
    ISP_GAMMA_CURVE_TYPE_E enCurveType;
} ISP_GAMMA_ATTR_S;

// ---- Sharpen ----
#ifndef ISP_SHARPEN_LUMA_NUM
#define ISP_SHARPEN_LUMA_NUM 32
#endif
#ifndef ISP_SHARPEN_GAIN_NUM
#define ISP_SHARPEN_GAIN_NUM 32
#endif

typedef struct {
    HI_U8  au8LumaWgt[ISP_SHARPEN_LUMA_NUM];
    HI_U16 au16TextureStr[ISP_SHARPEN_GAIN_NUM];
    HI_U16 au16EdgeStr[ISP_SHARPEN_GAIN_NUM];
    HI_U16 u16TextureFreq;
    HI_U16 u16EdgeFreq;
    HI_U8  u8OverShoot;
    HI_U8  u8UnderShoot;
    HI_U8  u8ShootSupStr;
    HI_U8  u8ShootSupAdj;
    HI_U8  u8DetailCtrl;
    HI_U8  u8DetailCtrlThr;
    HI_U8  u8EdgeFiltStr;
    HI_U8  u8EdgeFiltMaxCap;
    HI_U8  u8RGain;
    HI_U8  u8GGain;
    HI_U8  u8BGain;
    HI_U8  u8SkinGain;
    HI_U16 u16MaxSharpGain;
    HI_U8  u8WeakDetailGain;
} ISP_SHARPEN_MANUAL_ATTR_S;

typedef struct {
    HI_U8  au8LumaWgt[ISP_SHARPEN_LUMA_NUM][ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16 au16TextureStr[ISP_SHARPEN_GAIN_NUM][ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16 au16EdgeStr[ISP_SHARPEN_GAIN_NUM][ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16 au16TextureFreq[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16 au16EdgeFreq[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8OverShoot[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8UnderShoot[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8ShootSupStr[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8ShootSupAdj[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8DetailCtrl[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8DetailCtrlThr[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8EdgeFiltStr[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8EdgeFiltMaxCap[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8RGain[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8GGain[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8BGain[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8SkinGain[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U16 au16MaxSharpGain[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8  au8WeakDetailGain[ISP_AUTO_ISO_STRENGTH_NUM];
} ISP_SHARPEN_AUTO_ATTR_S;

typedef struct {
    HI_BOOL bEnable;
    HI_U8 u8SkinUmin;
    HI_U8 u8SkinVmin;
    HI_U8 u8SkinUmax;
    HI_U8 u8SkinVmax;
    ISP_OP_TYPE_E enOpType;
    ISP_SHARPEN_MANUAL_ATTR_S stManual;
    ISP_SHARPEN_AUTO_ATTR_S   stAuto;
} ISP_SHARPEN_ATTR_S;

static int v4_iq_parse_csv_u32(const char *s, HI_U32 *out, int max) {
    int n = 0;
    const char *p = s;
    while (p && *p && n < max) {
        while (*p && (isspace((unsigned char)*p) || *p == ',')) p++;
        if (!*p) break;
        char *end = NULL;
        unsigned long v = strtoul(p, &end, 0);
        if (end == p) break;
        out[n++] = (HI_U32)v;
        p = end;
        while (*p && (isspace((unsigned char)*p) || *p == ',')) p++;
    }
    return n;
}

static const char *v4_iq_skip_ws(const char *p, const char *end) {
    while (p < end && (*p == ' ' || *p == '\t' || *p == '\r'))
        p++;
    return p;
}

static const char *v4_iq_line_end(const char *p, const char *end) {
    while (p < end && *p && *p != '\n')
        p++;
    return p;
}

static int v4_iq_parse_csv_u32_into(const char *s, HI_U32 *out, int max, int *idx) {
    if (!s || !out || !idx) return 0;
    HI_U32 tmp[512];
    int n = v4_iq_parse_csv_u32(s, tmp, (int)(sizeof(tmp) / sizeof(tmp[0])));
    int wrote = 0;
    for (int i = 0; i < n && *idx < max; i++) {
        out[(*idx)++] = tmp[i];
        wrote++;
    }
    return wrote;
}

// Parse "key = \\" multiline lists into an array. Handles '\' line continuations.
static int v4_iq_parse_multiline_u32(
    struct IniConfig *ini, const char *section, const char *key,
    HI_U32 *out, int max) {
    if (!ini || !ini->str || !section || !key || !out || max <= 0)
        return 0;

    int start_pos = 0, end_pos = 0;
    if (section_pos(ini, section, &start_pos, &end_pos) != CONFIG_OK)
        return 0;
    const char *base = ini->str;
    const char *p = base + start_pos;
    const char *end = (end_pos >= 0) ? (base + end_pos) : (base + strlen(base));

    int idx = 0;
    while (p < end) {
        const char *ls = p;
        const char *le = v4_iq_line_end(ls, end);
        p = (le < end) ? (le + 1) : le;

        const char *q = v4_iq_skip_ws(ls, le);
        if (q >= le) continue;
        if (*q == ';' || *q == '#') continue;

        // Match key at line start (case-insensitive)
        size_t klen = strlen(key);
        if ((size_t)(le - q) < klen) continue;
        if (strncasecmp(q, key, klen) != 0) continue;
        const char *r = q + klen;
        r = v4_iq_skip_ws(r, le);
        if (r >= le || (*r != '=' && *r != ':')) continue;
        r++;
        r = v4_iq_skip_ws(r, le);

        bool cont = false;
        // Parse this line's numbers (if any)
        {
            const char *value_end = le;
            // stop at comment
            for (const char *c = r; c < le; c++) {
                if (*c == ';' || *c == '#') { value_end = c; break; }
            }
            // trim right
            while (value_end > r && isspace((unsigned char)value_end[-1]))
                value_end--;
            if (value_end > r && value_end[-1] == '\\') {
                cont = true;
                value_end--;
                while (value_end > r && isspace((unsigned char)value_end[-1]))
                    value_end--;
            }

            // strip quotes for this fragment
            if (value_end - r >= 2 && *r == '"' && value_end[-1] == '"') {
                r++;
                value_end--;
            }

            // special-case: line contains only "\" (i.e. begins continuation)
            if ((value_end - r) == 1 && *r == '\\') {
                cont = true;
            } else if (value_end > r) {
                char tmp[8192];
                size_t n = (size_t)(value_end - r);
                if (n >= sizeof(tmp)) n = sizeof(tmp) - 1;
                memcpy(tmp, r, n);
                tmp[n] = '\0';
                v4_iq_parse_csv_u32_into(tmp, out, max, &idx);
            }
        }

        // Continuation lines: consume until a line without trailing '\'
        while (cont && p < end && idx < max) {
            const char *nls = p;
            const char *nle = v4_iq_line_end(nls, end);
            p = (nle < end) ? (nle + 1) : nle;

            const char *nr = v4_iq_skip_ws(nls, nle);
            if (nr >= nle) { cont = false; break; }
            if (*nr == ';' || *nr == '#') continue;

            const char *value_end = nle;
            for (const char *c = nr; c < nle; c++) {
                if (*c == ';' || *c == '#') { value_end = c; break; }
            }
            while (value_end > nr && isspace((unsigned char)value_end[-1]))
                value_end--;
            cont = false;
            if (value_end > nr && value_end[-1] == '\\') {
                cont = true;
                value_end--;
                while (value_end > nr && isspace((unsigned char)value_end[-1]))
                    value_end--;
            }

            if (value_end - nr >= 2 && *nr == '"' && value_end[-1] == '"') {
                nr++;
                value_end--;
            }

            if (value_end > nr) {
                char tmp[8192];
                size_t n = (size_t)(value_end - nr);
                if (n >= sizeof(tmp)) n = sizeof(tmp) - 1;
                memcpy(tmp, nr, n);
                tmp[n] = '\0';
                v4_iq_parse_csv_u32_into(tmp, out, max, &idx);
            }
        }

        break; // key processed
    }

    return idx;
}

static int v4_iq_parse_csv_u16(const char *s, HI_U16 *out, int max) {
    HI_U32 tmp[CCM_MATRIX_SIZE];
    int n = v4_iq_parse_csv_u32(s, tmp, max);
    for (int i = 0; i < n; i++)
        out[i] = (HI_U16)tmp[i];
    return n;
}

static int v4_iq_parse_csv_u8(const char *s, HI_U8 *out, int max) {
    HI_U32 tmp[ISP_AUTO_ISO_STRENGTH_NUM];
    int n = v4_iq_parse_csv_u32(s, tmp, max);
    for (int i = 0; i < n; i++) {
        HI_U32 v = tmp[i];
        if (v > 255) v = 255;
        out[i] = (HI_U8)v;
    }
    return n;
}

// ---- Dynamic IQ (by ISO): [dynamic_dehaze], [dynamic_linear_drc] ----
#define V4_IQ_DYN_MAX_POINTS 32

typedef struct {
    bool enabled;
    int n;
    HI_U32 iso_thr[V4_IQ_DYN_MAX_POINTS];
    HI_U8  str[V4_IQ_DYN_MAX_POINTS];
} v4_iq_dyn_dehaze_cfg;

typedef struct {
    bool enabled;
    int n;
    HI_U32 iso[V4_IQ_DYN_MAX_POINTS];

    HI_U8  localMixBrightMax[V4_IQ_DYN_MAX_POINTS];
    HI_U8  localMixBrightMin[V4_IQ_DYN_MAX_POINTS];
    HI_U8  localMixDarkMax[V4_IQ_DYN_MAX_POINTS];
    HI_U8  localMixDarkMin[V4_IQ_DYN_MAX_POINTS];
    HI_U8  brightGainLmt[V4_IQ_DYN_MAX_POINTS];
    HI_U8  brightGainLmtStep[V4_IQ_DYN_MAX_POINTS];
    HI_U8  darkGainLmtY[V4_IQ_DYN_MAX_POINTS];
    HI_U8  darkGainLmtC[V4_IQ_DYN_MAX_POINTS];
    HI_U8  fltScaleCoarse[V4_IQ_DYN_MAX_POINTS];
    HI_U8  fltScaleFine[V4_IQ_DYN_MAX_POINTS];
    HI_U8  contrastControl[V4_IQ_DYN_MAX_POINTS];
    HI_S8  detailAdjustFactor[V4_IQ_DYN_MAX_POINTS];

    HI_U8  asymmetry[V4_IQ_DYN_MAX_POINTS];
    HI_U8  secondPole[V4_IQ_DYN_MAX_POINTS];
    HI_U8  compress[V4_IQ_DYN_MAX_POINTS];
    HI_U8  stretch[V4_IQ_DYN_MAX_POINTS];

    HI_U16 strength[V4_IQ_DYN_MAX_POINTS];
} v4_iq_dyn_linear_drc_cfg;

typedef struct {
    HI_U16 strength;
    HI_U8  localMixBrightMax, localMixBrightMin, localMixDarkMax, localMixDarkMin;
    HI_U8  brightGainLmt, brightGainLmtStep, darkGainLmtY, darkGainLmtC;
    HI_U8  fltScaleCoarse, fltScaleFine, contrastControl;
    HI_S8  detailAdjustFactor;
    HI_U8  asymmetry, secondPole, compress, stretch;
} v4_iq_dyn_drc_sig;

// ---- 3DNR (VPSS NRX) profile (parsed from [static_3dnr]/[ir_static_3dnr]) ----
typedef struct {
    HI_S32 sfc, tfc, tpc, trc, mode, presfc;
} v4_iq_3dnr_nrc;

typedef struct {
    bool enabled;
    int n;
    HI_U32 iso[V4_IQ_DYN_MAX_POINTS];
    v4_iq_3dnr_nrc nrc[V4_IQ_DYN_MAX_POINTS];
} v4_iq_3dnr_cfg;

typedef struct {
    pthread_mutex_t lock;
    bool thread_started;
    int pipe;
    // Dynamic configs can have separate day/IR sections:
    //   [dynamic_dehaze] / [ir_dynamic_dehaze]
    //   [dynamic_linear_drc] / [ir_dynamic_linear_drc]
    v4_iq_dyn_dehaze_cfg dehaze_day;
    v4_iq_dyn_dehaze_cfg dehaze_ir;
    v4_iq_dyn_linear_drc_cfg linear_drc_day;
    v4_iq_dyn_linear_drc_cfg linear_drc_ir;

    // all_param hysteresis for ISO-driven profile switching
    HI_U32 upFrameIso;
    HI_U32 downFrameIso;

    // Low-light auto AE (works even if user keeps DAY mode at night)
    HI_BOOL lowlight_auto_ae;
    HI_U32 lowlight_iso;
    HI_U32 lowlight_exptime;
    // Optional "fast lowlight" overrides to reduce motion blur in bright-enough lowlight scenes.
    // Triggered when u8AveLum >= lowlight_fast_lum.
    HI_BOOL have_lowlight_fast;
    HI_U8  lowlight_fast_lum;
    HI_U32 lowlight_fast_expmax;
    HI_U32 lowlight_fast_againmax;
    HI_U8  lowlight_fast_comp_boost;

    // Optional lowlight FX/marketing logic (enabled only if config section exists in IQ).
    HI_BOOL have_lowlight_fx;
    HI_U32 noisy_gamma_iso;
    HI_U32 noisy_gamma_dgain;
    HI_U32 noisy_gamma_ispdgain;
    HI_U32 noisy_sharpen_iso;
    HI_U32 noisy_sharpen_dgain;
    HI_U32 noisy_sharpen_ispdgain;

    // Optional lowlight anti-halo sharpen tuning (enabled only if section exists).
    HI_BOOL have_lowlight_sharpen_ah;
    HI_U8  ah_over_pct;
    HI_U8  ah_under_pct;
    HI_U8  ah_min_over;
    HI_U8  ah_min_under;
    HI_U8  ah_shootsup_min;
    HI_U8  ah_edgefilt_min;
    HI_U16 ah_maxsharpgain_cap;
    HI_U8  ah_edgestr_pct;
    HI_U8  ah_texstr_pct;

    // Optional lowlight DRC caps (enabled only if section exists).
    HI_BOOL have_lowlight_drc_caps;
    HI_U16 ll_drc_strength_max;
    HI_U16 fast_drc_strength_max;
    HI_U8  ll_bright_mix_max;
    HI_U8  ll_bright_mix_min;
    HI_U8  ll_bright_gain_lmt;
    HI_U8  ll_bright_gain_step;
    HI_U8  ll_dark_mix_max;
    HI_U8  ll_dark_mix_min;
    HI_U8  ll_contrast_max;
    HI_U8  fast_dark_mix_max;
    HI_U8  fast_dark_mix_min;
    HI_U8  fast_contrast_max;
    HI_BOOL have_ae_day;
    HI_BOOL have_ae_low;
    HI_BOOL last_ae_lowlight;
    HI_U8  ae_day_comp, ae_low_comp;
    HI_U32 ae_day_expmax, ae_low_expmax;
    HI_U32 ae_day_sysgainmax, ae_low_sysgainmax;
    HI_U32 ae_day_againmax, ae_low_againmax;
    HI_U8  ae_day_histoff, ae_low_histoff;
    HI_U16 ae_day_histslope, ae_low_histslope;
    HI_U8  ae_day_speed, ae_low_speed;

    // 3DNR (VPSS NRX) day/ir profiles
    v4_iq_3dnr_cfg nr3d_day;
    v4_iq_3dnr_cfg nr3d_ir;
    int last_nr3d_idx;
    HI_BOOL last_nr3d_is_ir;
    int nr3d_fail_count;
    HI_BOOL nr3d_disabled;
    HI_BOOL nr3d_probe_logged;

    // Cache "not supported / rejected by SDK" to avoid repeating warnings every IQ apply.
    // Also store presence of AE route tables so the dynamic thread can start even if
    // only mode-switchable AE route/route-ex is desired.
    HI_BOOL have_aerouteex_day;
    HI_BOOL have_aerouteex_ir;
    HI_BOOL aerouteex_disabled;
    HI_BOOL ccm_disabled;

    HI_BOOL have_last;
    HI_U8  last_dehaze_strength;
    v4_iq_dyn_drc_sig last_drc;
    int drc_fail_count;
    HI_BOOL drc_disabled;
} v4_iq_dyn_state;

static v4_iq_dyn_state _v4_iq_dyn = { .lock = PTHREAD_MUTEX_INITIALIZER };

int v4_get_iq_lowlight_state(unsigned int iso, unsigned int exp_time, int *active) {
    if (!active) return EXIT_FAILURE;

    // LowLightAutoAE is designed for "DAY-at-night" (color) usage.
    // In true night/IR mode we must NOT activate lowlight switching; instead use [ir_*] sections.
    if (night_mode_on()) {
        *active = 0;
        return EXIT_SUCCESS;
    }

    HI_BOOL ll_ae = HI_FALSE;
    HI_BOOL have_day = HI_FALSE;
    HI_BOOL have_low = HI_FALSE;
    HI_U32 ll_iso = 0;
    HI_U32 ll_exptime = 0;

    pthread_mutex_lock(&_v4_iq_dyn.lock);
    ll_ae = _v4_iq_dyn.lowlight_auto_ae;
    have_day = _v4_iq_dyn.have_ae_day;
    have_low = _v4_iq_dyn.have_ae_low;
    ll_iso = _v4_iq_dyn.lowlight_iso;
    ll_exptime = _v4_iq_dyn.lowlight_exptime;
    pthread_mutex_unlock(&_v4_iq_dyn.lock);

    *active = (ll_ae && have_day && have_low &&
        ((HI_U32)iso >= ll_iso || (HI_U32)exp_time >= ll_exptime)) ? 1 : 0;
    return EXIT_SUCCESS;
}

int v4_get_ae_auto_params(unsigned int *comp, unsigned int *expmax, unsigned int *sysgainmax) {
    if (!comp || !expmax || !sysgainmax) return EXIT_FAILURE;
    if (!v4_isp.fnGetExposureAttr) return EXIT_FAILURE;
    ISP_EXPOSURE_ATTR_S ea;
    memset(&ea, 0, sizeof(ea));
    if (v4_isp.fnGetExposureAttr(_v4_vi_pipe, &ea))
        return EXIT_FAILURE;
    *comp = (unsigned int)ea.stAuto.u8Compensation;
    *expmax = (unsigned int)ea.stAuto.stExpTimeRange.u32Max;
    *sysgainmax = (unsigned int)ea.stAuto.stSysGainRange.u32Max;
    return EXIT_SUCCESS;
}

static inline HI_U32 v4_iq_dyn_interp_u32(HI_U32 x, const HI_U32 *xp, const HI_U32 *yp, int n) {
    if (n <= 1) return (n == 1) ? yp[0] : 0;
    if (x <= xp[0]) return yp[0];
    if (x >= xp[n - 1]) return yp[n - 1];
    for (int i = 0; i < n - 1; i++) {
        HI_U32 x0 = xp[i], x1 = xp[i + 1];
        if (x >= x0 && x <= x1) {
            HI_U32 y0 = yp[i], y1 = yp[i + 1];
            if (x1 == x0) return y1;
            // IMPORTANT: y can be decreasing (e.g. Strength 260->250->...).
            // Use signed math to avoid unsigned underflow producing garbage.
            HI_S64 dy = (HI_S64)y1 - (HI_S64)y0;
            HI_S64 num = dy * (HI_S64)(x - x0);
            HI_S64 den = (HI_S64)(x1 - x0);
            HI_S64 val = (HI_S64)y0 + (den ? (num / den) : 0);
            if (val < 0) val = 0;
            return (HI_U32)val;
        }
    }
    return yp[n - 1];
}

static inline HI_S32 v4_iq_dyn_interp_s32(HI_U32 x, const HI_U32 *xp, const HI_S32 *yp, int n) {
    if (n <= 1) return (n == 1) ? yp[0] : 0;
    if (x <= xp[0]) return yp[0];
    if (x >= xp[n - 1]) return yp[n - 1];
    for (int i = 0; i < n - 1; i++) {
        HI_U32 x0 = xp[i], x1 = xp[i + 1];
        if (x >= x0 && x <= x1) {
            HI_S32 y0 = yp[i], y1 = yp[i + 1];
            if (x1 == x0) return y1;
            return (HI_S32)(y0 + (HI_S64)(y1 - y0) * (HI_S64)(x - x0) / (HI_S64)(x1 - x0));
        }
    }
    return yp[n - 1];
}

static int v4_iq_parse_multiline_str(
    struct IniConfig *ini, const char *section, const char *key,
    char *out, size_t out_sz) {
    if (!ini || !ini->str || !section || !key || !out || out_sz == 0)
        return 0;
    out[0] = '\0';

    int start_pos = 0, end_pos = 0;
    if (section_pos(ini, section, &start_pos, &end_pos) != CONFIG_OK)
        return 0;

    const char *base = ini->str;
    const char *p = base + start_pos;
    const char *end = (end_pos >= 0) ? (base + end_pos) : (base + strlen(base));
    size_t w = 0;

    while (p < end) {
        const char *ls = p;
        const char *le = v4_iq_line_end(ls, end);
        p = (le < end) ? (le + 1) : le;

        const char *q = v4_iq_skip_ws(ls, le);
        if (q >= le) continue;
        if (*q == ';' || *q == '#') continue;

        size_t klen = strlen(key);
        if ((size_t)(le - q) < klen) continue;
        if (strncasecmp(q, key, klen) != 0) continue;
        const char *r = q + klen;
        r = v4_iq_skip_ws(r, le);
        if (r >= le || (*r != '=' && *r != ':')) continue;
        r++;
        r = v4_iq_skip_ws(r, le);

        bool cont = false;
        const char *value_end = le;
        for (const char *c = r; c < le; c++) {
            if (*c == ';' || *c == '#') { value_end = c; break; }
        }
        while (value_end > r && isspace((unsigned char)value_end[-1]))
            value_end--;
        if (value_end > r && value_end[-1] == '\\') {
            cont = true;
            value_end--;
            while (value_end > r && isspace((unsigned char)value_end[-1]))
                value_end--;
        }
        if (value_end - r >= 2 && *r == '"' && value_end[-1] == '"') {
            r++; value_end--;
        }
        if ((value_end - r) == 1 && *r == '\\') {
            cont = true;
        } else if (value_end > r) {
            size_t n = (size_t)(value_end - r);
            if (w + n + 2 >= out_sz) n = (out_sz > w + 2) ? (out_sz - w - 2) : 0;
            if (n) {
                memcpy(out + w, r, n);
                w += n;
                out[w++] = ' ';
                out[w] = '\0';
            }
        }

        while (cont && p < end) {
            const char *nls = p;
            const char *nle = v4_iq_line_end(nls, end);
            p = (nle < end) ? (nle + 1) : nle;

            const char *nr = v4_iq_skip_ws(nls, nle);
            if (nr >= nle) { cont = false; break; }
            if (*nr == ';' || *nr == '#') continue;

            const char *ve = nle;
            for (const char *c = nr; c < nle; c++) {
                if (*c == ';' || *c == '#') { ve = c; break; }
            }
            while (ve > nr && isspace((unsigned char)ve[-1]))
                ve--;
            cont = false;
            if (ve > nr && ve[-1] == '\\') {
                cont = true;
                ve--;
                while (ve > nr && isspace((unsigned char)ve[-1]))
                    ve--;
            }
            if (ve - nr >= 2 && *nr == '"' && ve[-1] == '"') {
                nr++; ve--;
            }
            if (ve > nr) {
                size_t n = (size_t)(ve - nr);
                if (w + n + 2 >= out_sz) n = (out_sz > w + 2) ? (out_sz - w - 2) : 0;
                if (n) {
                    memcpy(out + w, nr, n);
                    w += n;
                    out[w++] = ' ';
                    out[w] = '\0';
                }
            }
        }
        break;
    }

    // trim
    while (w > 0 && isspace((unsigned char)out[w - 1]))
        out[--w] = '\0';
    return (int)w;
}

// Minimal VPSS NRX V3 (Hi3516EV200-style) structs to access NRc.
// We keep the full V3 layout so Get/SetGrpNRXParam doesn't overwrite the stack.
typedef int OPERATION_MODE_E;
typedef int VPSS_NR_VER_E;

typedef struct {
    HI_U8  IES0, IES1, IES2, IES3;
    HI_U16 IEDZ : 10, IEEn : 1, _rb_ : 5;
} tV200_VPSS_IEy;

typedef struct {
    HI_U8  SPN6 : 3, SFR  : 5;
    HI_U8  SBN6 : 3, PBR6 : 5;
    HI_U16 SRT0 : 5, SRT1 : 5, JMODE : 3, DeIdx : 3;
    HI_U8  SFR6[4], SBR6[2], DeRate;
    HI_U8  SFS1,  SFT1,  SBR1;
    HI_U8  SFS2,  SFT2,  SBR2;
    HI_U8  SFS4,  SFT4,  SBR4;
    HI_U16 STH1 : 9,  SFN1 : 3, SFN0  : 3, NRyEn   : 1;
    HI_U16 STHd1 : 9, _rb0_ : 7;
    HI_U16 STH2 : 9,  SFN2 : 3, kMode : 3, _rb1_   : 1;
    HI_U16 STHd2 : 9, _rb2_ : 7;
    HI_U16 SBSk[32], SDSk[32];
} tV200_VPSS_SFy;

typedef struct {
    HI_U16 TFS0 : 4,   TDZ0 : 10,  TDX0    : 2;
    HI_U16 TFS1 : 4,   TDZ1 : 10,  TDX1    : 2;
    HI_U16 SDZ0 : 10,  STR0 : 5,   DZMode0 : 1;
    HI_U16 SDZ1 : 10,  STR1 : 5,   DZMode1 : 1;
    HI_U8  TSS0 : 4,   TSI0 : 4,  TFR0[6];
    HI_U8  TSS1 : 4,   TSI1 : 4,  TFR1[6];
    HI_U8  TFRS : 4,   TED  : 2,   bRef    : 1,  _rb_ : 1;
} tV200_VPSS_TFy;

typedef struct {
    HI_U16 MADZ0   : 9,   MAI00 : 2,  MAI01  : 2, MAI02 : 2, _rb0_ : 1;
    HI_U16 MADZ1   : 9,   MAI10 : 2,  MAI11  : 2, MAI12 : 2, _rb1_ : 1;
    HI_U8  MABR0, MABR1;
    HI_U16 MATH0   : 10,  MATE0 : 4,  MATW   : 2;
    HI_U16 MATHd0  : 10,  _rb2_ : 6;
    HI_U16 MATH1   : 10,  _rb3_ : 6;
    HI_U16 MATHd1  : 10,  _rb4_ : 6;
    HI_U8  MASW    :  4,  MATE1 : 4;
    HI_U8  MABW0   :  4,  MABW1 : 4;
    HI_U16 AdvMATH : 1,   AdvTH : 12, _rb5_  : 3;
} tV200_VPSS_MDy;

typedef struct {
    HI_U8  SFC, TFC : 6, _rb0_ : 2;
    HI_U8  TRC, TPC : 6, _rb1_ : 2;
    HI_U8  MODE : 1, _rb2_ : 7;
    HI_U8  PRESFC : 6, _rb3_ : 2;
} tV200_VPSS_NRc;

typedef struct {
    tV200_VPSS_IEy IEy[5];
    tV200_VPSS_SFy SFy[5];
    tV200_VPSS_MDy MDy[2];
    tV200_VPSS_TFy TFy[3];
    tV200_VPSS_NRc NRc;
} VPSS_NRX_V3_S;

typedef struct { VPSS_NRX_V3_S stNRXParam; } VPSS_NRX_PARAM_MANUAL_V3_S;
typedef struct { HI_U32 u32ParamNum; HI_U32 *pau32ISO; VPSS_NRX_V3_S *pastNRXParam; } VPSS_NRX_PARAM_AUTO_V3_S;
typedef struct {
    OPERATION_MODE_E enOptMode;
    VPSS_NRX_PARAM_MANUAL_V3_S stNRXManual;
    VPSS_NRX_PARAM_AUTO_V3_S stNRXAuto;
} VPSS_NRX_PARAM_V3_S;

typedef struct {
    VPSS_NR_VER_E enNRVer;
    union {
        VPSS_NRX_PARAM_V3_S stNRXParam_V3;
        HI_U8 _pad[8192];
    };
} VPSS_GRP_NRX_PARAM_S;

static int v4_iq_apply_vpss_3dnr_nrc(int grp, const v4_iq_3dnr_nrc *cfg) {
    if (!cfg) return EXIT_SUCCESS;
    if (!v4_vpss.fnGetGrpNRXParam || !v4_vpss.fnSetGrpNRXParam) {
        HAL_INFO("v4_iq", "3DNR: VPSS NRX API not available, skipping\n");
        return EXIT_SUCCESS;
    }

    VPSS_GRP_NRX_PARAM_S p;
    int ret = -1;
    // Many SDKs require caller to specify which NRX version to get/set.
    // GK/HiSilicon forks vary: some accept 3 (V3), others use 0/1/2. Probe a few.
    int chosen_ver = -1;
    for (int ver_try = 3; ver_try >= 0; ver_try--) {
        memset(&p, 0, sizeof(p));
        p.enNRVer = ver_try;
        ret = v4_vpss.fnGetGrpNRXParam(grp, &p);
        if (!ret) { chosen_ver = ver_try; break; }
    }
    if (ret) {
        HAL_WARNING("v4_iq", "3DNR: HI_MPI_VPSS_GetGrpNRXParam failed with %#x\n", ret);
        return ret;
    }

    // Expect EV200-style V3 on GK7205V210; if not, don't risk corrupting union.
    if (p.enNRVer != 3) {
        HAL_INFO("v4_iq", "3DNR: VPSS NRX version %d (chosen=%d) not supported by our V3 struct, skipping\n",
                 (int)p.enNRVer, chosen_ver);
        return EXIT_SUCCESS;
    }

    tV200_VPSS_NRc *n = &p.stNRXParam_V3.stNRXManual.stNRXParam.NRc;
    if (cfg->sfc >= 0) { HI_S32 v = cfg->sfc; if (v > 255) v = 255; if (v < 0) v = 0; n->SFC = (HI_U8)v; }
    if (cfg->tfc >= 0) { HI_S32 v = cfg->tfc; if (v > 32) v = 32; if (v < 0) v = 0; n->TFC = (HI_U8)v; }
    if (cfg->trc >= 0) { HI_S32 v = cfg->trc; if (v > 255) v = 255; if (v < 0) v = 0; n->TRC = (HI_U8)v; }
    if (cfg->tpc >= 0) { HI_S32 v = cfg->tpc; if (v > 32) v = 32; if (v < 0) v = 0; n->TPC = (HI_U8)v; }
    if (cfg->mode >= 0) { HI_S32 v = cfg->mode; if (v > 1) v = 1; if (v < 0) v = 0; n->MODE = (HI_U8)v; }
    if (cfg->presfc >= 0) { HI_S32 v = cfg->presfc; if (v > 32) v = 32; if (v < 0) v = 0; n->PRESFC = (HI_U8)v; }

    ret = v4_vpss.fnSetGrpNRXParam(grp, &p);
    if (ret) {
        HAL_WARNING("v4_iq", "3DNR: HI_MPI_VPSS_SetGrpNRXParam failed with %#x\n", ret);
        return ret;
    }
    return EXIT_SUCCESS;
}

// Prefer VI pipe NRX (3DNR) API; fallback to VPSS group NRX API.
static int v4_iq_apply_3dnr_nrc(int pipe, int grp, const v4_iq_3dnr_nrc *cfg) {
    if (!cfg) return EXIT_SUCCESS;

    // Probe VI NRX param layout once (helps identify enNRVersion/struct layout on GK).
    if (v4_vi.fnGetPipeNRXParam) {
        HI_BOOL do_log = HI_FALSE;
        pthread_mutex_lock(&_v4_iq_dyn.lock);
        if (!_v4_iq_dyn.nr3d_probe_logged) {
            _v4_iq_dyn.nr3d_probe_logged = HI_TRUE;
            do_log = HI_TRUE;
        }
        pthread_mutex_unlock(&_v4_iq_dyn.lock);

        if (do_log) {
            unsigned char blob[256];
            memset(blob, 0, sizeof(blob));
            int gr = v4_vi.fnGetPipeNRXParam(pipe, blob);
            if (gr) {
                HAL_WARNING("v4_iq", "3DNR probe: VI_GetPipeNRXParam failed with %#x\n", gr);
            } else {
                // Interpret first 4 bytes as little-endian int (commonly enNRVersion).
                unsigned int ver = (unsigned int)blob[0] |
                    ((unsigned int)blob[1] << 8) |
                    ((unsigned int)blob[2] << 16) |
                    ((unsigned int)blob[3] << 24);

                char hex[3 * 64 + 1];
                size_t w = 0;
                for (int i = 0; i < 64; i++) {
                    if (w + 3 >= sizeof(hex)) break;
                    static const char *digits = "0123456789abcdef";
                    unsigned char b = blob[i];
                    hex[w++] = digits[(b >> 4) & 0xF];
                    hex[w++] = digits[b & 0xF];
                    hex[w++] = (i == 63) ? '\0' : ' ';
                }
                hex[sizeof(hex) - 1] = '\0';
                HAL_INFO("v4_iq", "3DNR probe: VI_GetPipeNRXParam OK, header_u32(le)=%u, first64=[%s]\n",
                    ver, hex);
            }
        }
    }

    if (v4_vi.fnGetPipeNRXParam && v4_vi.fnSetPipeNRXParam) {
        // For GK, VI NRX param type layout can vary. Use a large buffer and only patch a few NRC fields
        // by string signature is hard; instead try VPSS if VI isn't supported.
        // Here we attempt VPSS first if VI returns unsupported.
        HI_U8 blob[8192];
        memset(blob, 0, sizeof(blob));
        int gr = v4_vi.fnGetPipeNRXParam(pipe, blob);
        if (!gr) {
            // Try to interpret as VI_PIPE_NRX_PARAM_S V1 (starts with enNRVersion)
            // We only support V1 fast-path when it matches expected offsets.
            // If this is wrong, Set will fail and we will fallback to VPSS.
            typedef struct { int enNRVersion; HI_U8 rest[1]; } vi_nrx_hdr;
            vi_nrx_hdr *hdr = (vi_nrx_hdr *)blob;
            (void)hdr;
            // We can't safely patch without full struct here; use VPSS path instead for now.
        }
        // fall through to VPSS
    }
    return v4_iq_apply_vpss_3dnr_nrc(grp, cfg);
}

static int v4_iq_find_int_token(const char *txt, const char *tag, HI_S32 *out) {
    if (!txt || !tag || !out) return 0;
    // strcasestr() is a GNU extension and may be unavailable in some toolchains.
    // Implement a tiny ASCII-only case-insensitive substring search.
    const char *p = NULL;
    {
        const char *h = txt;
        const char *n = tag;
        size_t nl = strlen(n);
        if (nl == 0) p = h;
        else {
            for (; *h; h++) {
                size_t i = 0;
                while (i < nl) {
                    unsigned char hc = (unsigned char)h[i];
                    unsigned char nc = (unsigned char)n[i];
                    if (!hc) break;
                    if (tolower(hc) != tolower(nc)) break;
                    i++;
                }
                if (i == nl) { p = h; break; }
            }
        }
    }
    if (!p) return 0;
    p += strlen(tag);
    while (*p && isspace((unsigned char)*p)) p++;
    char *end = NULL;
    long v = strtol(p, &end, 0);
    if (!end || end == p) return 0;
    *out = (HI_S32)v;
    return 1;
}

static void v4_iq_load_3dnr_section(struct IniConfig *ini, const char *section, v4_iq_3dnr_cfg *dst) {
    if (!dst) return;
    memset(dst, 0, sizeof(*dst));
    if (!ini || !section) return;

    int sec_s = 0, sec_e = 0;
    if (section_pos(ini, section, &sec_s, &sec_e) != CONFIG_OK)
        return;

    int cnt = 0;
    if (parse_int(ini, section, "3DNRCount", 1, V4_IQ_DYN_MAX_POINTS, &cnt) != CONFIG_OK)
        return;

    char buf[4096];
    HI_U32 iso[V4_IQ_DYN_MAX_POINTS] = {0};
    if (parse_param_value(ini, section, "IsoThresh", buf) != CONFIG_OK)
        return;
    int nIso = v4_iq_parse_csv_u32(buf, iso, V4_IQ_DYN_MAX_POINTS);
    int n = cnt;
    if (nIso < n) n = nIso;
    if (n < 1) return;

    dst->enabled = true;
    dst->n = n;
    for (int i = 0; i < n; i++) {
        dst->iso[i] = iso[i];
        dst->nrc[i].sfc = dst->nrc[i].tfc = dst->nrc[i].tpc = dst->nrc[i].trc = -1;
        dst->nrc[i].mode = dst->nrc[i].presfc = -1;
    }

    for (int i = 0; i < n; i++) {
        char key[32];
        snprintf(key, sizeof(key), "3DnrParam_%d", i);
        char txt[16384];
        int tl = v4_iq_parse_multiline_str(ini, section, key, txt, sizeof(txt));
        if (tl <= 0) continue;

        HI_S32 v;
        if (v4_iq_find_int_token(txt, "-sfc", &v)) dst->nrc[i].sfc = v;
        if (v4_iq_find_int_token(txt, "-tfc", &v)) dst->nrc[i].tfc = v;
        if (v4_iq_find_int_token(txt, "-tpc", &v)) dst->nrc[i].tpc = v;
        if (v4_iq_find_int_token(txt, "-trc", &v)) dst->nrc[i].trc = v;
        if (v4_iq_find_int_token(txt, "-mode", &v)) dst->nrc[i].mode = v;
        if (v4_iq_find_int_token(txt, "-presfc", &v)) dst->nrc[i].presfc = v;
    }
}

static void v4_iq_dyn_unbypass_modules(int pipe) {
    if (!v4_isp.fnGetModuleControl || !v4_isp.fnSetModuleControl) return;
    ISP_MODULE_CTRL_U mc;
    memset(&mc, 0, sizeof(mc));
    if (v4_isp.fnGetModuleControl(pipe, &mc)) return;
    mc.u64Key &= ~(1ULL << 5);  // Dehaze
    mc.u64Key &= ~(1ULL << 8);  // DRC
    v4_isp.fnSetModuleControl(pipe, &mc);
}

static void v4_iq_dyn_parse_u8_list(struct IniConfig *ini, const char *section, const char *key, HI_U8 *out, int *ioN) {
    char buf[4096];
    HI_U32 tmp[V4_IQ_DYN_MAX_POINTS];
    if (parse_param_value(ini, section, key, buf) != CONFIG_OK) return;
    memset(tmp, 0, sizeof(tmp));
    int n = v4_iq_parse_csv_u32(buf, tmp, V4_IQ_DYN_MAX_POINTS);
    if (*ioN == 0 || n < *ioN) *ioN = n;
    for (int i = 0; i < *ioN; i++) {
        HI_U32 v = tmp[i];
        if (v > 255) v = 255;
        out[i] = (HI_U8)v;
    }
}

static void v4_iq_dyn_parse_u16_list(struct IniConfig *ini, const char *section, const char *key, HI_U16 *out, int *ioN) {
    char buf[4096];
    HI_U32 tmp[V4_IQ_DYN_MAX_POINTS];
    if (parse_param_value(ini, section, key, buf) != CONFIG_OK) return;
    memset(tmp, 0, sizeof(tmp));
    int n = v4_iq_parse_csv_u32(buf, tmp, V4_IQ_DYN_MAX_POINTS);
    if (*ioN == 0 || n < *ioN) *ioN = n;
    for (int i = 0; i < *ioN; i++) {
        HI_U32 v = tmp[i];
        if (v > 65535) v = 65535;
        out[i] = (HI_U16)v;
    }
}

static void v4_iq_dyn_parse_s8_list(struct IniConfig *ini, const char *section, const char *key, HI_S8 *out, int *ioN) {
    char buf[4096];
    HI_U32 tmp[V4_IQ_DYN_MAX_POINTS];
    if (parse_param_value(ini, section, key, buf) != CONFIG_OK) return;
    memset(tmp, 0, sizeof(tmp));
    int n = v4_iq_parse_csv_u32(buf, tmp, V4_IQ_DYN_MAX_POINTS);
    if (*ioN == 0 || n < *ioN) *ioN = n;
    for (int i = 0; i < *ioN; i++) {
        HI_S32 v = (HI_S32)tmp[i];
        if (v > 127) v = 127;
        if (v < -128) v = -128;
        out[i] = (HI_S8)v;
    }
}

static void v4_iq_dyn_parse_iso_list(struct IniConfig *ini, const char *section, const char *key, HI_U32 *out, int *ioN) {
    char buf[4096];
    if (parse_param_value(ini, section, key, buf) != CONFIG_OK) return;
    int n = v4_iq_parse_csv_u32(buf, out, V4_IQ_DYN_MAX_POINTS);
    if (*ioN == 0 || n < *ioN) *ioN = n;
}

static void v4_iq_dyn_load_ae_profile(struct IniConfig *ini, const char *sec,
                                      HI_BOOL *have, HI_U8 *comp, HI_U32 *expmax, HI_U32 *sysgainmax, HI_U32 *againmax,
                                      HI_U8 *histoff, HI_U16 *histslope, HI_U8 *speed) {
    int sec_s = 0, sec_e = 0;
    if (section_pos(ini, sec, &sec_s, &sec_e) != CONFIG_OK) return;
    int v;
    if (parse_int(ini, sec, "AutoCompensation", 0, 255, &v) == CONFIG_OK) *comp = (HI_U8)v;
    if (parse_int(ini, sec, "AutoExpTimeMax", 0, INT_MAX, &v) == CONFIG_OK) *expmax = (HI_U32)v;
    if (parse_int(ini, sec, "AutoSysGainMax", 0, INT_MAX, &v) == CONFIG_OK) *sysgainmax = (HI_U32)v;
    if (parse_int(ini, sec, "AutoAGainMax", 0, INT_MAX, &v) == CONFIG_OK) *againmax = (HI_U32)v;
    if (parse_int(ini, sec, "AutoMaxHistOffset", 0, 255, &v) == CONFIG_OK) *histoff = (HI_U8)v;
    if (parse_int(ini, sec, "AutoHistRatioSlope", 0, 65535, &v) == CONFIG_OK) *histslope = (HI_U16)v;
    if (parse_int(ini, sec, "AutoSpeed", 0, 255, &v) == CONFIG_OK) *speed = (HI_U8)v;
    *have = HI_TRUE;
}

static void v4_iq_dyn_load_from_ini(struct IniConfig *ini, bool enableDynDehaze, bool enableDynLinearDRC) {
    v4_iq_dyn_dehaze_cfg deh_day, deh_ir;
    v4_iq_dyn_linear_drc_cfg drc_day, drc_ir;
    memset(&deh_day, 0, sizeof(deh_day));
    memset(&deh_ir, 0, sizeof(deh_ir));
    memset(&drc_day, 0, sizeof(drc_day));
    memset(&drc_ir, 0, sizeof(drc_ir));

    // dynamic_dehaze
    {
        int sec_s = 0, sec_e = 0;
        if (section_pos(ini, "dynamic_dehaze", &sec_s, &sec_e) == CONFIG_OK && enableDynDehaze) {
            deh_day.enabled = true;
            int en = 1;
            if (parse_int(ini, "dynamic_dehaze", "Enable", 0, 1, &en) == CONFIG_OK)
                deh_day.enabled = (en != 0);
            int n = 0;
            v4_iq_dyn_parse_iso_list(ini, "dynamic_dehaze", "IsoThresh", deh_day.iso_thr, &n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_dehaze", "AutoDehazeStr", deh_day.str, &n);
            deh_day.n = n;
            if (deh_day.n < 2) deh_day.enabled = false;
        }
    }
    // Optional IR override: ir_dynamic_dehaze
    deh_ir = deh_day;
    {
        int sec_s = 0, sec_e = 0;
        if (section_pos(ini, "ir_dynamic_dehaze", &sec_s, &sec_e) == CONFIG_OK && enableDynDehaze) {
            memset(&deh_ir, 0, sizeof(deh_ir));
            deh_ir.enabled = true;
            int en = 1;
            if (parse_int(ini, "ir_dynamic_dehaze", "Enable", 0, 1, &en) == CONFIG_OK)
                deh_ir.enabled = (en != 0);
            int n = 0;
            v4_iq_dyn_parse_iso_list(ini, "ir_dynamic_dehaze", "IsoThresh", deh_ir.iso_thr, &n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_dehaze", "AutoDehazeStr", deh_ir.str, &n);
            deh_ir.n = n;
            if (deh_ir.n < 2) deh_ir.enabled = false;
        }
    }

    // dynamic_linear_drc
    // all_param hysteresis + low-light auto AE (even when user keeps DAY mode at night)
    {
        HI_U32 up = 0, down = 0;
        int v;
        if (parse_int(ini, "all_param", "UpFrameIso", 0, INT_MAX, &v) == CONFIG_OK)
            up = (HI_U32)v;
        if (parse_int(ini, "all_param", "DownFrameIso", 0, INT_MAX, &v) == CONFIG_OK)
            down = (HI_U32)v;

        HI_BOOL ll_en = HI_FALSE;
        HI_U32 ll_iso = 1500;
        HI_U32 ll_exptime = 15000;
        HI_BOOL have_fast = HI_FALSE;
        HI_U8  ll_fast_lum = 0;
        HI_U32 ll_fast_expmax = 0;
        HI_U32 ll_fast_againmax = 0;
        HI_U8  ll_fast_comp_boost = 0;
        if (parse_int(ini, "all_param", "LowLightAutoAE", 0, 1, &v) == CONFIG_OK)
            ll_en = (v != 0) ? HI_TRUE : HI_FALSE;
        if (parse_int(ini, "all_param", "LowLightIso", 0, INT_MAX, &v) == CONFIG_OK)
            ll_iso = (HI_U32)v;
        if (parse_int(ini, "all_param", "LowLightExpTime", 0, INT_MAX, &v) == CONFIG_OK)
            ll_exptime = (HI_U32)v;
        if (parse_int(ini, "all_param", "LowLightFastLum", 0, 255, &v) == CONFIG_OK) {
            ll_fast_lum = (HI_U8)v;
            have_fast = HI_TRUE;
        }
        if (parse_int(ini, "all_param", "LowLightFastExpTimeMax", 0, INT_MAX, &v) == CONFIG_OK) {
            ll_fast_expmax = (HI_U32)v;
            have_fast = HI_TRUE;
        }
        if (parse_int(ini, "all_param", "LowLightFastAGainMax", 0, INT_MAX, &v) == CONFIG_OK) {
            ll_fast_againmax = (HI_U32)v;
            have_fast = HI_TRUE;
        }
        if (parse_int(ini, "all_param", "LowLightFastCompBoost", 0, 64, &v) == CONFIG_OK) {
            ll_fast_comp_boost = (HI_U8)v;
            have_fast = HI_TRUE;
        }

        // Optional lowlight FX thresholds (only if section exists).
        HI_BOOL have_fx = HI_FALSE;
        HI_U32 noisy_gamma_iso = 0, noisy_gamma_dg = 0, noisy_gamma_ispg = 0;
        HI_U32 noisy_sh_iso = 0, noisy_sh_dg = 0, noisy_sh_ispg = 0;
        {
            int sec_s = 0, sec_e = 0;
            if (section_pos(ini, "lowlight_fx", &sec_s, &sec_e) == CONFIG_OK) {
                have_fx = HI_TRUE;
                int en = 1;
                if (parse_int(ini, "lowlight_fx", "Enable", 0, 1, &en) == CONFIG_OK)
                    have_fx = (en != 0) ? HI_TRUE : HI_FALSE;
                if (parse_int(ini, "lowlight_fx", "NoisyGammaIso", 0, INT_MAX, &v) == CONFIG_OK) noisy_gamma_iso = (HI_U32)v;
                if (parse_int(ini, "lowlight_fx", "NoisyGammaDGain", 0, INT_MAX, &v) == CONFIG_OK) noisy_gamma_dg = (HI_U32)v;
                if (parse_int(ini, "lowlight_fx", "NoisyGammaISPDGain", 0, INT_MAX, &v) == CONFIG_OK) noisy_gamma_ispg = (HI_U32)v;
                if (parse_int(ini, "lowlight_fx", "NoisySharpenIso", 0, INT_MAX, &v) == CONFIG_OK) noisy_sh_iso = (HI_U32)v;
                if (parse_int(ini, "lowlight_fx", "NoisySharpenDGain", 0, INT_MAX, &v) == CONFIG_OK) noisy_sh_dg = (HI_U32)v;
                if (parse_int(ini, "lowlight_fx", "NoisySharpenISPDGain", 0, INT_MAX, &v) == CONFIG_OK) noisy_sh_ispg = (HI_U32)v;
            }
        }

        // Optional anti-halo sharpen (only if section exists).
        HI_BOOL have_ah = HI_FALSE;
        HI_U8 ah_over_pct = 0, ah_under_pct = 0, ah_min_over = 0, ah_min_under = 0;
        HI_U8 ah_shootsup_min = 0, ah_edgefilt_min = 0, ah_edgestr_pct = 0, ah_texstr_pct = 0;
        HI_U16 ah_maxsharp_cap = 0;
        {
            int sec_s = 0, sec_e = 0;
            if (section_pos(ini, "lowlight_sharpen", &sec_s, &sec_e) == CONFIG_OK) {
                have_ah = HI_TRUE;
                int en = 1;
                if (parse_int(ini, "lowlight_sharpen", "Enable", 0, 1, &en) == CONFIG_OK)
                    have_ah = (en != 0) ? HI_TRUE : HI_FALSE;
                if (parse_int(ini, "lowlight_sharpen", "OverShootScalePct", 0, 100, &v) == CONFIG_OK) ah_over_pct = (HI_U8)v;
                if (parse_int(ini, "lowlight_sharpen", "UnderShootScalePct", 0, 100, &v) == CONFIG_OK) ah_under_pct = (HI_U8)v;
                if (parse_int(ini, "lowlight_sharpen", "MinOverShoot", 0, 255, &v) == CONFIG_OK) ah_min_over = (HI_U8)v;
                if (parse_int(ini, "lowlight_sharpen", "MinUnderShoot", 0, 255, &v) == CONFIG_OK) ah_min_under = (HI_U8)v;
                if (parse_int(ini, "lowlight_sharpen", "ShootSupStrMin", 0, 255, &v) == CONFIG_OK) ah_shootsup_min = (HI_U8)v;
                if (parse_int(ini, "lowlight_sharpen", "EdgeFiltStrMin", 0, 255, &v) == CONFIG_OK) ah_edgefilt_min = (HI_U8)v;
                if (parse_int(ini, "lowlight_sharpen", "MaxSharpGainCap", 0, INT_MAX, &v) == CONFIG_OK) ah_maxsharp_cap = (HI_U16)v;
                if (parse_int(ini, "lowlight_sharpen", "EdgeStrScalePct", 0, 100, &v) == CONFIG_OK) ah_edgestr_pct = (HI_U8)v;
                if (parse_int(ini, "lowlight_sharpen", "TextureStrScalePct", 0, 100, &v) == CONFIG_OK) ah_texstr_pct = (HI_U8)v;
            }
        }

        // Optional lowlight DRC caps (only if section exists).
        HI_BOOL have_drc_caps = HI_FALSE;
        HI_U16 ll_str_max = 0, fast_str_max = 0;
        HI_U8 ll_bm_max = 0, ll_bm_min = 0, ll_bg_lmt = 0, ll_bg_step = 0;
        HI_U8 ll_dm_max = 0, ll_dm_min = 0, ll_cc_max = 0;
        HI_U8 fast_dm_max = 0, fast_dm_min = 0, fast_cc_max = 0;
        {
            int sec_s = 0, sec_e = 0;
            if (section_pos(ini, "lowlight_drc_caps", &sec_s, &sec_e) == CONFIG_OK) {
                have_drc_caps = HI_TRUE;
                int en = 1;
                if (parse_int(ini, "lowlight_drc_caps", "Enable", 0, 1, &en) == CONFIG_OK)
                    have_drc_caps = (en != 0) ? HI_TRUE : HI_FALSE;
                if (parse_int(ini, "lowlight_drc_caps", "LowLightStrengthMax", 0, INT_MAX, &v) == CONFIG_OK) ll_str_max = (HI_U16)v;
                if (parse_int(ini, "lowlight_drc_caps", "FastLowLightStrengthMax", 0, INT_MAX, &v) == CONFIG_OK) fast_str_max = (HI_U16)v;
                if (parse_int(ini, "lowlight_drc_caps", "BrightMixMax", 0, 255, &v) == CONFIG_OK) ll_bm_max = (HI_U8)v;
                if (parse_int(ini, "lowlight_drc_caps", "BrightMixMin", 0, 255, &v) == CONFIG_OK) ll_bm_min = (HI_U8)v;
                if (parse_int(ini, "lowlight_drc_caps", "BrightGainLmt", 0, 255, &v) == CONFIG_OK) ll_bg_lmt = (HI_U8)v;
                if (parse_int(ini, "lowlight_drc_caps", "BrightGainStep", 0, 255, &v) == CONFIG_OK) ll_bg_step = (HI_U8)v;
                if (parse_int(ini, "lowlight_drc_caps", "DarkMixMax", 0, 255, &v) == CONFIG_OK) ll_dm_max = (HI_U8)v;
                if (parse_int(ini, "lowlight_drc_caps", "DarkMixMin", 0, 255, &v) == CONFIG_OK) ll_dm_min = (HI_U8)v;
                if (parse_int(ini, "lowlight_drc_caps", "ContrastMax", 0, 255, &v) == CONFIG_OK) ll_cc_max = (HI_U8)v;
                if (parse_int(ini, "lowlight_drc_caps", "FastDarkMixMax", 0, 255, &v) == CONFIG_OK) fast_dm_max = (HI_U8)v;
                if (parse_int(ini, "lowlight_drc_caps", "FastDarkMixMin", 0, 255, &v) == CONFIG_OK) fast_dm_min = (HI_U8)v;
                if (parse_int(ini, "lowlight_drc_caps", "FastContrastMax", 0, 255, &v) == CONFIG_OK) fast_cc_max = (HI_U8)v;
            }
        }

        // Load AE profiles (day + low-light)
        HI_BOOL have_day = HI_FALSE, have_low = HI_FALSE;
        HI_U8 day_comp = 0, low_comp = 0;
        HI_U32 day_expmax = 0, low_expmax = 0;
        HI_U32 day_sysgainmax = 0, low_sysgainmax = 0;
        HI_U32 day_againmax = 0, low_againmax = 0;
        HI_U8 day_histoff = 0, low_histoff = 0;
        HI_U16 day_histslope = 0, low_histslope = 0;
        HI_U8 day_speed = 0, low_speed = 0;
        v4_iq_dyn_load_ae_profile(ini, "static_ae", &have_day, &day_comp, &day_expmax, &day_sysgainmax, &day_againmax,
                                  &day_histoff, &day_histslope, &day_speed);
        // Prefer dedicated low-light color profile (for "DAY-at-night") if present.
        // Fallback to ir_static_ae for legacy configs.
        v4_iq_dyn_load_ae_profile(ini, "lowlight_static_ae", &have_low, &low_comp, &low_expmax, &low_sysgainmax, &low_againmax,
                                  &low_histoff, &low_histslope, &low_speed);
        if (!have_low) {
            v4_iq_dyn_load_ae_profile(ini, "ir_static_ae", &have_low, &low_comp, &low_expmax, &low_sysgainmax, &low_againmax,
                                      &low_histoff, &low_histslope, &low_speed);
        }
        if (!have_low) {
            have_low = have_day;
            low_comp = day_comp;
            low_expmax = day_expmax;
            low_sysgainmax = day_sysgainmax;
            low_againmax = day_againmax;
            low_histoff = day_histoff;
            low_histslope = day_histslope;
            low_speed = day_speed;
        }

        // Presence of RouteEx tables (used to trigger mode-switch AE apply even if other dynamics are off)
        HI_BOOL have_rx_day = HI_FALSE, have_rx_ir = HI_FALSE;
        {
            int sec_s = 0, sec_e = 0;
            have_rx_day = (section_pos(ini, "static_aerouteex", &sec_s, &sec_e) == CONFIG_OK) ? HI_TRUE : HI_FALSE;
            sec_s = sec_e = 0;
            have_rx_ir = (section_pos(ini, "ir_static_aerouteex", &sec_s, &sec_e) == CONFIG_OK) ? HI_TRUE : HI_FALSE;
        }

        pthread_mutex_lock(&_v4_iq_dyn.lock);
        _v4_iq_dyn.upFrameIso = up;
        _v4_iq_dyn.downFrameIso = down;
        _v4_iq_dyn.lowlight_auto_ae = ll_en;
        _v4_iq_dyn.lowlight_iso = ll_iso;
        _v4_iq_dyn.lowlight_exptime = ll_exptime;
        _v4_iq_dyn.have_lowlight_fast = have_fast;
        _v4_iq_dyn.lowlight_fast_lum = ll_fast_lum;
        _v4_iq_dyn.lowlight_fast_expmax = ll_fast_expmax;
        _v4_iq_dyn.lowlight_fast_againmax = ll_fast_againmax;
        _v4_iq_dyn.lowlight_fast_comp_boost = ll_fast_comp_boost;
        _v4_iq_dyn.have_lowlight_fx = have_fx;
        _v4_iq_dyn.noisy_gamma_iso = noisy_gamma_iso;
        _v4_iq_dyn.noisy_gamma_dgain = noisy_gamma_dg;
        _v4_iq_dyn.noisy_gamma_ispdgain = noisy_gamma_ispg;
        _v4_iq_dyn.noisy_sharpen_iso = noisy_sh_iso;
        _v4_iq_dyn.noisy_sharpen_dgain = noisy_sh_dg;
        _v4_iq_dyn.noisy_sharpen_ispdgain = noisy_sh_ispg;
        _v4_iq_dyn.have_lowlight_sharpen_ah = have_ah;
        _v4_iq_dyn.ah_over_pct = ah_over_pct;
        _v4_iq_dyn.ah_under_pct = ah_under_pct;
        _v4_iq_dyn.ah_min_over = ah_min_over;
        _v4_iq_dyn.ah_min_under = ah_min_under;
        _v4_iq_dyn.ah_shootsup_min = ah_shootsup_min;
        _v4_iq_dyn.ah_edgefilt_min = ah_edgefilt_min;
        _v4_iq_dyn.ah_maxsharpgain_cap = ah_maxsharp_cap;
        _v4_iq_dyn.ah_edgestr_pct = ah_edgestr_pct;
        _v4_iq_dyn.ah_texstr_pct = ah_texstr_pct;
        _v4_iq_dyn.have_lowlight_drc_caps = have_drc_caps;
        _v4_iq_dyn.ll_drc_strength_max = ll_str_max;
        _v4_iq_dyn.fast_drc_strength_max = fast_str_max;
        _v4_iq_dyn.ll_bright_mix_max = ll_bm_max;
        _v4_iq_dyn.ll_bright_mix_min = ll_bm_min;
        _v4_iq_dyn.ll_bright_gain_lmt = ll_bg_lmt;
        _v4_iq_dyn.ll_bright_gain_step = ll_bg_step;
        _v4_iq_dyn.ll_dark_mix_max = ll_dm_max;
        _v4_iq_dyn.ll_dark_mix_min = ll_dm_min;
        _v4_iq_dyn.ll_contrast_max = ll_cc_max;
        _v4_iq_dyn.fast_dark_mix_max = fast_dm_max;
        _v4_iq_dyn.fast_dark_mix_min = fast_dm_min;
        _v4_iq_dyn.fast_contrast_max = fast_cc_max;
        _v4_iq_dyn.have_ae_day = have_day;
        _v4_iq_dyn.have_ae_low = have_low;
        _v4_iq_dyn.have_aerouteex_day = have_rx_day;
        _v4_iq_dyn.have_aerouteex_ir = have_rx_ir;
        _v4_iq_dyn.last_ae_lowlight = HI_FALSE;
        _v4_iq_dyn.ae_day_comp = day_comp;
        _v4_iq_dyn.ae_low_comp = low_comp;
        _v4_iq_dyn.ae_day_expmax = day_expmax;
        _v4_iq_dyn.ae_low_expmax = low_expmax;
        _v4_iq_dyn.ae_day_sysgainmax = day_sysgainmax;
        _v4_iq_dyn.ae_low_sysgainmax = low_sysgainmax;
        _v4_iq_dyn.ae_day_againmax = day_againmax;
        _v4_iq_dyn.ae_low_againmax = low_againmax;
        _v4_iq_dyn.ae_day_histoff = day_histoff;
        _v4_iq_dyn.ae_low_histoff = low_histoff;
        _v4_iq_dyn.ae_day_histslope = day_histslope;
        _v4_iq_dyn.ae_low_histslope = low_histslope;
        _v4_iq_dyn.ae_day_speed = day_speed;
        _v4_iq_dyn.ae_low_speed = low_speed;
        pthread_mutex_unlock(&_v4_iq_dyn.lock);

        if (up || down)
            HAL_INFO("v4_iq", "all_param: UpFrameIso=%u DownFrameIso=%u\n", (unsigned)up, (unsigned)down);
        if (ll_en)
            HAL_INFO("v4_iq", "all_param: LowLightAutoAE=1 LowLightIso=%u LowLightExpTime=%u\n",
                     (unsigned)ll_iso, (unsigned)ll_exptime);
    }

    // static_3dnr (day) and ir_static_3dnr (night)
    {
        v4_iq_3dnr_cfg day, ir;
        v4_iq_load_3dnr_section(ini, "static_3dnr", &day);
        v4_iq_load_3dnr_section(ini, "ir_static_3dnr", &ir);

        pthread_mutex_lock(&_v4_iq_dyn.lock);
        _v4_iq_dyn.nr3d_day = day;
        _v4_iq_dyn.nr3d_ir = ir;
        _v4_iq_dyn.last_nr3d_idx = -1;
        _v4_iq_dyn.last_nr3d_is_ir = HI_FALSE;
        pthread_mutex_unlock(&_v4_iq_dyn.lock);

        if (day.enabled) HAL_INFO("v4_iq", "3DNR: loaded [static_3dnr] (%d points)\n", day.n);
        if (ir.enabled)  HAL_INFO("v4_iq", "3DNR: loaded [ir_static_3dnr] (%d points)\n", ir.n);
    }
    {
        int sec_s = 0, sec_e = 0;
        if (section_pos(ini, "dynamic_linear_drc", &sec_s, &sec_e) == CONFIG_OK && enableDynLinearDRC) {
            drc_day.enabled = true;
            int en = 1;
            if (parse_int(ini, "dynamic_linear_drc", "Enable", 0, 1, &en) == CONFIG_OK)
                drc_day.enabled = (en != 0);

            int n = 0;
            v4_iq_dyn_parse_iso_list(ini, "dynamic_linear_drc", "IsoLevel", drc_day.iso, &n);
            drc_day.n = n;

            // Per-ISO fields (shrink n to common length across lists)
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "LocalMixingBrightMax", drc_day.localMixBrightMax, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "LocalMixingBrightMin", drc_day.localMixBrightMin, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "LocalMixingDarkMax", drc_day.localMixDarkMax, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "LocalMixingDarkMin", drc_day.localMixDarkMin, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "BrightGainLmt", drc_day.brightGainLmt, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "BrightGainLmtStep", drc_day.brightGainLmtStep, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "DarkGainLmtY", drc_day.darkGainLmtY, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "DarkGainLmtC", drc_day.darkGainLmtC, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "FltScaleCoarse", drc_day.fltScaleCoarse, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "FltScaleFine", drc_day.fltScaleFine, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "ContrastControl", drc_day.contrastControl, &drc_day.n);
            v4_iq_dyn_parse_s8_list(ini, "dynamic_linear_drc", "DetailAdjustFactor", drc_day.detailAdjustFactor, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "Asymmetry", drc_day.asymmetry, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "SecondPole", drc_day.secondPole, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "Compress", drc_day.compress, &drc_day.n);
            v4_iq_dyn_parse_u8_list(ini, "dynamic_linear_drc", "Stretch", drc_day.stretch, &drc_day.n);
            v4_iq_dyn_parse_u16_list(ini, "dynamic_linear_drc", "Strength", drc_day.strength, &drc_day.n);

            if (drc_day.n < 2) drc_day.enabled = false;
        }
    }
    // Optional IR override: ir_dynamic_linear_drc
    drc_ir = drc_day;
    {
        int sec_s = 0, sec_e = 0;
        if (section_pos(ini, "ir_dynamic_linear_drc", &sec_s, &sec_e) == CONFIG_OK && enableDynLinearDRC) {
            memset(&drc_ir, 0, sizeof(drc_ir));
            drc_ir.enabled = true;
            int en = 1;
            if (parse_int(ini, "ir_dynamic_linear_drc", "Enable", 0, 1, &en) == CONFIG_OK)
                drc_ir.enabled = (en != 0);

            int n = 0;
            v4_iq_dyn_parse_iso_list(ini, "ir_dynamic_linear_drc", "IsoLevel", drc_ir.iso, &n);
            drc_ir.n = n;

            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "LocalMixingBrightMax", drc_ir.localMixBrightMax, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "LocalMixingBrightMin", drc_ir.localMixBrightMin, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "LocalMixingDarkMax", drc_ir.localMixDarkMax, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "LocalMixingDarkMin", drc_ir.localMixDarkMin, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "BrightGainLmt", drc_ir.brightGainLmt, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "BrightGainLmtStep", drc_ir.brightGainLmtStep, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "DarkGainLmtY", drc_ir.darkGainLmtY, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "DarkGainLmtC", drc_ir.darkGainLmtC, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "FltScaleCoarse", drc_ir.fltScaleCoarse, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "FltScaleFine", drc_ir.fltScaleFine, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "ContrastControl", drc_ir.contrastControl, &drc_ir.n);
            v4_iq_dyn_parse_s8_list(ini, "ir_dynamic_linear_drc", "DetailAdjustFactor", drc_ir.detailAdjustFactor, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "Asymmetry", drc_ir.asymmetry, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "SecondPole", drc_ir.secondPole, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "Compress", drc_ir.compress, &drc_ir.n);
            v4_iq_dyn_parse_u8_list(ini, "ir_dynamic_linear_drc", "Stretch", drc_ir.stretch, &drc_ir.n);
            v4_iq_dyn_parse_u16_list(ini, "ir_dynamic_linear_drc", "Strength", drc_ir.strength, &drc_ir.n);

            if (drc_ir.n < 2) drc_ir.enabled = false;
        }
    }

    pthread_mutex_lock(&_v4_iq_dyn.lock);
    _v4_iq_dyn.dehaze_day = deh_day;
    _v4_iq_dyn.dehaze_ir = deh_ir;
    _v4_iq_dyn.linear_drc_day = drc_day;
    _v4_iq_dyn.linear_drc_ir = drc_ir;
    _v4_iq_dyn.have_last = HI_FALSE;
    _v4_iq_dyn.nr3d_fail_count = 0;
    _v4_iq_dyn.nr3d_disabled = HI_FALSE;
    pthread_mutex_unlock(&_v4_iq_dyn.lock);

    if (deh_day.enabled)
        HAL_INFO("v4_iq", "Dynamic Dehaze: loaded [dynamic_dehaze] (%d points)\n", deh_day.n);
    if (deh_ir.enabled && memcmp(&deh_ir, &deh_day, sizeof(deh_day)) != 0)
        HAL_INFO("v4_iq", "Dynamic Dehaze: loaded [ir_dynamic_dehaze] (%d points)\n", deh_ir.n);
    if (drc_day.enabled)
        HAL_INFO("v4_iq", "Dynamic Linear DRC: loaded [dynamic_linear_drc] (%d points)\n", drc_day.n);
    if (drc_ir.enabled && memcmp(&drc_ir, &drc_day, sizeof(drc_day)) != 0)
        HAL_INFO("v4_iq", "Dynamic Linear DRC: loaded [ir_dynamic_linear_drc] (%d points)\n", drc_ir.n);
}

static void v4_iq_apply_ae_route_for_current_mode(int pipe);

static void *v4_iq_dynamic_thread(void *arg) {
    int pipe = (int)(intptr_t)arg;
    usleep(700 * 1000);

    if (!v4_isp.fnQueryExposureInfo) {
        HAL_INFO("v4_iq", "Dynamic IQ: QueryExposureInfo API not available, skipping dynamics\n");
        return NULL;
    }

    v4_iq_dyn_unbypass_modules(pipe);

    HI_BOOL last_mode_valid = HI_FALSE;
    bool last_ir_mode = false;

    for (;;) {
        v4_iq_dyn_dehaze_cfg deh_day, deh_ir;
        v4_iq_dyn_linear_drc_cfg drc_day, drc_ir;
        v4_iq_3dnr_cfg nr_day, nr_ir;
        HI_U32 upIso = 0, downIso = 0;
        HI_BOOL ll_ae = HI_FALSE;
        HI_U32 ll_iso = 0, ll_exptime = 0;
        HI_BOOL have_ae_day = HI_FALSE, have_ae_low = HI_FALSE;
        HI_BOOL last_ae_low = HI_FALSE;
        HI_U8 day_comp = 0, low_comp = 0;
        HI_U32 day_expmax = 0, low_expmax = 0;
        HI_U32 day_sysgainmax = 0, low_sysgainmax = 0;
        HI_U32 day_againmax = 0, low_againmax = 0;
        HI_U8 day_histoff = 0, low_histoff = 0;
        HI_U16 day_histslope = 0, low_histslope = 0;
        HI_U8 day_speed = 0, low_speed = 0;
        HI_BOOL have_fast = HI_FALSE;
        HI_U8 ll_fast_lum = 0;
        HI_U32 ll_fast_expmax = 0;
        HI_U32 ll_fast_againmax = 0;
        HI_U8 ll_fast_comp_boost = 0;
        HI_BOOL have_fx = HI_FALSE;
        HI_U32 noisy_gamma_iso = 0, noisy_gamma_dg = 0, noisy_gamma_ispg = 0;
        HI_U32 noisy_sh_iso = 0, noisy_sh_dg = 0, noisy_sh_ispg = 0;
        HI_BOOL have_ah = HI_FALSE;
        HI_U8 ah_over_pct = 0, ah_under_pct = 0, ah_min_over = 0, ah_min_under = 0;
        HI_U8 ah_shootsup_min = 0, ah_edgefilt_min = 0, ah_edgestr_pct = 0, ah_texstr_pct = 0;
        HI_U16 ah_maxsharp_cap = 0;
        HI_BOOL have_drc_caps = HI_FALSE;
        HI_U16 ll_str_max = 0, fast_str_max = 0;
        HI_U8 ll_bm_max = 0, ll_bm_min = 0, ll_bg_lmt = 0, ll_bg_step = 0;
        HI_U8 ll_dm_max = 0, ll_dm_min = 0, ll_cc_max = 0;
        HI_U8 fast_dm_max = 0, fast_dm_min = 0, fast_cc_max = 0;
        int last_nr_idx = -1;
        HI_BOOL last_nr_ir = HI_FALSE;
        int nr3d_fail_count = 0;
        HI_BOOL nr3d_disabled = HI_FALSE;
        HI_BOOL have_last;
        HI_U8 last_deh;
        v4_iq_dyn_drc_sig last_drc;

        pthread_mutex_lock(&_v4_iq_dyn.lock);
        deh_day = _v4_iq_dyn.dehaze_day;
        deh_ir = _v4_iq_dyn.dehaze_ir;
        drc_day = _v4_iq_dyn.linear_drc_day;
        drc_ir = _v4_iq_dyn.linear_drc_ir;
        nr_day = _v4_iq_dyn.nr3d_day;
        nr_ir = _v4_iq_dyn.nr3d_ir;
        upIso = _v4_iq_dyn.upFrameIso;
        downIso = _v4_iq_dyn.downFrameIso;
        ll_ae = _v4_iq_dyn.lowlight_auto_ae;
        ll_iso = _v4_iq_dyn.lowlight_iso;
        ll_exptime = _v4_iq_dyn.lowlight_exptime;
        have_ae_day = _v4_iq_dyn.have_ae_day;
        have_ae_low = _v4_iq_dyn.have_ae_low;
        last_ae_low = _v4_iq_dyn.last_ae_lowlight;
        day_comp = _v4_iq_dyn.ae_day_comp;
        low_comp = _v4_iq_dyn.ae_low_comp;
        day_expmax = _v4_iq_dyn.ae_day_expmax;
        low_expmax = _v4_iq_dyn.ae_low_expmax;
        day_sysgainmax = _v4_iq_dyn.ae_day_sysgainmax;
        low_sysgainmax = _v4_iq_dyn.ae_low_sysgainmax;
        day_againmax = _v4_iq_dyn.ae_day_againmax;
        low_againmax = _v4_iq_dyn.ae_low_againmax;
        day_histoff = _v4_iq_dyn.ae_day_histoff;
        low_histoff = _v4_iq_dyn.ae_low_histoff;
        day_histslope = _v4_iq_dyn.ae_day_histslope;
        low_histslope = _v4_iq_dyn.ae_low_histslope;
        day_speed = _v4_iq_dyn.ae_day_speed;
        low_speed = _v4_iq_dyn.ae_low_speed;
        have_fast = _v4_iq_dyn.have_lowlight_fast;
        ll_fast_lum = _v4_iq_dyn.lowlight_fast_lum;
        ll_fast_expmax = _v4_iq_dyn.lowlight_fast_expmax;
        ll_fast_againmax = _v4_iq_dyn.lowlight_fast_againmax;
        ll_fast_comp_boost = _v4_iq_dyn.lowlight_fast_comp_boost;
        have_fx = _v4_iq_dyn.have_lowlight_fx;
        noisy_gamma_iso = _v4_iq_dyn.noisy_gamma_iso;
        noisy_gamma_dg = _v4_iq_dyn.noisy_gamma_dgain;
        noisy_gamma_ispg = _v4_iq_dyn.noisy_gamma_ispdgain;
        noisy_sh_iso = _v4_iq_dyn.noisy_sharpen_iso;
        noisy_sh_dg = _v4_iq_dyn.noisy_sharpen_dgain;
        noisy_sh_ispg = _v4_iq_dyn.noisy_sharpen_ispdgain;
        have_ah = _v4_iq_dyn.have_lowlight_sharpen_ah;
        ah_over_pct = _v4_iq_dyn.ah_over_pct;
        ah_under_pct = _v4_iq_dyn.ah_under_pct;
        ah_min_over = _v4_iq_dyn.ah_min_over;
        ah_min_under = _v4_iq_dyn.ah_min_under;
        ah_shootsup_min = _v4_iq_dyn.ah_shootsup_min;
        ah_edgefilt_min = _v4_iq_dyn.ah_edgefilt_min;
        ah_maxsharp_cap = _v4_iq_dyn.ah_maxsharpgain_cap;
        ah_edgestr_pct = _v4_iq_dyn.ah_edgestr_pct;
        ah_texstr_pct = _v4_iq_dyn.ah_texstr_pct;
        have_drc_caps = _v4_iq_dyn.have_lowlight_drc_caps;
        ll_str_max = _v4_iq_dyn.ll_drc_strength_max;
        fast_str_max = _v4_iq_dyn.fast_drc_strength_max;
        ll_bm_max = _v4_iq_dyn.ll_bright_mix_max;
        ll_bm_min = _v4_iq_dyn.ll_bright_mix_min;
        ll_bg_lmt = _v4_iq_dyn.ll_bright_gain_lmt;
        ll_bg_step = _v4_iq_dyn.ll_bright_gain_step;
        ll_dm_max = _v4_iq_dyn.ll_dark_mix_max;
        ll_dm_min = _v4_iq_dyn.ll_dark_mix_min;
        ll_cc_max = _v4_iq_dyn.ll_contrast_max;
        fast_dm_max = _v4_iq_dyn.fast_dark_mix_max;
        fast_dm_min = _v4_iq_dyn.fast_dark_mix_min;
        fast_cc_max = _v4_iq_dyn.fast_contrast_max;
        last_nr_idx = _v4_iq_dyn.last_nr3d_idx;
        last_nr_ir = _v4_iq_dyn.last_nr3d_is_ir;
        nr3d_fail_count = _v4_iq_dyn.nr3d_fail_count;
        nr3d_disabled = _v4_iq_dyn.nr3d_disabled;
        have_last = _v4_iq_dyn.have_last;
        last_deh = _v4_iq_dyn.last_dehaze_strength;
        last_drc = _v4_iq_dyn.last_drc;
        pthread_mutex_unlock(&_v4_iq_dyn.lock);

        if (!deh_day.enabled && !deh_ir.enabled &&
            !drc_day.enabled && !drc_ir.enabled &&
            !nr_day.enabled && !nr_ir.enabled &&
            !(ll_ae && have_ae_day && have_ae_low)) {
            usleep(1000 * 1000);
            continue;
        }

        ISP_EXP_INFO_S expi;
        memset(&expi, 0, sizeof(expi));
        if (v4_isp.fnQueryExposureInfo(pipe, &expi)) {
            usleep(500 * 1000);
            continue;
        }
        HI_U32 iso = expi.u32ISO;
        const bool is_ir_mode = night_mode_on();

        if (!last_mode_valid || is_ir_mode != last_ir_mode) {
            last_mode_valid = HI_TRUE;
            last_ir_mode = is_ir_mode;
            HAL_INFO("v4_iq", "AE route: mode switch -> apply %s tables\n", is_ir_mode ? "IR" : "DAY");
            v4_iq_apply_ae_route_for_current_mode(pipe);
        }

        const HI_BOOL is_lowlight =
            (!is_ir_mode &&
             ll_ae && have_ae_day && have_ae_low &&
             (iso >= ll_iso || expi.u32ExpTime >= ll_exptime)) ? HI_TRUE : HI_FALSE;

        // Debug: log exposure state periodically, and always when very bright (snow clipping).
        // This makes AE/DRC tuning deterministic.
        static time_t last_dbg = 0;
        time_t now = time(NULL);
        HI_BOOL very_bright = (expi.u8AveLum >= 210) ? HI_TRUE : HI_FALSE;
        if (now != (time_t)-1 && (very_bright || (now - last_dbg) >= 5)) {
            last_dbg = now;
            HAL_INFO("v4_iq",
                "ISP exp: iso=%u expTime=%u again=%u dgain=%u ispdgain=%u aveLum=%u expIsMax=%d (mode=%s)\n",
                (unsigned)expi.u32ISO,
                (unsigned)expi.u32ExpTime,
                (unsigned)expi.u32AGain,
                (unsigned)expi.u32DGain,
                (unsigned)expi.u32ISPDGain,
                (unsigned)expi.u8AveLum,
                (int)expi.bExposureIsMAX,
                night_mode_on() ? "IR" : "DAY");
        }

        const v4_iq_dyn_dehaze_cfg *deh = is_ir_mode ? &deh_ir : &deh_day;
        const v4_iq_dyn_linear_drc_cfg *drc = is_ir_mode ? &drc_ir : &drc_day;
        int drc_fail_count = 0;
        HI_BOOL drc_disabled = HI_FALSE;
        pthread_mutex_lock(&_v4_iq_dyn.lock);
        drc_fail_count = _v4_iq_dyn.drc_fail_count;
        drc_disabled = _v4_iq_dyn.drc_disabled;
        pthread_mutex_unlock(&_v4_iq_dyn.lock);

        // Dehaze by ISO
        if (deh->enabled && v4_isp.fnGetDehazeAttr && v4_isp.fnSetDehazeAttr) {
            HI_U32 str32[V4_IQ_DYN_MAX_POINTS];
            for (int i = 0; i < deh->n; i++) str32[i] = deh->str[i];
            HI_U8 target = (HI_U8)v4_iq_dyn_interp_u32(iso, deh->iso_thr, str32, deh->n);
            if (!have_last || target != last_deh) {
                ISP_DEHAZE_ATTR_S dh;
                memset(&dh, 0, sizeof(dh));
                if (!v4_isp.fnGetDehazeAttr(pipe, &dh)) {
                    dh.bEnable = HI_TRUE;
                    dh.enOpType = OP_TYPE_AUTO;
                    dh.stAuto.u8strength = target;
                    if (!v4_isp.fnSetDehazeAttr(pipe, &dh)) {
                        pthread_mutex_lock(&_v4_iq_dyn.lock);
                        _v4_iq_dyn.last_dehaze_strength = target;
                        _v4_iq_dyn.have_last = HI_TRUE;
                        pthread_mutex_unlock(&_v4_iq_dyn.lock);
                        HAL_INFO("v4_iq", "Dynamic Dehaze: iso=%u strength=%u\n", (unsigned)iso, (unsigned)target);
                    }
                }
            }
        }

        // Linear DRC by ISO
        // NOTE: In low-light (DAY-at-night) we intentionally reduce shadow lift to avoid amplifying noise.
        if (!drc_disabled && drc->enabled && v4_isp.fnGetDRCAttr && v4_isp.fnSetDRCAttr) {
            HI_U32 u16vals[V4_IQ_DYN_MAX_POINTS];
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->strength[i];
            HI_U16 strength = (HI_U16)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);

            // Compute full "signature" of applied values to avoid redundant sets.
            v4_iq_dyn_drc_sig cur;
            memset(&cur, 0, sizeof(cur));
            cur.strength = strength;

            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->localMixBrightMax[i];
            cur.localMixBrightMax = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->localMixBrightMin[i];
            cur.localMixBrightMin = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->localMixDarkMax[i];
            cur.localMixDarkMax = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->localMixDarkMin[i];
            cur.localMixDarkMin = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->brightGainLmt[i];
            cur.brightGainLmt = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->brightGainLmtStep[i];
            cur.brightGainLmtStep = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->darkGainLmtY[i];
            cur.darkGainLmtY = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->darkGainLmtC[i];
            cur.darkGainLmtC = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->fltScaleCoarse[i];
            cur.fltScaleCoarse = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->fltScaleFine[i];
            cur.fltScaleFine = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->contrastControl[i];
            cur.contrastControl = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);

            HI_S32 svals[V4_IQ_DYN_MAX_POINTS];
            for (int i = 0; i < drc->n; i++) svals[i] = (HI_S32)drc->detailAdjustFactor[i];
            cur.detailAdjustFactor = (HI_S8)v4_iq_dyn_interp_s32(iso, drc->iso, svals, drc->n);

            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->asymmetry[i];
            cur.asymmetry = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->secondPole[i];
            cur.secondPole = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->compress[i];
            cur.compress = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);
            for (int i = 0; i < drc->n; i++) u16vals[i] = drc->stretch[i];
            cur.stretch = (HI_U8)v4_iq_dyn_interp_u32(iso, drc->iso, u16vals, drc->n);

            if (is_lowlight && have_drc_caps) {
                const HI_BOOL fast_ll = (have_fast && ll_fast_expmax > 0 && expi.u8AveLum >= ll_fast_lum) ? HI_TRUE : HI_FALSE;
                // Choose caps (fast caps fallback to normal caps if not provided)
                HI_U16 max_strength = fast_ll ? fast_str_max : ll_str_max;
                if (max_strength == 0) max_strength = cur.strength; // no cap if not configured

                HI_U8 bm_max = ll_bm_max, bm_min = ll_bm_min, bg_lmt = ll_bg_lmt, bg_step = ll_bg_step;
                HI_U8 dm_max = fast_ll ? (fast_dm_max ? fast_dm_max : ll_dm_max) : ll_dm_max;
                HI_U8 dm_min = fast_ll ? (fast_dm_min ? fast_dm_min : ll_dm_min) : ll_dm_min;
                HI_U8 cc_max = fast_ll ? (fast_cc_max ? fast_cc_max : ll_cc_max) : ll_cc_max;

                if (max_strength && cur.strength > max_strength) cur.strength = max_strength;
                if (bm_max && cur.localMixBrightMax > bm_max) cur.localMixBrightMax = bm_max;
                if (bm_min && cur.localMixBrightMin > bm_min) cur.localMixBrightMin = bm_min;
                if (bg_lmt && cur.brightGainLmt > bg_lmt) cur.brightGainLmt = bg_lmt;
                if (bg_step && cur.brightGainLmtStep > bg_step) cur.brightGainLmtStep = bg_step;
                if (dm_max && cur.localMixDarkMax > dm_max) cur.localMixDarkMax = dm_max;
                if (dm_min && cur.localMixDarkMin > dm_min) cur.localMixDarkMin = dm_min;
                if (cc_max && cur.contrastControl > cc_max) cur.contrastControl = cc_max;
            }

            if (!have_last || memcmp(&cur, &last_drc, sizeof(cur)) != 0) {
                ISP_DRC_ATTR_S da;
                memset(&da, 0, sizeof(da));
                if (!v4_isp.fnGetDRCAttr(pipe, &da)) {
                    // Ensure DRC module isn't bypassed (some firmwares default to bypass in linear mode).
                    if (v4_isp.fnGetModuleControl && v4_isp.fnSetModuleControl) {
                        ISP_MODULE_CTRL_U mc;
                        memset(&mc, 0, sizeof(mc));
                        if (!v4_isp.fnGetModuleControl(pipe, &mc)) {
                            mc.u64Key &= ~(1ULL << 8); // bitBypassDRC
                            (void)v4_isp.fnSetModuleControl(pipe, &mc);
                        }
                    }

                    ISP_DRC_ATTR_S orig = da;
                    da.bEnable = HI_TRUE;
                    da.enOpType = OP_TYPE_AUTO;
                    da.enCurveSelect = DRC_CURVE_ASYMMETRY;
                    da.stAuto.u16Strength = cur.strength;
                    da.u8LocalMixingBrightMax = cur.localMixBrightMax;
                    da.u8LocalMixingBrightMin = cur.localMixBrightMin;
                    da.u8LocalMixingDarkMax = cur.localMixDarkMax;
                    da.u8LocalMixingDarkMin = cur.localMixDarkMin;
                    da.u8BrightGainLmt = cur.brightGainLmt;
                    da.u8BrightGainLmtStep = cur.brightGainLmtStep;
                    da.u8DarkGainLmtY = cur.darkGainLmtY;
                    da.u8DarkGainLmtC = cur.darkGainLmtC;
                    da.u8FltScaleCoarse = cur.fltScaleCoarse;
                    da.u8FltScaleFine = cur.fltScaleFine;
                    da.u8ContrastControl = cur.contrastControl;
                    da.s8DetailAdjustFactor = cur.detailAdjustFactor;
                    da.stAsymmetryCurve.u8Asymmetry = cur.asymmetry;
                    da.stAsymmetryCurve.u8SecondPole = cur.secondPole;
                    da.stAsymmetryCurve.u8Compress = cur.compress;
                    da.stAsymmetryCurve.u8Stretch = cur.stretch;

                    int sr = v4_isp.fnSetDRCAttr(pipe, &da);
                    if (!sr) {
                        pthread_mutex_lock(&_v4_iq_dyn.lock);
                        _v4_iq_dyn.last_drc = cur;
                        _v4_iq_dyn.have_last = HI_TRUE;
                        _v4_iq_dyn.drc_fail_count = 0;
                        _v4_iq_dyn.drc_disabled = HI_FALSE;
                        pthread_mutex_unlock(&_v4_iq_dyn.lock);
                        HAL_INFO("v4_iq", "Dynamic Linear DRC: iso=%u strength=%u contrast=%u\n",
                            (unsigned)iso, (unsigned)cur.strength, (unsigned)cur.contrastControl);
                    } else {
                        // Some SDK builds reject ASYMMETRY params (enum/layout differences).
                        // Fallback: enable DRC with minimal fields and keep existing curve select.
                        HAL_WARNING("v4_iq",
                            "Dynamic Linear DRC: SetDRCAttr(ASYMMETRY) failed with %#x (iso=%u strength=%u); retrying minimal\n",
                            sr, (unsigned)iso, (unsigned)cur.strength);

                        ISP_DRC_ATTR_S fb = orig;
                        fb.bEnable = HI_TRUE;
                        fb.enOpType = OP_TYPE_AUTO;
                        fb.stAuto.u16Strength = cur.strength;
                        // Try a safer curve select if orig is out of range.
                        if (fb.enCurveSelect < DRC_CURVE_ASYMMETRY || fb.enCurveSelect > DRC_CURVE_USER)
                            fb.enCurveSelect = DRC_CURVE_CUBIC;
                        sr = v4_isp.fnSetDRCAttr(pipe, &fb);
                        if (!sr) {
                            pthread_mutex_lock(&_v4_iq_dyn.lock);
                            _v4_iq_dyn.last_drc = cur;
                            _v4_iq_dyn.have_last = HI_TRUE;
                            _v4_iq_dyn.drc_fail_count = 0;
                            _v4_iq_dyn.drc_disabled = HI_FALSE;
                            pthread_mutex_unlock(&_v4_iq_dyn.lock);
                            HAL_INFO("v4_iq", "Dynamic Linear DRC: applied via fallback (curve=%d strength=%u)\n",
                                (int)fb.enCurveSelect, (unsigned)cur.strength);
                        } else {
                            HAL_WARNING("v4_iq",
                                "Dynamic Linear DRC: fallback SetDRCAttr failed with %#x (curve=%d strength=%u)\n",
                                sr, (int)fb.enCurveSelect, (unsigned)cur.strength);

                            pthread_mutex_lock(&_v4_iq_dyn.lock);
                            _v4_iq_dyn.drc_fail_count++;
                            int fc = _v4_iq_dyn.drc_fail_count;
                            if (fc >= 3) {
                                _v4_iq_dyn.drc_disabled = HI_TRUE;
                                HAL_WARNING("v4_iq",
                                    "Dynamic Linear DRC: disabling due to repeated failures (last=%#x)\n", sr);
                            }
                            pthread_mutex_unlock(&_v4_iq_dyn.lock);
                        }
                    }
                }
            }
        }

        // Auto AE switching by low-light (even if user keeps DAY mode at night)
        // IMPORTANT: do NOT apply this logic in true IR mode; IR must use [ir_static_ae].
        if (!is_ir_mode && ll_ae && have_ae_day && have_ae_low &&
            v4_isp.fnGetExposureAttr && v4_isp.fnSetExposureAttr) {
            ISP_EXPOSURE_ATTR_S ea;
            memset(&ea, 0, sizeof(ea));
            if (v4_isp.fnGetExposureAttr(pipe, &ea) == 0) {
                // Compute desired knobs for current lowlight state
                HI_U8 want_comp = is_lowlight ? low_comp : day_comp;
                HI_U32 want_expmax = is_lowlight ? low_expmax : day_expmax;
                HI_U32 want_sysgainmax = is_lowlight ? low_sysgainmax : day_sysgainmax;
                HI_U32 want_againmax = is_lowlight ? low_againmax : day_againmax;
                HI_U8 want_histoff = is_lowlight ? low_histoff : day_histoff;
                HI_U16 want_histslope = is_lowlight ? low_histslope : day_histslope;
                HI_U8 want_speed = is_lowlight ? low_speed : day_speed;

                // Fast lowlight: if configured and scene is bright enough, prefer shorter shutter to reduce motion blur.
                // Comp boost helps keep the image from looking "too dark" when we cut shutter time.
                const HI_BOOL fast_ll = (is_lowlight && have_fast && ll_fast_expmax > 0 && expi.u8AveLum >= ll_fast_lum) ? HI_TRUE : HI_FALSE;
                if (fast_ll) {
                    if (want_expmax > ll_fast_expmax) want_expmax = ll_fast_expmax;
                    if (ll_fast_againmax > 0) want_againmax = ll_fast_againmax;
                    if (ll_fast_comp_boost > 0) {
                        HI_U32 c = (HI_U32)want_comp + (HI_U32)ll_fast_comp_boost;
                        if (c > 255) c = 255;
                        want_comp = (HI_U8)c;
                    }
                }

                // Apply if state changed OR if current params don't match desired
                const HI_BOOL mismatch =
                    (ea.stAuto.u8Compensation != want_comp) ||
                    (ea.stAuto.stExpTimeRange.u32Max != want_expmax) ||
                    (ea.stAuto.stSysGainRange.u32Max != want_sysgainmax) ||
                    ((want_againmax != 0) && (ea.stAuto.stAGainRange.u32Max != want_againmax)) ||
                    (ea.stAuto.u8MaxHistOffset != want_histoff) ||
                    (ea.stAuto.u16HistRatioSlope != want_histslope) ||
                    (ea.stAuto.u8Speed != want_speed);

                // Rate limit retries to avoid log spam
                static time_t last_ae_try = 0;
                time_t now2 = time(NULL);
                const HI_BOOL allow_try =
                    (now2 == (time_t)-1) ? HI_TRUE : ((now2 - last_ae_try) >= 2);

                if ((is_lowlight != last_ae_low || mismatch) && allow_try) {
                    last_ae_try = now2;

                    ea.stAuto.u8Compensation = want_comp;
                    ea.stAuto.stExpTimeRange.u32Max = want_expmax;
                    ea.stAuto.stSysGainRange.u32Max = want_sysgainmax;
                    if (want_againmax != 0)
                        ea.stAuto.stAGainRange.u32Max = want_againmax;
                    ea.stAuto.u8MaxHistOffset = want_histoff;
                    ea.stAuto.u16HistRatioSlope = want_histslope;
                    ea.stAuto.u8Speed = want_speed;

                    int sr = v4_isp.fnSetExposureAttr(pipe, &ea);
                    if (!sr) {
                        pthread_mutex_lock(&_v4_iq_dyn.lock);
                        _v4_iq_dyn.last_ae_lowlight = is_lowlight;
                        pthread_mutex_unlock(&_v4_iq_dyn.lock);
                        HAL_INFO("v4_iq",
                            "Dynamic AE: lowlight=%d iso=%u expTime=%u aveLum=%u comp=%u expMax=%u aGainMax=%u sysGainMax=%u\n",
                            (int)is_lowlight, (unsigned)iso, (unsigned)expi.u32ExpTime, (unsigned)expi.u8AveLum,
                            (unsigned)want_comp, (unsigned)want_expmax, (unsigned)want_againmax, (unsigned)want_sysgainmax);
                    } else {
                        HAL_WARNING("v4_iq",
                            "Dynamic AE: SetExposureAttr failed with %#x (lowlight=%d wantComp=%u wantExpMax=%u)\n",
                            sr, (int)is_lowlight, (unsigned)want_comp, (unsigned)want_expmax);
                    }
                }
            }
        }

        // Optional lowlight FX/marketing processing (enabled only if configured in IQ):
        // When lowlight is active and the image is truly noisy, reduce "marketing" processing:
        // - Gamma often raises shadows and reveals noise
        // - Sharpen turns noise into grain
        // But if ISO/gains are low (clean image), keep original IQ look.
        {
            if (!have_fx) {
                // No lowlight FX section in IQ => do not touch gamma/sharpen dynamically.
                goto fx_done;
            }
            static HI_BOOL baseline_init = HI_FALSE;
            static HI_BOOL baseline_gamma = HI_TRUE;
            static HI_BOOL baseline_sharpen = HI_TRUE;
            static HI_BOOL baseline_sh_attr_valid = HI_FALSE;
            static ISP_SHARPEN_ATTR_S baseline_sh_attr;
            static HI_BOOL last_noisy_fx = HI_FALSE;
            static time_t last_fx_try = 0;
            time_t now3 = time(NULL);
            HI_BOOL allow_fx = (now3 == (time_t)-1) ? HI_TRUE : ((now3 - last_fx_try) >= 2);
            // Consider it "noisy" only when we actually need to hide noise.
            // Thresholds are conservative; tune later if needed.
            // Separate thresholds: keep gamma longer, disable sharpen earlier.
            //
            // IMPORTANT: Apply this not only in "DAY-at-night" lowlight, but also in true IR mode.
            // In IR the scene often sits at max exposure and high digital/ISP gain; leaving gamma/sharpen
            // enabled makes noise look much worse ("sand"). This is NOT about "crank gain then denoise";
            // it's about avoiding noise amplification by marketing processing.
            const HI_BOOL allow_noisy_fx = (is_lowlight == HI_TRUE || is_ir_mode) ? HI_TRUE : HI_FALSE;
            const HI_BOOL noisy_gamma =
                allow_noisy_fx &&
                ((noisy_gamma_iso && iso >= noisy_gamma_iso) ||
                 (noisy_gamma_dg && expi.u32DGain > noisy_gamma_dg) ||
                 (noisy_gamma_ispg && expi.u32ISPDGain > noisy_gamma_ispg));
            const HI_BOOL noisy_sharpen =
                allow_noisy_fx &&
                ((noisy_sh_iso && iso >= noisy_sh_iso) ||
                 (noisy_sh_dg && expi.u32DGain > noisy_sh_dg) ||
                 (noisy_sh_ispg && expi.u32ISPDGain > noisy_sh_ispg));
            const HI_BOOL noisy_lowlight = (noisy_gamma || noisy_sharpen) ? HI_TRUE : HI_FALSE;

            if (allow_fx && (!baseline_init || noisy_lowlight != last_noisy_fx)) {
                last_fx_try = now3;

                if (v4_isp.fnGetGammaAttr && v4_isp.fnSetGammaAttr) {
                    ISP_GAMMA_ATTR_S ga;
                    memset(&ga, 0, sizeof(ga));
                    if (!v4_isp.fnGetGammaAttr(pipe, &ga)) {
                        if (!baseline_init) baseline_gamma = ga.bEnable;
                        ga.bEnable = noisy_gamma ? HI_FALSE : baseline_gamma;
                        (void)v4_isp.fnSetGammaAttr(pipe, &ga);
                    }
                }

                if (v4_isp.fnGetIspSharpenAttr && v4_isp.fnSetIspSharpenAttr) {
                    ISP_SHARPEN_ATTR_S sh;
                    memset(&sh, 0, sizeof(sh));
                    if (!v4_isp.fnGetIspSharpenAttr(pipe, &sh)) {
                        if (!baseline_init) baseline_sharpen = sh.bEnable;
                        if (!baseline_sh_attr_valid) {
                            baseline_sh_attr = sh;
                            baseline_sh_attr_valid = HI_TRUE;
                        }

                        // Start from baseline each time to avoid accumulating modifications.
                        if (baseline_sh_attr_valid) sh = baseline_sh_attr;

                        // Anti-halo sharpen in lowlight (optional, enabled only if configured):
                        // keep sharpening ON for marketing, but reduce overshoot/undershoot which causes glow around lamps.
                        if (is_lowlight && !noisy_sharpen && have_ah) {
                            sh.bEnable = baseline_sharpen;
                            if (sh.enOpType == OP_TYPE_AUTO) {
                                for (int i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
                                    // Soften ringing/halos
                                    if (ah_over_pct)  sh.stAuto.au8OverShoot[i]  = (HI_U8)((sh.stAuto.au8OverShoot[i]  * ah_over_pct) / 100);
                                    if (ah_under_pct) sh.stAuto.au8UnderShoot[i] = (HI_U8)((sh.stAuto.au8UnderShoot[i] * ah_under_pct) / 100);
                                    if (ah_min_over && sh.stAuto.au8OverShoot[i] < ah_min_over)   sh.stAuto.au8OverShoot[i] = ah_min_over;
                                    if (ah_min_under && sh.stAuto.au8UnderShoot[i] < ah_min_under) sh.stAuto.au8UnderShoot[i] = ah_min_under;
                                    // Increase suppression/filter a bit to reduce edge glow
                                    if (ah_shootsup_min && sh.stAuto.au8ShootSupStr[i] < ah_shootsup_min) sh.stAuto.au8ShootSupStr[i] = ah_shootsup_min;
                                    if (ah_edgefilt_min && sh.stAuto.au8EdgeFiltStr[i]  < ah_edgefilt_min) sh.stAuto.au8EdgeFiltStr[i] = ah_edgefilt_min;
                                    if (ah_maxsharp_cap && sh.stAuto.au16MaxSharpGain[i] > ah_maxsharp_cap) sh.stAuto.au16MaxSharpGain[i] = ah_maxsharp_cap;
                                }
                                // Slightly reduce overall edge/texture strength to reduce halos, but keep crispness.
                                for (int g = 0; g < ISP_SHARPEN_GAIN_NUM; g++) {
                                    for (int i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
                                        if (ah_texstr_pct)  sh.stAuto.au16TextureStr[g][i] = (HI_U16)((sh.stAuto.au16TextureStr[g][i] * ah_texstr_pct) / 100);
                                        if (ah_edgestr_pct) sh.stAuto.au16EdgeStr[g][i]    = (HI_U16)((sh.stAuto.au16EdgeStr[g][i]    * ah_edgestr_pct) / 100);
                                    }
                                }
                            }
                        } else {
                            // Noisy: disable sharpening to avoid turning noise into grain
                            sh.bEnable = noisy_sharpen ? HI_FALSE : baseline_sharpen;
                        }
                        (void)v4_isp.fnSetIspSharpenAttr(pipe, &sh);
                    }
                }

                baseline_init = HI_TRUE;
                last_noisy_fx = noisy_lowlight;
                HAL_INFO("v4_iq", "Dynamic FX: lowlight=%d noisy=%d gamma=%s sharpen=%s (iso=%u dg=%u ispdg=%u)\n",
                    (int)is_lowlight, (int)noisy_lowlight,
                    noisy_gamma ? "off" : (baseline_gamma ? "on" : "off"),
                    noisy_sharpen ? "off" : (baseline_sharpen ? "on" : "off"),
                    (unsigned)iso, (unsigned)expi.u32DGain, (unsigned)expi.u32ISPDGain);
            }
fx_done:
            ;
        }

        // 3DNR (VPSS NRX) by ISO + night mode
        {
            if (nr3d_disabled) {
                // disabled due to repeated errors
            } else {
            const bool is_ir = is_ir_mode;
            const v4_iq_3dnr_cfg *cfg = NULL;
            if (is_ir && nr_ir.enabled) cfg = &nr_ir;
            else if (nr_day.enabled) cfg = &nr_day;

            if (cfg && cfg->enabled && cfg->n > 0) {
                int desired = 0;
                for (int i = 0; i < cfg->n; i++) {
                    if (iso >= cfg->iso[i]) desired = i;
                    else break;
                }

                bool allow = false;
                if (last_nr_idx < 0 || (last_nr_ir != (is_ir ? HI_TRUE : HI_FALSE))) {
                    allow = true; // first apply or IR mode changed
                } else if (desired != last_nr_idx) {
                    if (upIso == 0 && downIso == 0) {
                        allow = true;
                    } else if (desired > last_nr_idx) {
                        if (downIso == 0 || iso >= downIso) allow = true;
                    } else if (desired < last_nr_idx) {
                        if (upIso == 0 || iso <= upIso) allow = true;
                    }
                }

                if (allow && (desired != last_nr_idx || last_nr_ir != (is_ir ? HI_TRUE : HI_FALSE))) {
                    int ar = v4_iq_apply_3dnr_nrc(_v4_vi_pipe, _v4_vpss_grp, &cfg->nrc[desired]);
                    if (!ar) {
                        pthread_mutex_lock(&_v4_iq_dyn.lock);
                        _v4_iq_dyn.last_nr3d_idx = desired;
                        _v4_iq_dyn.last_nr3d_is_ir = is_ir ? HI_TRUE : HI_FALSE;
                        _v4_iq_dyn.nr3d_fail_count = 0;
                        pthread_mutex_unlock(&_v4_iq_dyn.lock);
                        HAL_INFO("v4_iq", "3DNR: %s idx=%d iso=%u (sfc=%d tfc=%d tpc=%d trc=%d)\n",
                            is_ir ? "IR" : "DAY", desired, (unsigned)iso,
                            (int)cfg->nrc[desired].sfc, (int)cfg->nrc[desired].tfc,
                            (int)cfg->nrc[desired].tpc, (int)cfg->nrc[desired].trc);
                    } else {
                        // Disable after a few failures to avoid log spam
                        pthread_mutex_lock(&_v4_iq_dyn.lock);
                        _v4_iq_dyn.nr3d_fail_count++;
                        int fc = _v4_iq_dyn.nr3d_fail_count;
                        if (fc == 1) {
                            HAL_WARNING("v4_iq", "3DNR: apply failed with %#x (will retry a few times)\n", ar);
                        }
                        if (fc >= 3) {
                            _v4_iq_dyn.nr3d_disabled = HI_TRUE;
                            HAL_WARNING("v4_iq", "3DNR: disabling due to repeated failures (last=%#x)\n", ar);
                        }
                        pthread_mutex_unlock(&_v4_iq_dyn.lock);
                    }
                }
            }
            }
        }

        usleep(500 * 1000);
    }
    return NULL;
}

static void v4_iq_dyn_update_from_ini(struct IniConfig *ini, int pipe, bool enableDynDehaze, bool enableDynLinearDRC) {
    pthread_mutex_lock(&_v4_iq_dyn.lock);
    _v4_iq_dyn.pipe = pipe;
    pthread_mutex_unlock(&_v4_iq_dyn.lock);
    v4_iq_dyn_load_from_ini(ini, enableDynDehaze, enableDynLinearDRC);
}

static void v4_iq_dyn_maybe_start(int pipe) {
    bool need = false;
    bool start = false;
    pthread_mutex_lock(&_v4_iq_dyn.lock);
    need = (_v4_iq_dyn.dehaze_day.enabled || _v4_iq_dyn.dehaze_ir.enabled ||
            _v4_iq_dyn.linear_drc_day.enabled || _v4_iq_dyn.linear_drc_ir.enabled ||
            _v4_iq_dyn.nr3d_day.enabled || _v4_iq_dyn.nr3d_ir.enabled ||
            _v4_iq_dyn.have_aerouteex_day || _v4_iq_dyn.have_aerouteex_ir ||
            (_v4_iq_dyn.lowlight_auto_ae && _v4_iq_dyn.have_ae_day && _v4_iq_dyn.have_ae_low));
    if (need && !_v4_iq_dyn.thread_started) {
        _v4_iq_dyn.thread_started = true;
        start = true;
    }
    pthread_mutex_unlock(&_v4_iq_dyn.lock);

    if (!start) return;

    pthread_t tid;
    if (pthread_create(&tid, NULL, v4_iq_dynamic_thread, (void *)(intptr_t)pipe) == 0) {
        pthread_detach(tid);
    } else {
        pthread_mutex_lock(&_v4_iq_dyn.lock);
        _v4_iq_dyn.thread_started = false;
        pthread_mutex_unlock(&_v4_iq_dyn.lock);
    }
}

static int v4_iq_apply_static_ae(struct IniConfig *ini, int pipe);
static int v4_iq_apply_static_aerouteex(struct IniConfig *ini, int pipe);

static void v4_iq_apply_ae_route_for_current_mode(int pipe) {
    if (!_v4_iq_cfg_path[0])
        return;

    struct IniConfig ini;
    memset(&ini, 0, sizeof(ini));
    FILE *file = fopen(_v4_iq_cfg_path, "r");
    if (!open_config(&ini, &file))
        return;
    find_sections(&ini);

    bool doStaticAE = true;
    parse_bool(&ini, "module_state", "bStaticAE", &doStaticAE);
    if (doStaticAE) {
        (void)v4_iq_apply_static_ae(&ini, pipe);
        (void)v4_iq_apply_static_aerouteex(&ini, pipe);
    }

    free(ini.str);
}

static int v4_iq_apply_static_ae(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetExposureAttr || !v4_isp.fnSetExposureAttr) {
        HAL_INFO("v4_iq", "AE: API not available, skipping\n");
        return EXIT_SUCCESS;
    }
    const char *sec = "static_ae";
    int sec_s = 0, sec_e = 0;
    if (night_mode_on() && section_pos(ini, "ir_static_ae", &sec_s, &sec_e) == CONFIG_OK) {
        sec = "ir_static_ae";
    } else if (section_pos(ini, "static_ae", &sec_s, &sec_e) != CONFIG_OK) {
        HAL_INFO("v4_iq", "AE: no [static_ae] section, skipping\n");
        return EXIT_SUCCESS;
    }

    ISP_EXPOSURE_ATTR_S exp;
    memset(&exp, 0, sizeof(exp));
    int ret = v4_isp.fnGetExposureAttr(pipe, &exp);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetExposureAttr failed with %#x\n", ret);
        return ret;
    }

    int val;
    // Top-level AE/exposure switches
    if (parse_int(ini, sec, "ByPass", 0, 1, &val) == CONFIG_OK)
        exp.bByPass = (HI_BOOL)val;
    if (parse_int(ini, sec, "HistStatAdjust", 0, 1, &val) == CONFIG_OK)
        exp.bHistStatAdjust = (HI_BOOL)val;
    if (parse_int(ini, sec, "AERunInterval", 1, 255, &val) == CONFIG_OK)
        exp.u8AERunInterval = (HI_U8)val;
    if (parse_int(ini, sec, "AERouteExValid", 0, 1, &val) == CONFIG_OK)
        exp.bAERouteExValid = (HI_BOOL)val;
    // If RouteEx table is known to be rejected by this SDK, don't let firmware try to use it.
    if (exp.bAERouteExValid) {
        pthread_mutex_lock(&_v4_iq_dyn.lock);
        HI_BOOL disabled = _v4_iq_dyn.aerouteex_disabled;
        pthread_mutex_unlock(&_v4_iq_dyn.lock);
        if (disabled) exp.bAERouteExValid = HI_FALSE;
    }
    if (parse_int(ini, sec, "PriorFrame", 0, 2, &val) == CONFIG_OK)
        exp.enPriorFrame = (ISP_PRIOR_FRAME_E)val;
    if (parse_int(ini, sec, "AEGainSepCfg", 0, 1, &val) == CONFIG_OK)
        exp.bAEGainSepCfg = (HI_BOOL)val;
    if (parse_int(ini, sec, "AEOpType", 0, 1, &val) == CONFIG_OK)
        exp.enOpType = (ISP_OP_TYPE_E)val;

    // Auto ranges (the most common knobs in vendor IQ)
    if (parse_int(ini, sec, "AutoExpTimeMin", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stAuto.stExpTimeRange.u32Min = (HI_U32)val;
    if (parse_int(ini, sec, "AutoExpTimeMax", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stAuto.stExpTimeRange.u32Max = (HI_U32)val;
    if (parse_int(ini, sec, "AutoAGainMin", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stAuto.stAGainRange.u32Min = (HI_U32)val;
    if (parse_int(ini, sec, "AutoAGainMax", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stAuto.stAGainRange.u32Max = (HI_U32)val;
    if (parse_int(ini, sec, "AutoDGainMin", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stAuto.stDGainRange.u32Min = (HI_U32)val;
    if (parse_int(ini, sec, "AutoDGainMax", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stAuto.stDGainRange.u32Max = (HI_U32)val;
    if (parse_int(ini, sec, "AutoISPDGainMin", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stAuto.stISPDGainRange.u32Min = (HI_U32)val;
    if (parse_int(ini, sec, "AutoISPDGainMax", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stAuto.stISPDGainRange.u32Max = (HI_U32)val;
    if (parse_int(ini, sec, "AutoSysGainMin", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stAuto.stSysGainRange.u32Min = (HI_U32)val;
    if (parse_int(ini, sec, "AutoSysGainMax", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stAuto.stSysGainRange.u32Max = (HI_U32)val;
    if (parse_int(ini, sec, "AutoGainThreshold", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stAuto.u32GainThreshold = (HI_U32)val;

    // Auto behavior
    if (parse_int(ini, sec, "AutoSpeed", 0, 255, &val) == CONFIG_OK)
        exp.stAuto.u8Speed = (HI_U8)val;
    if (parse_int(ini, sec, "AutoBlackSpeedBias", 0, 65535, &val) == CONFIG_OK)
        exp.stAuto.u16BlackSpeedBias = (HI_U16)val;
    if (parse_int(ini, sec, "AutoTolerance", 0, 255, &val) == CONFIG_OK)
        exp.stAuto.u8Tolerance = (HI_U8)val;
    if (parse_int(ini, sec, "AutoCompensation", 0, 255, &val) == CONFIG_OK)
        exp.stAuto.u8Compensation = (HI_U8)val;
    if (parse_int(ini, sec, "AutoEVBias", 0, 65535, &val) == CONFIG_OK)
        exp.stAuto.u16EVBias = (HI_U16)val;
    if (parse_int(ini, sec, "AutoAEStrategyMode", 0, 1, &val) == CONFIG_OK)
        exp.stAuto.enAEStrategyMode = (ISP_AE_STRATEGY_E)val;
    if (parse_int(ini, sec, "AutoHistRatioSlope", 0, 65535, &val) == CONFIG_OK)
        exp.stAuto.u16HistRatioSlope = (HI_U16)val;
    if (parse_int(ini, sec, "AutoMaxHistOffset", 0, 255, &val) == CONFIG_OK)
        exp.stAuto.u8MaxHistOffset = (HI_U8)val;
    if (parse_int(ini, sec, "AutoAEMode", 0, 1, &val) == CONFIG_OK)
        exp.stAuto.enAEMode = (ISP_AE_MODE_E)val;

    // Anti-flicker (optional)
    if (parse_int(ini, sec, "AntiFlickerEnable", 0, 1, &val) == CONFIG_OK)
        exp.stAuto.stAntiflicker.bEnable = (HI_BOOL)val;
    if (parse_int(ini, sec, "AntiFlickerFrequency", 0, 255, &val) == CONFIG_OK)
        exp.stAuto.stAntiflicker.u8Frequency = (HI_U8)val;
    if (parse_int(ini, sec, "AntiFlickerMode", 0, 1, &val) == CONFIG_OK)
        exp.stAuto.stAntiflicker.enMode = (ISP_ANTIFLICKER_MODE_E)val;
    if (parse_int(ini, sec, "SubFlickerEnable", 0, 1, &val) == CONFIG_OK)
        exp.stAuto.stSubflicker.bEnable = (HI_BOOL)val;
    if (parse_int(ini, sec, "SubFlickerLumaDiff", 0, 255, &val) == CONFIG_OK)
        exp.stAuto.stSubflicker.u8LumaDiff = (HI_U8)val;

    // Manual exposure (optional)
    if (parse_int(ini, sec, "ManualExpTimeOpType", 0, 1, &val) == CONFIG_OK)
        exp.stManual.enExpTimeOpType = (ISP_OP_TYPE_E)val;
    if (parse_int(ini, sec, "ManualAGainOpType", 0, 1, &val) == CONFIG_OK)
        exp.stManual.enAGainOpType = (ISP_OP_TYPE_E)val;
    if (parse_int(ini, sec, "ManualDGainOpType", 0, 1, &val) == CONFIG_OK)
        exp.stManual.enDGainOpType = (ISP_OP_TYPE_E)val;
    if (parse_int(ini, sec, "ManualISPDGainOpType", 0, 1, &val) == CONFIG_OK)
        exp.stManual.enISPDGainOpType = (ISP_OP_TYPE_E)val;
    if (parse_int(ini, sec, "ManualExpTime", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stManual.u32ExpTime = (HI_U32)val;
    if (parse_int(ini, sec, "ManualAGain", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stManual.u32AGain = (HI_U32)val;
    if (parse_int(ini, sec, "ManualDGain", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stManual.u32DGain = (HI_U32)val;
    if (parse_int(ini, sec, "ManualISPDGain", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stManual.u32ISPDGain = (HI_U32)val;

    if (parse_int(ini, sec, "AutoBlackDelayFrame", 0, 65535, &val) == CONFIG_OK)
        exp.stAuto.stAEDelayAttr.u16BlackDelayFrame = (HI_U16)val;
    if (parse_int(ini, sec, "AutoWhiteDelayFrame", 0, 65535, &val) == CONFIG_OK)
        exp.stAuto.stAEDelayAttr.u16WhiteDelayFrame = (HI_U16)val;

    ret = v4_isp.fnSetExposureAttr(pipe, &exp);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetExposureAttr failed with %#x\n", ret);
    } else {
        ISP_EXPOSURE_ATTR_S rb;
        memset(&rb, 0, sizeof(rb));
        int gr = v4_isp.fnGetExposureAttr(pipe, &rb);
        if (!gr) {
            HAL_INFO("v4_iq",
                "AE: applied (runInt=%u routeExValid=%d comp=%u expMax=%u aGainMax=%u dGainMax=%u ispdGainMax=%u sysGainMax=%u speed=%u histOff=%u histSlope=%u)\n",
                (unsigned)rb.u8AERunInterval,
                (int)rb.bAERouteExValid,
                (unsigned)rb.stAuto.u8Compensation,
                (unsigned)rb.stAuto.stExpTimeRange.u32Max,
                (unsigned)rb.stAuto.stAGainRange.u32Max,
                (unsigned)rb.stAuto.stDGainRange.u32Max,
                (unsigned)rb.stAuto.stISPDGainRange.u32Max,
                (unsigned)rb.stAuto.stSysGainRange.u32Max,
                (unsigned)rb.stAuto.u8Speed,
                (unsigned)rb.stAuto.u8MaxHistOffset,
                (unsigned)rb.stAuto.u16HistRatioSlope);
        } else {
            HAL_INFO("v4_iq", "AE: applied\n");
        }
    }
    return ret;
}

static int v4_iq_apply_static_aerouteex(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetAERouteAttrEx || !v4_isp.fnSetAERouteAttrEx) {
        HAL_INFO("v4_iq", "AE route-ex: API not available, skipping\n");
        return EXIT_SUCCESS;
    }
    pthread_mutex_lock(&_v4_iq_dyn.lock);
    HI_BOOL disabled = _v4_iq_dyn.aerouteex_disabled;
    pthread_mutex_unlock(&_v4_iq_dyn.lock);
    if (disabled) {
        HAL_INFO("v4_iq", "AE route-ex: disabled (previous SDK reject), skipping\n");
        return EXIT_SUCCESS;
    }
    const char *sec = "static_aerouteex";
    int sec_s = 0, sec_e = 0;
    if (night_mode_on() && section_pos(ini, "ir_static_aerouteex", &sec_s, &sec_e) == CONFIG_OK) {
        sec = "ir_static_aerouteex";
    } else if (section_pos(ini, "static_aerouteex", &sec_s, &sec_e) != CONFIG_OK) {
        HAL_INFO("v4_iq", "AE route-ex: no [static_aerouteex] section, skipping\n");
        return EXIT_SUCCESS;
    }

    ISP_AE_ROUTE_EX_S route;
    memset(&route, 0, sizeof(route));
    int ret = v4_isp.fnGetAERouteAttrEx(pipe, &route);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetAERouteAttrEx failed with %#x\n", ret);
        return ret;
    }

    // Get current AE range limits so we can clamp route tables to what the firmware accepts.
    HI_U32 exp_max = 0, exp_min = 0;
    HI_U32 again_max = 0, dgain_max = 0, ispdgain_max = 0, sysgain_max = 0;
    if (v4_isp.fnGetExposureAttr) {
        ISP_EXPOSURE_ATTR_S exp;
        memset(&exp, 0, sizeof(exp));
        if (v4_isp.fnGetExposureAttr(pipe, &exp) == 0) {
            exp_min = exp.stAuto.stExpTimeRange.u32Min;
            exp_max = exp.stAuto.stExpTimeRange.u32Max;
            again_max = exp.stAuto.stAGainRange.u32Max;
            dgain_max = exp.stAuto.stDGainRange.u32Max;
            ispdgain_max = exp.stAuto.stISPDGainRange.u32Max;
            sysgain_max = exp.stAuto.stSysGainRange.u32Max;
        }
    }

    int total = 0;
    if (parse_int(ini, sec, "TotalNum", 0, ISP_AE_ROUTE_EX_MAX_NODES, &total) != CONFIG_OK) {
        HAL_INFO("v4_iq", "AE route-ex: no %s/TotalNum, skipping\n", sec);
        return EXIT_SUCCESS;
    }

    HI_U32 ints[ISP_AE_ROUTE_EX_MAX_NODES] = {0};
    HI_U32 again[ISP_AE_ROUTE_EX_MAX_NODES] = {0};
    HI_U32 dgain[ISP_AE_ROUTE_EX_MAX_NODES] = {0};
    HI_U32 ispdgain[ISP_AE_ROUTE_EX_MAX_NODES] = {0};

    int nInts = v4_iq_parse_multiline_u32(ini, sec, "RouteEXIntTime", ints, ISP_AE_ROUTE_EX_MAX_NODES);
    int nAgain = v4_iq_parse_multiline_u32(ini, sec, "RouteEXAGain", again, ISP_AE_ROUTE_EX_MAX_NODES);
    int nDgain = v4_iq_parse_multiline_u32(ini, sec, "RouteEXDGain", dgain, ISP_AE_ROUTE_EX_MAX_NODES);
    int nIspDgain = v4_iq_parse_multiline_u32(ini, sec, "RouteEXISPDGain", ispdgain, ISP_AE_ROUTE_EX_MAX_NODES);

    route.u32TotalNum = (HI_U32)total;
    for (int i = 0; i < total && i < ISP_AE_ROUTE_EX_MAX_NODES; i++) {
        if (i < nInts) route.astRouteExNode[i].u32IntTime = ints[i];
        if (i < nAgain) route.astRouteExNode[i].u32Again = again[i];
        if (i < nDgain) route.astRouteExNode[i].u32Dgain = dgain[i];
        if (i < nIspDgain) route.astRouteExNode[i].u32IspDgain = ispdgain[i];
    }

    // Build a sanitized table before first Set call:
    // - strictly increasing exposure time
    // - respects current AE exposure range (ExpTimeRange)
    // - clamps gains into common ranges + current AE ranges (when available)
    ISP_AE_ROUTE_EX_S clean;
    memset(&clean, 0, sizeof(clean));
    HI_U32 prevT = 0, prevA = 0, prevD = 0, prevG = 0;
    ISP_IRIS_F_NO_E prevFno = ISP_IRIS_F_NO_1_0;
    HI_U32 prevFnoLin = 0x400;
    HI_BOOL havePrev = HI_FALSE;
    int outN = 0;
    for (int i = 0; i < total && i < ISP_AE_ROUTE_EX_MAX_NODES; i++) {
        HI_U32 t = route.astRouteExNode[i].u32IntTime;
        HI_U32 a = route.astRouteExNode[i].u32Again;
        HI_U32 d = route.astRouteExNode[i].u32Dgain;
        HI_U32 g = route.astRouteExNode[i].u32IspDgain;
        ISP_IRIS_F_NO_E fno = route.astRouteExNode[i].enIrisFNO;
        HI_U32 fnolin = route.astRouteExNode[i].u32IrisFNOLin;

        if (t == 0) continue;
        if (exp_min && t < exp_min) t = exp_min;
        if (exp_max && t > exp_max) continue; // don't feed firmware out-of-range times

        // Clamp to common ranges from hi_comm_isp.h (safe for GK too)
        if (a < 0x400) a = 0x400;
        if (d < 0x400) d = 0x400;
        if (g < 0x400) g = 0x400;
        if (a > 0x3FFFFF) a = 0x3FFFFF;
        if (d > 0x3FFFFF) d = 0x3FFFFF;
        if (g > 0x40000) g = 0x40000;

        // Further clamp to current AE ranges when available.
        if (again_max >= 0x400 && a > again_max) a = again_max;
        if (dgain_max >= 0x400 && d > dgain_max) d = dgain_max;
        if (ispdgain_max >= 0x400 && g > ispdgain_max) g = ispdgain_max;

        // hi_comm_isp.h: u32IrisFNOLin Range:[0x1, 0x400]
        if (fnolin < 0x1 || fnolin > 0x400) fnolin = 0x400;
        if ((int)fno < 0 || (int)fno >= (int)ISP_IRIS_F_NO_BUTT) fno = ISP_IRIS_F_NO_1_0;

        // Some GK SDK builds require "axis-aligned" route changes:
        // do not change IntTime and gain(s) simultaneously between adjacent nodes.
        // If we detect a diagonal step (t increases AND any gain changes), split into:
        //  - (t, prevGains)
        //  - (t, newGains)
        if (!havePrev) {
            clean.astRouteExNode[outN].u32IntTime = t;
            clean.astRouteExNode[outN].u32Again = a;
            clean.astRouteExNode[outN].u32Dgain = d;
            clean.astRouteExNode[outN].u32IspDgain = g;
            clean.astRouteExNode[outN].enIrisFNO = fno;
            clean.astRouteExNode[outN].u32IrisFNOLin = fnolin;
            outN++;
            havePrev = HI_TRUE;
            prevT = t; prevA = a; prevD = d; prevG = g; prevFno = fno; prevFnoLin = fnolin;
            if (outN >= ISP_AE_ROUTE_EX_MAX_NODES) break;
            continue;
        }

        if (t < prevT) continue;

        const HI_BOOL gains_changed = (a != prevA || d != prevD || g != prevG) ? HI_TRUE : HI_FALSE;
        if (t == prevT && !gains_changed) continue; // exact duplicate

        if (t > prevT && gains_changed && outN < ISP_AE_ROUTE_EX_MAX_NODES) {
            clean.astRouteExNode[outN].u32IntTime = t;
            clean.astRouteExNode[outN].u32Again = prevA;
            clean.astRouteExNode[outN].u32Dgain = prevD;
            clean.astRouteExNode[outN].u32IspDgain = prevG;
            clean.astRouteExNode[outN].enIrisFNO = prevFno;
            clean.astRouteExNode[outN].u32IrisFNOLin = prevFnoLin;
            outN++;
            prevT = t; // advance time, keep gains
            if (outN >= ISP_AE_ROUTE_EX_MAX_NODES) break;
        } else if (t > prevT && !gains_changed && outN < ISP_AE_ROUTE_EX_MAX_NODES) {
            clean.astRouteExNode[outN].u32IntTime = t;
            clean.astRouteExNode[outN].u32Again = a;
            clean.astRouteExNode[outN].u32Dgain = d;
            clean.astRouteExNode[outN].u32IspDgain = g;
            clean.astRouteExNode[outN].enIrisFNO = fno;
            clean.astRouteExNode[outN].u32IrisFNOLin = fnolin;
            outN++;
            prevT = t; prevA = a; prevD = d; prevG = g; prevFno = fno; prevFnoLin = fnolin;
            if (outN >= ISP_AE_ROUTE_EX_MAX_NODES) break;
            continue;
        }

        // Now push gain-change steps at current time (t == prevT).
        // New GK SDK prints "RouteExCheck node[i]&node[i+1] illegal" when too many params
        // change at once, so change gains one-by-one: Again -> Dgain -> IspDgain.
        if ((t == prevT) && gains_changed) {
            HI_U32 curA = prevA, curD = prevD, curG = prevG;
            if (a != curA && outN < ISP_AE_ROUTE_EX_MAX_NODES) {
                clean.astRouteExNode[outN].u32IntTime = prevT;
                clean.astRouteExNode[outN].u32Again = a;
                clean.astRouteExNode[outN].u32Dgain = curD;
                clean.astRouteExNode[outN].u32IspDgain = curG;
                clean.astRouteExNode[outN].enIrisFNO = fno;
                clean.astRouteExNode[outN].u32IrisFNOLin = fnolin;
                outN++;
                curA = a;
            }
            if (d != curD && outN < ISP_AE_ROUTE_EX_MAX_NODES) {
                clean.astRouteExNode[outN].u32IntTime = prevT;
                clean.astRouteExNode[outN].u32Again = curA;
                clean.astRouteExNode[outN].u32Dgain = d;
                clean.astRouteExNode[outN].u32IspDgain = curG;
                clean.astRouteExNode[outN].enIrisFNO = fno;
                clean.astRouteExNode[outN].u32IrisFNOLin = fnolin;
                outN++;
                curD = d;
            }
            if (g != curG && outN < ISP_AE_ROUTE_EX_MAX_NODES) {
                clean.astRouteExNode[outN].u32IntTime = prevT;
                clean.astRouteExNode[outN].u32Again = curA;
                clean.astRouteExNode[outN].u32Dgain = curD;
                clean.astRouteExNode[outN].u32IspDgain = g;
                clean.astRouteExNode[outN].enIrisFNO = fno;
                clean.astRouteExNode[outN].u32IrisFNOLin = fnolin;
                outN++;
                curG = g;
            }
            prevA = curA; prevD = curD; prevG = curG; prevFno = fno; prevFnoLin = fnolin;
            if (outN >= ISP_AE_ROUTE_EX_MAX_NODES) break;
        }
    }
    clean.u32TotalNum = (HI_U32)outN;

    if (outN < 2) {
        HAL_WARNING("v4_iq", "AE route-ex: not enough valid nodes after clamp (%s outN=%d exp=[%u..%u]), skipping\n",
            sec, outN, (unsigned)exp_min, (unsigned)exp_max);
        return EXIT_SUCCESS;
    }

    HAL_INFO("v4_iq", "AE route-ex: current ranges exp=[%u..%u] againMax=%u dgainMax=%u ispdMax=%u sysGainMax=%u\n",
        (unsigned)exp_min, (unsigned)exp_max,
        (unsigned)again_max, (unsigned)dgain_max, (unsigned)ispdgain_max, (unsigned)sysgain_max);

    // Some firmwares validate route nodes against current auto ranges.
    // Ensure ExposureAttr ranges cover the route's max values (best-effort).
    if (v4_isp.fnGetExposureAttr && v4_isp.fnSetExposureAttr) {
        ISP_EXPOSURE_ATTR_S exp;
        memset(&exp, 0, sizeof(exp));
        if (v4_isp.fnGetExposureAttr(pipe, &exp) == 0) {
            HI_U32 need_again = 0, need_dgain = 0, need_ispdgain = 0;
            HI_U32 need_t = 0;
            for (int i = 0; i < outN; i++) {
                HI_U32 t = clean.astRouteExNode[i].u32IntTime;
                HI_U32 a = clean.astRouteExNode[i].u32Again;
                HI_U32 d = clean.astRouteExNode[i].u32Dgain;
                HI_U32 g = clean.astRouteExNode[i].u32IspDgain;
                if (t > need_t) need_t = t;
                if (a > need_again) need_again = a;
                if (d > need_dgain) need_dgain = d;
                if (g > need_ispdgain) need_ispdgain = g;
            }

            HI_BOOL changed = HI_FALSE;
            if (need_t > 0 && exp.stAuto.stExpTimeRange.u32Max < need_t) {
                exp.stAuto.stExpTimeRange.u32Max = need_t;
                changed = HI_TRUE;
            }
            if (need_again > 0 && exp.stAuto.stAGainRange.u32Max < need_again) {
                exp.stAuto.stAGainRange.u32Max = need_again;
                changed = HI_TRUE;
            }
            if (need_dgain > 0 && exp.stAuto.stDGainRange.u32Max < need_dgain) {
                exp.stAuto.stDGainRange.u32Max = need_dgain;
                changed = HI_TRUE;
            }
            if (need_ispdgain > 0 && exp.stAuto.stISPDGainRange.u32Max < need_ispdgain) {
                exp.stAuto.stISPDGainRange.u32Max = need_ispdgain;
                changed = HI_TRUE;
            }

            if (changed) {
                int sr = v4_isp.fnSetExposureAttr(pipe, &exp);
                if (sr) {
                    HAL_WARNING("v4_iq", "AE route-ex: SetExposureAttr pre-adjust failed with %#x (need t=%u again=%u dgain=%u ispd=%u)\n",
                        sr, (unsigned)need_t, (unsigned)need_again, (unsigned)need_dgain, (unsigned)need_ispdgain);
                } else {
                    HAL_INFO("v4_iq", "AE route-ex: adjusted ranges (tmax=%u againMax=%u dgainMax=%u ispdMax=%u)\n",
                        (unsigned)need_t, (unsigned)need_again, (unsigned)need_dgain, (unsigned)need_ispdgain);
                }
            }
        }
    }

    ret = v4_isp.fnSetAERouteAttrEx(pipe, &clean);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetAERouteAttrEx failed with %#x (sanitizing & retry)\n", ret);
        {
            // Dump a compact summary to understand what the SDK rejects.
            // Keep it small: first/last nodes and monotonicity hint.
            unsigned int t0 = (total > 0) ? (unsigned)route.astRouteExNode[0].u32IntTime : 0;
            unsigned int a0 = (total > 0) ? (unsigned)route.astRouteExNode[0].u32Again : 0;
            unsigned int d0 = (total > 0) ? (unsigned)route.astRouteExNode[0].u32Dgain : 0;
            unsigned int g0 = (total > 0) ? (unsigned)route.astRouteExNode[0].u32IspDgain : 0;
            unsigned int tl = (total > 0) ? (unsigned)route.astRouteExNode[total - 1].u32IntTime : 0;
            unsigned int al = (total > 0) ? (unsigned)route.astRouteExNode[total - 1].u32Again : 0;
            unsigned int dl = (total > 0) ? (unsigned)route.astRouteExNode[total - 1].u32Dgain : 0;
            unsigned int gl = (total > 0) ? (unsigned)route.astRouteExNode[total - 1].u32IspDgain : 0;

            int nonmono = -1;
            HI_U32 prev = 0;
            for (int i = 0; i < total && i < ISP_AE_ROUTE_EX_MAX_NODES; i++) {
                HI_U32 t = route.astRouteExNode[i].u32IntTime;
                if (t == 0) continue;
                if (prev && t <= prev) { nonmono = i; break; }
                prev = t;
            }

            HAL_WARNING("v4_iq",
                "AE route-ex reject dump: total=%d nInts=%d nAgain=%d nDgain=%d nIspDgain=%d "
                "first={t=%u again=%u dgain=%u ispd=%u} last={t=%u again=%u dgain=%u ispd=%u} nonmono_idx=%d\n",
                total, nInts, nAgain, nDgain, nIspDgain, t0, a0, d0, g0, tl, al, dl, gl, nonmono);
            for (int i = 0; i < total && i < 4; i++) {
                HAL_WARNING("v4_iq",
                    "AE route-ex node[%d]={t=%u again=%u dgain=%u ispd=%u}\n",
                    i,
                    (unsigned)route.astRouteExNode[i].u32IntTime,
                    (unsigned)route.astRouteExNode[i].u32Again,
                    (unsigned)route.astRouteExNode[i].u32Dgain,
                    (unsigned)route.astRouteExNode[i].u32IspDgain);
            }
            if (total > 4) {
                int i = total - 1;
                HAL_WARNING("v4_iq",
                    "AE route-ex node[last=%d]={t=%u again=%u dgain=%u ispd=%u}\n",
                    i,
                    (unsigned)route.astRouteExNode[i].u32IntTime,
                    (unsigned)route.astRouteExNode[i].u32Again,
                    (unsigned)route.astRouteExNode[i].u32Dgain,
                    (unsigned)route.astRouteExNode[i].u32IspDgain);
            }
        }

        // Some SDKs require strictly increasing exposure time and valid gain ranges.
        // Build a sanitized table and retry once.
        // We already built a sanitized table as `clean` above; just retry once more
        // to keep previous behavior/logs consistent.
        if ((int)clean.u32TotalNum >= 2) {
            HAL_INFO("v4_iq", "AE route-ex: retry with sanitized table (%u nodes, from %d)\n",
                (unsigned)clean.u32TotalNum, total);
            int ret2 = v4_isp.fnSetAERouteAttrEx(pipe, &clean);
            if (!ret2) {
                route = clean;
                ret = 0;
            } else {
                HAL_WARNING("v4_iq", "HI_MPI_ISP_SetAERouteAttrEx retry failed with %#x\n", ret2);
                {
                    int outN2 = (int)clean.u32TotalNum;
                    unsigned int t0 = (outN2 > 0) ? (unsigned)clean.astRouteExNode[0].u32IntTime : 0;
                    unsigned int a0 = (outN2 > 0) ? (unsigned)clean.astRouteExNode[0].u32Again : 0;
                    unsigned int d0 = (outN2 > 0) ? (unsigned)clean.astRouteExNode[0].u32Dgain : 0;
                    unsigned int g0 = (outN2 > 0) ? (unsigned)clean.astRouteExNode[0].u32IspDgain : 0;
                    unsigned int tl = (outN2 > 0) ? (unsigned)clean.astRouteExNode[outN2 - 1].u32IntTime : 0;
                    unsigned int al = (outN2 > 0) ? (unsigned)clean.astRouteExNode[outN2 - 1].u32Again : 0;
                    unsigned int dl = (outN2 > 0) ? (unsigned)clean.astRouteExNode[outN2 - 1].u32Dgain : 0;
                    unsigned int gl = (outN2 > 0) ? (unsigned)clean.astRouteExNode[outN2 - 1].u32IspDgain : 0;
                    HAL_WARNING("v4_iq",
                        "AE route-ex reject dump (sanitized): outN=%d exp=[%u..%u] first={t=%u again=%u dgain=%u ispd=%u} last={t=%u again=%u dgain=%u ispd=%u}\n",
                        outN2, (unsigned)exp_min, (unsigned)exp_max, t0, a0, d0, g0, tl, al, dl, gl);
                }
                ret = ret2;
            }
        } else {
            HAL_WARNING("v4_iq", "AE route-ex: not enough valid nodes after sanitize, skipping\n");
            // continue to fallback below
        }

        // If it still fails, determine whether this SDK supports route tables at all.
        // Some GK builds export the symbols but always return ILLEGAL_PARAM.
        if (ret && v4_isp.fnGetAERouteAttrEx && v4_isp.fnSetAERouteAttrEx) {
            ISP_AE_ROUTE_EX_S cur;
            memset(&cur, 0, sizeof(cur));
            int gr = v4_isp.fnGetAERouteAttrEx(pipe, &cur);
            if (!gr) {
                int sr = v4_isp.fnSetAERouteAttrEx(pipe, &cur);
                if (sr) {
                    pthread_mutex_lock(&_v4_iq_dyn.lock);
                    if (!_v4_iq_dyn.aerouteex_disabled) {
                        _v4_iq_dyn.aerouteex_disabled = HI_TRUE;
                        HAL_WARNING("v4_iq",
                            "AE route-ex: SDK rejects even noop SetAERouteAttrEx(GetAERouteAttrEx) (%#x); disabling route tables\n",
                            sr);
                    }
                    pthread_mutex_unlock(&_v4_iq_dyn.lock);
                }
            }
        }
    } else {
        // Ensure AE is configured to actually use the RouteEx table.
        // Some SDKs require bAERouteExValid=1 in ExposureAttr.
        if (v4_isp.fnGetExposureAttr && v4_isp.fnSetExposureAttr) {
            ISP_EXPOSURE_ATTR_S exp;
            memset(&exp, 0, sizeof(exp));
            int gr = v4_isp.fnGetExposureAttr(pipe, &exp);
            if (!gr && !exp.bAERouteExValid) {
                exp.bAERouteExValid = HI_TRUE;
                int sr = v4_isp.fnSetExposureAttr(pipe, &exp);
                if (!sr)
                    HAL_INFO("v4_iq", "AE route-ex: enabled AERouteExValid=1\n");
                else
                    HAL_WARNING("v4_iq", "AE route-ex: failed to set AERouteExValid=1 (%#x)\n", sr);
            }
        }

        ISP_AE_ROUTE_EX_S rb;
        memset(&rb, 0, sizeof(rb));
        int gr = v4_isp.fnGetAERouteAttrEx(pipe, &rb);
        if (!gr && rb.u32TotalNum > 0) {
            HAL_INFO("v4_iq", "AE route-ex: applied (%s %u nodes) first={t=%u again=%u dgain=%u ispd=%u}\n",
                sec,
                (unsigned)rb.u32TotalNum,
                (unsigned)rb.astRouteExNode[0].u32IntTime,
                (unsigned)rb.astRouteExNode[0].u32Again,
                (unsigned)rb.astRouteExNode[0].u32Dgain,
                (unsigned)rb.astRouteExNode[0].u32IspDgain);
        } else {
            HAL_INFO("v4_iq", "AE route-ex: applied (%s %d nodes)\n", sec, total);
        }
    }

    // If RouteEx is not supported/accepted by this SDK, try to approximate using non-Ex AERouteAttr.
    if (ret && v4_isp.fnSetAERouteAttr) {
        ISP_AE_ROUTE_S r;
        memset(&r, 0, sizeof(r));
        int outN = 0;
        HI_U32 prevT = 0;
        HI_U32 prevSG = 0;
        ISP_IRIS_F_NO_E prevFno = ISP_IRIS_F_NO_1_0;
        HI_U32 prevFnoLin = 0x400;
        HI_BOOL havePrev = HI_FALSE;
        for (int i = 0; i < total && i < ISP_AE_ROUTE_EX_MAX_NODES; i++) {
            HI_U32 t = route.astRouteExNode[i].u32IntTime;
            if (t == 0) continue;
            if (exp_min && t < exp_min) t = exp_min;
            if (exp_max && t > exp_max) continue;
            HI_U32 sg = v4_iq_sysgain_from_routeex(
                route.astRouteExNode[i].u32Again,
                route.astRouteExNode[i].u32Dgain,
                route.astRouteExNode[i].u32IspDgain);
            if (sg < 0x400) sg = 0x400;
            if (sysgain_max >= 0x400 && sg > sysgain_max) sg = sysgain_max;

            ISP_IRIS_F_NO_E fno = route.astRouteExNode[i].enIrisFNO;
            HI_U32 fnolin = route.astRouteExNode[i].u32IrisFNOLin;
            if (fnolin < 0x1 || fnolin > 0x400) fnolin = 0x400;
            if ((int)fno < 0 || (int)fno >= (int)ISP_IRIS_F_NO_BUTT) fno = ISP_IRIS_F_NO_1_0;

            if (!havePrev) {
                r.astRouteNode[outN].u32IntTime = t;
                r.astRouteNode[outN].u32SysGain = sg;
                r.astRouteNode[outN].enIrisFNO = fno;
                r.astRouteNode[outN].u32IrisFNOLin = fnolin;
                outN++;
                havePrev = HI_TRUE;
                prevT = t; prevSG = sg; prevFno = fno; prevFnoLin = fnolin;
                if (outN >= ISP_AE_ROUTE_MAX_NODES) break;
                continue;
            }

            if (t < prevT) continue;
            const HI_BOOL sg_changed = (sg != prevSG) ? HI_TRUE : HI_FALSE;
            if (t == prevT && !sg_changed) continue;

            // Axis-aligned rule: don't change IntTime and SysGain simultaneously.
            if (t > prevT && sg_changed && outN < ISP_AE_ROUTE_MAX_NODES) {
                r.astRouteNode[outN].u32IntTime = t;
                r.astRouteNode[outN].u32SysGain = prevSG;
                r.astRouteNode[outN].enIrisFNO = prevFno;
                r.astRouteNode[outN].u32IrisFNOLin = prevFnoLin;
                outN++;
                prevT = t;
                if (outN >= ISP_AE_ROUTE_MAX_NODES) break;
            } else if (t > prevT && !sg_changed && outN < ISP_AE_ROUTE_MAX_NODES) {
                r.astRouteNode[outN].u32IntTime = t;
                r.astRouteNode[outN].u32SysGain = sg;
                r.astRouteNode[outN].enIrisFNO = fno;
                r.astRouteNode[outN].u32IrisFNOLin = fnolin;
                outN++;
                prevT = t; prevSG = sg; prevFno = fno; prevFnoLin = fnolin;
                if (outN >= ISP_AE_ROUTE_MAX_NODES) break;
                continue;
            }

            if ((t == prevT) && sg_changed && outN < ISP_AE_ROUTE_MAX_NODES) {
                r.astRouteNode[outN].u32IntTime = prevT;
                r.astRouteNode[outN].u32SysGain = sg;
                r.astRouteNode[outN].enIrisFNO = fno;
                r.astRouteNode[outN].u32IrisFNOLin = fnolin;
                outN++;
                prevSG = sg; prevFno = fno; prevFnoLin = fnolin;
                if (outN >= ISP_AE_ROUTE_MAX_NODES) break;
            }
        }
        r.u32TotalNum = (HI_U32)outN;

        if (outN >= 2) {
            int rret = v4_isp.fnSetAERouteAttr(pipe, &r);
            if (!rret) {
                HAL_INFO("v4_iq", "AE route-ex: RouteEx rejected (%#x), applied fallback AERouteAttr (%d nodes)\n", ret, outN);
                // Disable RouteExValid so firmware uses the non-Ex route.
                if (v4_isp.fnGetExposureAttr && v4_isp.fnSetExposureAttr) {
                    ISP_EXPOSURE_ATTR_S exp;
                    memset(&exp, 0, sizeof(exp));
                    if (!v4_isp.fnGetExposureAttr(pipe, &exp) && exp.bAERouteExValid) {
                        exp.bAERouteExValid = HI_FALSE;
                        v4_isp.fnSetExposureAttr(pipe, &exp);
                    }
                }
                return EXIT_SUCCESS;
            } else {
                HAL_WARNING("v4_iq", "AE route-ex: fallback HI_MPI_ISP_SetAERouteAttr failed with %#x\n", rret);
                // Same check for non-Ex route: noop Set(Get()).
                if (v4_isp.fnGetAERouteAttr) {
                    ISP_AE_ROUTE_S cur;
                    memset(&cur, 0, sizeof(cur));
                    int gr = v4_isp.fnGetAERouteAttr(pipe, &cur);
                    if (!gr) {
                        int sr = v4_isp.fnSetAERouteAttr(pipe, &cur);
                        if (sr) {
                            HAL_WARNING("v4_iq",
                                "AE route: SDK rejects even noop SetAERouteAttr(GetAERouteAttr) (%#x); route tables unsupported\n",
                                sr);
                        }
                    }
                }
            }
        }
    }

    // Don't fail full IQ apply because RouteEx isn't accepted on this SDK.
    if (ret) {
        pthread_mutex_lock(&_v4_iq_dyn.lock);
        if (!_v4_iq_dyn.aerouteex_disabled) {
            _v4_iq_dyn.aerouteex_disabled = HI_TRUE;
            HAL_WARNING("v4_iq", "AE route-ex: disabling due to SDK rejection (%#x)\n", ret);
        }
        pthread_mutex_unlock(&_v4_iq_dyn.lock);

        // If firmware can't accept route tables, ensure it doesn't try to use RouteEx.
        if (v4_isp.fnGetExposureAttr && v4_isp.fnSetExposureAttr) {
            ISP_EXPOSURE_ATTR_S exp;
            memset(&exp, 0, sizeof(exp));
            if (!v4_isp.fnGetExposureAttr(pipe, &exp) && exp.bAERouteExValid) {
                exp.bAERouteExValid = HI_FALSE;
                if (!v4_isp.fnSetExposureAttr(pipe, &exp))
                    HAL_INFO("v4_iq", "AE route-ex: SDK rejected route; forced AERouteExValid=0\n");
            }
        }
        HAL_WARNING("v4_iq", "AE route-ex: skipping (SDK rejected), continuing\n");
        return EXIT_SUCCESS;
    }
    return ret;
}

static int v4_iq_apply_static_ccm(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetCCMAttr || !v4_isp.fnSetCCMAttr) {
        HAL_INFO("v4_iq", "CCM: API not available, skipping\n");
        return EXIT_SUCCESS;
    }
    pthread_mutex_lock(&_v4_iq_dyn.lock);
    HI_BOOL ccm_disabled = _v4_iq_dyn.ccm_disabled;
    pthread_mutex_unlock(&_v4_iq_dyn.lock);
    if (ccm_disabled) {
        HAL_INFO("v4_iq", "CCM: disabled (previous SDK reject), skipping\n");
        return EXIT_SUCCESS;
    }

    // We don't know which exact CCM struct layout this GK/SDK expects.
    // Use a sufficiently large, aligned blob, seed it with GetCCMAttr(),
    // then try applying using both known layouts (tabbed and H/M/L).
    union {
        HI_U64 _align;
        unsigned char b[1024];
    } u;
    memset(&u, 0, sizeof(u));
    int ret = v4_isp.fnGetCCMAttr(pipe, u.b);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetCCMAttr failed with %#x\n", ret);
        pthread_mutex_lock(&_v4_iq_dyn.lock);
        _v4_iq_dyn.ccm_disabled = HI_TRUE;
        pthread_mutex_unlock(&_v4_iq_dyn.lock);
        HAL_WARNING("v4_iq", "CCM: skipping (SDK rejected), continuing\n");
        return EXIT_SUCCESS;
    }

    // Keep a pristine copy for retries with other layout.
    unsigned char orig[1024];
    memcpy(orig, u.b, sizeof(orig));

    int val;
    int requested_op = -1; // -1 = keep current
    if (parse_int(ini, "static_ccm", "CCMOpType", 0, 1, &val) == CONFIG_OK)
        requested_op = val;
    int isoActEn = -1, tempActEn = -1;
    if (parse_int(ini, "static_ccm", "ISOActEn", 0, 1, &val) == CONFIG_OK)
        isoActEn = val;
    if (parse_int(ini, "static_ccm", "TempActEn", 0, 1, &val) == CONFIG_OK)
        tempActEn = val;

    // Manual CCM (optional)
    char buf[1024];
    HI_BOOL have_manual = HI_FALSE;
    HI_U16 manual_ccm[CCM_MATRIX_SIZE] = {0};
    if (parse_param_value(ini, "static_ccm", "ManualCCMTable", buf) == CONFIG_OK) {
        v4_iq_parse_csv_u16(buf, manual_ccm, CCM_MATRIX_SIZE);
        have_manual = HI_TRUE;
    }

    // Auto CCM tables as provided by IQ (tabbed form).
    int total = 0;
    HI_BOOL have_auto = HI_FALSE;
    HI_U32 temps_u32[CCM_MATRIX_NUM] = {0};
    HI_U16 auto_ccm[CCM_MATRIX_NUM][CCM_MATRIX_SIZE];
    memset(auto_ccm, 0, sizeof(auto_ccm));
    int ntemps = 0;
    if (parse_int(ini, "static_ccm", "TotalNum", 0, CCM_MATRIX_NUM, &total) == CONFIG_OK && total > 0) {
        if (parse_param_value(ini, "static_ccm", "AutoColorTemp", buf) == CONFIG_OK) {
            ntemps = v4_iq_parse_csv_u32(buf, temps_u32, CCM_MATRIX_NUM);
        }
        for (int i = 0; i < total && i < CCM_MATRIX_NUM; i++) {
            char key[32];
            snprintf(key, sizeof(key), "AutoCCMTable_%d", i);
            if (parse_param_value(ini, "static_ccm", key, buf) == CONFIG_OK) {
                v4_iq_parse_csv_u16(buf, auto_ccm[i], CCM_MATRIX_SIZE);
                have_auto = HI_TRUE;
            }
        }
        if (ntemps > 0) have_auto = HI_TRUE;
    }

    // Apply opType logic.
    // IQ files commonly use: 0=Auto, 1=Manual. Some SDKs reverse it.
    // - In Manual mode, keep only stManual matrix and disable auto activation fields/tables.
    // - In Auto mode, keep auto tables; stManual can still be present as fallback.
    HI_BOOL want_manual = HI_FALSE;
    if (requested_op == 1) want_manual = HI_TRUE;
    else if (requested_op == 0) want_manual = HI_FALSE;
    else {
        // Default to current mode by reading tabbed layout header (safe if wrong too).
        ISP_COLORMATRIX_ATTR_S *cur = (ISP_COLORMATRIX_ATTR_S *)(void *)u.b;
        want_manual = (cur->enOpType == OP_TYPE_MANUAL) ? HI_TRUE : HI_FALSE;
    }

    // If user asked for manual but no manual table provided, don't force manual.
    if (want_manual && !have_manual) want_manual = HI_FALSE;
    // If user asked for auto but no auto tables provided, don't force auto.
    if (!want_manual && !have_auto) want_manual = HI_TRUE;

    // Try TABBED then HML (most GK builds seem to accept HML, but not sure).
    // Attempt 1: tabbed layout
    {
        ISP_COLORMATRIX_ATTR_S *a = (ISP_COLORMATRIX_ATTR_S *)(void *)u.b;
        if (want_manual) {
            a->enOpType = OP_TYPE_MANUAL;
            a->stManual.bSatEn = HI_FALSE;
            if (have_manual) memcpy(a->stManual.au16CCM, manual_ccm, sizeof(manual_ccm));
            // Keep stAuto as-is (as returned by GetCCMAttr). Some GK SDK builds
            // validate u16CCMTabNum even in manual mode and warn if we zero it.
        } else {
            a->enOpType = OP_TYPE_AUTO;
            if (isoActEn >= 0) a->stAuto.bISOActEn = (HI_BOOL)isoActEn;
            if (tempActEn >= 0) a->stAuto.bTempActEn = (HI_BOOL)tempActEn;
            int n = total;
            if (n < 3) n = 3;
            if (n > CCM_MATRIX_NUM) n = CCM_MATRIX_NUM;
            a->stAuto.u16CCMTabNum = (HI_U16)n;
            for (int i = 0; i < n && i < CCM_MATRIX_NUM; i++) {
                HI_U16 ct = (HI_U16)((i < ntemps) ? temps_u32[i] : 6500);
                a->stAuto.astCCMTab[i].u16ColorTemp = ct;
                memcpy(a->stAuto.astCCMTab[i].au16CCM, auto_ccm[i], CCM_MATRIX_SIZE * sizeof(HI_U16));
            }
            if (have_manual) memcpy(a->stManual.au16CCM, manual_ccm, sizeof(manual_ccm));
        }
        ret = v4_isp.fnSetCCMAttr(pipe, u.b);
    }
    const char *used = "TAB";
    if (ret) {
        memcpy(u.b, orig, sizeof(orig));
        // Attempt 2: H/M/L layout
        {
            ISP_COLORMATRIX_ATTR_HML_S *a = (ISP_COLORMATRIX_ATTR_HML_S *)(void *)u.b;
            if (want_manual) {
                a->enOpType = OP_TYPE_MANUAL;
                a->stManual.bSatEn = HI_FALSE;
                if (have_manual) memcpy(a->stManual.au16CCM, manual_ccm, sizeof(manual_ccm));
                a->stAuto.bISOActEn = HI_FALSE;
                a->stAuto.bTempActEn = HI_FALSE;
            } else {
                a->enOpType = OP_TYPE_AUTO;
                if (isoActEn >= 0) a->stAuto.bISOActEn = (HI_BOOL)isoActEn;
                if (tempActEn >= 0) a->stAuto.bTempActEn = (HI_BOOL)tempActEn;

                // Map up to 3 provided tables into Low/Mid/High.
                struct Node { HI_U16 t; const HI_U16 *m; } nodes[3];
                int n = 0;
                for (int i = 0; i < total && i < 3; i++) {
                    HI_U16 t = (HI_U16)((i < ntemps) ? temps_u32[i] : (i == 0 ? 4500 : (i == 1 ? 6500 : 8000)));
                    nodes[n].t = t;
                    nodes[n].m = auto_ccm[i];
                    n++;
                }
                while (n < 3) {
                    HI_U16 t = (n == 0 ? 4500 : (n == 1 ? 6500 : 8000));
                    nodes[n].t = t;
                    nodes[n].m = auto_ccm[n - 1];
                    n++;
                }
                // sort ascending by t
                for (int i = 0; i < 3; i++) for (int j = i + 1; j < 3; j++)
                    if (nodes[j].t < nodes[i].t) { struct Node tmp = nodes[i]; nodes[i] = nodes[j]; nodes[j] = tmp; }

                HI_U16 lowT = nodes[0].t;
                HI_U16 midT = nodes[1].t;
                HI_U16 highT = nodes[2].t;
                if (lowT < 2000) lowT = 2000;
                if (midT + 400 > highT) midT = (highT > 400) ? (highT - 400) : highT;
                if (lowT + 400 > midT) lowT = (midT > 400) ? (midT - 400) : lowT;

                a->stAuto.u16LowColorTemp = lowT;
                a->stAuto.u16MidColorTemp = midT;
                a->stAuto.u16HighColorTemp = highT;
                memcpy(a->stAuto.au16LowCCM, nodes[0].m, CCM_MATRIX_SIZE * sizeof(HI_U16));
                memcpy(a->stAuto.au16MidCCM, nodes[1].m, CCM_MATRIX_SIZE * sizeof(HI_U16));
                memcpy(a->stAuto.au16HighCCM, nodes[2].m, CCM_MATRIX_SIZE * sizeof(HI_U16));

                if (have_manual) memcpy(a->stManual.au16CCM, manual_ccm, sizeof(manual_ccm));
            }
            ret = v4_isp.fnSetCCMAttr(pipe, u.b);
        }
        used = "HML";
    }
    if (ret) {
        // As a sanity check: can we Set() exactly what Get() returned?
        memcpy(u.b, orig, sizeof(orig));
        int roundtrip = v4_isp.fnSetCCMAttr(pipe, u.b);
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetCCMAttr failed with %#x (other layout failed too). roundtrip_set=%#x\n", ret, roundtrip);

        pthread_mutex_lock(&_v4_iq_dyn.lock);
        _v4_iq_dyn.ccm_disabled = HI_TRUE;
        pthread_mutex_unlock(&_v4_iq_dyn.lock);
        HAL_WARNING("v4_iq", "CCM: skipping (SDK rejected), continuing\n");
        return EXIT_SUCCESS;
    }

    // Read-back (best-effort)
    union { HI_U64 _a; unsigned char b[1024]; } rb;
    memset(&rb, 0, sizeof(rb));
    int gr = v4_isp.fnGetCCMAttr(pipe, rb.b);
    if (!gr) {
        ISP_COLORMATRIX_ATTR_S *rbt = (ISP_COLORMATRIX_ATTR_S *)(void *)rb.b;
        HAL_INFO("v4_iq", "CCM: applied (%s opType=%d manualCCM[0..2]=%u,%u,%u)\n",
            used, (int)rbt->enOpType,
            (unsigned)rbt->stManual.au16CCM[0],
            (unsigned)rbt->stManual.au16CCM[1],
            (unsigned)rbt->stManual.au16CCM[2]);
    } else {
        HAL_INFO("v4_iq", "CCM: applied (%s)\n", used);
    }
    // Don't fail full IQ apply because CCM isn't accepted on this SDK/build.
    return EXIT_SUCCESS;
}

static int v4_iq_apply_static_saturation(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetSaturationAttr || !v4_isp.fnSetSaturationAttr) {
        HAL_INFO("v4_iq", "Saturation: API not available, skipping\n");
        return EXIT_SUCCESS;
    }

    ISP_SATURATION_ATTR_S sat;
    memset(&sat, 0, sizeof(sat));
    int ret = v4_isp.fnGetSaturationAttr(pipe, &sat);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetSaturationAttr failed with %#x\n", ret);
        return ret;
    }

    sat.enOpType = OP_TYPE_AUTO;
    char buf[512];
    if (parse_param_value(ini, "static_saturation", "AutoSat", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, sat.stAuto.au8Sat, ISP_AUTO_ISO_STRENGTH_NUM);

    ret = v4_isp.fnSetSaturationAttr(pipe, &sat);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetSaturationAttr failed with %#x\n", ret);
    } else {
        ISP_SATURATION_ATTR_S rb;
        memset(&rb, 0, sizeof(rb));
        int gr = v4_isp.fnGetSaturationAttr(pipe, &rb);
        if (!gr) {
            HAL_INFO("v4_iq", "Saturation: applied (opType=%d auto[0]=%u auto[15]=%u)\n",
                (int)rb.enOpType,
                (unsigned)rb.stAuto.au8Sat[0],
                (unsigned)rb.stAuto.au8Sat[15]);
        } else {
            HAL_INFO("v4_iq", "Saturation: applied\n");
        }
    }
    return ret;
}

static int v4_iq_apply_static_aeweight(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetStatisticsConfig || !v4_isp.fnSetStatisticsConfig) {
        HAL_INFO("v4_iq", "AE weight: API not available, skipping\n");
        return EXIT_SUCCESS;
    }
    const char *sec = "static_aeweight";
    int sec_s = 0, sec_e = 0;
    if (night_mode_on() && section_pos(ini, "ir_static_aeweight", &sec_s, &sec_e) == CONFIG_OK) {
        sec = "ir_static_aeweight";
    } else if (section_pos(ini, "static_aeweight", &sec_s, &sec_e) != CONFIG_OK) {
        HAL_INFO("v4_iq", "AE weight: no [static_aeweight] section, skipping\n");
        return EXIT_SUCCESS;
    }

    ISP_STATISTICS_CFG_S cfg;
    memset(&cfg, 0, sizeof(cfg));
    int ret = v4_isp.fnGetStatisticsConfig(pipe, &cfg);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetStatisticsConfig failed with %#x\n", ret);
        return ret;
    }

    // Parse rows ExpWeight_0..14, each has 17 entries.
    char buf[4096];
    HI_U32 row[AE_ZONE_COLUMN];
    int rowsParsed = 0;
    for (int r = 0; r < AE_ZONE_ROW; r++) {
        char key[32];
        snprintf(key, sizeof(key), "ExpWeight_%d", r);
        if (parse_param_value(ini, sec, key, buf) != CONFIG_OK)
            continue;
        memset(row, 0, sizeof(row));
        int n = v4_iq_parse_csv_u32(buf, row, AE_ZONE_COLUMN);
        if (n <= 0) continue;
        for (int c = 0; c < n && c < AE_ZONE_COLUMN; c++) {
            HI_U32 v = row[c];
            if (v > 0xF) v = 0xF;
            cfg.stAECfg.au8Weight[r][c] = (HI_U8)v;
        }
        rowsParsed++;
    }

    if (rowsParsed == 0) {
        HAL_INFO("v4_iq", "AE weight: no ExpWeight_* rows found, skipping\n");
        return EXIT_SUCCESS;
    }

    ret = v4_isp.fnSetStatisticsConfig(pipe, &cfg);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetStatisticsConfig failed with %#x\n", ret);
    } else {
        HAL_INFO("v4_iq", "AE weight: applied (%d/%d rows)\n", rowsParsed, AE_ZONE_ROW);
    }
    return ret;
}

static int v4_iq_apply_static_ldci(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetLDCIAttr || !v4_isp.fnSetLDCIAttr) {
        HAL_INFO("v4_iq", "LDCI: API not available, skipping\n");
        return EXIT_SUCCESS;
    }
    int sec_s = 0, sec_e = 0;
    if (section_pos(ini, "static_ldci", &sec_s, &sec_e) != CONFIG_OK) {
        HAL_INFO("v4_iq", "LDCI: no [static_ldci] section, skipping\n");
        return EXIT_SUCCESS;
    }

    // Ensure LDCI module is not bypassed
    if (v4_isp.fnGetModuleControl && v4_isp.fnSetModuleControl) {
        ISP_MODULE_CTRL_U mc;
        memset(&mc, 0, sizeof(mc));
        int gr = v4_isp.fnGetModuleControl(pipe, &mc);
        if (!gr) {
            HI_U64 before = mc.u64Key;
            mc.u64Key &= ~(1ULL << 21); // bitBypassLdci
            int sr = v4_isp.fnSetModuleControl(pipe, &mc);
            if (!sr) {
                ISP_MODULE_CTRL_U mc2;
                memset(&mc2, 0, sizeof(mc2));
                if (!v4_isp.fnGetModuleControl(pipe, &mc2))
                    HAL_INFO("v4_iq", "LDCI: module control u64Key %#llx -> %#llx\n",
                        (unsigned long long)before, (unsigned long long)mc2.u64Key);
            }
        }
    }

    ISP_LDCI_ATTR_S ldci;
    memset(&ldci, 0, sizeof(ldci));
    int ret = v4_isp.fnGetLDCIAttr(pipe, &ldci);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetLDCIAttr failed with %#x\n", ret);
        return ret;
    }

    int val;
    if (parse_int(ini, "static_ldci", "Enable", 0, 1, &val) == CONFIG_OK)
        ldci.bEnable = (HI_BOOL)val;
    if (parse_int(ini, "static_ldci", "LDCIOpType", 0, 1, &val) == CONFIG_OK)
        ldci.enOpType = (ISP_OP_TYPE_E)val;
    if (parse_int(ini, "static_ldci", "LDCIGaussLPFSigma", 0, 255, &val) == CONFIG_OK)
        ldci.u8GaussLPFSigma = (HI_U8)val;

    char buf[1024];
    HI_U32 tmp[ISP_AUTO_ISO_STRENGTH_NUM];

    // Auto per-ISO HE parameters
    if (parse_param_value(ini, "static_ldci", "AutoHePosWgt", buf) == CONFIG_OK) {
        memset(tmp, 0, sizeof(tmp));
        int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
        for (int i = 0; i < n; i++) ldci.stAuto.astHeWgt[i].stHePosWgt.u8Wgt = (HI_U8)tmp[i];
    }
    if (parse_param_value(ini, "static_ldci", "AutoHePosSigma", buf) == CONFIG_OK) {
        memset(tmp, 0, sizeof(tmp));
        int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
        for (int i = 0; i < n; i++) ldci.stAuto.astHeWgt[i].stHePosWgt.u8Sigma = (HI_U8)tmp[i];
    }
    if (parse_param_value(ini, "static_ldci", "AutoHePosMean", buf) == CONFIG_OK) {
        memset(tmp, 0, sizeof(tmp));
        int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
        for (int i = 0; i < n; i++) ldci.stAuto.astHeWgt[i].stHePosWgt.u8Mean = (HI_U8)tmp[i];
    }

    if (parse_param_value(ini, "static_ldci", "AutoHeNegWgt", buf) == CONFIG_OK) {
        memset(tmp, 0, sizeof(tmp));
        int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
        for (int i = 0; i < n; i++) ldci.stAuto.astHeWgt[i].stHeNegWgt.u8Wgt = (HI_U8)tmp[i];
    }
    if (parse_param_value(ini, "static_ldci", "AutoHeNegSigma", buf) == CONFIG_OK) {
        memset(tmp, 0, sizeof(tmp));
        int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
        for (int i = 0; i < n; i++) ldci.stAuto.astHeWgt[i].stHeNegWgt.u8Sigma = (HI_U8)tmp[i];
    }
    if (parse_param_value(ini, "static_ldci", "AutoHeNegMean", buf) == CONFIG_OK) {
        memset(tmp, 0, sizeof(tmp));
        int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
        for (int i = 0; i < n; i++) ldci.stAuto.astHeWgt[i].stHeNegWgt.u8Mean = (HI_U8)tmp[i];
    }
    if (parse_param_value(ini, "static_ldci", "AutoBlcCtrl", buf) == CONFIG_OK) {
        memset(tmp, 0, sizeof(tmp));
        int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
        for (int i = 0; i < n; i++) ldci.stAuto.au16BlcCtrl[i] = (HI_U16)tmp[i];
    }

    ret = v4_isp.fnSetLDCIAttr(pipe, &ldci);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetLDCIAttr failed with %#x\n", ret);
    } else {
        ISP_LDCI_ATTR_S rb;
        memset(&rb, 0, sizeof(rb));
        int gr = v4_isp.fnGetLDCIAttr(pipe, &rb);
        if (!gr) {
            HAL_INFO("v4_iq", "LDCI: applied (en=%d opType=%d posWgt[0]=%u negWgt[0]=%u blc[0]=%u)\n",
                (int)rb.bEnable, (int)rb.enOpType,
                (unsigned)rb.stAuto.astHeWgt[0].stHePosWgt.u8Wgt,
                (unsigned)rb.stAuto.astHeWgt[0].stHeNegWgt.u8Wgt,
                (unsigned)rb.stAuto.au16BlcCtrl[0]);
        } else {
            HAL_INFO("v4_iq", "LDCI: applied\n");
        }
    }
    return ret;
}

static int v4_iq_apply_static_dehaze(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetDehazeAttr || !v4_isp.fnSetDehazeAttr) {
        HAL_INFO("v4_iq", "Dehaze: API not available, skipping\n");
        return EXIT_SUCCESS;
    }
    int sec_s = 0, sec_e = 0;
    if (section_pos(ini, "static_dehaze", &sec_s, &sec_e) != CONFIG_OK) {
        HAL_INFO("v4_iq", "Dehaze: no [static_dehaze] section, skipping\n");
        return EXIT_SUCCESS;
    }

    // Ensure Dehaze module is not bypassed
    if (v4_isp.fnGetModuleControl && v4_isp.fnSetModuleControl) {
        ISP_MODULE_CTRL_U mc;
        memset(&mc, 0, sizeof(mc));
        int gr = v4_isp.fnGetModuleControl(pipe, &mc);
        if (!gr) {
            HI_U64 before = mc.u64Key;
            mc.u64Key &= ~(1ULL << 5); // bitBypassDehaze
            int sr = v4_isp.fnSetModuleControl(pipe, &mc);
            if (!sr) {
                ISP_MODULE_CTRL_U mc2;
                memset(&mc2, 0, sizeof(mc2));
                if (!v4_isp.fnGetModuleControl(pipe, &mc2))
                    HAL_INFO("v4_iq", "Dehaze: module control u64Key %#llx -> %#llx\n",
                        (unsigned long long)before, (unsigned long long)mc2.u64Key);
            }
        }
    }

    ISP_DEHAZE_ATTR_S dh;
    memset(&dh, 0, sizeof(dh));
    int ret = v4_isp.fnGetDehazeAttr(pipe, &dh);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetDehazeAttr failed with %#x\n", ret);
        return ret;
    }

    int val;
    if (parse_int(ini, "static_dehaze", "Enable", 0, 1, &val) == CONFIG_OK)
        dh.bEnable = (HI_BOOL)val;
    if (parse_int(ini, "static_dehaze", "DehazeUserLutEnable", 0, 1, &val) == CONFIG_OK)
        dh.bUserLutEnable = (HI_BOOL)val;
    if (parse_int(ini, "static_dehaze", "DehazeOpType", 0, 1, &val) == CONFIG_OK)
        dh.enOpType = (ISP_OP_TYPE_E)val;

    // User LUT (256 nodes), often multiline.
    {
        HI_U32 tmp[256];
        memset(tmp, 0, sizeof(tmp));
        int n = v4_iq_parse_multiline_u32(ini, "static_dehaze", "DehazeLut", tmp, 256);
        if (n > 0) {
            for (int i = 0; i < n && i < 256; i++) {
                HI_U32 v = tmp[i];
                if (v > 255) v = 255;
                dh.au8DehazeLut[i] = (HI_U8)v;
            }
        }
    }

    ret = v4_isp.fnSetDehazeAttr(pipe, &dh);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetDehazeAttr failed with %#x\n", ret);
    } else {
        ISP_DEHAZE_ATTR_S rb;
        memset(&rb, 0, sizeof(rb));
        int gr = v4_isp.fnGetDehazeAttr(pipe, &rb);
        if (!gr) {
            HAL_INFO("v4_iq", "Dehaze: applied (en=%d opType=%d userLut=%d lut[0]=%u lut[255]=%u)\n",
                (int)rb.bEnable, (int)rb.enOpType, (int)rb.bUserLutEnable,
                (unsigned)rb.au8DehazeLut[0], (unsigned)rb.au8DehazeLut[255]);
        } else {
            HAL_INFO("v4_iq", "Dehaze: applied\n");
        }
    }
    return ret;
}

static int v4_iq_apply_static_drc(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetDRCAttr || !v4_isp.fnSetDRCAttr) {
        HAL_INFO("v4_iq", "DRC: API not available, skipping\n");
        return EXIT_SUCCESS;
    }
    int sec_s = 0, sec_e = 0;
    if (section_pos(ini, "static_drc", &sec_s, &sec_e) != CONFIG_OK) {
        HAL_INFO("v4_iq", "DRC: no [static_drc] section, skipping\n");
        return EXIT_SUCCESS;
    }

    ISP_DRC_ATTR_S drc;
    memset(&drc, 0, sizeof(drc));
    int ret = v4_isp.fnGetDRCAttr(pipe, &drc);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetDRCAttr failed with %#x\n", ret);
        return ret;
    }

    // Ensure DRC module is not bypassed (some firmwares default to bypass in linear mode).
    if (v4_isp.fnGetModuleControl && v4_isp.fnSetModuleControl) {
        ISP_MODULE_CTRL_U mc;
        memset(&mc, 0, sizeof(mc));
        int gr = v4_isp.fnGetModuleControl(pipe, &mc);
        if (!gr) {
            HI_U64 before = mc.u64Key;
            mc.u64Key &= ~(1ULL << 8); // bitBypassDRC
            int sr = v4_isp.fnSetModuleControl(pipe, &mc);
            if (!sr) {
                ISP_MODULE_CTRL_U mc2;
                memset(&mc2, 0, sizeof(mc2));
                if (!v4_isp.fnGetModuleControl(pipe, &mc2))
                    HAL_INFO("v4_iq", "DRC: module control u64Key %#llx -> %#llx\n",
                        (unsigned long long)before, (unsigned long long)mc2.u64Key);
                else
                    HAL_INFO("v4_iq", "DRC: module control u64Key %#llx -> (set ok)\n",
                        (unsigned long long)before);
            } else {
                HAL_WARNING("v4_iq", "DRC: SetModuleControl failed with %#x\n", sr);
            }
        } else {
            HAL_WARNING("v4_iq", "DRC: GetModuleControl failed with %#x\n", gr);
        }
    }

    int val;
    if (parse_int(ini, "static_drc", "Enable", 0, 1, &val) == CONFIG_OK)
        drc.bEnable = (HI_BOOL)val;
    if (parse_int(ini, "static_drc", "CurveSelect", 0, 2, &val) == CONFIG_OK)
        drc.enCurveSelect = (ISP_DRC_CURVE_SELECT_E)val;
    if (parse_int(ini, "static_drc", "DRCOpType", 0, 1, &val) == CONFIG_OK)
        drc.enOpType = (ISP_OP_TYPE_E)val;
    if (parse_int(ini, "static_drc", "DRCAutoStr", 0, INT_MAX, &val) == CONFIG_OK)
        drc.stAuto.u16Strength = (HI_U16)val;
    if (parse_int(ini, "static_drc", "DRCAutoStrMin", 0, INT_MAX, &val) == CONFIG_OK)
        drc.stAuto.u16StrengthMin = (HI_U16)val;
    if (parse_int(ini, "static_drc", "DRCAutoStrMax", 0, INT_MAX, &val) == CONFIG_OK)
        drc.stAuto.u16StrengthMax = (HI_U16)val;

    // User curve (optional; 200 nodes). This INI often provides it as a multiline list.
    {
        HI_U32 tmp[HI_ISP_DRC_TM_NODE_NUM];
        memset(tmp, 0, sizeof(tmp));
        int n = v4_iq_parse_multiline_u32(ini, "static_drc", "DRCToneMappingValue", tmp, HI_ISP_DRC_TM_NODE_NUM);
        if (n > 0) {
            for (int i = 0; i < n && i < HI_ISP_DRC_TM_NODE_NUM; i++)
                drc.au16ToneMappingValue[i] = (HI_U16)tmp[i];
            // If a user-defined tone mapping LUT is provided, force user curve selection.
            // Otherwise, some SDKs ignore au16ToneMappingValue when CurveSelect=0 (asymmetry).
            if (drc.enCurveSelect != DRC_CURVE_USER) {
                HAL_INFO("v4_iq", "DRC: DRCToneMappingValue present (%d nodes), forcing CurveSelect=2 (USER)\n", n);
                drc.enCurveSelect = DRC_CURVE_USER;
            }
        }
    }

    ret = v4_isp.fnSetDRCAttr(pipe, &drc);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetDRCAttr failed with %#x\n", ret);
    } else {
        ISP_DRC_ATTR_S rb;
        memset(&rb, 0, sizeof(rb));
        int gr = v4_isp.fnGetDRCAttr(pipe, &rb);
        if (!gr) {
            HAL_INFO("v4_iq", "DRC: applied (en=%d opType=%d strength=%u tm[0]=%u tm[last]=%u)\n",
                (int)rb.bEnable, (int)rb.enOpType,
                (unsigned)rb.stAuto.u16Strength,
                (unsigned)rb.au16ToneMappingValue[0],
                (unsigned)rb.au16ToneMappingValue[HI_ISP_DRC_TM_NODE_NUM - 1]);
        } else {
            HAL_INFO("v4_iq", "DRC: applied\n");
        }
    }
    return ret;
}

static int v4_iq_apply_static_nr(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetNRAttr || !v4_isp.fnSetNRAttr) {
        HAL_INFO("v4_iq", "NR: API not available, skipping\n");
        return EXIT_SUCCESS;
    }
    const char *sec = "static_nr";
    int sec_s = 0, sec_e = 0;
    if (night_mode_on() && section_pos(ini, "ir_static_nr", &sec_s, &sec_e) == CONFIG_OK) {
        sec = "ir_static_nr";
    } else if (section_pos(ini, "static_nr", &sec_s, &sec_e) != CONFIG_OK) {
        HAL_INFO("v4_iq", "NR: no [static_nr] section, skipping\n");
        return EXIT_SUCCESS;
    }

    ISP_NR_ATTR_S nr;
    memset(&nr, 0, sizeof(nr));
    int ret = v4_isp.fnGetNRAttr(pipe, &nr);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetNRAttr failed with %#x\n", ret);
        return ret;
    }

    int val;
    if (parse_int(ini, sec, "Enable", 0, 1, &val) == CONFIG_OK)
        nr.bEnable = (HI_BOOL)val;
    nr.enOpType = OP_TYPE_AUTO;

    char buf[512];
    if (parse_param_value(ini, sec, "FineStr", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, nr.stAuto.au8FineStr, ISP_AUTO_ISO_STRENGTH_NUM);
    {
        HI_U32 tmp[ISP_AUTO_ISO_STRENGTH_NUM];
        if (parse_param_value(ini, sec, "CoringWgt", buf) == CONFIG_OK) {
            int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
            for (int i = 0; i < n; i++)
                nr.stAuto.au16CoringWgt[i] = (HI_U16)tmp[i];
        }
    }

    // Safety clamp: some SDKs reject out-of-range values with HI_ERR_ISP_ILLEGAL_PARAM.
    // Ranges from hi_comm_isp.h:
    // - FineStr:   [0x0, 0x80]
    // - CoringWgt: [0x0, 0xc80]
    for (int i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
        if (nr.stAuto.au8FineStr[i] > 0x80)
            nr.stAuto.au8FineStr[i] = 0x80;
        if (nr.stAuto.au16CoringWgt[i] > 0x0C80)
            nr.stAuto.au16CoringWgt[i] = 0x0C80;
    }

    ret = v4_isp.fnSetNRAttr(pipe, &nr);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetNRAttr failed with %#x\n", ret);
        // Dump a compact view of what we attempted to apply.
        HAL_WARNING("v4_iq",
            "NR reject dump: sec=%s en=%d opType=%d fine[0..3]=%u,%u,%u,%u coring[0..3]=%u,%u,%u,%u\n",
            sec, (int)nr.bEnable, (int)nr.enOpType,
            (unsigned)nr.stAuto.au8FineStr[0],
            (unsigned)nr.stAuto.au8FineStr[1],
            (unsigned)nr.stAuto.au8FineStr[2],
            (unsigned)nr.stAuto.au8FineStr[3],
            (unsigned)nr.stAuto.au16CoringWgt[0],
            (unsigned)nr.stAuto.au16CoringWgt[1],
            (unsigned)nr.stAuto.au16CoringWgt[2],
            (unsigned)nr.stAuto.au16CoringWgt[3]);
    } else {
        ISP_NR_ATTR_S rb;
        memset(&rb, 0, sizeof(rb));
        int gr = v4_isp.fnGetNRAttr(pipe, &rb);
        if (!gr) {
            HAL_INFO("v4_iq", "NR: applied (en=%d opType=%d fine[0]=%u coringWgt[0]=%u)\n",
                (int)rb.bEnable, (int)rb.enOpType,
                (unsigned)rb.stAuto.au8FineStr[0],
                (unsigned)rb.stAuto.au16CoringWgt[0]);
        } else {
            HAL_INFO("v4_iq", "NR: applied\n");
        }
    }
    return ret;
}

static int v4_iq_apply_gamma(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetGammaAttr || !v4_isp.fnSetGammaAttr) {
        HAL_INFO("v4_iq", "Gamma: API not available, skipping\n");
        return EXIT_SUCCESS;
    }

    // Allow disabling custom gamma from IQ.
    // Some scenes (e.g. snowy dawn) are extremely sensitive to tone curve choice.
    int en = 1;
    if (parse_int(ini, "dynamic_gamma", "Enable", 0, 1, &en) == CONFIG_OK && en == 0) {
        HAL_INFO("v4_iq", "Gamma: disabled by dynamic_gamma/Enable=0\n");
        return EXIT_SUCCESS;
    }

    ISP_GAMMA_ATTR_S gamma;
    memset(&gamma, 0, sizeof(gamma));
    int ret = v4_isp.fnGetGammaAttr(pipe, &gamma);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetGammaAttr failed with %#x\n", ret);
        return ret;
    }

    // This IQ format uses [dynamic_gamma] with Table_0..; apply a user-defined gamma curve at init.
    // We support selecting which table to use via dynamic_gamma/UseTable (0..2).
    HI_U32 tmp[GAMMA_NODE_NUM];
    memset(tmp, 0, sizeof(tmp));
    int use_table = 0;
    (void)parse_int(ini, "dynamic_gamma", "UseTable", 0, 2, &use_table);
    char key[32];
    snprintf(key, sizeof(key), "Table_%d", use_table);

    int n = v4_iq_parse_multiline_u32(ini, "dynamic_gamma", key, tmp, GAMMA_NODE_NUM);
    if (n <= 0 && use_table != 0) {
        // Backward compatible fallback
        n = v4_iq_parse_multiline_u32(ini, "dynamic_gamma", "Table_0", tmp, GAMMA_NODE_NUM);
    }
    if (n <= 0) {
        // Some IQ files may have a static section.
        n = v4_iq_parse_multiline_u32(ini, "static_gamma", "Table", tmp, GAMMA_NODE_NUM);
    }
    if (n > 0) {
        gamma.bEnable = HI_TRUE;
        gamma.enCurveType = ISP_GAMMA_CURVE_USER_DEFINE;
        for (int i = 0; i < n && i < GAMMA_NODE_NUM; i++) {
            HI_U32 v = tmp[i];
            if (v > 4095) v = 4095;
            gamma.u16Table[i] = (HI_U16)v;
        }
        ret = v4_isp.fnSetGammaAttr(pipe, &gamma);
        if (ret) {
            HAL_WARNING("v4_iq", "HI_MPI_ISP_SetGammaAttr failed with %#x\n", ret);
        } else {
            ISP_GAMMA_ATTR_S rb;
            memset(&rb, 0, sizeof(rb));
            int gr = v4_isp.fnGetGammaAttr(pipe, &rb);
            if (!gr) {
                HAL_INFO("v4_iq", "Gamma: applied (table=%d type=%d node0=%u nodeLast=%u)\n",
                    use_table,
                    (int)rb.enCurveType,
                    (unsigned)rb.u16Table[0],
                    (unsigned)rb.u16Table[GAMMA_NODE_NUM - 1]);
            } else {
                HAL_INFO("v4_iq", "Gamma: applied (table=%d %d nodes)\n", use_table, n);
            }
        }
        return ret;
    }

    HAL_INFO("v4_iq", "Gamma: no table found, skipping\n");
    return EXIT_SUCCESS;
}

static int v4_iq_apply_static_sharpen(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetIspSharpenAttr || !v4_isp.fnSetIspSharpenAttr) {
        HAL_INFO("v4_iq", "Sharpen: API not available, skipping\n");
        return EXIT_SUCCESS;
    }
    const char *sec = "static_sharpen";
    int sec_s = 0, sec_e = 0;
    if (night_mode_on() && section_pos(ini, "ir_static_sharpen", &sec_s, &sec_e) == CONFIG_OK) {
        sec = "ir_static_sharpen";
    } else if (section_pos(ini, "static_sharpen", &sec_s, &sec_e) != CONFIG_OK) {
        HAL_INFO("v4_iq", "Sharpen: no [static_sharpen] section, skipping\n");
        return EXIT_SUCCESS;
    }

    ISP_SHARPEN_ATTR_S shp;
    memset(&shp, 0, sizeof(shp));
    int ret = v4_isp.fnGetIspSharpenAttr(pipe, &shp);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetIspSharpenAttr failed with %#x\n", ret);
        return ret;
    }

    int val;
    if (parse_int(ini, sec, "Enable", 0, 1, &val) == CONFIG_OK)
        shp.bEnable = (HI_BOOL)val;
    shp.enOpType = OP_TYPE_AUTO;

    char buf[1024];
    // Auto luma weights: AutoLumaWgt_0..31 (each has 16 values)
    for (int i = 0; i < ISP_SHARPEN_LUMA_NUM; i++) {
        char key[32];
        snprintf(key, sizeof(key), "AutoLumaWgt_%d", i);
        if (parse_param_value(ini, sec, key, buf) == CONFIG_OK)
            v4_iq_parse_csv_u8(buf, shp.stAuto.au8LumaWgt[i], ISP_AUTO_ISO_STRENGTH_NUM);
    }

    // Auto texture/edge strength: AutoTextureStr_0..31, AutoEdgeStr_0..31
    for (int i = 0; i < ISP_SHARPEN_GAIN_NUM; i++) {
        char key[32];
        snprintf(key, sizeof(key), "AutoTextureStr_%d", i);
        if (parse_param_value(ini, sec, key, buf) == CONFIG_OK) {
            HI_U32 tmp[ISP_AUTO_ISO_STRENGTH_NUM];
            int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
            for (int j = 0; j < n; j++)
                shp.stAuto.au16TextureStr[i][j] = (HI_U16)tmp[j];
        }
        snprintf(key, sizeof(key), "AutoEdgeStr_%d", i);
        if (parse_param_value(ini, sec, key, buf) == CONFIG_OK) {
            HI_U32 tmp[ISP_AUTO_ISO_STRENGTH_NUM];
            int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
            for (int j = 0; j < n; j++)
                shp.stAuto.au16EdgeStr[i][j] = (HI_U16)tmp[j];
        }
    }

    // Per-ISO scalars
    if (parse_param_value(ini, "static_sharpen", "AutoTextureFreq", buf) == CONFIG_OK) {
        HI_U32 tmp[ISP_AUTO_ISO_STRENGTH_NUM];
        int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
        for (int j = 0; j < n; j++)
            shp.stAuto.au16TextureFreq[j] = (HI_U16)tmp[j];
    }
    if (parse_param_value(ini, "static_sharpen", "AutoEdgeFreq", buf) == CONFIG_OK) {
        HI_U32 tmp[ISP_AUTO_ISO_STRENGTH_NUM];
        int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
        for (int j = 0; j < n; j++)
            shp.stAuto.au16EdgeFreq[j] = (HI_U16)tmp[j];
    }
    if (parse_param_value(ini, "static_sharpen", "AutoOverShoot", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, shp.stAuto.au8OverShoot, ISP_AUTO_ISO_STRENGTH_NUM);
    if (parse_param_value(ini, "static_sharpen", "AutoUnderShoot", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, shp.stAuto.au8UnderShoot, ISP_AUTO_ISO_STRENGTH_NUM);
    if (parse_param_value(ini, "static_sharpen", "AutoShootSupStr", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, shp.stAuto.au8ShootSupStr, ISP_AUTO_ISO_STRENGTH_NUM);
    if (parse_param_value(ini, "static_sharpen", "AutoShootSupAdj", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, shp.stAuto.au8ShootSupAdj, ISP_AUTO_ISO_STRENGTH_NUM);
    if (parse_param_value(ini, "static_sharpen", "AutoDetailCtrl", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, shp.stAuto.au8DetailCtrl, ISP_AUTO_ISO_STRENGTH_NUM);
    if (parse_param_value(ini, "static_sharpen", "AutoDetailCtrlThr", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, shp.stAuto.au8DetailCtrlThr, ISP_AUTO_ISO_STRENGTH_NUM);
    if (parse_param_value(ini, "static_sharpen", "AutoEdgeFiltStr", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, shp.stAuto.au8EdgeFiltStr, ISP_AUTO_ISO_STRENGTH_NUM);
    if (parse_param_value(ini, "static_sharpen", "AutoEdgeFiltMaxCap", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, shp.stAuto.au8EdgeFiltMaxCap, ISP_AUTO_ISO_STRENGTH_NUM);
    if (parse_param_value(ini, "static_sharpen", "AutoRGain", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, shp.stAuto.au8RGain, ISP_AUTO_ISO_STRENGTH_NUM);
    if (parse_param_value(ini, "static_sharpen", "AutoGGain", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, shp.stAuto.au8GGain, ISP_AUTO_ISO_STRENGTH_NUM);
    if (parse_param_value(ini, "static_sharpen", "AutoBGain", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, shp.stAuto.au8BGain, ISP_AUTO_ISO_STRENGTH_NUM);
    if (parse_param_value(ini, "static_sharpen", "AutoSkinGain", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, shp.stAuto.au8SkinGain, ISP_AUTO_ISO_STRENGTH_NUM);
    if (parse_param_value(ini, "static_sharpen", "AutoMaxSharpGain", buf) == CONFIG_OK) {
        HI_U32 tmp[ISP_AUTO_ISO_STRENGTH_NUM];
        int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
        for (int j = 0; j < n; j++)
            shp.stAuto.au16MaxSharpGain[j] = (HI_U16)tmp[j];
    }

    ret = v4_isp.fnSetIspSharpenAttr(pipe, &shp);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetIspSharpenAttr failed with %#x\n", ret);
    } else {
        ISP_SHARPEN_ATTR_S rb;
        memset(&rb, 0, sizeof(rb));
        int gr = v4_isp.fnGetIspSharpenAttr(pipe, &rb);
        if (!gr) {
            HAL_INFO("v4_iq", "Sharpen: applied (en=%d opType=%d texFreq[0]=%u edgeFreq[0]=%u)\n",
                (int)rb.bEnable, (int)rb.enOpType,
                (unsigned)rb.stAuto.au16TextureFreq[0],
                (unsigned)rb.stAuto.au16EdgeFreq[0]);
        } else {
            HAL_INFO("v4_iq", "Sharpen: applied\n");
        }
    }
    return ret;
}

static int v4_iq_apply(const char *path, int pipe) {
    if (!path || !*path)
        return EXIT_SUCCESS;
    if (access(path, F_OK)) {
        HAL_WARNING("v4_iq", "IQ config '%s' not found, skipping\n", path);
        return EXIT_SUCCESS;
    }

    struct IniConfig ini;
    memset(&ini, 0, sizeof(ini));
    FILE *file = fopen(path, "r");
    if (!open_config(&ini, &file))
        return EXIT_FAILURE;
    find_sections(&ini);

    // Respect module_state toggles when present; default to "apply" if section/key missing.
    bool doStaticAE = true, doStaticCCM = true, doStaticSat = true;
    bool doStaticLDCI = true, doStaticDehaze = true;
    bool doStaticDRC = true, doStaticNR = true, doStaticSharpen = true;
    bool doGamma = true;
    bool doDynDehaze = true, doDynLinearDRC = true;
    parse_bool(&ini, "module_state", "bStaticAE", &doStaticAE);
    parse_bool(&ini, "module_state", "bStaticCCM", &doStaticCCM);
    parse_bool(&ini, "module_state", "bStaticSaturation", &doStaticSat);
    parse_bool(&ini, "module_state", "bStaticLDCI", &doStaticLDCI);
    parse_bool(&ini, "module_state", "bStaticDehaze", &doStaticDehaze);
    parse_bool(&ini, "module_state", "bStaticDRC", &doStaticDRC);
    parse_bool(&ini, "module_state", "bStaticNr", &doStaticNR);
    parse_bool(&ini, "module_state", "bStaticSharpen", &doStaticSharpen);
    // Gamma in this IQ is described under "dynamic_gamma"
    parse_bool(&ini, "module_state", "bDynamicGamma", &doGamma);
    // Optional dynamic controls
    parse_bool(&ini, "module_state", "bDynamicDehaze", &doDynDehaze);
    parse_bool(&ini, "module_state", "bDynamicLinearDRC", &doDynLinearDRC);

    // Parse dynamic sections now (used by background thread).
    v4_iq_dyn_update_from_ini(&ini, pipe, doDynDehaze, doDynLinearDRC);

    int ret = EXIT_SUCCESS;
    HAL_INFO("v4_iq", "Loading IQ config '%s'\n", path);
    if (doStaticAE) {
        int r = v4_iq_apply_static_ae(&ini, pipe);
        if (r) ret = r;
        r = v4_iq_apply_static_aerouteex(&ini, pipe);
        if (r) ret = r;
    }
    {
        int r = v4_iq_apply_static_aeweight(&ini, pipe);
        if (r) ret = r;
    }
    if (doStaticCCM) {
        int r = v4_iq_apply_static_ccm(&ini, pipe);
        if (r) ret = r;
    }
    if (doStaticSat) {
        int r = v4_iq_apply_static_saturation(&ini, pipe);
        if (r) ret = r;
    }
    if (doStaticLDCI) {
        int r = v4_iq_apply_static_ldci(&ini, pipe);
        if (r) ret = r;
    }
    if (doStaticDehaze) {
        int r = v4_iq_apply_static_dehaze(&ini, pipe);
        if (r) ret = r;
    }
    if (doStaticDRC) {
        int r = v4_iq_apply_static_drc(&ini, pipe);
        if (r) ret = r;
    }
    if (doStaticNR) {
        int r = v4_iq_apply_static_nr(&ini, pipe);
        if (r) ret = r;
    }
    if (doStaticSharpen) {
        int r = v4_iq_apply_static_sharpen(&ini, pipe);
        if (r) ret = r;
    }
    if (doGamma) {
        int r = v4_iq_apply_gamma(&ini, pipe);
        if (r) ret = r;
    }

    free(ini.str);
    if (ret) HAL_WARNING("v4_iq", "IQ apply finished with error %#x\n", ret);
    else HAL_INFO("v4_iq", "IQ apply finished OK\n");
    return ret;
}

int v4_pipeline_create(const char *iqConfig)
{
    int ret;
    if (iqConfig && *iqConfig)
        snprintf(_v4_iq_cfg_path, sizeof(_v4_iq_cfg_path), "%s", iqConfig);
    else
        _v4_iq_cfg_path[0] = '\0';

    {
        v4_sys_oper mode[4];
        v4_sys.fnGetViVpssMode((v4_sys_oper*)mode);
        for (char i = 0; i < 4; i++)
            mode[i] = V4_SYS_OPER_VIOFF_VPSSON;
        if (ret = v4_sys.fnSetViVpssMode((v4_sys_oper*)mode))
            return ret;
    }

    if (ret = v4_sensor_config())
        return ret;

    if (ret = v4_vi.fnSetDeviceConfig(_v4_vi_dev, &v4_config.videv))
        return ret;
    if (ret = v4_vi.fnEnableDevice(_v4_vi_dev))
        return ret;

    {
        v4_vi_bind bind;
        bind.num = 1;
        bind.pipeId[0] = _v4_vi_pipe;
        if (ret = v4_vi.fnBindPipe(_v4_vi_dev, &bind))
            return ret;
    }

    {
        v4_vi_pipe pipe;
        pipe.bypass = 0;
        pipe.yuvSkipOn = 0;
        pipe.ispBypassOn = 0;
        pipe.maxSize.width = v4_config.isp.capt.width;
        pipe.maxSize.height = v4_config.isp.capt.height;
        pipe.pixFmt = V4_PIXFMT_RGB_BAYER_8BPP + v4_config.mipi.prec;
        pipe.compress = V4_COMPR_NONE;
        pipe.prec = v4_config.mipi.prec;
        pipe.nRedOn = 0;
        pipe.nRed.pixFmt = 0;
        pipe.nRed.prec = 0;
        pipe.nRed.srcRfrOrChn0 = 0;
        pipe.nRed.compress = V4_COMPR_NONE;
        pipe.sharpenOn = 1;
        pipe.srcFps = -1;
        pipe.dstFps = -1;
        pipe.discProPic = 0;
        if (ret = v4_vi.fnCreatePipe(_v4_vi_pipe, &pipe))
            return ret;
    }
    if (ret = v4_vi.fnStartPipe(_v4_vi_pipe))
        return ret;

    {
        v4_vi_chn channel;
        channel.size.width = v4_config.isp.capt.width;
        channel.size.height = v4_config.isp.capt.height;
        channel.pixFmt = V4_PIXFMT_YVU420SP;
        channel.dynRange = V4_HDR_SDR8;
        channel.videoFmt = 0;
        channel.compress = V4_COMPR_NONE;
        channel.mirror = 0;
        channel.flip = 0;
        channel.depth = 0;
        channel.srcFps = -1;
        channel.dstFps = -1;
        if (ret = v4_vi.fnSetChannelConfig(_v4_vi_pipe, _v4_vi_chn, &channel))
            return ret;
    }
    if (ret = v4_vi.fnEnableChannel(_v4_vi_pipe, _v4_vi_chn))
        return ret;

    {
        v4_snr_bus bus;
        bus.i2c = 0;
        if (ret = v4_snr_drv.obj->pfnSetBusInfo(_v4_vi_pipe, bus))
            return ret;
    }

    if (!v4_isp.handleGoke) {
        strcpy(v4_ae_lib.libName, "hisi_ae_lib");
        strcpy(v4_awb_lib.libName, "hisi_awb_lib");
    }

    if (ret = v4_snr_drv.obj->pfnRegisterCallback(_v4_vi_pipe, &v4_ae_lib, &v4_awb_lib))
        return ret;
    
    if (ret = v4_isp.fnRegisterAE(_v4_vi_pipe, &v4_ae_lib))
        return ret;
    if (ret = v4_isp.fnRegisterAWB(_v4_vi_pipe, &v4_awb_lib))
        return ret;
    if (ret = v4_isp.fnMemInit(_v4_vi_pipe))
        return ret;

    v4_config.isp.capt.x = 0;
    v4_config.isp.capt.y = 0;
    if (ret = v4_isp.fnSetDeviceConfig(_v4_vi_pipe, &v4_config.isp))
        return ret;
    if (ret = v4_isp.fnInit(_v4_vi_pipe))
        return ret;

    // Apply optional IQ/scene profile before ISP Run thread starts.
    ret = v4_iq_apply(iqConfig, _v4_vi_pipe);
    if (ret)
        HAL_WARNING("v4_iq", "IQ application returned %#x (continuing)\n", ret);
    
    {
        v4_vpss_grp group;
        memset(&group, 0, sizeof(group));
        group.dest.width = v4_config.isp.capt.width;
        group.dest.height = v4_config.isp.capt.height;
        group.pixFmt = V4_PIXFMT_YVU420SP;
        group.hdr = V4_HDR_SDR8;
        group.srcFps = -1;
        group.dstFps = -1;
        group.nRedOn = 1;
        group.nRed.mode = V4_VPSS_NMODE_VIDEO;
        group.nRed.compress = V4_COMPR_NONE;
        group.nRed.motionCompOn = 0;
        if (ret = v4_vpss.fnCreateGroup(_v4_vpss_grp, &group))
            return ret;
    }
    if (ret = v4_vpss.fnStartGroup(_v4_vpss_grp))
        return ret;

    {
        v4_sys_bind source = { .module = V4_SYS_MOD_VIU, 
            .device = _v4_vi_dev, .channel = _v4_vi_chn };
        v4_sys_bind dest = { .module = V4_SYS_MOD_VPSS, 
            .device = _v4_vpss_grp, .channel = 0 };
        if (ret = v4_sys.fnBind(&source, &dest))
            return ret;
    }

    return EXIT_SUCCESS;
}

void v4_pipeline_destroy(void)
{
    v4_isp.fnExit(_v4_vi_pipe);
    v4_isp.fnUnregisterAWB(_v4_vi_pipe, &v4_awb_lib);
    v4_isp.fnUnregisterAE(_v4_vi_pipe, &v4_ae_lib);

    v4_snr_drv.obj->pfnUnRegisterCallback(_v4_vi_pipe, &v4_ae_lib, &v4_awb_lib);

    for (char grp = 0; grp < V4_VPSS_GRP_NUM; grp++)
    {
        for (char chn = 0; chn < V4_VPSS_CHN_NUM; chn++)
            v4_vpss.fnDisableChannel(grp, chn);

        {
            v4_sys_bind source = { .module = V4_SYS_MOD_VIU, 
                .device = _v4_vi_dev, .channel = _v4_vi_chn };
            v4_sys_bind dest = { .module = V4_SYS_MOD_VPSS,
                .device = grp, .channel = 0 };
            v4_sys.fnUnbind(&source, &dest);
        }

        v4_vpss.fnStopGroup(grp);
        v4_vpss.fnDestroyGroup(grp);
    }
    
    v4_vi.fnDisableChannel(_v4_vi_pipe, _v4_vi_chn);

    v4_vi.fnStopPipe(_v4_vi_pipe);
    v4_vi.fnDestroyPipe(_v4_vi_pipe);

    v4_vi.fnDisableDevice(_v4_vi_dev);

    v4_sensor_deconfig();
}

int v4_region_create(char handle, hal_rect rect, short opacity)
{
    // Backwards compatible wrapper: no background alpha.
    return v4_region_create_ex(handle, rect, opacity, 0);
}

int v4_region_create_ex(char handle, hal_rect rect, short fg_opacity, short bg_opacity)
{
    int ret;

    v4_sys_bind dest = { .module = V4_SYS_MOD_VENC, .device = _v4_venc_dev, .channel = 0 };
    v4_rgn_cnf region, regionCurr;
    v4_rgn_chn attrib, attribCurr;

    rect.height += rect.height & 1;
    rect.width += rect.width & 1;

    memset(&region, 0, sizeof(region));
    region.type = V4_RGN_TYPE_OVERLAY;
    region.overlay.pixFmt = V4_PIXFMT_ARGB1555;
    region.overlay.size.width = rect.width;
    region.overlay.size.height = rect.height;
    region.overlay.canvas = handle + 1;

    if (v4_rgn.fnGetRegionConfig(handle, &regionCurr)) {
        HAL_INFO("v4_rgn", "Creating region %d...\n", handle);
        if (ret = v4_rgn.fnCreateRegion(handle, &region))
            return ret;
    } else if (regionCurr.type != region.type ||
        regionCurr.overlay.size.height != region.overlay.size.height || 
        regionCurr.overlay.size.width != region.overlay.size.width) {
        HAL_INFO("v4_rgn", "Parameters are different, recreating "
            "region %d...\n", handle);
        for (char i = 0; i < V4_VENC_CHN_NUM; i++) {
            if (!v4_state[i].enable) continue;
            dest.channel = i;
            v4_rgn.fnDetachChannel(handle, &dest);
        }
        v4_rgn.fnDestroyRegion(handle);
        if (ret = v4_rgn.fnCreateRegion(handle, &region))
            return ret;
    }

    if (v4_rgn.fnGetChannelConfig(handle, &dest, &attribCurr))
        HAL_INFO("v4_rgn", "Attaching region %d...\n", handle);
    else if (attribCurr.overlay.point.x != rect.x || attribCurr.overlay.point.y != rect.y) {
        HAL_INFO("v4_rgn", "Position has changed, reattaching "
            "region %d...\n", handle);
        for (char i = 0; i < V4_VENC_CHN_NUM; i++) {
            if (!v4_state[i].enable) continue;
            dest.channel = i;
            v4_rgn.fnDetachChannel(handle, &dest);
        }
    }

    memset(&attrib, 0, sizeof(attrib));
    attrib.show = 1;
    attrib.type = V4_RGN_TYPE_OVERLAY;
    attrib.overlay.bgAlpha = bg_opacity >> 1;
    attrib.overlay.fgAlpha = fg_opacity >> 1;
    attrib.overlay.point.x = rect.x;
    attrib.overlay.point.y = rect.y;
    attrib.overlay.layer = 7;
    attrib.overlay.attachDest = V4_RGN_DEST_MAIN;

    for (char i = 0; i < V4_VENC_CHN_NUM; i++) {
        if (!v4_state[i].enable) continue;
        dest.channel = i;
        if (!hal_osd_is_allowed_for_channel(&v4_state[i])) {
            v4_rgn.fnDetachChannel(handle, &dest);
            continue;
        }
        v4_rgn.fnAttachChannel(handle, &dest, &attrib);
    }

    return EXIT_SUCCESS;
}

void v4_region_destroy(char handle)
{
    v4_sys_bind dest = { .module = V4_SYS_MOD_VENC, .device = _v4_venc_dev, .channel = 0 };
    
    for (char i = 0; i < V4_VENC_CHN_NUM; i++) {
        if (!v4_state[i].enable) continue;
        dest.channel = i;
        v4_rgn.fnDetachChannel(handle, &dest);
    }
    v4_rgn.fnDestroyRegion(handle);
}

int v4_region_setbitmap(int handle, hal_bitmap *bitmap)
{
    v4_rgn_bmp nativeBmp = { .data = bitmap->data, .pixFmt = V4_PIXFMT_ARGB1555,
        .size.height = bitmap->dim.height, .size.width = bitmap->dim.width };

    return v4_rgn.fnSetBitmap(handle, &nativeBmp);
}

int v4_sensor_config(void) {
    int fd;
    char v4a_device = 0;
    if (EQUALS(chip, "Hi3516AV300") ||
        EQUALS(chip, "Hi3516DV300") ||
        EQUALS(chip, "Hi3516CV500"))
        v4a_device = 1;

    if (v4a_device) {
        v4a_snr_dev config;
        memset(&config, 0, sizeof(config));
        config.device = 0;
        config.input = v4_config.input_mode;
        config.rect.width = v4_config.isp.capt.width;
        config.rect.height = v4_config.isp.capt.height;
        if (config.input == V4_SNR_INPUT_MIPI)
            memcpy(&config.mipi, &v4_config.mipi, sizeof(v4_snr_mipi));
        else if (config.input == V4_SNR_INPUT_LVDS)
            memcpy(&config.lvds, &v4_config.lvds, sizeof(v4_snr_lvds));

        if (!access(v4_snr_endp, F_OK))
            fd = open(v4_snr_endp, O_RDWR);
        else fd = open("/dev/mipi", O_RDWR);
        if (fd < 0)
            HAL_ERROR("v4_snr", "Opening imaging device has failed!\n");

        int laneMode = 0;
        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_CONF_HSMODE, int), &laneMode);

        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_CLKON_MIPI, unsigned int), &config.device);

        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_RST_MIPI, unsigned int), &config.device);

        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_CLKON_SENS, unsigned int), &config.device);

        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_RST_SENS, unsigned int), &config.device);
        
        if (ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_CONF_DEV, v4a_snr_dev), &config))
            HAL_ERROR("v4_snr", "Configuring imaging device has failed!\n");

        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_UNRST_MIPI, unsigned int), &config.device);

        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_UNRST_SENS, unsigned int), &config.device);

        close(fd);
    } else {
        v4_snr_dev config;
        memset(&config, 0, sizeof(config));
        config.device = 0;
        config.input = v4_config.input_mode;
        config.rect.width = v4_config.isp.capt.width;
        config.rect.height = v4_config.isp.capt.height;
        if (config.input == V4_SNR_INPUT_MIPI)
            memcpy(&config.mipi, &v4_config.mipi, sizeof(v4_snr_mipi));
        else if (config.input == V4_SNR_INPUT_LVDS)
            memcpy(&config.lvds, &v4_config.lvds, sizeof(v4_snr_lvds));

        if (!access(v4_snr_endp, F_OK))
            fd = open(v4_snr_endp, O_RDWR);
        else fd = open("/dev/mipi", O_RDWR);
        if (fd < 0)
            HAL_ERROR("v4_snr", "Opening imaging device has failed!\n");

        int laneMode = 0;
        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_CONF_HSMODE, int), &laneMode);

        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_CLKON_MIPI, unsigned int), &config.device);

        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_RST_MIPI, unsigned int), &config.device);

        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_CLKON_SENS, unsigned int), &config.device);

        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_RST_SENS, unsigned int), &config.device);
        
        if (ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_CONF_DEV, v4_snr_dev), &config))
            HAL_ERROR("v4_snr", "Configuring imaging device has failed!\n");

        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_UNRST_MIPI, unsigned int), &config.device);

        ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_UNRST_SENS, unsigned int), &config.device);

        close(fd);
    }

    return EXIT_SUCCESS;
}

void v4_sensor_deconfig(void) {
    int fd;
    v4_snr_dev config;
    config.device = 0;

    if (!access(v4_snr_endp, F_OK))
        fd = open(v4_snr_endp, O_RDWR);
    else fd = open("/dev/mipi", O_RDWR);
    if (fd < 0)
        HAL_DANGER("v4_snr", "Opening imaging device has failed!\n");

    ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_CLKOFF_SENS, unsigned int), &config.device);

    ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_RST_SENS, unsigned int), &config.device);

    ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_CLKON_MIPI, unsigned int), &config.device);

    ioctl(fd, _IOW(V4_SNR_IOC_MAGIC, V4_SNR_CMD_RST_MIPI, unsigned int), &config.device);

    close(fd);
}

void v4_sensor_deinit(void)
{
    dlclose(v4_snr_drv.handle);
    v4_snr_drv.handle = NULL;
}

int v4_sensor_init(char *name, char *obj)
{
    char path[128];
    char* dirs[] = {"%s", "./%s", "/usr/lib/sensors/%s", NULL};
    char **dir = dirs;

    while (*dir) {
        sprintf(path, *dir++, name);
        if (v4_snr_drv.handle = dlopen(path, RTLD_LAZY | RTLD_GLOBAL))
            break;
    } if (!v4_snr_drv.handle)
        HAL_ERROR("v4_snr", "Failed to load the sensor driver\n");
    
    if (!(v4_snr_drv.obj = (v4_snr_obj*)dlsym(v4_snr_drv.handle, obj)))
        HAL_ERROR("v4_snr", "Failed to connect the sensor object\n");

    return EXIT_SUCCESS;
}

int v4_video_create(char index, hal_vidconfig *config)
{
    int ret;
    v4_venc_chn channel;
    memset(&channel, 0, sizeof(channel));
    const int h264_plus =
        (config->codec == HAL_VIDCODEC_H264) && (config->flags & HAL_VIDOPT_H264_PLUS);
    channel.gop.mode = h264_plus ? V4_VENC_GOPMODE_ADVSMARTP : V4_VENC_GOPMODE_NORMALP;
    if (config->codec == HAL_VIDCODEC_JPG) {
        // Dedicated JPEG snapshot channel.
        // Some vendor SDKs validate the codec and attribute union strictly, so JPEG must
        // use V4_VENC_CODEC_JPEG (not MJPG), and a sane buffer size.
        channel.attrib.codec = V4_VENC_CODEC_JPEG;
        channel.attrib.maxPic.width = config->width;
        channel.attrib.maxPic.height = config->height;
        channel.attrib.bufSize = ALIGN_UP(config->height, 16) * ALIGN_UP(config->width, 16);
        channel.attrib.byFrame = 1;
        channel.attrib.pic.width = config->width;
        channel.attrib.pic.height = config->height;
        channel.attrib.jpg.dcfThumbs = 0;
        channel.attrib.jpg.numThumbs = 0;
        memset(channel.attrib.jpg.sizeThumbs, 0, sizeof(channel.attrib.jpg.sizeThumbs));
        channel.attrib.jpg.multiReceiveOn = 0;
        goto create;
    } else if (config->codec == HAL_VIDCODEC_MJPG) {
        // MJPEG stream channel (rate control still applies).
        channel.attrib.codec = V4_VENC_CODEC_MJPG;
        switch (config->mode) {
            case HAL_VIDMODE_CBR:
                channel.rate.mode = V4_VENC_RATEMODE_MJPGCBR;
                channel.rate.mjpgCbr = (v4_venc_rate_mjpgbr){ .statTime = 1, .srcFps = config->framerate,
                    .dstFps = config->framerate, .maxBitrate = config->bitrate }; break;
            case HAL_VIDMODE_VBR:
                channel.rate.mode = V4_VENC_RATEMODE_MJPGVBR;
                channel.rate.mjpgVbr = (v4_venc_rate_mjpgbr){ .statTime = 1, 
                    .srcFps = config->framerate, .dstFps = config->framerate, 
                    .maxBitrate = MAX(config->bitrate, config->maxBitrate) }; break;
            case HAL_VIDMODE_QP:
                channel.rate.mode = V4_VENC_RATEMODE_MJPGQP;
                channel.rate.mjpgQp = (v4_venc_rate_mjpgqp){ .srcFps = config->framerate,
                    .dstFps = config->framerate, .quality = config->maxQual }; break;
            default:
                HAL_ERROR("v4_venc", "MJPEG encoder can only support CBR, VBR or fixed QP modes!");
        }
    } else if (config->codec == HAL_VIDCODEC_H265) {
        channel.attrib.codec = V4_VENC_CODEC_H265;
        channel.gop.normalP.ipQualDelta = config->gop / config->framerate;
        switch (config->mode) {
            case HAL_VIDMODE_CBR:
                channel.rate.mode = V4_VENC_RATEMODE_H265CBR;
                channel.rate.h265Cbr = (v4_venc_rate_h26xbr){ .gop = config->gop,
                    .statTime = 1, .srcFps = config->framerate, .dstFps = config->framerate,
                    .maxBitrate = config->bitrate }; break;
            case HAL_VIDMODE_VBR:
                channel.rate.mode = V4_VENC_RATEMODE_H265VBR;
                channel.rate.h265Vbr = (v4_venc_rate_h26xbr){ .gop = config->gop,
                    .statTime = 1, .srcFps = config->framerate, .dstFps = config->framerate, 
                    .maxBitrate = MAX(config->bitrate, config->maxBitrate) }; break;
            case HAL_VIDMODE_QP:
                channel.rate.mode = V4_VENC_RATEMODE_H265QP;
                channel.rate.h265Qp = (v4_venc_rate_h26xqp){ .gop = config->gop,
                    .srcFps = config->framerate, .dstFps = config->framerate, .interQual = config->maxQual, 
                    .predQual = config->minQual, .bipredQual = config->minQual }; break;
            case HAL_VIDMODE_AVBR:
                channel.rate.mode = V4_VENC_RATEMODE_H265AVBR;
                channel.rate.h265Avbr = (v4_venc_rate_h26xbr){ .gop = config->gop,
                    .statTime = 1, .srcFps = config->framerate, .dstFps = config->framerate,
                    .maxBitrate = config->bitrate }; break;
            default:
                HAL_ERROR("v4_venc", "H.265 encoder does not support this mode!");
        }
    } else if (config->codec == HAL_VIDCODEC_H264) {
        channel.attrib.codec = V4_VENC_CODEC_H264;
        if (h264_plus) {
            channel.gop.advSmartP.bgInterv = config->gop;
            channel.gop.advSmartP.bgQualDelta = 6;
            channel.gop.advSmartP.viQualDelta = 3;
        } else {
            channel.gop.normalP.ipQualDelta = config->gop / config->framerate;
        }
        hal_vidmode mode = config->mode;
        if (h264_plus && mode != HAL_VIDMODE_QP)
            mode = HAL_VIDMODE_AVBR;
        switch (mode) {
            case HAL_VIDMODE_CBR:
                channel.rate.mode = V4_VENC_RATEMODE_H264CBR;
                channel.rate.h264Cbr = (v4_venc_rate_h26xbr){ .gop = config->gop,
                    .statTime = 1, .srcFps = config->framerate, .dstFps = config->framerate,
                    .maxBitrate = config->bitrate }; break;
            case HAL_VIDMODE_VBR:
                channel.rate.mode = V4_VENC_RATEMODE_H264VBR;
                channel.rate.h264Vbr = (v4_venc_rate_h26xbr){ .gop = config->gop,
                    .statTime = 1, .srcFps = config->framerate, .dstFps = config->framerate, 
                    .maxBitrate = MAX(config->bitrate, config->maxBitrate) }; break;
            case HAL_VIDMODE_QP:
                channel.rate.mode = V4_VENC_RATEMODE_H264QP;
                channel.rate.h264Qp = (v4_venc_rate_h26xqp){ .gop = config->gop,
                    .srcFps = config->framerate, .dstFps = config->framerate, .interQual = config->maxQual, 
                    .predQual = config->minQual, .bipredQual = config->minQual }; break;
            case HAL_VIDMODE_AVBR:
                channel.rate.mode = V4_VENC_RATEMODE_H264AVBR;
                channel.rate.h264Avbr = (v4_venc_rate_h26xbr){ .gop = config->gop,
                    .statTime = 1, .srcFps = config->framerate, .dstFps = config->framerate,
                    .maxBitrate = config->bitrate }; break;
            default:
                HAL_ERROR("v4_venc", "H.264 encoder does not support this mode!");
        }
    } else HAL_ERROR("v4_venc", "This codec is not supported by the hardware!");
    channel.attrib.maxPic.width = config->width;
    channel.attrib.maxPic.height = config->height;
    // NOTE: For MJPEG/JPEG many SDKs require at least aligned W*H (and often reject smaller).
    if (channel.attrib.codec == V4_VENC_CODEC_MJPG || channel.attrib.codec == V4_VENC_CODEC_JPEG)
        channel.attrib.bufSize = ALIGN_UP(config->height, 16) * ALIGN_UP(config->width, 16);
    else
    channel.attrib.bufSize = ALIGN_UP(config->height * config->width * 3 / 4, 64);
    if (channel.attrib.codec == V4_VENC_CODEC_H264)
        channel.attrib.profile = MAX(config->profile, 2);
    channel.attrib.byFrame = 1;
    channel.attrib.pic.width = config->width;
    channel.attrib.pic.height = config->height;

create:
    // Debug: dump what we're asking the SDK to create.
    if (channel.attrib.codec == V4_VENC_CODEC_H264) {
        HAL_INFO("v4_venc", "CreateChannel ch=%d H264 plus=%d %dx%d fps=%d gop=%d gopMode=%d rateMode=%d bitrate=%d maxBitrate=%d qp=[%d..%d] profile=%u (cfgProfile=%d)\n",
            index, h264_plus ? 1 : 0, config->width, config->height, config->framerate, config->gop,
            (int)channel.gop.mode, (int)channel.rate.mode,
            (int)config->bitrate, (int)MAX(config->bitrate, config->maxBitrate),
            (int)config->minQual, (int)config->maxQual,
            channel.attrib.profile, (int)config->profile);
    } else if (channel.attrib.codec == V4_VENC_CODEC_MJPG) {
        HAL_INFO("v4_venc", "CreateChannel ch=%d MJPEG %dx%d fps=%d rateMode=%d bitrate=%d maxBitrate=%d quality=%d\n",
            index, config->width, config->height, config->framerate,
            (int)channel.rate.mode, config->bitrate, MAX(config->bitrate, config->maxBitrate),
            config->maxQual);
    } else if (channel.attrib.codec == V4_VENC_CODEC_JPEG) {
        HAL_INFO("v4_venc", "CreateChannel ch=%d JPEG %dx%d\n",
            index, config->width, config->height);
    } else if (channel.attrib.codec == V4_VENC_CODEC_H265) {
        HAL_INFO("v4_venc", "CreateChannel ch=%d H265 %dx%d fps=%d gop=%d gopMode=%d rateMode=%d maxBitrate=%d\n",
            index, config->width, config->height, config->framerate, config->gop,
            (int)channel.gop.mode, (int)channel.rate.mode,
            MAX(config->bitrate, config->maxBitrate));
    }

    ret = v4_venc.fnCreateChannel(index, &channel);
    HAL_INFO("v4_venc", "CreateChannel ch=%d -> ret=%#x (%s)\n", index, ret, errstr(ret));
    if (ret) {
        // Best-effort fallback for H.264+: try to reduce features until the SDK accepts it.
        if (h264_plus && channel.attrib.codec == V4_VENC_CODEC_H264) {
            // Try a few combinations to find what the firmware actually supports.
            // 1) SMARTP + AVBR (drop ADVSMARTP, keep AVBR)
            HAL_WARNING("v4_venc", "H264+ create failed, retrying with SMARTP + AVBR\n");
            memset(&channel, 0, sizeof(channel));
            channel.attrib.codec = V4_VENC_CODEC_H264;
            channel.gop.mode = V4_VENC_GOPMODE_SMARTP;
            channel.gop.smartP.bgInterv = config->gop;
            channel.gop.smartP.bgQualDelta = 6;
            channel.gop.smartP.viQualDelta = 3;
            channel.rate.mode = V4_VENC_RATEMODE_H264AVBR;
            channel.rate.h264Avbr = (v4_venc_rate_h26xbr){ .gop = config->gop,
                .statTime = 1, .srcFps = config->framerate, .dstFps = config->framerate,
                .maxBitrate = config->bitrate };
            channel.attrib.maxPic.width = config->width;
            channel.attrib.maxPic.height = config->height;
            channel.attrib.bufSize = ALIGN_UP(config->height * config->width * 3 / 4, 64);
            channel.attrib.profile = config->profile;
            channel.attrib.byFrame = 1;
            channel.attrib.pic.width = config->width;
            channel.attrib.pic.height = config->height;
            ret = v4_venc.fnCreateChannel(index, &channel);
            HAL_INFO("v4_venc", "CreateChannel ch=%d retry(SMARTP+AVBR) -> ret=%#x (%s)\n",
                index, ret, errstr(ret));

            // 2) NORMALP + AVBR (keep AVBR, drop Smart GOP)
            if (ret) {
                HAL_WARNING("v4_venc", "H264+ create failed, retrying with NORMALP + AVBR\n");
                memset(&channel, 0, sizeof(channel));
                channel.attrib.codec = V4_VENC_CODEC_H264;
                channel.gop.mode = V4_VENC_GOPMODE_NORMALP;
                channel.gop.normalP.ipQualDelta = config->gop / config->framerate;
                channel.rate.mode = V4_VENC_RATEMODE_H264AVBR;
                channel.rate.h264Avbr = (v4_venc_rate_h26xbr){ .gop = config->gop,
                    .statTime = 1, .srcFps = config->framerate, .dstFps = config->framerate,
                    .maxBitrate = config->bitrate };
                channel.attrib.maxPic.width = config->width;
                channel.attrib.maxPic.height = config->height;
                channel.attrib.bufSize = ALIGN_UP(config->height * config->width * 3 / 4, 64);
                channel.attrib.profile = config->profile;
                channel.attrib.byFrame = 1;
                channel.attrib.pic.width = config->width;
                channel.attrib.pic.height = config->height;
                ret = v4_venc.fnCreateChannel(index, &channel);
                HAL_INFO("v4_venc", "CreateChannel ch=%d retry(NORMALP+AVBR) -> ret=%#x (%s)\n",
                    index, ret, errstr(ret));
            }

            // 3) SMARTP + VBR (keep Smart GOP, drop AVBR)
            if (ret) {
                HAL_WARNING("v4_venc", "H264+ create failed, retrying with SMARTP + VBR\n");
                memset(&channel, 0, sizeof(channel));
                channel.attrib.codec = V4_VENC_CODEC_H264;
                channel.gop.mode = V4_VENC_GOPMODE_SMARTP;
                channel.gop.smartP.bgInterv = config->gop;
                channel.gop.smartP.bgQualDelta = 6;
                channel.gop.smartP.viQualDelta = 3;
                channel.rate.mode = V4_VENC_RATEMODE_H264VBR;
                channel.rate.h264Vbr = (v4_venc_rate_h26xbr){ .gop = config->gop,
                    .statTime = 1, .srcFps = config->framerate, .dstFps = config->framerate,
                    .maxBitrate = MAX(config->bitrate, config->maxBitrate) };
                channel.attrib.maxPic.width = config->width;
                channel.attrib.maxPic.height = config->height;
                channel.attrib.bufSize = ALIGN_UP(config->height * config->width * 3 / 4, 64);
                channel.attrib.profile = config->profile;
                channel.attrib.byFrame = 1;
                channel.attrib.pic.width = config->width;
                channel.attrib.pic.height = config->height;
                ret = v4_venc.fnCreateChannel(index, &channel);
                HAL_INFO("v4_venc", "CreateChannel ch=%d retry(SMARTP+VBR) -> ret=%#x (%s)\n",
                    index, ret, errstr(ret));
            }

            // 4) NORMALP + VBR (baseline fallback)
            if (ret) {
                HAL_WARNING("v4_venc", "H264+ create failed, retrying with NORMALP + VBR\n");
                memset(&channel, 0, sizeof(channel));
                channel.attrib.codec = V4_VENC_CODEC_H264;
                channel.gop.mode = V4_VENC_GOPMODE_NORMALP;
                channel.gop.normalP.ipQualDelta = config->gop / config->framerate;
                channel.rate.mode = V4_VENC_RATEMODE_H264VBR;
                channel.rate.h264Vbr = (v4_venc_rate_h26xbr){ .gop = config->gop,
                    .statTime = 1, .srcFps = config->framerate, .dstFps = config->framerate,
                    .maxBitrate = MAX(config->bitrate, config->maxBitrate) };
                channel.attrib.maxPic.width = config->width;
                channel.attrib.maxPic.height = config->height;
                channel.attrib.bufSize = ALIGN_UP(config->height * config->width * 3 / 4, 64);
                channel.attrib.profile = config->profile;
                channel.attrib.byFrame = 1;
                channel.attrib.pic.width = config->width;
                channel.attrib.pic.height = config->height;
                ret = v4_venc.fnCreateChannel(index, &channel);
                HAL_INFO("v4_venc", "CreateChannel ch=%d retry(NORMALP+VBR) -> ret=%#x (%s)\n",
                    index, ret, errstr(ret));
            }
        }
        if (ret)
            return ret;
    }

    {
        int count = -1;
        if (config->codec != HAL_VIDCODEC_JPG && 
            (ret = v4_venc.fnStartReceivingEx(index, &count))) {
            HAL_WARNING("v4_venc", "StartReceivingEx ch=%d -> ret=%#x (%s)\n",
                index, ret, errstr(ret));
            return ret;
        }
    }
    
    v4_state[index].payload = config->codec;

    return EXIT_SUCCESS;
}

int v4_video_destroy(char index)
{
    int ret;

    v4_state[index].enable = 0;
    v4_state[index].payload = HAL_VIDCODEC_UNSPEC;

    v4_venc.fnStopReceiving(index);

    {
        v4_sys_bind source = { .module = V4_SYS_MOD_VPSS, 
            .device = _v4_vpss_grp, .channel = index };
        v4_sys_bind dest = { .module = V4_SYS_MOD_VENC,
            .device = _v4_venc_dev, .channel = index };
        if (ret = v4_sys.fnUnbind(&source, &dest))
            return ret;
    }

    if (ret = v4_venc.fnDestroyChannel(index))
        return ret;
    
    if (ret = v4_vpss.fnDisableChannel(_v4_vpss_grp, index))
        return ret;

    return EXIT_SUCCESS;
}
    
int v4_video_destroy_all(void)
{
    int ret;

    for (char i = 0; i < V4_VENC_CHN_NUM; i++)
        if (v4_state[i].enable)
            if (ret = v4_video_destroy(i))
                return ret;

    return EXIT_SUCCESS;
}

void v4_video_request_idr(char index)
{
    v4_venc.fnRequestIdr(index, 1);
}

int v4_video_snapshot_grab(char index, hal_jpegdata *jpeg)
{
    int ret;

    if (ret = v4_channel_bind(index)) {
        HAL_DANGER("v4_venc", "Binding the encoder channel "
            "%d failed with %#x!\n", index, ret);
        goto abort;
    }

    unsigned int count = 1;
    if (ret = v4_venc.fnStartReceivingEx(index, &count)) {
        HAL_DANGER("v4_venc", "Requesting one frame "
            "%d failed with %#x!\n", index, ret);
        goto abort;
    }

    int fd = v4_venc.fnGetDescriptor(index);

    struct timeval timeout = { .tv_sec = 2, .tv_usec = 0 };
    fd_set readFds;
    FD_ZERO(&readFds);
    FD_SET(fd, &readFds);
    ret = select(fd + 1, &readFds, NULL, NULL, &timeout);
    if (ret < 0) {
        HAL_DANGER("v4_venc", "Select operation failed!\n");
        goto abort;
    } else if (ret == 0) {
        HAL_DANGER("v4_venc", "Capture stream timed out!\n");
        goto abort;
    }

    if (FD_ISSET(fd, &readFds)) {
        v4_venc_stat stat;
        if (ret = v4_venc.fnQuery(index, &stat)) {
            HAL_DANGER("v4_venc", "Querying the encoder channel "
                "%d failed with %#x!\n", index, ret);
            goto abort;
        }

        if (!stat.curPacks) {
            HAL_DANGER("v4_venc", "Current frame is empty, skipping it!\n");
            goto abort;
        }

        v4_venc_strm strm;
        memset(&strm, 0, sizeof(strm));
        strm.packet = (v4_venc_pack*)malloc(sizeof(v4_venc_pack) * stat.curPacks);
        if (!strm.packet) {
            HAL_DANGER("v4_venc", "Memory allocation on channel %d failed!\n", index);
            goto abort;
        }
        strm.count = stat.curPacks;

        if (ret = v4_venc.fnGetStream(index, &strm, stat.curPacks)) {
            HAL_DANGER("v4_venc", "Getting the stream on "
                "channel %d failed with %#x!\n", index, ret);
            free(strm.packet);
            strm.packet = NULL;
            goto abort;
        }

        {
            jpeg->jpegSize = 0;
            for (unsigned int i = 0; i < strm.count; i++) {
                v4_venc_pack *pack = &strm.packet[i];
                unsigned int packLen = pack->length - pack->offset;
                unsigned char *packData = pack->data + pack->offset;

                unsigned int newLen = jpeg->jpegSize + packLen;
                if (newLen > jpeg->length) {
                    jpeg->data = realloc(jpeg->data, newLen);
                    jpeg->length = newLen;
                }
                memcpy(jpeg->data + jpeg->jpegSize, packData, packLen);
                jpeg->jpegSize += packLen;
            }
        }

abort:
        v4_venc.fnFreeStream(index, &strm);
    }

    v4_venc.fnFreeDescriptor(index);

    v4_venc.fnStopReceiving(index);

    v4_channel_unbind(index);

    return ret;
}

void *v4_video_thread(void)
{
    int ret, maxFd = 0;

    for (int i = 0; i < V4_VENC_CHN_NUM; i++) {
        if (!v4_state[i].enable) continue;
        if (!v4_state[i].mainLoop) continue;

        ret = v4_venc.fnGetDescriptor(i);
        if (ret < 0) {
            HAL_DANGER("v4_venc", "Getting the encoder descriptor failed with %#x!\n", ret);
            return (void*)0;
        }
        v4_state[i].fileDesc = ret;

        if (maxFd <= v4_state[i].fileDesc)
            maxFd = v4_state[i].fileDesc;
    }

    v4_venc_stat stat;
    v4_venc_strm stream;
    struct timeval timeout;
    fd_set readFds;

    while (keepRunning) {
        FD_ZERO(&readFds);
        for(int i = 0; i < V4_VENC_CHN_NUM; i++) {
            if (!v4_state[i].enable) continue;
            if (!v4_state[i].mainLoop) continue;
            FD_SET(v4_state[i].fileDesc, &readFds);
        }

        timeout.tv_sec = 2;
        timeout.tv_usec = 0;
        ret = select(maxFd + 1, &readFds, NULL, NULL, &timeout);
        if (ret < 0) {
            HAL_DANGER("v4_venc", "Select operation failed!\n");
            break;
        } else if (ret == 0) {
            HAL_WARNING("v4_venc", "Main stream loop timed out!\n");
            continue;
        } else {
            for (int i = 0; i < V4_VENC_CHN_NUM; i++) {
                if (!v4_state[i].enable) continue;
                if (!v4_state[i].mainLoop) continue;
                if (FD_ISSET(v4_state[i].fileDesc, &readFds)) {
                    memset(&stream, 0, sizeof(stream));
                    
                    if (ret = v4_venc.fnQuery(i, &stat)) {
                        HAL_DANGER("v4_venc", "Querying the encoder channel "
                            "%d failed with %#x!\n", i, ret);
                        break;
                    }

                    if (!stat.curPacks) {
                        HAL_WARNING("v4_venc", "Current frame is empty, skipping it!\n");
                        continue;
                    }

                    stream.packet = (v4_venc_pack*)malloc(
                        sizeof(v4_venc_pack) * stat.curPacks);
                    if (!stream.packet) {
                        HAL_DANGER("v4_venc", "Memory allocation on channel %d failed!\n", i);
                        break;
                    }
                    stream.count = stat.curPacks;

                    if (ret = v4_venc.fnGetStream(i, &stream, 40)) {
                        HAL_DANGER("v4_venc", "Getting the stream on "
                            "channel %d failed with %#x!\n", i, ret);
                        break;
                    }

                    if (v4_vid_cb) {
                        hal_vidstream outStrm;
                        hal_vidpack outPack[stream.count];
                        outStrm.count = stream.count;
                        outStrm.seq = stream.sequence;
                        for (int j = 0; j < stream.count; j++) {
                            v4_venc_pack *pack = &stream.packet[j];
                            outPack[j].data = pack->data;
                            outPack[j].length = pack->length;
                            outPack[j].naluCnt = 1;
                            outPack[j].nalu[0].length = pack->length;
                            outPack[j].nalu[0].offset = pack->offset;
                            switch (v4_state[i].payload) {
                                case HAL_VIDCODEC_H264:
                                    outPack[j].nalu[0].type = pack->naluType.h264Nalu;
                                    break;
                                case HAL_VIDCODEC_H265:
                                    outPack[j].nalu[0].type = pack->naluType.h265Nalu;
                                    break;
                            }
                            outPack[j].offset = pack->offset;
                            outPack[j].timestamp = pack->timestamp;
                        }
                        outStrm.pack = outPack;
                        (*v4_vid_cb)(i, &outStrm);
                    }

                    if (ret = v4_venc.fnFreeStream(i, &stream)) {
                        HAL_WARNING("v4_venc", "Releasing the stream on "
                            "channel %d failed with %#x!\n", i, ret);
                    }
                    free(stream.packet);
                    stream.packet = NULL;
                }
            }
        }
    }

    HAL_INFO("v4_venc", "Shutting down encoding thread...\n");
}

void v4_system_deinit(void)
{
    v4_sys.fnExit();
    v4_vb.fnExit();

    v4_sensor_deinit();
}

int v4_system_init(char *snrConfig)
{
    int ret;

    printf("App built with headers v%s\n", V4_SYS_API);

    {
        v4_sys_ver version;
        v4_sys.fnGetVersion(&version);
        puts(version.version);
    }

    if (v4_parse_sensor_config(snrConfig, &v4_config) != CONFIG_OK)
        HAL_ERROR("v4_sys", "Can't load sensor config\n");

    if (ret = v4_sensor_init(v4_config.dll_file, v4_config.sensor_type))
        return ret;

    v4_sys.fnExit();
    v4_vb.fnExit();

    {
        v4_vb_pool pool;
        memset(&pool, 0, sizeof(pool)); 
        
        pool.count = 2;
        pool.comm[0].blockSize = v4_buffer_calculate_vi(v4_config.isp.size.width,
            v4_config.isp.size.height, V4_PIXFMT_RGB_BAYER_8BPP + v4_config.mipi.prec,
            V4_COMPR_NONE, 8);
        pool.comm[0].blockCnt = 3;
        pool.comm[1].blockSize = v4_buffer_calculate_venc(
            v4_config.isp.size.width, v4_config.isp.size.height, 
            V4_PIXFMT_YVU420SP, 8);
        pool.comm[1].blockCnt = 2;

        if (ret = v4_vb.fnConfigPool(&pool))
            return ret;
    }
    if (ret = v4_vb.fnInit())
        return ret;

    if (ret = v4_sys.fnInit())
        return ret;

    return EXIT_SUCCESS;
}

float v4_system_readtemp(void)
{
    char v4a_device = 0;
    int val, prep;
    float result = 0.0 / 0.0;

    if (EQUALS(chip, "Hi3516AV300") ||
        EQUALS(chip, "Hi3516DV300") ||
        EQUALS(chip, "Hi3516CV500"))
        v4a_device = 1;

    prep = v4a_device ? 0x60fa0000 : 0xc3200000;

    if (hal_registry(v4a_device ? 0x120300b4 : 0x120280b4, &val, OP_READ) && prep != val)
        hal_registry(v4a_device ? 0x120300b4 : 0x120280b4, &prep, OP_WRITE);

    if (!hal_registry(v4a_device ? 0x120300bc : 0x120280bc, &val, OP_READ))
        return result;

    result = val & ((1 << 10) - 1);
    return ((result - (v4a_device ? 136 : 117)) / (v4a_device ? 793 : 798)) * 165 - 40;
}

#endif