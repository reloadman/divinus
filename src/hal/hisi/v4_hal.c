#if defined(__arm__) && !defined(__ARM_PCS_VFP)

#include "v4_hal.h"
#include <ctype.h>
#include <limits.h>
#include <stdbool.h>
#include <strings.h>
#include <pthread.h>

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

// Forward decl (used by delayed apply thread)
static int v4_iq_apply(const char *path, int pipe);

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

int v4_audio_init(int samplerate)
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
    int ret;

    for (char i = 0; i < V4_VENC_CHN_NUM; i++) {
        v4_venc_para param;
        if (!v4_state[i].enable) continue;
        if (ret = v4_venc.fnGetChannelParam(i, &param))
            return ret;
        param.grayscaleOn = enable;
        if (ret = v4_venc.fnSetChannelParam(i, &param))
            return ret;
    }

    return EXIT_SUCCESS;
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

    if (ret = v4_isp.fnRun(_v4_isp_dev))
        HAL_DANGER("v4_isp", "Operation failed with %#x!\n", ret);
    HAL_INFO("v4_isp", "Shutting down ISP thread...\n");
}

// ---- IQ (scene_auto-style) INI loader (Goke/HiSilicon v4) ----
typedef int HI_BOOL;
typedef unsigned char HI_U8;
typedef unsigned short HI_U16;
typedef unsigned int HI_U32;
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

// Minimal ModuleControl union (we only need u64Key bitmask).
// In Hi/GK SDK, bit 8 is bitBypassDRC.
typedef union {
    HI_U64 u64Key;
} ISP_MODULE_CTRL_U;

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

static int v4_iq_apply_static_ae(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetExposureAttr || !v4_isp.fnSetExposureAttr) {
        HAL_INFO("v4_iq", "AE: API not available, skipping\n");
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
    if (parse_int(ini, "static_ae", "AERunInterval", 1, 255, &val) == CONFIG_OK)
        exp.u8AERunInterval = (HI_U8)val;
    if (parse_int(ini, "static_ae", "AERouteExValid", 0, 1, &val) == CONFIG_OK)
        exp.bAERouteExValid = (HI_BOOL)val;
    if (parse_int(ini, "static_ae", "AutoSysGainMax", 0, INT_MAX, &val) == CONFIG_OK)
        exp.stAuto.stSysGainRange.u32Max = (HI_U32)val;
    if (parse_int(ini, "static_ae", "AutoSpeed", 0, 255, &val) == CONFIG_OK)
        exp.stAuto.u8Speed = (HI_U8)val;
    if (parse_int(ini, "static_ae", "AutoTolerance", 0, 255, &val) == CONFIG_OK)
        exp.stAuto.u8Tolerance = (HI_U8)val;
    if (parse_int(ini, "static_ae", "AutoBlackDelayFrame", 0, 65535, &val) == CONFIG_OK)
        exp.stAuto.stAEDelayAttr.u16BlackDelayFrame = (HI_U16)val;
    if (parse_int(ini, "static_ae", "AutoWhiteDelayFrame", 0, 65535, &val) == CONFIG_OK)
        exp.stAuto.stAEDelayAttr.u16WhiteDelayFrame = (HI_U16)val;

    ret = v4_isp.fnSetExposureAttr(pipe, &exp);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetExposureAttr failed with %#x\n", ret);
    } else {
        ISP_EXPOSURE_ATTR_S rb;
        memset(&rb, 0, sizeof(rb));
        int gr = v4_isp.fnGetExposureAttr(pipe, &rb);
        if (!gr) {
            HAL_INFO("v4_iq", "AE: applied (runInt=%u routeExValid=%d)\n",
                (unsigned)rb.u8AERunInterval, (int)rb.bAERouteExValid);
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

    ISP_AE_ROUTE_EX_S route;
    memset(&route, 0, sizeof(route));
    int ret = v4_isp.fnGetAERouteAttrEx(pipe, &route);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetAERouteAttrEx failed with %#x\n", ret);
        return ret;
    }

    int total = 0;
    if (parse_int(ini, "static_aerouteex", "TotalNum", 0, ISP_AE_ROUTE_EX_MAX_NODES, &total) != CONFIG_OK) {
        HAL_INFO("v4_iq", "AE route-ex: no static_aerouteex/TotalNum, skipping\n");
        return EXIT_SUCCESS;
    }

    char buf[1024];
    HI_U32 ints[ISP_AE_ROUTE_EX_MAX_NODES] = {0};
    HI_U32 again[ISP_AE_ROUTE_EX_MAX_NODES] = {0};
    HI_U32 dgain[ISP_AE_ROUTE_EX_MAX_NODES] = {0};
    HI_U32 ispdgain[ISP_AE_ROUTE_EX_MAX_NODES] = {0};

    if (parse_param_value(ini, "static_aerouteex", "RouteEXIntTime", buf) == CONFIG_OK)
        v4_iq_parse_csv_u32(buf, ints, ISP_AE_ROUTE_EX_MAX_NODES);
    if (parse_param_value(ini, "static_aerouteex", "RouteEXAGain", buf) == CONFIG_OK)
        v4_iq_parse_csv_u32(buf, again, ISP_AE_ROUTE_EX_MAX_NODES);
    if (parse_param_value(ini, "static_aerouteex", "RouteEXDGain", buf) == CONFIG_OK)
        v4_iq_parse_csv_u32(buf, dgain, ISP_AE_ROUTE_EX_MAX_NODES);
    if (parse_param_value(ini, "static_aerouteex", "RouteEXISPDGain", buf) == CONFIG_OK)
        v4_iq_parse_csv_u32(buf, ispdgain, ISP_AE_ROUTE_EX_MAX_NODES);

    route.u32TotalNum = (HI_U32)total;
    for (int i = 0; i < total && i < ISP_AE_ROUTE_EX_MAX_NODES; i++) {
        if (ints[i]) route.astRouteExNode[i].u32IntTime = ints[i];
        if (again[i]) route.astRouteExNode[i].u32Again = again[i];
        if (dgain[i]) route.astRouteExNode[i].u32Dgain = dgain[i];
        if (ispdgain[i]) route.astRouteExNode[i].u32IspDgain = ispdgain[i];
    }

    ret = v4_isp.fnSetAERouteAttrEx(pipe, &route);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetAERouteAttrEx failed with %#x\n", ret);
    } else {
        ISP_AE_ROUTE_EX_S rb;
        memset(&rb, 0, sizeof(rb));
        int gr = v4_isp.fnGetAERouteAttrEx(pipe, &rb);
        if (!gr && rb.u32TotalNum > 0) {
            HAL_INFO("v4_iq", "AE route-ex: applied (%u nodes) first={t=%u again=%u dgain=%u ispd=%u}\n",
                (unsigned)rb.u32TotalNum,
                (unsigned)rb.astRouteExNode[0].u32IntTime,
                (unsigned)rb.astRouteExNode[0].u32Again,
                (unsigned)rb.astRouteExNode[0].u32Dgain,
                (unsigned)rb.astRouteExNode[0].u32IspDgain);
        } else {
            HAL_INFO("v4_iq", "AE route-ex: applied (%d nodes)\n", total);
        }
    }
    return ret;
}

static int v4_iq_apply_static_ccm(struct IniConfig *ini, int pipe) {
    if (!v4_isp.fnGetCCMAttr || !v4_isp.fnSetCCMAttr) {
        HAL_INFO("v4_iq", "CCM: API not available, skipping\n");
        return EXIT_SUCCESS;
    }

    ISP_COLORMATRIX_ATTR_S ccm;
    memset(&ccm, 0, sizeof(ccm));
    int ret = v4_isp.fnGetCCMAttr(pipe, &ccm);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetCCMAttr failed with %#x\n", ret);
        return ret;
    }

    int val;
    if (parse_int(ini, "static_ccm", "CCMOpType", 0, 1, &val) == CONFIG_OK)
        ccm.enOpType = (ISP_OP_TYPE_E)val;
    if (parse_int(ini, "static_ccm", "ISOActEn", 0, 1, &val) == CONFIG_OK)
        ccm.stAuto.bISOActEn = (HI_BOOL)val;
    if (parse_int(ini, "static_ccm", "TempActEn", 0, 1, &val) == CONFIG_OK)
        ccm.stAuto.bTempActEn = (HI_BOOL)val;

    // Manual CCM (optional)
    char buf[1024];
    if (parse_param_value(ini, "static_ccm", "ManualCCMTable", buf) == CONFIG_OK)
        v4_iq_parse_csv_u16(buf, ccm.stManual.au16CCM, CCM_MATRIX_SIZE);

    // Auto CCM tables
    int total = 0;
    if (parse_int(ini, "static_ccm", "TotalNum", 0, CCM_MATRIX_NUM, &total) == CONFIG_OK) {
        ccm.stAuto.u16CCMTabNum = (HI_U16)total;

        if (parse_param_value(ini, "static_ccm", "AutoColorTemp", buf) == CONFIG_OK) {
            HI_U32 temps[CCM_MATRIX_NUM] = {0};
            int n = v4_iq_parse_csv_u32(buf, temps, CCM_MATRIX_NUM);
            for (int i = 0; i < n && i < total; i++)
                ccm.stAuto.astCCMTab[i].u16ColorTemp = (HI_U16)temps[i];
        }

        for (int i = 0; i < total && i < CCM_MATRIX_NUM; i++) {
            char key[32];
            snprintf(key, sizeof(key), "AutoCCMTable_%d", i);
            if (parse_param_value(ini, "static_ccm", key, buf) == CONFIG_OK)
                v4_iq_parse_csv_u16(buf, ccm.stAuto.astCCMTab[i].au16CCM, CCM_MATRIX_SIZE);
        }
    }

    ret = v4_isp.fnSetCCMAttr(pipe, &ccm);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetCCMAttr failed with %#x\n", ret);
    } else {
        ISP_COLORMATRIX_ATTR_S rb;
        memset(&rb, 0, sizeof(rb));
        int gr = v4_isp.fnGetCCMAttr(pipe, &rb);
        if (!gr) {
            HAL_INFO("v4_iq", "CCM: applied (opType=%d manualCCM[0..2]=%u,%u,%u)\n",
                (int)rb.enOpType,
                (unsigned)rb.stManual.au16CCM[0],
                (unsigned)rb.stManual.au16CCM[1],
                (unsigned)rb.stManual.au16CCM[2]);
        } else {
            HAL_INFO("v4_iq", "CCM: applied\n");
        }
    }
    return ret;
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
    int sec_s = 0, sec_e = 0;
    if (section_pos(ini, "static_nr", &sec_s, &sec_e) != CONFIG_OK) {
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
    if (parse_int(ini, "static_nr", "Enable", 0, 1, &val) == CONFIG_OK)
        nr.bEnable = (HI_BOOL)val;
    nr.enOpType = OP_TYPE_AUTO;

    char buf[512];
    if (parse_param_value(ini, "static_nr", "FineStr", buf) == CONFIG_OK)
        v4_iq_parse_csv_u8(buf, nr.stAuto.au8FineStr, ISP_AUTO_ISO_STRENGTH_NUM);
    {
        HI_U32 tmp[ISP_AUTO_ISO_STRENGTH_NUM];
        if (parse_param_value(ini, "static_nr", "CoringWgt", buf) == CONFIG_OK) {
            int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
            for (int i = 0; i < n; i++)
                nr.stAuto.au16CoringWgt[i] = (HI_U16)tmp[i];
        }
    }

    ret = v4_isp.fnSetNRAttr(pipe, &nr);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_SetNRAttr failed with %#x\n", ret);
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

    ISP_GAMMA_ATTR_S gamma;
    memset(&gamma, 0, sizeof(gamma));
    int ret = v4_isp.fnGetGammaAttr(pipe, &gamma);
    if (ret) {
        HAL_WARNING("v4_iq", "HI_MPI_ISP_GetGammaAttr failed with %#x\n", ret);
        return ret;
    }

    // This IQ format uses [dynamic_gamma] with Table_0..; apply Table_0 as a user-defined gamma curve at init.
    HI_U32 tmp[GAMMA_NODE_NUM];
    memset(tmp, 0, sizeof(tmp));
    int n = v4_iq_parse_multiline_u32(ini, "dynamic_gamma", "Table_0", tmp, GAMMA_NODE_NUM);
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
                HAL_INFO("v4_iq", "Gamma: applied (type=%d node0=%u nodeLast=%u)\n",
                    (int)rb.enCurveType,
                    (unsigned)rb.u16Table[0],
                    (unsigned)rb.u16Table[GAMMA_NODE_NUM - 1]);
            } else {
                HAL_INFO("v4_iq", "Gamma: applied (%d nodes)\n", n);
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
    int sec_s = 0, sec_e = 0;
    if (section_pos(ini, "static_sharpen", &sec_s, &sec_e) != CONFIG_OK) {
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
    if (parse_int(ini, "static_sharpen", "Enable", 0, 1, &val) == CONFIG_OK)
        shp.bEnable = (HI_BOOL)val;
    shp.enOpType = OP_TYPE_AUTO;

    char buf[1024];
    // Auto luma weights: AutoLumaWgt_0..31 (each has 16 values)
    for (int i = 0; i < ISP_SHARPEN_LUMA_NUM; i++) {
        char key[32];
        snprintf(key, sizeof(key), "AutoLumaWgt_%d", i);
        if (parse_param_value(ini, "static_sharpen", key, buf) == CONFIG_OK)
            v4_iq_parse_csv_u8(buf, shp.stAuto.au8LumaWgt[i], ISP_AUTO_ISO_STRENGTH_NUM);
    }

    // Auto texture/edge strength: AutoTextureStr_0..31, AutoEdgeStr_0..31
    for (int i = 0; i < ISP_SHARPEN_GAIN_NUM; i++) {
        char key[32];
        snprintf(key, sizeof(key), "AutoTextureStr_%d", i);
        if (parse_param_value(ini, "static_sharpen", key, buf) == CONFIG_OK) {
            HI_U32 tmp[ISP_AUTO_ISO_STRENGTH_NUM];
            int n = v4_iq_parse_csv_u32(buf, tmp, ISP_AUTO_ISO_STRENGTH_NUM);
            for (int j = 0; j < n; j++)
                shp.stAuto.au16TextureStr[i][j] = (HI_U16)tmp[j];
        }
        snprintf(key, sizeof(key), "AutoEdgeStr_%d", i);
        if (parse_param_value(ini, "static_sharpen", key, buf) == CONFIG_OK) {
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
    bool doStaticDRC = true, doStaticNR = true, doStaticSharpen = true;
    bool doGamma = true;
    parse_bool(&ini, "module_state", "bStaticAE", &doStaticAE);
    parse_bool(&ini, "module_state", "bStaticCCM", &doStaticCCM);
    parse_bool(&ini, "module_state", "bStaticSaturation", &doStaticSat);
    parse_bool(&ini, "module_state", "bStaticDRC", &doStaticDRC);
    parse_bool(&ini, "module_state", "bStaticNr", &doStaticNR);
    parse_bool(&ini, "module_state", "bStaticSharpen", &doStaticSharpen);
    // Gamma in this IQ is described under "dynamic_gamma"
    parse_bool(&ini, "module_state", "bDynamicGamma", &doGamma);

    int ret = EXIT_SUCCESS;
    HAL_INFO("v4_iq", "Loading IQ config '%s'\n", path);
    if (doStaticAE) {
        int r = v4_iq_apply_static_ae(&ini, pipe);
        if (r) ret = r;
        r = v4_iq_apply_static_aerouteex(&ini, pipe);
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
    attrib.overlay.bgAlpha = 0;
    attrib.overlay.fgAlpha = opacity >> 1;
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
    if (config->codec == HAL_VIDCODEC_JPG || config->codec == HAL_VIDCODEC_MJPG) {
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
    channel.attrib.bufSize = ALIGN_UP(config->height * config->width * 3 / 4, 64);
    if (channel.attrib.codec == V4_VENC_CODEC_H264)
        channel.attrib.profile = MAX(config->profile, 2);
    channel.attrib.byFrame = 1;
    channel.attrib.pic.width = config->width;
    channel.attrib.pic.height = config->height;

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