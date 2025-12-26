#if defined(__ARM_PCS_VFP)

#include "i6_hal.h"

i6_aud_impl  i6_aud;
i6_isp_impl  i6_isp;
i6_rgn_impl  i6_rgn;
i6_snr_impl  i6_snr;
i6_sys_impl  i6_sys;
i6_venc_impl i6_venc;
i6_vif_impl  i6_vif;
i6_vpe_impl  i6_vpe;

hal_chnstate i6_state[I6_VENC_CHN_NUM] = {0};
int (*i6_aud_cb)(hal_audframe*);
int (*i6_vid_cb)(char, hal_vidstream*);

i6_snr_pad _i6_snr_pad;
i6_snr_plane _i6_snr_plane;
char _i6_snr_framerate, _i6_snr_hdr, _i6_snr_index, _i6_snr_profile;

char _i6_aud_chn = 0;
char _i6_aud_dev = 0;
char _i6_isp_chn = 0;
char _i6_venc_port = 0;
char _i6_vif_chn = 0;
char _i6_vif_dev = 0;
char _i6_vif_port = 0;
char _i6_vpe_chn = 0;
char _i6_vpe_dev = 0;
char _i6_vpe_port = 0;

// Track where each region is attached (VPE vs VENC) to detach correctly and allow fallback.
#define I6_RGN_MAX_HANDLES 16
static i6_sys_mod _i6_rgn_mod[I6_RGN_MAX_HANDLES] = {
    [0 ... I6_RGN_MAX_HANDLES - 1] = I6_SYS_MOD_VPE
};

static inline i6_sys_mod i6_rgn_mod_get(char handle) {
    if ((unsigned char)handle < I6_RGN_MAX_HANDLES)
        return _i6_rgn_mod[(unsigned char)handle];
    return I6_SYS_MOD_VPE;
}

static inline void i6_rgn_mod_set(char handle, i6_sys_mod mod) {
    if ((unsigned char)handle < I6_RGN_MAX_HANDLES)
        _i6_rgn_mod[(unsigned char)handle] = mod;
}

static int i6_rgn_fill_dest(i6_sys_bind *dest, i6_sys_mod mod, char port) {
    if (!dest)
        return -1;
    memset(dest, 0, sizeof(*dest));
    dest->module = mod;
    switch (mod) {
        case I6_SYS_MOD_VPE:
            dest->device = _i6_vpe_dev;
            dest->channel = _i6_vpe_chn;
            // Region overlay attaches to VPE output port (fixed), not per-stream index.
            dest->port = _i6_vpe_port;
            return 0;
        case I6_SYS_MOD_VENC: {
            // VENC device is fixed to 0 on i6; channel selects stream.
            dest->device = 0;
            // Attach all OSD regions to main VENC channel 0.
            dest->channel = 0;
            dest->port = _i6_venc_port;
            return 0;
        }
        default:
            return -1;
    }
}

static void i6_rgn_detach(char handle, i6_sys_mod mod, char port) {
    i6_sys_bind dest;
    if (i6_rgn_fill_dest(&dest, mod, port) == 0)
        i6_rgn.fnDetachChannel(handle, &dest);
}

void i6_hal_deinit(void)
{
    i6_vpe_unload(&i6_vpe);
    i6_vif_unload(&i6_vif);
    i6_venc_unload(&i6_venc);
    i6_snr_unload(&i6_snr);
    i6_rgn_unload(&i6_rgn);
    i6_isp_unload(&i6_isp);
    i6_aud_unload(&i6_aud);
    i6_sys_unload(&i6_sys);
}

int i6_hal_init(void)
{
    int ret;

    if (ret = i6_sys_load(&i6_sys))
        return ret;
    if (ret = i6_aud_load(&i6_aud))
        return ret;
    if (ret = i6_isp_load(&i6_isp))
        return ret;
    if (ret = i6_rgn_load(&i6_rgn))
        return ret;
    if (ret = i6_snr_load(&i6_snr))
        return ret;
    if (ret = i6_venc_load(&i6_venc))
        return ret;
    if (ret = i6_vif_load(&i6_vif))
        return ret;
    if (ret = i6_vpe_load(&i6_vpe))
        return ret;

    return EXIT_SUCCESS;
}

void i6_audio_deinit(void)
{
    i6_aud.fnDisableChannel(_i6_aud_dev, _i6_aud_chn);

    i6_aud.fnDisableDevice(_i6_aud_dev);
}

int i6_audio_init(int samplerate, int gain)
{
    int ret;

    {
        i6_aud_cnf config;
        config.rate = samplerate;
        config.bit24On = 0;
        config.intf = I6_AUD_INTF_I2S_SLAVE;
        config.sound = I6_AUD_SND_MONO;
        config.frmNum = 0;
        config.packNumPerFrm = 640;
        config.codecChnNum = 0;
        config.chnNum = 1;
        config.i2s.leftJustOn = 0;
        config.i2s.clock = I6_AUD_CLK_OFF;
        config.i2s.syncRxClkOn = 0;
        config.i2s.tdmSlotNum = 0;
        config.i2s.bit24On = 0;
        if (ret = i6_aud.fnSetDeviceConfig(_i6_aud_dev, &config))
            return ret;
    }
    if (ret = i6_aud.fnEnableDevice(_i6_aud_dev))
        return ret;
    
    if (ret = i6_aud.fnEnableChannel(_i6_aud_dev, _i6_aud_chn))
        return ret;
    if (ret = i6_aud.fnSetVolume(_i6_aud_dev, _i6_aud_chn, gain))
        return ret;

    {
        i6_sys_bind bind = { .module = I6_SYS_MOD_AI, 
            .device = _i6_aud_dev, .channel = _i6_aud_chn };
        if (ret = i6_sys.fnSetOutputDepth(&bind, 2, 4))
            return ret;
    }

    return EXIT_SUCCESS;
}

void *i6_audio_thread(void)
{
    int ret;

    i6_aud_frm frame;
    memset(&frame, 0, sizeof(frame));

    while (keepRunning  && audioOn) {
        if (ret = i6_aud.fnGetFrame(_i6_aud_dev, _i6_aud_chn, 
            &frame, NULL, 128)) {
            HAL_WARNING("i6_aud", "Getting the frame failed "
                "with %#x!\n", ret);
            continue;
        }

        if (i6_aud_cb) {
            hal_audframe outFrame;
            outFrame.channelCnt = 1;
            outFrame.data[0] = frame.addr[0];
            outFrame.length[0] = frame.length;
            outFrame.seq = frame.sequence;
            outFrame.timestamp = frame.timestamp;
            (i6_aud_cb)(&outFrame);
        }

        if (ret = i6_aud.fnFreeFrame(_i6_aud_dev, _i6_aud_chn,
            &frame, NULL)) {
            HAL_WARNING("i6_aud", "Releasing the frame failed"
                " with %#x!\n", ret);
        }
    }
    HAL_INFO("i6_aud", "Shutting down capture thread...\n");
}

int i6_channel_bind(char index, char framerate)
{
    int ret;

    if (ret = i6_vpe.fnEnablePort(_i6_vpe_chn, index))
        return ret;

    {
        unsigned int device;
        if (ret = i6_venc.fnGetChannelDeviceId(index, &device))
            return ret;
        i6_sys_bind source = { .module = I6_SYS_MOD_VPE, 
            .device = _i6_vpe_dev, .channel = _i6_vpe_chn, .port = index };
        i6_sys_bind dest = { .module = I6_SYS_MOD_VENC,
            .device = device, .channel = index, .port = _i6_venc_port };
        if (ret = i6_sys.fnBindExt(&source, &dest, framerate, framerate,
            I6_SYS_LINK_FRAMEBASE, 0))
            return ret;
    }

    return EXIT_SUCCESS;
}

int i6_channel_create(char index, short width, short height, char jpeg)
{
    i6_vpe_port port;
    port.output.width = width;
    port.output.height = height;
    port.mirror = 0;
    port.flip = 0;
    port.compress = I6_COMPR_NONE;
    port.pixFmt = jpeg ? I6_PIXFMT_YUV422_YUYV : I6_PIXFMT_YUV420SP;

    return i6_vpe.fnSetPortConfig(_i6_vpe_chn, index, &port);
}

int i6_channel_grayscale(char enable)
{
    return i6_isp.fnSetColorToGray(0, &enable);
}

int i6_channel_unbind(char index)
{
    int ret;

    if (ret = i6_vpe.fnDisablePort(_i6_vpe_chn, index))
        return ret;

    {
        unsigned int device;
        if (ret = i6_venc.fnGetChannelDeviceId(index, &device))
            return ret;
        i6_sys_bind source = { .module = I6_SYS_MOD_VPE, 
            .device = _i6_vpe_dev, .channel = _i6_vpe_chn, .port = index };
        i6_sys_bind dest = { .module = I6_SYS_MOD_VENC,
            .device = device, .channel = index, .port = _i6_venc_port };
        if (ret = i6_sys.fnUnbind(&source, &dest))
            return ret;
    }

    return EXIT_SUCCESS; 
}

int i6_config_load(char *path)
{
    return i6_isp.fnLoadChannelConfig(_i6_isp_chn, path, 1234);
}

int i6_pipeline_create(char sensor, short width, short height, char mirror, char flip, char framerate)
{
    int ret;

    _i6_snr_index = sensor;
    _i6_snr_profile = -1;

    {
        unsigned int count;
        i6_snr_res resolution;
        if (ret = i6_snr.fnSetPlaneMode(_i6_snr_index, 0))
            return ret;

        if (ret = i6_snr.fnGetResolutionCount(_i6_snr_index, &count))
            return ret;
        for (char i = 0; i < count; i++) {
            if (ret = i6_snr.fnGetResolution(_i6_snr_index, i, &resolution))
                return ret;

            if (width > resolution.crop.width ||
                height > resolution.crop.height ||
                framerate > resolution.maxFps)
                continue;
        
            _i6_snr_profile = i;
            if (ret = i6_snr.fnSetResolution(_i6_snr_index, _i6_snr_profile))
                return ret;
            _i6_snr_framerate = framerate;
            if (ret = i6_snr.fnSetFramerate(_i6_snr_index, _i6_snr_framerate))
                return ret;
            break;
        }
        if (_i6_snr_profile < 0)
            return EXIT_FAILURE;
    }

    if (ret = i6_snr.fnSetOrientation(_i6_snr_index, mirror, flip))
        return ret;

    if (ret = i6_snr.fnGetPadInfo(_i6_snr_index, &_i6_snr_pad))
        return ret;
    if (ret = i6_snr.fnGetPlaneInfo(_i6_snr_index, 0, &_i6_snr_plane))
        return ret;
    if (ret = i6_snr.fnEnable(_i6_snr_index))
        return ret;

    {
        i6_vif_dev device;
        memset(&device, 0, sizeof(device));
        device.intf = _i6_snr_pad.intf;
        device.work = device.intf == I6_INTF_BT656 ? 
            I6_VIF_WORK_1MULTIPLEX : I6_VIF_WORK_RGB_REALTIME;
        device.hdr = I6_HDR_OFF;
        if (device.intf == I6_INTF_MIPI) {
            device.edge = I6_EDGE_DOUBLE;
            device.input = _i6_snr_pad.intfAttr.mipi.input;
        } else if (device.intf == I6_INTF_BT656) {
            device.edge = _i6_snr_pad.intfAttr.bt656.edge;
            device.sync = _i6_snr_pad.intfAttr.bt656.sync;
        }
        if (ret = i6_vif.fnSetDeviceConfig(_i6_vif_dev, &device))
            return ret;
    }
    if (ret = i6_vif.fnEnableDevice(_i6_vif_dev))
        return ret;

    {
        i6_vif_port port;
        port.capt = _i6_snr_plane.capt;
        port.dest.height = _i6_snr_plane.capt.height;
        port.dest.width = _i6_snr_plane.capt.width;
        port.field = 0;
        port.interlaceOn = 0;
        port.pixFmt = (i6_common_pixfmt)(_i6_snr_plane.bayer > I6_BAYER_END ? 
            _i6_snr_plane.pixFmt : (I6_PIXFMT_RGB_BAYER + _i6_snr_plane.precision * I6_BAYER_END + _i6_snr_plane.bayer));
        port.frate = I6_VIF_FRATE_FULL;
        port.frameLineCnt = 0;
        if (ret = i6_vif.fnSetPortConfig(_i6_vif_chn, _i6_vif_port, &port))
            return ret;
    }
    if (ret = i6_vif.fnEnablePort(_i6_vif_chn, _i6_vif_port))
        return ret;

    if (series == 0xF1) {
        i6e_vpe_chn channel;
        memset(&channel, 0, sizeof(channel));
        channel.capt.height = _i6_snr_plane.capt.height;
        channel.capt.width = _i6_snr_plane.capt.width;
        channel.pixFmt = (i6_common_pixfmt)(_i6_snr_plane.bayer > I6_BAYER_END ? 
            _i6_snr_plane.pixFmt : (I6_PIXFMT_RGB_BAYER + _i6_snr_plane.precision * I6_BAYER_END + _i6_snr_plane.bayer));
        channel.hdr = I6_HDR_OFF;
        channel.sensor = (i6_vpe_sens)(_i6_snr_index + 1);
        channel.mode = I6_VPE_MODE_REALTIME;
        if (ret = i6_vpe.fnCreateChannel(_i6_vpe_chn, (i6_vpe_chn*)&channel))
            return ret;

        i6e_vpe_para param;
        memset(&param, 0, sizeof(param));
        param.hdr = I6_HDR_OFF;
        param.level3DNR = 1;
        param.mirror = 0;
        param.flip = 0;
        param.lensAdjOn = 0;
        if (ret = i6_vpe.fnSetChannelParam(_i6_vpe_chn, (i6_vpe_para*)&param))
            return ret;
    } else {
        i6_vpe_chn channel;
        memset(&channel, 0, sizeof(channel));
        channel.capt.height = _i6_snr_plane.capt.height;
        channel.capt.width = _i6_snr_plane.capt.width;
        channel.pixFmt = (i6_common_pixfmt)(_i6_snr_plane.bayer > I6_BAYER_END ? 
            _i6_snr_plane.pixFmt : (I6_PIXFMT_RGB_BAYER + _i6_snr_plane.precision * I6_BAYER_END + _i6_snr_plane.bayer));
        channel.hdr = I6_HDR_OFF;
        channel.sensor = (i6_vpe_sens)(_i6_snr_index + 1);
        channel.mode = I6_VPE_MODE_REALTIME;
        if (ret = i6_vpe.fnCreateChannel(_i6_vpe_chn, &channel))
            return ret;

        i6_vpe_para param;
        memset(&param, 0, sizeof(param));
        param.hdr = I6_HDR_OFF;
        param.level3DNR = 1;
        param.mirror = 0;
        param.flip = 0;
        param.lensAdjOn = 0;
        if (ret = i6_vpe.fnSetChannelParam(_i6_vpe_chn, &param))
            return ret;
    }
    if (ret = i6_vpe.fnStartChannel(_i6_vpe_chn))
        return ret;

    {
        i6_sys_bind source = { .module = I6_SYS_MOD_VIF, 
            .device = _i6_vif_dev, .channel = _i6_vif_chn, .port = _i6_vif_port };
        i6_sys_bind dest = { .module = I6_SYS_MOD_VPE,
            .device = _i6_vpe_dev, .channel = _i6_vpe_chn, .port = _i6_vpe_port };
        return i6_sys.fnBindExt(&source, &dest, _i6_snr_framerate, _i6_snr_framerate,
            I6_SYS_LINK_REALTIME, 0);
    }

    return EXIT_SUCCESS;
}

int i6_set_orientation(char mirror, char flip)
{
    return i6_snr.fnSetOrientation(_i6_snr_index, mirror, flip);
}

void i6_pipeline_destroy(void)
{
    for (char i = 0; i < 4; i++)
        i6_vpe.fnDisablePort(_i6_vpe_chn, i);

    {
        i6_sys_bind source = { .module = I6_SYS_MOD_VIF, 
            .device = _i6_vif_dev, .channel = _i6_vif_chn, .port = _i6_vif_port };
        i6_sys_bind dest = { .module = I6_SYS_MOD_VPE,
            .device = _i6_vif_dev, .channel = _i6_vpe_chn, .port = _i6_vpe_port };
        i6_sys.fnUnbind(&source, &dest);
    }

    i6_vpe.fnStopChannel(_i6_vpe_chn);
    i6_vpe.fnDestroyChannel(_i6_vpe_chn);

    i6_vif.fnDisablePort(_i6_vif_chn, 0);
    i6_vif.fnDisableDevice(_i6_vif_dev);

    i6_snr.fnDisable(_i6_snr_index);
}

int i6_region_create(char handle, hal_rect rect, short opacity)
{
    // Backwards compatible wrapper: no background alpha.
    return i6_region_create_ex(handle, rect, opacity, 0);
}

int i6_region_create_ex(char handle, hal_rect rect, short fg_opacity, short bg_opacity)
{
    int ret;
    unsigned int h = (unsigned int)((unsigned char)handle + 1); // avoid handle 0

    i6_sys_bind dest = { .module = I6_SYS_MOD_VENC, .port = _i6_venc_port };
    i6_rgn_cnf region, regionCurr;
    i6_rgn_chn attrib, attribCurr;

    region.type = I6_RGN_TYPE_OSD;
    region.pixFmt = I6_RGN_PIXFMT_ARGB1555;
    region.size.width = rect.width;
    region.size.height = rect.height;

    if (i6_rgn.fnGetRegionConfig(h, &regionCurr)) {
        HAL_INFO("i6_rgn", "Creating region %d...\n", handle);
        if (ret = i6_rgn.fnCreateRegion(h, &region))
            return ret;
    } else if (regionCurr.type != region.type ||
        regionCurr.size.height != region.size.height || 
        regionCurr.size.width != region.size.width) {
        HAL_INFO("i6_rgn", "Parameters are different, recreating "
            "region %d...\n", handle);
        // Detach from all targets: VENC channels and VPE output ports.
        for (char i = 0; i < I6_VENC_CHN_NUM; i++) {
            unsigned int dev = 0;
            if (i6_venc.fnGetChannelDeviceId(i, &dev) != 0)
                dev = 0;
            dest.device = dev;
            dest.channel = i;
            i6_rgn.fnDetachChannel(h, &dest);
        }
        {
            i6_sys_bind vpe = { .module = I6_SYS_MOD_VPE,
                .device = _i6_vpe_dev, .channel = _i6_vpe_chn };
            for (char i = 0; i < I6_VENC_CHN_NUM; i++) {
                vpe.port = i; // VPE output port corresponds to encoder channel index
                i6_rgn.fnDetachChannel(h, &vpe);
            }
        }
        i6_rgn.fnDestroyRegion(h);
        if (ret = i6_rgn.fnCreateRegion(h, &region))
            return ret;
    }

    // Pick a representative enabled/allowed channel for config comparison.
    int have_ref = 0;
    for (char i = 0; i < I6_VENC_CHN_NUM; i++) {
        if (!i6_state[i].enable) continue;
        if (!hal_osd_is_allowed_for_channel(&i6_state[i])) continue;
        unsigned int dev = 0;
        if (i6_venc.fnGetChannelDeviceId(i, &dev) != 0)
            dev = 0;
        dest.device = dev;
        dest.channel = i;
        have_ref = 1;
        break;
    }

    if (!have_ref || i6_rgn.fnGetChannelConfig(h, &dest, &attribCurr)) {
        HAL_INFO("i6_rgn", "Attaching region %d...\n", handle);
    } else if (attribCurr.point.x != rect.x || attribCurr.point.y != rect.y ||
        attribCurr.osd.bgFgAlpha[0] != bg_opacity ||
        attribCurr.osd.bgFgAlpha[1] != fg_opacity) {
        HAL_INFO("i6_rgn", "Parameters are different, reattaching "
            "region %d...\n", handle);
        for (char i = 0; i < I6_VENC_CHN_NUM; i++) {
            unsigned int dev = 0;
            if (i6_venc.fnGetChannelDeviceId(i, &dev) != 0)
                dev = 0;
            dest.device = dev;
            dest.channel = i;
            i6_rgn.fnDetachChannel(h, &dest);
        }
        {
            i6_sys_bind vpe = { .module = I6_SYS_MOD_VPE,
                .device = _i6_vpe_dev, .channel = _i6_vpe_chn };
            for (char i = 0; i < I6_VENC_CHN_NUM; i++) {
                vpe.port = i;
                i6_rgn.fnDetachChannel(h, &vpe);
            }
        }
    }

    memset(&attrib, 0, sizeof(attrib));
    attrib.show = 1;
    attrib.point.x = rect.x;
    attrib.point.y = rect.y;
    attrib.osd.layer = 0;
    attrib.osd.constAlphaOn = 0;
    attrib.osd.bgFgAlpha[0] = bg_opacity;
    attrib.osd.bgFgAlpha[1] = fg_opacity;

    // SigmaStar i6 firmwares differ:
    // - some blend OSD in VENC
    // - some blend OSD in VPE (pre-encode)
    // Attach to both to maximize compatibility.

    // 1) Attach to all enabled VENC channels (order can put MJPEG on ch0 and H26x on ch1+).
    for (char i = 0; i < I6_VENC_CHN_NUM; i++) {
        unsigned int dev = 0;
        if (i6_venc.fnGetChannelDeviceId(i, &dev) != 0)
            dev = 0;
        dest.device = dev;
        dest.channel = i;

        if (!i6_state[i].enable) {
            i6_rgn.fnDetachChannel(h, &dest);
            continue;
        }
        if (!hal_osd_is_allowed_for_channel(&i6_state[i])) {
            i6_rgn.fnDetachChannel(h, &dest);
            continue;
        }

        int rc = i6_rgn.fnAttachChannel(h, &dest, &attrib);
        if (rc) {
            HAL_ERROR("i6_rgn", "reg%d attach VENC failed (dev=%u ch=%u port=%u rc=%d)\n",
                handle, dest.device, dest.channel, dest.port, rc);
            // Keep going: other channels might still succeed.
        } else {
            // Some SigmaStar SDK variants require SetDisplayAttr after attach.
            int sc = i6_rgn.fnSetChannelConfig(h, &dest, &attrib);
            if (sc) {
                HAL_WARNING("i6_rgn", "reg%d SetDisplayAttr(VENC) failed (dev=%u ch=%u port=%u rc=%d)\n",
                    handle, dest.device, dest.channel, dest.port, sc);
            }
        }
    }

    // 2) Attach to VPE output ports (pre-encode), matching encoder channel index.
    {
        i6_sys_bind vpe = { .module = I6_SYS_MOD_VPE,
            .device = _i6_vpe_dev, .channel = _i6_vpe_chn };
        for (char i = 0; i < I6_VENC_CHN_NUM; i++) {
            vpe.port = i;
            if (!i6_state[i].enable) {
                i6_rgn.fnDetachChannel(h, &vpe);
                continue;
            }
            if (!hal_osd_is_allowed_for_channel(&i6_state[i])) {
                i6_rgn.fnDetachChannel(h, &vpe);
                continue;
            }
            int rc = i6_rgn.fnAttachChannel(h, &vpe, &attrib);
            if (rc) {
                HAL_ERROR("i6_rgn", "reg%d attach VPE failed (dev=%u ch=%u port=%u rc=%d)\n",
                    handle, vpe.device, vpe.channel, vpe.port, rc);
            } else {
                int sc = i6_rgn.fnSetChannelConfig(h, &vpe, &attrib);
                if (sc) {
                    HAL_WARNING("i6_rgn", "reg%d SetDisplayAttr(VPE) failed (dev=%u ch=%u port=%u rc=%d)\n",
                        handle, vpe.device, vpe.channel, vpe.port, sc);
                }
            }
        }
    }

    return EXIT_SUCCESS;
}

void i6_region_deinit(void)
{
    i6_rgn.fnDeinit();
}

void i6_region_destroy(char handle)
{
    unsigned int h = (unsigned int)((unsigned char)handle + 1);
    i6_sys_bind dest = { .module = I6_SYS_MOD_VENC, .port = _i6_venc_port };
    for (char i = 0; i < I6_VENC_CHN_NUM; i++) {
        unsigned int dev = 0;
        if (i6_venc.fnGetChannelDeviceId(i, &dev) != 0)
            dev = 0;
        dest.device = dev;
        dest.channel = i;
        i6_rgn.fnDetachChannel(h, &dest);
    }
    {
        i6_sys_bind vpe = { .module = I6_SYS_MOD_VPE,
            .device = _i6_vpe_dev, .channel = _i6_vpe_chn };
        for (char i = 0; i < I6_VENC_CHN_NUM; i++) {
            vpe.port = i;
            i6_rgn.fnDetachChannel(h, &vpe);
        }
    }
    i6_rgn.fnDestroyRegion(h);
}

void i6_region_init(void)
{
    i6_rgn_pal palette = {{{0, 0, 0, 0}}};
    i6_rgn.fnInit(&palette);
}

int i6_region_setbitmap(int handle, hal_bitmap *bitmap)
{
    unsigned int h = (unsigned int)((unsigned char)handle + 1);
    i6_rgn_bmp nativeBmp = { .data = bitmap->data, .pixFmt = I6_RGN_PIXFMT_ARGB1555,
        .size.height = bitmap->dim.height, .size.width = bitmap->dim.width };

    return i6_rgn.fnSetBitmap(h, &nativeBmp);
}

int i6_sensor_exposure(unsigned int micros)
{
    int ret;

    {
        i6_isp_exp config;
        if (ret = i6_isp.fnGetExposureLimit(0, &config))
            return ret;

        config.maxShutterUs = micros;
        if (ret = i6_isp.fnSetExposureLimit(0, &config))
            return ret;
    }

    return ret;
}

int i6_get_isp_exposure_info(unsigned int *iso, unsigned int *exp_time,
    unsigned int *again, unsigned int *dgain, unsigned int *ispdgain,
    int *exposure_is_max)
{
    if (!iso || !exp_time || !again || !dgain || !ispdgain || !exposure_is_max)
        return EXIT_FAILURE;

    i6_snr_plane p;
    int ret = i6_snr.fnGetPlaneInfo(_i6_snr_index, 0, &p);
    if (ret)
        return ret;

    *exp_time = p.shutter;       // us
    *again = p.sensGain;         // x1024
    *dgain = p.compGain;         // x1024
    *ispdgain = p.compGain;      // best-effort

    // Best-effort "ISO-like" metric: combined gain (still scaled by 1024).
    unsigned long long comb = (unsigned long long)p.sensGain * (unsigned long long)p.compGain;
    comb /= 1024ull;
    if (comb > 0xFFFFFFFFull) comb = 0xFFFFFFFFull;
    *iso = (unsigned int)comb;

    // We don't have an explicit "exposure max" flag from MI_SNR.
    *exposure_is_max = 0;
    return EXIT_SUCCESS;
}

int i6_video_create(char index, hal_vidconfig *config)
{
    int ret;
    i6_venc_chn channel;
    i6_venc_attr_h26x *attrib;
    memset(&channel, 0, sizeof(channel));
    const int h264_plus =
        (config->codec == HAL_VIDCODEC_H264) && (config->flags & HAL_VIDOPT_H264_PLUS);
    
    if (config->codec == HAL_VIDCODEC_JPG || config->codec == HAL_VIDCODEC_MJPG) {
        channel.attrib.codec = I6_VENC_CODEC_MJPG;
        switch (config->mode) {
            case HAL_VIDMODE_CBR:
                channel.rate.mode = series == 0xEF ? I6OG_VENC_RATEMODE_MJPGCBR : I6_VENC_RATEMODE_MJPGCBR;
                channel.rate.mjpgCbr.bitrate = config->bitrate << 10;
                channel.rate.mjpgCbr.fpsNum = 
                    config->codec == HAL_VIDCODEC_JPG ? 1 : config->framerate;
                channel.rate.mjpgCbr.fpsDen = 1;
                break;
            case HAL_VIDMODE_QP:
                channel.rate.mode = series == 0xEF ? I6OG_VENC_RATEMODE_MJPGQP : I6_VENC_RATEMODE_MJPGQP;
                channel.rate.mjpgQp.fpsNum = 
                    config->codec == HAL_VIDCODEC_JPG ? 1 : config->framerate;
                channel.rate.mjpgCbr.fpsDen = 1;
                channel.rate.mjpgQp.quality = MAX(config->minQual, config->maxQual);
                break;
            default:
                HAL_ERROR("i6_venc", "MJPEG encoder can only support CBR or fixed QP modes!");
        }

        channel.attrib.mjpg.maxHeight = config->height;
        channel.attrib.mjpg.maxWidth = config->width;
        channel.attrib.mjpg.bufSize = config->width * config->height;
        channel.attrib.mjpg.byFrame = 1;
        channel.attrib.mjpg.height = config->height;
        channel.attrib.mjpg.width = config->width;
        channel.attrib.mjpg.dcfThumbs = 0;
        channel.attrib.mjpg.markPerRow = 0;

        goto attach;
    } else if (config->codec == HAL_VIDCODEC_H265) {
        channel.attrib.codec = I6_VENC_CODEC_H265;
        attrib = &channel.attrib.h265;
        switch (config->mode) {
            case HAL_VIDMODE_CBR:
                channel.rate.mode = series == 0xEF ? I6OG_VENC_RATEMODE_H265CBR :
                    I6_VENC_RATEMODE_H265CBR;
                channel.rate.h265Cbr = (i6_venc_rate_h26xcbr){ .gop = config->gop,
                    .statTime = 1, .fpsNum = config->framerate, .fpsDen = 1, .bitrate = 
                    (unsigned int)(config->bitrate) << 10, .avgLvl = 1 }; break;
            case HAL_VIDMODE_VBR:
                channel.rate.mode = series == 0xEF ? I6OG_VENC_RATEMODE_H265VBR :
                    I6_VENC_RATEMODE_H265VBR;
                channel.rate.h265Vbr = (i6_venc_rate_h26xvbr){ .gop = config->gop,
                    .statTime = 1, .fpsNum = config->framerate, .fpsDen = 1, .maxBitrate = 
                    (unsigned int)(MAX(config->bitrate, config->maxBitrate)) << 10,
                    .maxQual = config->maxQual, .minQual = config->minQual }; break;
            case HAL_VIDMODE_QP:
                channel.rate.mode = series == 0xEF ? I6OG_VENC_RATEMODE_H265QP :
                    I6_VENC_RATEMODE_H265QP;
                channel.rate.h265Qp = (i6_venc_rate_h26xqp){ .gop = config->gop,
                    .fpsNum = config->framerate, .fpsDen = 1, .interQual = config->maxQual,
                    .predQual = config->minQual }; break;
            case HAL_VIDMODE_ABR:
                HAL_ERROR("i6_venc", "H.265 encoder does not support ABR mode!");
            case HAL_VIDMODE_AVBR:
                channel.rate.mode = series == 0xEF ? I6OG_VENC_RATEMODE_H265AVBR :
                    I6_VENC_RATEMODE_H265AVBR;
                channel.rate.h265Avbr = (i6_venc_rate_h26xvbr){ .gop = config->gop,
                    .statTime = 1, .fpsNum = config->framerate, .fpsDen = 1, .maxBitrate = 
                    (unsigned int)(MAX(config->bitrate, config->maxBitrate)) << 10,
                    .maxQual = config->maxQual, .minQual = config->minQual }; break;
            default:
                HAL_ERROR("i6_venc", "H.265 encoder does not support this mode!");
        }  
    } else if (config->codec == HAL_VIDCODEC_H264) {
        channel.attrib.codec = I6_VENC_CODEC_H264;
        attrib = &channel.attrib.h264;
        hal_vidmode mode = config->mode;
        if (h264_plus && mode != HAL_VIDMODE_QP)
            mode = HAL_VIDMODE_AVBR;
        if (series == 0xEF && mode == HAL_VIDMODE_ABR)
            mode = (hal_vidmode)-1;
        switch (mode) {
            case HAL_VIDMODE_CBR:
                channel.rate.mode = series == 0xEF ? I6OG_VENC_RATEMODE_H264CBR :
                    I6_VENC_RATEMODE_H264CBR;
                channel.rate.h264Cbr = (i6_venc_rate_h26xcbr){ .gop = config->gop,
                    .statTime = 1, .fpsNum = config->framerate, .fpsDen = 1, .bitrate = 
                    (unsigned int)(config->bitrate) << 10, .avgLvl = 1 }; break;
            case HAL_VIDMODE_VBR:
                channel.rate.mode = series == 0xEF ? I6OG_VENC_RATEMODE_H264VBR :
                    I6_VENC_RATEMODE_H264VBR;
                channel.rate.h264Vbr = (i6_venc_rate_h26xvbr){ .gop = config->gop,
                    .statTime = 1, .fpsNum = config->framerate, .fpsDen = 1, .maxBitrate = 
                    (unsigned int)(MAX(config->bitrate, config->maxBitrate)) << 10,
                    .maxQual = config->maxQual, .minQual = config->minQual }; break;
            case HAL_VIDMODE_QP:
                channel.rate.mode = series == 0xEF ? I6OG_VENC_RATEMODE_H264QP :
                    I6_VENC_RATEMODE_H264QP;
                channel.rate.h264Qp = (i6_venc_rate_h26xqp){ .gop = config->gop,
                    .fpsNum = config->framerate, .fpsDen = 1, .interQual = config->maxQual,
                    .predQual = config->minQual }; break;
            case HAL_VIDMODE_ABR:
                channel.rate.mode = I6_VENC_RATEMODE_H264ABR;
                channel.rate.h264Abr = (i6_venc_rate_h26xabr){ .gop = config->gop,
                    .statTime = 1, .fpsNum = config->framerate, .fpsDen = 1,
                    .avgBitrate = (unsigned int)(config->bitrate) << 10,
                    .maxBitrate = (unsigned int)(config->maxBitrate) << 10 }; break;
            case HAL_VIDMODE_AVBR:
                channel.rate.mode = series == 0xEF ? I6OG_VENC_RATEMODE_H264AVBR :
                    I6_VENC_RATEMODE_H264AVBR;
                channel.rate.h264Avbr = (i6_venc_rate_h26xvbr){ .gop = config->gop, .statTime = 1,
                    .fpsNum = config->framerate, .fpsDen = 1, .maxBitrate = 
                    (unsigned int)(MAX(config->bitrate, config->maxBitrate)) << 10,
                    .maxQual = config->maxQual, .minQual = config->minQual }; break;
            default:
                HAL_ERROR("i6_venc", "H.264 encoder does not support this mode!");
        }
    } else HAL_ERROR("i6_venc", "This codec is not supported by the hardware!");
    attrib->maxHeight = config->height;
    attrib->maxWidth = config->width;
    attrib->bufSize = config->height * config->width;
    attrib->profile = MIN((series == 0xEF || config->codec == HAL_VIDCODEC_H265) ? 1 : 2,
        config->profile);
    attrib->byFrame = 1;
    attrib->height = config->height;
    attrib->width = config->width;
    attrib->bFrameNum = 0;
    attrib->refNum = 1;
attach:
    if (ret = i6_venc.fnCreateChannel(index, &channel))
        return ret;

    if (config->codec != HAL_VIDCODEC_JPG && 
        (ret = i6_venc.fnStartReceiving(index)))
        return ret;

    i6_state[index].payload = config->codec;

    return EXIT_SUCCESS;
}

int i6_video_destroy(char index)
{
    int ret;

    i6_state[index].enable = 0;
    i6_state[index].payload = HAL_VIDCODEC_UNSPEC;

    i6_venc.fnStopReceiving(index);

    {
        unsigned int device;
        if (ret = i6_venc.fnGetChannelDeviceId(index, &device))
            return ret;
        i6_sys_bind source = { .module = I6_SYS_MOD_VPE, 
            .device = _i6_vpe_dev, .channel = _i6_vpe_chn, .port = index };
        i6_sys_bind dest = { .module = I6_SYS_MOD_VENC,
            .device = device, .channel = index, .port = _i6_venc_port };
        if (ret = i6_sys.fnUnbind(&source, &dest))
            return ret;
    }

    if (ret = i6_venc.fnDestroyChannel(index))
        return ret;
    
    if (ret = i6_vpe.fnDisablePort(_i6_vpe_chn, index))
        return ret;

    return EXIT_SUCCESS;
}

int i6_video_destroy_all(void)
{
    int ret;

    for (char i = 0; i < I6_VENC_CHN_NUM; i++)
        if (i6_state[i].enable)
            if (ret = i6_video_destroy(i))
                return ret;

    return EXIT_SUCCESS;
}

void i6_video_request_idr(char index)
{
    i6_venc.fnRequestIdr(index, 1);
}

int i6_video_snapshot_grab(char index, char quality, hal_jpegdata *jpeg)
{
    int ret;

    if (ret = i6_channel_bind(index, 1)) {
        HAL_DANGER("i6_venc", "Binding the encoder channel "
            "%d failed with %#x!\n", index, ret);
        goto abort;
    }

    i6_venc_jpg param;
    memset(&param, 0, sizeof(param));
    if (ret = i6_venc.fnGetJpegParam(index, &param)) {
        HAL_DANGER("i6_venc", "Reading the JPEG settings "
            "%d failed with %#x!\n", index, ret);
        goto abort;
    }

    param.quality = quality;
    if (ret = i6_venc.fnSetJpegParam(index, &param)) {
        HAL_DANGER("i6_venc", "Writing the JPEG settings "
            "%d failed with %#x!\n", index, ret);
        goto abort;
    }

    unsigned int count = 1;
    if (ret = i6_venc.fnStartReceivingEx(index, &count)) {
        HAL_DANGER("i6_venc", "Requesting one frame "
            "%d failed with %#x!\n", index, ret);
        goto abort;
    }

    int fd = i6_venc.fnGetDescriptor(index);

    struct timeval timeout = { .tv_sec = 2, .tv_usec = 0 };
    fd_set readFds;
    FD_ZERO(&readFds);
    FD_SET(fd, &readFds);
    ret = select(fd + 1, &readFds, NULL, NULL, &timeout);
    if (ret < 0) {
        HAL_DANGER("i6_venc", "Select operation failed!\n");
        goto abort;
    } else if (ret == 0) {
        HAL_DANGER("i6_venc", "Capture stream timed out!\n");
        goto abort;
    }

    if (FD_ISSET(fd, &readFds)) {
        i6_venc_stat stat;
        if (ret = i6_venc.fnQuery(index, &stat)) {
            HAL_DANGER("i6_venc", "Querying the encoder channel "
                "%d failed with %#x!\n", index, ret);
            goto abort;
        }

        if (!stat.curPacks) {
            HAL_DANGER("i6_venc", "Current frame is empty, skipping it!\n");
            goto abort;
        }

        i6_venc_strm strm;
        memset(&strm, 0, sizeof(strm));
        strm.packet = (i6_venc_pack*)malloc(sizeof(i6_venc_pack) * stat.curPacks);
        if (!strm.packet) {
            HAL_DANGER("i6_venc", "Memory allocation on channel %d failed!\n", index);
            goto abort;
        }
        strm.count = stat.curPacks;

        if (ret = i6_venc.fnGetStream(index, &strm, stat.curPacks)) {
            HAL_DANGER("i6_venc", "Getting the stream on "
                "channel %d failed with %#x!\n", index, ret);
            free(strm.packet);
            strm.packet = NULL;
            goto abort;
        }

        {
            jpeg->jpegSize = 0;
            for (unsigned int i = 0; i < strm.count; i++) {
                i6_venc_pack *pack = &strm.packet[i];
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
        i6_venc.fnFreeStream(index, &strm);
    }

    i6_venc.fnFreeDescriptor(index);

    i6_venc.fnStopReceiving(index);

    i6_channel_unbind(index);

    return ret;
}

void *i6_video_thread(void)
{
    int ret, maxFd = 0;

    for (int i = 0; i < I6_VENC_CHN_NUM; i++) {
        if (!i6_state[i].enable) continue;
        if (!i6_state[i].mainLoop) continue;

        ret = i6_venc.fnGetDescriptor(i);
        if (ret < 0) {
            HAL_DANGER("i6_venc", "Getting the encoder descriptor failed with %#x!\n", ret);
            return (void*)0;
        }
        i6_state[i].fileDesc = ret;

        if (maxFd <= i6_state[i].fileDesc)
            maxFd = i6_state[i].fileDesc;
    }

    i6_venc_stat stat;
    i6_venc_strm stream;
    struct timeval timeout;
    fd_set readFds;

    while (keepRunning) {
        FD_ZERO(&readFds);
        for(int i = 0; i < I6_VENC_CHN_NUM; i++) {
            if (!i6_state[i].enable) continue;
            if (!i6_state[i].mainLoop) continue;
            FD_SET(i6_state[i].fileDesc, &readFds);
        }

        timeout.tv_sec = 2;
        timeout.tv_usec = 0;
        ret = select(maxFd + 1, &readFds, NULL, NULL, &timeout);
        if (ret < 0) {
            HAL_DANGER("i6_venc", "Select operation failed!\n");
            break;
        } else if (ret == 0) {
            HAL_WARNING("i6_venc", "Main stream loop timed out!\n");
            continue;
        } else {
            for (int i = 0; i < I6_VENC_CHN_NUM; i++) {
                if (!i6_state[i].enable) continue;
                if (!i6_state[i].mainLoop) continue;
                if (FD_ISSET(i6_state[i].fileDesc, &readFds)) {
                    memset(&stream, 0, sizeof(stream));
                    
                    if (ret = i6_venc.fnQuery(i, &stat)) {
                        HAL_DANGER("i6_venc", "Querying the encoder channel "
                            "%d failed with %#x!\n", i, ret);
                        break;
                    }

                    if (!stat.curPacks) {
                        HAL_WARNING("i6_venc", "Current frame is empty, skipping it!\n");
                        continue;
                    }

                    stream.packet = (i6_venc_pack*)malloc(
                        sizeof(i6_venc_pack) * stat.curPacks);
                    if (!stream.packet) {
                        HAL_DANGER("i6_venc", "Memory allocation on channel %d failed!\n", i);
                        break;
                    }
                    stream.count = stat.curPacks;

                    if (ret = i6_venc.fnGetStream(i, &stream, 40)) {
                        HAL_DANGER("i6_venc", "Getting the stream on "
                            "channel %d failed with %#x!\n", i, ret);
                        break;
                    }

                    if (i6_vid_cb) {
                        hal_vidstream outStrm;
                        hal_vidpack outPack[stream.count];
                        outStrm.count = stream.count;
                        outStrm.seq = stream.sequence;
                        for (int j = 0; j < stream.count; j++) {
                            i6_venc_pack *pack = &stream.packet[j];
                            outPack[j].data = pack->data;
                            outPack[j].length = pack->length;
                            outPack[j].naluCnt = pack->packNum;
                            if (series == 0xEF) {
                                signed char n = 0;
                                switch (i6_state[i].payload) {
                                    case HAL_VIDCODEC_H264:
                                        for (unsigned int p = 0; p < pack->length - 4; p++) {
                                            if (outPack[j].data[p] || outPack[j].data[p + 1] ||
                                                outPack[j].data[p + 2] || outPack[j].data[p + 3] != 1) continue;
                                            outPack[0].nalu[n].type = outPack[j].data[p + 4] & 0x1F;
                                            outPack[0].nalu[n++].offset = p;
                                            if (n == (outPack[j].naluCnt)) break;
                                        }
                                        break;
                                    case HAL_VIDCODEC_H265:
                                        for (unsigned int p = 0; p < pack->length - 4; p++) {
                                            if (outPack[j].data[p] || outPack[j].data[p + 1] ||
                                                outPack[j].data[p + 2] || outPack[j].data[p + 3] != 1) continue;
                                            outPack[0].nalu[n].type = (outPack[j].data[p + 4] & 0x7E) >> 1;
                                            outPack[0].nalu[n++].offset = p;
                                            if (n == (outPack[j].naluCnt)) break;
                                        }
                                        break;
                                }

                                outPack[0].naluCnt = n;
                                outPack[0].nalu[n].offset = pack->length;
                                for (n = 0; n < outPack[0].naluCnt; n++)
                                    outPack[0].nalu[n].length = 
                                        outPack[0].nalu[n + 1].offset -
                                        outPack[0].nalu[n].offset;
                            } else switch (i6_state[i].payload) {
                                case HAL_VIDCODEC_H264:
                                    for (char k = 0; k < outPack[j].naluCnt; k++) {
                                        outPack[j].nalu[k].length =
                                            pack->packetInfo[k].length;
                                        outPack[j].nalu[k].offset =
                                            pack->packetInfo[k].offset;
                                        outPack[j].nalu[k].type =
                                            pack->packetInfo[k].packType.h264Nalu;
                                    }
                                    break;
                                case HAL_VIDCODEC_H265:
                                    for (char k = 0; k < outPack[j].naluCnt; k++) {
                                        outPack[j].nalu[k].length =
                                            pack->packetInfo[k].length;
                                        outPack[j].nalu[k].offset =
                                            pack->packetInfo[k].offset;
                                        outPack[j].nalu[k].type =
                                            pack->packetInfo[k].packType.h265Nalu;
                                    }
                                    break;
                            }
                            outPack[j].offset = pack->offset;
                            outPack[j].timestamp = pack->timestamp;
                        }
                        outStrm.pack = outPack;
                        (*i6_vid_cb)(i, &outStrm);
                    }

                    if (ret = i6_venc.fnFreeStream(i, &stream)) {
                        HAL_DANGER("i6_venc", "Releasing the stream on "
                            "channel %d failed with %#x!\n", i, ret);
                    }
                    free(stream.packet);
                    stream.packet = NULL;
                }
            }
        }
    }

    HAL_INFO("i6_venc", "Shutting down encoding thread...\n");
}

void i6_system_deinit(void)
{
    i6_sys.fnExit();
}

int i6_system_init(void)
{
    int ret;

    printf("App built with headers v%s\n", I6_SYS_API);

    {
        i6_sys_ver version;
        if (ret = i6_sys.fnGetVersion(&version))
            return ret;
        puts(version.version);
    }

    if (ret = i6_sys.fnInit())
        return ret;

    return EXIT_SUCCESS;
}

#endif