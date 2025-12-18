#include "jpeg.h"
#include <time.h>

int jpeg_index;
bool jpeg_module_init = false;

pthread_mutex_t jpeg_mutex;

int jpeg_init() {  
    int ret;

    pthread_mutex_lock(&jpeg_mutex);

    // If MJPEG is enabled, serve snapshots from the last MJPEG frame.
    // This avoids creating a dedicated JPEG/VENC channel which some SDKs reject.
    if (app_config.jpeg_enable)
        goto active;

    switch (plat) {
#if defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_GM: goto active;
#elif defined(__mips__)
        case HAL_PLATFORM_T31:
            if (app_config.jpeg_enable) goto active;
            break;
#endif
    }

    jpeg_index = take_next_free_channel(false);

    if (ret = create_channel(jpeg_index, app_config.jpeg_width, app_config.jpeg_height, 1, 1)) {
        pthread_mutex_unlock(&jpeg_mutex);
        HAL_ERROR("jpeg", "Creating channel %d failed with %#x!\n%s\n", 
            jpeg_index, ret, errstr(ret));
    }

    {
        hal_vidconfig config = {0};
        config.width = app_config.jpeg_width;
        config.height = app_config.jpeg_height;
        config.codec = HAL_VIDCODEC_JPG;
        config.mode = HAL_VIDMODE_QP;
        // Some vendor HALs reject MJPEG/JPEG channels with src/dst fps = 0.
        // Snapshot encoders use "one-shot" StartReceivingEx later, but the channel
        // still needs a sane fps in its rate control struct.
        config.framerate = 1;
        // Keep safe defaults for vendor HALs that still peek at bitrate fields.
        config.bitrate = 1024;
        config.maxBitrate = 1024 * 5 / 4;
        config.minQual = config.maxQual = app_config.jpeg_qfactor;

        switch (plat) {
#if defined(__ARM_PCS_VFP)
            case HAL_PLATFORM_I6:  ret = i6_video_create(jpeg_index, &config); break;
            case HAL_PLATFORM_I6C: ret = i6c_video_create(jpeg_index, &config); break;
            case HAL_PLATFORM_M6:  ret = m6_video_create(jpeg_index, &config); break;
            case HAL_PLATFORM_RK:  ret = rk_video_create(jpeg_index, &config); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
            case HAL_PLATFORM_V1:  ret = v1_video_create(jpeg_index, &config); break;
            case HAL_PLATFORM_V2:  ret = v2_video_create(jpeg_index, &config); break;
            case HAL_PLATFORM_V3:  ret = v3_video_create(jpeg_index, &config); break;
            case HAL_PLATFORM_V4:  ret = v4_video_create(jpeg_index, &config); break;
#elif defined(__mips__)
            case HAL_PLATFORM_T31: ret = t31_video_create(jpeg_index, &config); break;
#elif defined(__riscv) || defined(__riscv__)
            case HAL_PLATFORM_CVI: ret = cvi_video_create(jpeg_index, &config); break;
#endif
            default: 
                pthread_mutex_unlock(&jpeg_mutex);
                return EXIT_FAILURE;      
        }

        if (ret) {
            pthread_mutex_unlock(&jpeg_mutex);
            HAL_ERROR("jpeg", "Creating encoder %d failed with %#x!\n%s\n", 
                jpeg_index, ret, errstr(ret));
        }
    }

active:
    jpeg_module_init = true;
    pthread_mutex_unlock(&jpeg_mutex);
    HAL_INFO("jpeg", "Module enabled!\n");

    return EXIT_SUCCESS;
}

void jpeg_deinit() {
    pthread_mutex_lock(&jpeg_mutex);

    // If MJPEG is enabled, we did not create a dedicated JPEG channel.
    if (app_config.jpeg_enable)
        goto active;

    switch (plat) {
#if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  i6_video_destroy(jpeg_index); break;
        case HAL_PLATFORM_I6C: i6c_video_destroy(jpeg_index); break;
        case HAL_PLATFORM_M6:  m6_video_destroy(jpeg_index); break;
        case HAL_PLATFORM_RK:  rk_video_destroy(jpeg_index); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_GM:  goto active;
        case HAL_PLATFORM_V1:  v1_video_destroy(jpeg_index); break;
        case HAL_PLATFORM_V2:  v2_video_destroy(jpeg_index); break;
        case HAL_PLATFORM_V3:  v3_video_destroy(jpeg_index); break;
        case HAL_PLATFORM_V4:  v4_video_destroy(jpeg_index); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31:
            if (app_config.jpeg_enable) goto active;
            t31_video_destroy(jpeg_index);
            break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: cvi_video_destroy(jpeg_index); break;
#endif
        default: 
            pthread_mutex_unlock(&jpeg_mutex);
            return;    
    }

active:
    jpeg_module_init = false;
    pthread_mutex_unlock(&jpeg_mutex);
    HAL_INFO("jpeg", "Module disabled!\n");
}

int jpeg_get(short width, short height, char quality, char grayscale, 
    hal_jpegdata *jpeg) {
    pthread_mutex_lock(&jpeg_mutex);
    if (!jpeg_module_init) {
        pthread_mutex_unlock(&jpeg_mutex);
        HAL_ERROR("jpeg", "Module is not enabled!\n");
    }
    int ret;

    // When MJPEG is enabled, treat snapshots as "last MJPEG frame".
    // This makes /image.jpg and http_post snapshots robust on firmwares that
    // can't create a dedicated JPEG channel.
    if (app_config.jpeg_enable) {
        ret = media_get_last_mjpeg_frame(jpeg, 2000);
        if (ret) {
            static time_t last_fail_log = 0;
            time_t now = time(NULL);
            if (!last_fail_log || now - last_fail_log >= 5) {
                HAL_WARNING("jpeg", "Snapshot requested, but no MJPEG cached frame yet\n");
                last_fail_log = now;
            }
        } else {
            static unsigned int ok_logs = 0;
            static time_t last_ok_log = 0;
            time_t now = time(NULL);
            if (ok_logs < 5 || !last_ok_log || now - last_ok_log >= 5) {
                HAL_INFO("jpeg", "Snapshot served from MJPEG cached frame (%u bytes)\n",
                    jpeg->jpegSize);
                ok_logs++;
                last_ok_log = now;
            }
        }
        pthread_mutex_unlock(&jpeg_mutex);
        return ret;
    }

    switch (plat) {
 #if defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_I6:  ret = i6_video_snapshot_grab(jpeg_index, quality, jpeg); break;
        case HAL_PLATFORM_I6C: ret = i6c_video_snapshot_grab(jpeg_index, quality, jpeg); break;
        case HAL_PLATFORM_M6:  ret = m6_video_snapshot_grab(jpeg_index, quality, jpeg); break;
        case HAL_PLATFORM_RK:  ret = rk_video_snapshot_grab(jpeg_index, jpeg); break;
#elif defined(__arm__) && !defined(__ARM_PCS_VFP)
        case HAL_PLATFORM_GM:  ret = gm_video_snapshot_grab(width, height, quality, jpeg); break;
        case HAL_PLATFORM_V1:  ret = v1_video_snapshot_grab(jpeg_index, jpeg); break;
        case HAL_PLATFORM_V2:  ret = v2_video_snapshot_grab(jpeg_index, jpeg); break;
        case HAL_PLATFORM_V3:  ret = v3_video_snapshot_grab(jpeg_index, jpeg); break;
        case HAL_PLATFORM_V4:  ret = v4_video_snapshot_grab(jpeg_index, jpeg); break;
#elif defined(__mips__)
        case HAL_PLATFORM_T31: ret = t31_video_snapshot_grab(app_config.jpeg_enable ? 
            -1 : jpeg_index, jpeg); break;
#elif defined(__riscv) || defined(__riscv__)
        case HAL_PLATFORM_CVI: ret = cvi_video_snapshot_grab(jpeg_index, jpeg); break;
#endif
    }
    if (ret && jpeg->data) { 
        free(jpeg->data);
        jpeg->data = NULL;
    }

    pthread_mutex_unlock(&jpeg_mutex);
    return ret;
}