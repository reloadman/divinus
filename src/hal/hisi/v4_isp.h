#pragma once

#include "v4_common.h"

extern void* (*fnIsp_Malloc)(unsigned long);
extern int   (*fnISP_AlgRegisterAcs)(int);
extern int   (*fnISP_AlgRegisterDehaze)(int);
extern int   (*fnISP_AlgRegisterDrc)(int);
extern int   (*fnISP_AlgRegisterLdci)(int);
extern int   (*fnMPI_ISP_IrAutoRunOnce)(int, void*);

typedef enum {
    V4_ISP_DIR_NORMAL,
    V4_ISP_DIR_MIRROR,
    V4_ISP_DIR_FLIP,
    V4_ISP_DIR_MIRROR_FLIP,
    V4_ISP_DIR_END
} v4_isp_dir;

typedef struct {
    int id;
    char libName[20];
} v4_isp_alg;

typedef struct {
    v4_common_rect capt;
    v4_common_dim size;
    float framerate;
    v4_common_bayer bayer;
    v4_common_wdr wdr;
    unsigned char mode;
} v4_isp_dev;

typedef struct {
    void *handleCalcFlick, *handle, *handleMpi, *handleAcs, *handleDehaze, *handleDrc, *handleLdci, *handleIrAuto, *handleAwb, 
        *handleGokeAwb, *handleAe, *handleGokeAe,  *handleGoke;

    int (*fnExit)(int pipe);
    int (*fnInit)(int pipe);
    int (*fnMemInit)(int pipe);
    int (*fnRun)(int pipe);

    int (*fnSetDeviceConfig)(int pipe, v4_isp_dev *config);

    int (*fnRegisterAE)(int pipe, v4_isp_alg *library);
    int (*fnRegisterAWB)(int pipe, v4_isp_alg *library);
    int (*fnUnregisterAE)(int pipe, v4_isp_alg *library);
    int (*fnUnregisterAWB)(int pipe, v4_isp_alg *library);

    // Optional ISP/IQ controls (not required for base pipeline)
    int (*fnGetExposureAttr)(int pipe, void *attr);
    int (*fnSetExposureAttr)(int pipe, const void *attr);
    int (*fnGetAERouteAttrEx)(int pipe, void *attr);
    int (*fnSetAERouteAttrEx)(int pipe, const void *attr);
    int (*fnGetAERouteAttr)(int pipe, void *attr);
    int (*fnSetAERouteAttr)(int pipe, const void *attr);
    int (*fnQueryExposureInfo)(int pipe, void *expInfo);
    int (*fnGetCCMAttr)(int pipe, void *attr);
    int (*fnSetCCMAttr)(int pipe, const void *attr);
    int (*fnGetSaturationAttr)(int pipe, void *attr);
    int (*fnSetSaturationAttr)(int pipe, const void *attr);
    int (*fnGetDRCAttr)(int pipe, void *attr);
    int (*fnSetDRCAttr)(int pipe, const void *attr);
    int (*fnGetModuleControl)(int pipe, void *ctrl);
    int (*fnSetModuleControl)(int pipe, const void *ctrl);
    int (*fnGetDehazeAttr)(int pipe, void *attr);
    int (*fnSetDehazeAttr)(int pipe, const void *attr);
    int (*fnGetLDCIAttr)(int pipe, void *attr);
    int (*fnSetLDCIAttr)(int pipe, const void *attr);
    int (*fnGetNRAttr)(int pipe, void *attr);
    int (*fnSetNRAttr)(int pipe, const void *attr);
    int (*fnGetGammaAttr)(int pipe, void *attr);
    int (*fnSetGammaAttr)(int pipe, const void *attr);
    int (*fnGetIspSharpenAttr)(int pipe, void *attr);
    int (*fnSetIspSharpenAttr)(int pipe, const void *attr);

    int (*fnGetStatisticsConfig)(int pipe, void *cfg);
    int (*fnSetStatisticsConfig)(int pipe, const void *cfg);
} v4_isp_impl;

// Helper: resolve a symbol from multiple handles and multiple name variants.
// Must be file-scope (C does not support nested function definitions).
static inline void *v4_dlsym_multi(void *handles[], const char *names[]) {
    for (int h = 0; handles[h]; h++) {
        for (int n = 0; names[n]; n++) {
            void *sym = dlsym(handles[h], names[n]);
            if (sym) return sym;
        }
    }
    return NULL;
}

static int v4_isp_load(v4_isp_impl *isp_lib) {
    isp_lib->handleCalcFlick = dlopen("lib_hicalcflicker.so", RTLD_LAZY | RTLD_GLOBAL);
    // Optional: some SDKs export HI_MPI_* symbols from libhi_mpi.so instead of lib*_isp.so.
    isp_lib->handleMpi = dlopen("libhi_mpi.so", RTLD_LAZY | RTLD_GLOBAL);

    if ((isp_lib->handle = dlopen("libisp.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleAe = dlopen("lib_hiae.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleAwb = dlopen("lib_hiawb.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleIrAuto = dlopen("lib_hiir_auto.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleLdci = dlopen("lib_hildci.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleDehaze = dlopen("lib_hidehaze.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleDrc = dlopen("lib_hidrc.so", RTLD_LAZY | RTLD_GLOBAL)))
        goto loaded;

    if ((isp_lib->handleGoke = dlopen("libgk_isp.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleGokeAe = dlopen("libgk_ae.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleAe = dlopen("libhi_ae.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleGokeAwb = dlopen("libgk_awb.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleAwb = dlopen("libhi_awb.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleIrAuto = dlopen("libir_auto.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleLdci = dlopen("libldci.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleDrc = dlopen("libdrc.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handleDehaze = dlopen("libdehaze.so", RTLD_LAZY | RTLD_GLOBAL)) &&
        (isp_lib->handle = dlopen("libhi_isp.so", RTLD_LAZY | RTLD_GLOBAL)))
        goto loaded;

    HAL_ERROR("v4_isp", "Failed to load library!\nError: %s\n", dlerror());

loaded:
    isp_lib->handleAcs = dlopen("lib_hiacs.so", RTLD_LAZY | RTLD_GLOBAL);

    if (!isp_lib->handleGoke) {
        hal_symbol_load("v4_isp", isp_lib->handle, "isp_malloc");
            
        if (!(fnISP_AlgRegisterAcs = (int(*)(int))
            hal_symbol_load("v4_isp", isp_lib->handleAcs, "isp_alg_register_acs")))
            return EXIT_FAILURE;

        if (!(fnISP_AlgRegisterDehaze = (int(*)(int))hal_symbol_load("v4_isp", isp_lib->handleDehaze, "ISP_AlgRegisterDehaze")))
            fnISP_AlgRegisterDehaze = (int(*)(int))hal_symbol_load("v4_isp", isp_lib->handleDehaze, "isp_alg_register_dehaze");

        if (!(fnISP_AlgRegisterDrc = (int(*)(int))hal_symbol_load("v4_isp", isp_lib->handleDrc, "ISP_AlgRegisterDrc")))
            fnISP_AlgRegisterDrc = (int(*)(int))hal_symbol_load("v4_isp", isp_lib->handleDrc, "isp_alg_register_drc");

        if (!(fnISP_AlgRegisterLdci = (int(*)(int))hal_symbol_load("v4_isp", isp_lib->handleLdci, "ISP_AlgRegisterLdci")))
            fnISP_AlgRegisterLdci = (int(*)(int))hal_symbol_load("v4_isp", isp_lib->handleLdci, "isp_alg_register_ldci");

        if (!(fnMPI_ISP_IrAutoRunOnce = (int(*)(int, void*))hal_symbol_load("v4_isp", isp_lib->handleIrAuto, "HI_MPI_ISP_IrAutoRunOnce")))
            fnMPI_ISP_IrAutoRunOnce = (int(*)(int, void*))hal_symbol_load("v4_isp", isp_lib->handleIrAuto, "isp_ir_auto_run_once");
    } else {
        if (!(fnISP_AlgRegisterDehaze = (int(*)(int))
            hal_symbol_load("v4_isp", isp_lib->handleDehaze, "ISP_AlgRegisterDehaze")))
            return EXIT_FAILURE;

        if (!(fnISP_AlgRegisterDrc = (int(*)(int))
            hal_symbol_load("v4_isp", isp_lib->handleDrc, "ISP_AlgRegisterDrc")))
            return EXIT_FAILURE;

        if (!(fnISP_AlgRegisterLdci = (int(*)(int))
            hal_symbol_load("v4_isp", isp_lib->handleLdci, "ISP_AlgRegisterLdci")))
            return EXIT_FAILURE;

        if (!(fnMPI_ISP_IrAutoRunOnce = (int(*)(int, void*))
            hal_symbol_load("v4_isp", isp_lib->handleIrAuto, "MPI_ISP_IrAutoRunOnce")))
            return EXIT_FAILURE;
    }

    if (!(isp_lib->fnExit = (int(*)(int pipe))
        hal_symbol_load("v4_isp", isp_lib->handle, "HI_MPI_ISP_Exit")))
        return EXIT_FAILURE;

    if (!(isp_lib->fnInit = (int(*)(int pipe))
        hal_symbol_load("v4_isp", isp_lib->handle, "HI_MPI_ISP_Init")))
        return EXIT_FAILURE;

    if (!(isp_lib->fnMemInit = (int(*)(int pipe))
        hal_symbol_load("v4_isp", isp_lib->handle, "HI_MPI_ISP_MemInit")))
        return EXIT_FAILURE;

    if (!(isp_lib->fnRun = (int(*)(int pipe))
        hal_symbol_load("v4_isp", isp_lib->handle, "HI_MPI_ISP_Run")))
        return EXIT_FAILURE;

    if (!(isp_lib->fnSetDeviceConfig = (int(*)(int pipe, v4_isp_dev *config))
        hal_symbol_load("v4_isp", isp_lib->handle, "HI_MPI_ISP_SetPubAttr")))
        return EXIT_FAILURE;

    if (!(isp_lib->fnRegisterAE = (int(*)(int pipe, v4_isp_alg *library))
        hal_symbol_load("v4_isp", isp_lib->handleAe, "HI_MPI_AE_Register")))
        return EXIT_FAILURE;

    if (!(isp_lib->fnRegisterAWB = (int(*)(int pipe, v4_isp_alg *library))
        hal_symbol_load("v4_isp", isp_lib->handleAwb, "HI_MPI_AWB_Register")))
        return EXIT_FAILURE;

    if (!(isp_lib->fnUnregisterAE = (int(*)(int pipe, v4_isp_alg *library))
        hal_symbol_load("v4_isp", isp_lib->handleAe, "HI_MPI_AE_UnRegister")))
        return EXIT_FAILURE;

    if (!(isp_lib->fnUnregisterAWB = (int(*)(int pipe, v4_isp_alg *library))
        hal_symbol_load("v4_isp", isp_lib->handleAwb, "HI_MPI_AWB_UnRegister")))
        return EXIT_FAILURE;

    // Optional symbols used for applying IQ profiles. These may be absent on some SDKs.
    // Prefer libhi_mpi.so when present, fallback to the ISP library handle.
    void *handles_isp[] = { isp_lib->handleMpi, isp_lib->handle, NULL };
    void *handles_isp_ext[] = { isp_lib->handleMpi, isp_lib->handle, isp_lib->handleDehaze, isp_lib->handleLdci, NULL };
    void *handles_ae[]  = { isp_lib->handleMpi, isp_lib->handle, isp_lib->handleAe, isp_lib->handleGokeAe, NULL };
    void *handles_awb[] = { isp_lib->handleMpi, isp_lib->handle, isp_lib->handleAwb, isp_lib->handleGokeAwb, NULL };

    // Exposure / AE route
    // On some Goke/GK SDKs, HI_MPI_* symbols exist but are stubs for parts of AE.
    // Prefer GK_API_* when available, fallback to HI_MPI_*/MPI_*.
    isp_lib->fnGetExposureAttr = (int(*)(int, void*))v4_dlsym_multi(
        handles_ae, (const char*[]){ "GK_API_ISP_GetExposureAttr", "HI_MPI_ISP_GetExposureAttr", "MPI_ISP_GetExposureAttr", NULL });
    isp_lib->fnSetExposureAttr = (int(*)(int, const void*))v4_dlsym_multi(
        handles_ae, (const char*[]){ "GK_API_ISP_SetExposureAttr", "HI_MPI_ISP_SetExposureAttr", "MPI_ISP_SetExposureAttr", NULL });
    isp_lib->fnQueryExposureInfo = (int(*)(int, void*))v4_dlsym_multi(
        handles_ae, (const char*[]){ "GK_API_ISP_QueryExposureInfo", "HI_MPI_ISP_QueryExposureInfo", "MPI_ISP_QueryExposureInfo", NULL });
    isp_lib->fnGetAERouteAttrEx = (int(*)(int, void*))v4_dlsym_multi(
        handles_ae, (const char*[]){ "GK_API_ISP_GetAERouteAttrEx", "HI_MPI_ISP_GetAERouteAttrEx", "MPI_ISP_GetAERouteAttrEx", NULL });
    isp_lib->fnSetAERouteAttrEx = (int(*)(int, const void*))v4_dlsym_multi(
        handles_ae, (const char*[]){ "GK_API_ISP_SetAERouteAttrEx", "HI_MPI_ISP_SetAERouteAttrEx", "MPI_ISP_SetAERouteAttrEx", NULL });
    isp_lib->fnGetAERouteAttr = (int(*)(int, void*))v4_dlsym_multi(
        handles_ae, (const char*[]){ "GK_API_ISP_GetAERouteAttr", "HI_MPI_ISP_GetAERouteAttr", "MPI_ISP_GetAERouteAttr", NULL });
    isp_lib->fnSetAERouteAttr = (int(*)(int, const void*))v4_dlsym_multi(
        handles_ae, (const char*[]){ "GK_API_ISP_SetAERouteAttr", "HI_MPI_ISP_SetAERouteAttr", "MPI_ISP_SetAERouteAttr", NULL });

    // Statistics config (used for AE weight table)
    isp_lib->fnGetStatisticsConfig = (int(*)(int, void*))v4_dlsym_multi(
        handles_isp, (const char*[]){ "HI_MPI_ISP_GetStatisticsConfig", "MPI_ISP_GetStatisticsConfig", "GK_API_ISP_GetStatisticsConfig", NULL });
    isp_lib->fnSetStatisticsConfig = (int(*)(int, const void*))v4_dlsym_multi(
        handles_isp, (const char*[]){ "HI_MPI_ISP_SetStatisticsConfig", "MPI_ISP_SetStatisticsConfig", "GK_API_ISP_SetStatisticsConfig", NULL });

    // AWB/Color helpers (often exported from AWB libs)
    isp_lib->fnGetCCMAttr = (int(*)(int, void*))v4_dlsym_multi(
        handles_awb, (const char*[]){ "HI_MPI_ISP_GetCCMAttr", "MPI_ISP_GetCCMAttr", "GK_API_ISP_GetCCMAttr", NULL });
    isp_lib->fnSetCCMAttr = (int(*)(int, const void*))v4_dlsym_multi(
        handles_awb, (const char*[]){ "HI_MPI_ISP_SetCCMAttr", "MPI_ISP_SetCCMAttr", "GK_API_ISP_SetCCMAttr", NULL });
    isp_lib->fnGetSaturationAttr = (int(*)(int, void*))v4_dlsym_multi(
        handles_awb, (const char*[]){ "HI_MPI_ISP_GetSaturationAttr", "MPI_ISP_GetSaturationAttr", "GK_API_ISP_GetSaturationAttr", NULL });
    isp_lib->fnSetSaturationAttr = (int(*)(int, const void*))v4_dlsym_multi(
        handles_awb, (const char*[]){ "HI_MPI_ISP_SetSaturationAttr", "MPI_ISP_SetSaturationAttr", "GK_API_ISP_SetSaturationAttr", NULL });

    // Core ISP attributes
    isp_lib->fnGetDRCAttr = (int(*)(int, void*))v4_dlsym_multi(
        handles_isp, (const char*[]){ "HI_MPI_ISP_GetDRCAttr", "MPI_ISP_GetDRCAttr", "GK_API_ISP_GetDRCAttr", NULL });
    isp_lib->fnSetDRCAttr = (int(*)(int, const void*))v4_dlsym_multi(
        handles_isp, (const char*[]){ "HI_MPI_ISP_SetDRCAttr", "MPI_ISP_SetDRCAttr", "GK_API_ISP_SetDRCAttr", NULL });
    isp_lib->fnGetModuleControl = (int(*)(int, void*))v4_dlsym_multi(
        handles_isp, (const char*[]){ "HI_MPI_ISP_GetModuleControl", "MPI_ISP_GetModuleControl", "GK_API_ISP_GetModuleControl", NULL });
    isp_lib->fnSetModuleControl = (int(*)(int, const void*))v4_dlsym_multi(
        handles_isp, (const char*[]){ "HI_MPI_ISP_SetModuleControl", "MPI_ISP_SetModuleControl", "GK_API_ISP_SetModuleControl", NULL });
    isp_lib->fnGetDehazeAttr = (int(*)(int, void*))v4_dlsym_multi(
        handles_isp_ext, (const char*[]){ "HI_MPI_ISP_GetDehazeAttr", "MPI_ISP_GetDehazeAttr", "GK_API_ISP_GetDehazeAttr", NULL });
    isp_lib->fnSetDehazeAttr = (int(*)(int, const void*))v4_dlsym_multi(
        handles_isp_ext, (const char*[]){ "HI_MPI_ISP_SetDehazeAttr", "MPI_ISP_SetDehazeAttr", "GK_API_ISP_SetDehazeAttr", NULL });
    isp_lib->fnGetLDCIAttr = (int(*)(int, void*))v4_dlsym_multi(
        handles_isp_ext, (const char*[]){ "HI_MPI_ISP_GetLDCIAttr", "MPI_ISP_GetLDCIAttr", "GK_API_ISP_GetLDCIAttr", NULL });
    isp_lib->fnSetLDCIAttr = (int(*)(int, const void*))v4_dlsym_multi(
        handles_isp_ext, (const char*[]){ "HI_MPI_ISP_SetLDCIAttr", "MPI_ISP_SetLDCIAttr", "GK_API_ISP_SetLDCIAttr", NULL });
    isp_lib->fnGetNRAttr = (int(*)(int, void*))v4_dlsym_multi(
        handles_isp, (const char*[]){ "HI_MPI_ISP_GetNRAttr", "MPI_ISP_GetNRAttr", "GK_API_ISP_GetNRAttr", NULL });
    isp_lib->fnSetNRAttr = (int(*)(int, const void*))v4_dlsym_multi(
        handles_isp, (const char*[]){ "HI_MPI_ISP_SetNRAttr", "MPI_ISP_SetNRAttr", "GK_API_ISP_SetNRAttr", NULL });
    isp_lib->fnGetGammaAttr = (int(*)(int, void*))v4_dlsym_multi(
        handles_isp, (const char*[]){ "HI_MPI_ISP_GetGammaAttr", "MPI_ISP_GetGammaAttr", "GK_API_ISP_GetGammaAttr", NULL });
    isp_lib->fnSetGammaAttr = (int(*)(int, const void*))v4_dlsym_multi(
        handles_isp, (const char*[]){ "HI_MPI_ISP_SetGammaAttr", "MPI_ISP_SetGammaAttr", "GK_API_ISP_SetGammaAttr", NULL });
    isp_lib->fnGetIspSharpenAttr = (int(*)(int, void*))v4_dlsym_multi(
        handles_isp, (const char*[]){ "HI_MPI_ISP_GetIspSharpenAttr", "MPI_ISP_GetIspSharpenAttr", "GK_API_ISP_GetIspSharpenAttr", NULL });
    isp_lib->fnSetIspSharpenAttr = (int(*)(int, const void*))v4_dlsym_multi(
        handles_isp, (const char*[]){ "HI_MPI_ISP_SetIspSharpenAttr", "MPI_ISP_SetIspSharpenAttr", "GK_API_ISP_SetIspSharpenAttr", NULL });

    return EXIT_SUCCESS;
}

static void v4_isp_unload(v4_isp_impl *isp_lib) {
    if (isp_lib->handleGoke) dlclose(isp_lib->handleGoke);
    isp_lib->handleGoke = NULL;
    if (isp_lib->handleGokeAe) dlclose(isp_lib->handleGokeAe);
    isp_lib->handleGokeAe = NULL;
    if (isp_lib->handleAe) dlclose(isp_lib->handleAe);
    isp_lib->handleAe = NULL;
    if (isp_lib->handleGokeAwb) dlclose(isp_lib->handleGokeAwb);
    isp_lib->handleGokeAwb = NULL;
    if (isp_lib->handleAwb) dlclose(isp_lib->handleAwb);
    isp_lib->handleAwb = NULL;
    if (isp_lib->handleIrAuto) dlclose(isp_lib->handleIrAuto);
    isp_lib->handleIrAuto = NULL;
    if (isp_lib->handleLdci) dlclose(isp_lib->handleLdci);
    isp_lib->handleLdci = NULL;
    if (isp_lib->handleDrc) dlclose(isp_lib->handleDrc);
    isp_lib->handleDrc = NULL;
    if (isp_lib->handleDehaze) dlclose(isp_lib->handleDehaze);
    isp_lib->handleDehaze = NULL;
    if (isp_lib->handleAcs) dlclose(isp_lib->handleAcs);
    isp_lib->handleAcs = NULL;
    if (isp_lib->handleMpi) dlclose(isp_lib->handleMpi);
    isp_lib->handleMpi = NULL;
    if (isp_lib->handle) dlclose(isp_lib->handle);
    isp_lib->handle = NULL;
    if (isp_lib->handleCalcFlick) dlclose(isp_lib->handleCalcFlick);
    isp_lib->handleCalcFlick = NULL;
    memset(isp_lib, 0, sizeof(*isp_lib));
}
