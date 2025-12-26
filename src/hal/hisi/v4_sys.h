#pragma once

#include "../sys_mod.h"
#include "v4_common.h"

#define V4_SYS_API "1.0"

typedef enum {
    V4_SYS_OPER_VIOFF_VPSSOFF,
    V4_SYS_OPER_VIOFF_VPSSON,
    V4_SYS_OPER_VION_VPSSOFF,
    V4_SYS_OPER_VION_VPSSON,
    V4_SYS_OPER_VIPARA_VPSSOFF,
    V4_SYS_OPER_VIPARA_VPSSPARA,
    V4_SYS_OPER_END
} v4_sys_oper;

typedef struct {
    v4_sys_mod module;
    int device;
    int channel;
} v4_sys_bind;

typedef struct {
    char version[64];
} v4_sys_ver;

typedef struct {
    void *handle, *handleGoke, *handleVoiceEngine, *handleDnvqe, *handleUpvqe, *handleSecureC;
    
    int (*fnExit)(void);
    int (*fnGetChipId)(unsigned int *chip);
    int (*fnGetVersion)(v4_sys_ver *version);
    
    int (*fnInit)(void);
    int (*fnSetAlignment)(unsigned int *width);

    int (*fnBind)(v4_sys_bind *source, v4_sys_bind *dest);
    int (*fnUnbind)(v4_sys_bind *source, v4_sys_bind *dest);

    int (*fnGetViVpssMode)(v4_sys_oper *mode);
    int (*fnSetViVpssMode)(v4_sys_oper *mode);
} v4_sys_impl;

static int v4_sys_load(v4_sys_impl *sys_lib) {
    if ((!(sys_lib->handleSecureC = dlopen("libsecurec.so", RTLD_LAZY | RTLD_GLOBAL)) ||
         !(sys_lib->handleUpvqe = dlopen("libupvqe.so", RTLD_LAZY | RTLD_GLOBAL)) ||
         !(sys_lib->handleDnvqe = dlopen("libdnvqe.so", RTLD_LAZY | RTLD_GLOBAL))) ||

       ((!(sys_lib->handleVoiceEngine = dlopen("libVoiceEngine.so", RTLD_LAZY | RTLD_GLOBAL)) ||
         !(sys_lib->handle = dlopen("libmpi.so", RTLD_LAZY | RTLD_GLOBAL))) &&

        (!(sys_lib->handleVoiceEngine = dlopen("libvoice_engine.so", RTLD_LAZY | RTLD_GLOBAL)) ||
         !(sys_lib->handleGoke = dlopen("libgk_api.so", RTLD_LAZY | RTLD_GLOBAL)) ||
         !(sys_lib->handle = dlopen("libhi_mpi.so", RTLD_LAZY | RTLD_GLOBAL)))))
        HAL_ERROR("v4_sys", "Failed to load library!\nError: %s\n", dlerror());

    if (!(sys_lib->fnExit = (int(*)(void))
        hal_symbol_load("v4_sys", sys_lib->handle, "HI_MPI_SYS_Exit")))
        return EXIT_FAILURE;

    if (!(sys_lib->fnGetChipId = (int(*)(unsigned int *chip))
        hal_symbol_load("v4_sys", sys_lib->handle, "HI_MPI_SYS_GetChipId")))
        return EXIT_FAILURE;

    if (!(sys_lib->fnGetVersion = (int(*)(v4_sys_ver *version))
        hal_symbol_load("v4_sys", sys_lib->handle, "HI_MPI_SYS_GetVersion")))
        return EXIT_FAILURE;

    if (!(sys_lib->fnInit = (int(*)(void))
        hal_symbol_load("v4_sys", sys_lib->handle, "HI_MPI_SYS_Init")))
        return EXIT_FAILURE;

    if (!(sys_lib->fnSetAlignment = (int(*)(unsigned int *width))
        hal_symbol_load("v4_sys", sys_lib->handle, "HI_MPI_SYS_SetConfig")))
        return EXIT_FAILURE;

    if (!(sys_lib->fnBind = (int(*)(v4_sys_bind *source, v4_sys_bind *dest))
        hal_symbol_load("v4_sys", sys_lib->handle, "HI_MPI_SYS_Bind")))
        return EXIT_FAILURE;

    if (!(sys_lib->fnUnbind = (int(*)(v4_sys_bind *source, v4_sys_bind *dest))
        hal_symbol_load("v4_sys", sys_lib->handle, "HI_MPI_SYS_UnBind")))
        return EXIT_FAILURE;

    if (!(sys_lib->fnGetViVpssMode = (int(*)(v4_sys_oper *mode))
        hal_symbol_load("v4_sys", sys_lib->handle, "HI_MPI_SYS_GetVIVPSSMode")))
        return EXIT_FAILURE;

    if (!(sys_lib->fnSetViVpssMode = (int(*)(v4_sys_oper *mode))
        hal_symbol_load("v4_sys", sys_lib->handle, "HI_MPI_SYS_SetVIVPSSMode")))
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}

static void v4_sys_unload(v4_sys_impl *sys_lib) {
    if (sys_lib->handleSecureC) dlclose(sys_lib->handleSecureC);
    sys_lib->handleSecureC = NULL;
    if (sys_lib->handleUpvqe) dlclose(sys_lib->handleUpvqe);
    sys_lib->handleUpvqe = NULL;
    if (sys_lib->handleDnvqe) dlclose(sys_lib->handleDnvqe);
    sys_lib->handleDnvqe = NULL;
    if (sys_lib->handleVoiceEngine) dlclose(sys_lib->handleVoiceEngine);
    sys_lib->handleVoiceEngine = NULL;
    if (sys_lib->handleGoke) dlclose(sys_lib->handleGoke);
    sys_lib->handleGoke = NULL;
    if (sys_lib->handle) dlclose(sys_lib->handle);
    sys_lib->handle = NULL;
    memset(sys_lib, 0, sizeof(*sys_lib));
}