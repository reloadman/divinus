#ifdef __arm__

void* (*fnIsp_Malloc)(unsigned long);
int   (*fnISP_AlgRegisterAcs)(int);
int   (*fnISP_AlgRegisterDehaze)(int);
int   (*fnISP_AlgRegisterDrc)(int);
int   (*fnISP_AlgRegisterLdci)(int);
int   (*fnMPI_ISP_IrAutoRunOnce)(int, void*);

__attribute__((used)) void *isp_malloc(unsigned long size) {
    return fnIsp_Malloc(size);
}
__attribute__((used)) int isp_alg_register_acs(int pipeId) {
    return fnISP_AlgRegisterAcs(pipeId);
}
__attribute__((used)) int isp_alg_register_dehaze(int pipeId) {
    return fnISP_AlgRegisterDehaze(pipeId);
}
__attribute__((used)) int ISP_AlgRegisterDehaze(int pipeId) {
    return fnISP_AlgRegisterDehaze(pipeId);
}
__attribute__((used)) int isp_alg_register_drc(int pipeId) {
    return fnISP_AlgRegisterDrc(pipeId);
}
__attribute__((used)) int ISP_AlgRegisterDrc(int pipeId) {
    return fnISP_AlgRegisterDrc(pipeId);
}
__attribute__((used)) int isp_alg_register_ldci(int pipeId) {
    return fnISP_AlgRegisterLdci(pipeId);
}
__attribute__((used)) int ISP_AlgRegisterLdci(int pipeId) {
    return fnISP_AlgRegisterLdci(pipeId);
}
__attribute__((used)) int isp_ir_auto_run_once(int pipeId, void *irAttr) {
    return fnMPI_ISP_IrAutoRunOnce(pipeId, irAttr);
}
__attribute__((used)) int MPI_ISP_IrAutoRunOnce(int pipeId, void *irAttr) {
    return fnMPI_ISP_IrAutoRunOnce(pipeId, irAttr);
}

/* Keep wrappers alive even with aggressive GC/linker stripping. */
static __attribute__((used)) void *isp_quirks_keep[] = {
    (void *)isp_malloc,
    (void *)isp_alg_register_acs,
    (void *)isp_alg_register_dehaze,
    (void *)ISP_AlgRegisterDehaze,
    (void *)isp_alg_register_drc,
    (void *)ISP_AlgRegisterDrc,
    (void *)isp_alg_register_ldci,
    (void *)ISP_AlgRegisterLdci,
    (void *)isp_ir_auto_run_once,
    (void *)MPI_ISP_IrAutoRunOnce,
};

#endif