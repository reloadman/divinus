#ifndef __SPEEX_TYPES_H__
#define __SPEEX_TYPES_H__

// Compatibility shim for vendored SpeexDSP build without autotools.
// Upstream `speexdsp_types.h` includes "speexdsp_config_types.h" (no "speex/" prefix),
// so provide this header at include root and forward to the actual file.

#include "speex/speexdsp_config_types.h"

#endif

