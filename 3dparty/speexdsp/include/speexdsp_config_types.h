#ifndef __SPEEX_TYPES_H__
#define __SPEEX_TYPES_H__

// Generated for Divinus vendored SpeexDSP build (no autotools).
//
// Upstream generates `include/speex/speexdsp_config_types.h` from
// `include/speex/speexdsp_config_types.h.in`, but that generated header is
// ignored by upstream `.gitignore`. We keep a stable, tracked version here
// because `speex/speexdsp_types.h` includes "speexdsp_config_types.h" without
// any prefix.

#include <stdint.h>

typedef int16_t spx_int16_t;
typedef uint16_t spx_uint16_t;
typedef int32_t spx_int32_t;
typedef uint32_t spx_uint32_t;

/*
 * Upstream SpeexDSP normally defines `EXPORT` in an autotools-generated
 * `config.h` (see `configure.ac`). Our vendored build compiles the sources
 * without that generated header, so provide a safe default here.
 */
#ifndef EXPORT
#  if defined(__GNUC__) || defined(__clang__)
#    define EXPORT __attribute__((visibility("default")))
#  else
#    define EXPORT
#  endif
#endif

#endif

