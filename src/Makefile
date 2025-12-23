# MP3 (Shine) support removed; don't compile Shine sources.
SRC_LOCAL = $(shell find ./ -name '*.c' ! -path './lib/shine/*')
SMOLRTSP_LOCAL = $(shell find ../3dparty/smolrtsp/src -name '*.c')
SMOLLE_LOCAL = $(shell find ../3dparty/smolrtsp-libevent/src -name '*.c')
FAAC_LOCAL = $(shell find ../3dparty/faac/libfaac -name '*.c')

# Store sources as paths relative to repo root (no leading ./ or ../).
# This lets us build all objects into a toolchain/ABI-specific OBJDIR without
# accidentally escaping it via ".." path segments.
SRC = $(patsubst ./%,src/%,$(SRC_LOCAL))
SMOLRTSP_SRC = $(patsubst ../%,%,$(SMOLRTSP_LOCAL))
SMOLLE_SRC = $(patsubst ../%,%,$(SMOLLE_LOCAL))
FAAC_SRC = $(patsubst ../%,%,$(FAAC_LOCAL))

# libfyaml (YAML 1.2 parser/emitter) is vendored as sources.
# We build it as a per-toolchain/per-ABI static archive to avoid autotools/cmake.
LIBFYAML_SRC = \
	3dparty/libfyaml/src/lib/fy-parse.c \
	3dparty/libfyaml/src/lib/fy-types.c \
	3dparty/libfyaml/src/lib/fy-diag.c \
	3dparty/libfyaml/src/lib/fy-dump.c \
	3dparty/libfyaml/src/lib/fy-atom.c \
	3dparty/libfyaml/src/lib/fy-token.c \
	3dparty/libfyaml/src/lib/fy-input.c \
	3dparty/libfyaml/src/lib/fy-docstate.c \
	3dparty/libfyaml/src/lib/fy-doc.c \
	3dparty/libfyaml/src/lib/fy-docbuilder.c \
	3dparty/libfyaml/src/lib/fy-emit.c \
	3dparty/libfyaml/src/lib/fy-event.c \
	3dparty/libfyaml/src/lib/fy-accel.c \
	3dparty/libfyaml/src/lib/fy-walk.c \
	3dparty/libfyaml/src/lib/fy-path.c \
	3dparty/libfyaml/src/lib/fy-composer.c \
	3dparty/libfyaml/src/lib/fy-parse-diag.c \
	3dparty/libfyaml/src/lib/fy-input-diag.c \
	3dparty/libfyaml/src/lib/fy-doc-diag.c \
	3dparty/libfyaml/src/lib/fy-docbuilder-diag.c \
	3dparty/libfyaml/src/lib/fy-composer-diag.c \
	3dparty/libfyaml/src/util/fy-ctype.c \
	3dparty/libfyaml/src/util/fy-utf8.c \
	3dparty/libfyaml/src/util/fy-utils.c \
	3dparty/libfyaml/src/util/fy-blob.c \
	3dparty/libfyaml/src/xxhash/xxhash.c \
	3dparty/libfyaml/src/thread/fy-thread.c \
	3dparty/libfyaml/src/allocator/fy-allocator.c \
	3dparty/libfyaml/src/allocator/fy-allocator-linear.c \
	3dparty/libfyaml/src/allocator/fy-allocator-malloc.c \
	3dparty/libfyaml/src/allocator/fy-allocator-mremap.c \
	3dparty/libfyaml/src/allocator/fy-allocator-dedup.c \
	3dparty/libfyaml/src/allocator/fy-allocator-auto.c

# SpeexDSP (preprocessor: denoise/AGC/VAD/dereverb) is vendored as sources.
# We build it as a static archive similar to FAAC to avoid autotools.
SPEEXDSP_SRC = \
	3dparty/speexdsp/libspeexdsp/preprocess.c \
	3dparty/speexdsp/libspeexdsp/jitter.c \
	3dparty/speexdsp/libspeexdsp/mdf.c \
	3dparty/speexdsp/libspeexdsp/fftwrap.c \
	3dparty/speexdsp/libspeexdsp/filterbank.c \
	3dparty/speexdsp/libspeexdsp/resample.c \
	3dparty/speexdsp/libspeexdsp/buffer.c \
	3dparty/speexdsp/libspeexdsp/scal.c \
	3dparty/speexdsp/libspeexdsp/kiss_fft.c \
	3dparty/speexdsp/libspeexdsp/kiss_fftr.c

OBJ = ../divinus
INCLUDES = -I../3dparty -I../3dparty/metalang99/include -I../3dparty/smolrtsp/include -I../3dparty/smolrtsp-libevent/include -I../3dparty/faac/include -I../3dparty/faac/libfaac -I../3dparty/speexdsp/include -I../3dparty/speexdsp/libspeexdsp -I../3dparty/libfyaml/include

# libfyaml internal includes (required to build vendored sources).
LIBFYAML_INCLUDES = \
	-I../3dparty/libfyaml/src \
	-I../3dparty/libfyaml/src/lib \
	-I../3dparty/libfyaml/src/util \
	-I../3dparty/libfyaml/src/thread \
	-I../3dparty/libfyaml/src/allocator \
	-I../3dparty/libfyaml/src/xxhash \
	-I../3dparty/libfyaml/include

# Put all objects into a per-toolchain/per-ABI directory to avoid mixing .o
# built with different float ABIs (soft/softfp/hard) across toolchains.
TOOLCHAIN ?= $(shell $(CC) -dumpmachine 2>/dev/null || echo unknown)

# NOTE ABOUT LTO CACHES:
# If LTO is enabled, cached static libraries (like libfaac.a) contain GCC LTO
# bytecode. GCC 10+ cannot read LTO bytecode produced by GCC <10, so upgrading
# the toolchain while reusing cached archives can fail at link with:
#   "bytecode stream ... generated with GCC compiler older than 10.0"
# To make builds robust across toolchain upgrades, include an LTO/compiler tag
# in cache directories.
GCC_VERSION := $(shell $(CC) -dumpfullversion 2>/dev/null || $(CC) -dumpversion 2>/dev/null || echo unknown)
GCC_MAJOR := $(shell echo "$(GCC_VERSION)" | sed 's/\..*//')
CACHE_TAG = $(if $(filter 1,$(LTO)),lto-gcc$(GCC_MAJOR),nolto)
FLOATABI ?= unknown
ifneq (,$(findstring -mfloat-abi=hard,$(CFLAGS) $(CPPFLAGS)))
  FLOATABI := hard
else ifneq (,$(findstring -mfloat-abi=softfp,$(CFLAGS) $(CPPFLAGS)))
  FLOATABI := softfp
else ifneq (,$(findstring -mfloat-abi=soft,$(CFLAGS) $(CPPFLAGS)))
  FLOATABI := soft
endif
# If flags don't specify ABI, infer a sensible default from the compiler tuple.
# OpenIPC toolchains use ...eabi(hf) naming for ARM.
ifeq ($(FLOATABI),unknown)
  ifneq (,$(filter arm%,$(TOOLCHAIN)))
    ifneq (,$(findstring hf,$(TOOLCHAIN)))
      FLOATABI := hard
    else
      FLOATABI := soft
    endif
  endif
endif
OBJDIR ?= ../build/$(TOOLCHAIN)-$(FLOATABI)-$(CACHE_TAG)

# Cache FAAC separately so switching/cleaning app objects doesn't rebuild it.
FAAC_CACHEDIR ?= ../build/faac/$(TOOLCHAIN)-$(FLOATABI)-$(CACHE_TAG)
FAAC_LIB = $(FAAC_CACHEDIR)/libfaac.a

# Cache libfyaml separately as a static archive.
LIBFYAML_CACHEDIR ?= ../build/libfyaml/$(TOOLCHAIN)-$(FLOATABI)-$(CACHE_TAG)
LIBFYAML_LIB = $(LIBFYAML_CACHEDIR)/libfyaml.a

# Cache smolrtsp + smolrtsp-libevent separately as static archives.
SMOLRTSP_CACHEDIR ?= ../build/smolrtsp/$(TOOLCHAIN)-$(FLOATABI)-$(CACHE_TAG)
SMOLRTSP_LIB = $(SMOLRTSP_CACHEDIR)/libsmolrtsp.a
SMOLLE_CACHEDIR ?= ../build/smolrtsp-libevent/$(TOOLCHAIN)-$(FLOATABI)-$(CACHE_TAG)
SMOLLE_LIB = $(SMOLLE_CACHEDIR)/libsmolrtsp-libevent.a

# Cache libevent separately as a per-toolchain/per-ABI static build.
LIBEVENT_SRCDIR := $(abspath ../3dparty/libevent-2.1.12)
LIBEVENT_CACHEDIR ?= $(abspath ../build/libevent/$(TOOLCHAIN)-$(FLOATABI)-$(CACHE_TAG))
LIBEVENT_CONFIG_H := $(LIBEVENT_CACHEDIR)/include/event2/event-config.h
LIBEVENT_CORE_LIB := $(LIBEVENT_CACHEDIR)/.libs/libevent_core.a
LIBEVENT_EXTRA_LIB := $(LIBEVENT_CACHEDIR)/.libs/libevent_extra.a
LIBEVENT_PTHREADS_LIB := $(LIBEVENT_CACHEDIR)/.libs/libevent_pthreads.a
LIBEVENT_LIBS := $(LIBEVENT_CORE_LIB) $(LIBEVENT_EXTRA_LIB) $(LIBEVENT_PTHREADS_LIB)
LIBEVENT_STAMP := $(LIBEVENT_CACHEDIR)/.built

# Prefer our vendored libevent headers. NOTE: event-config.h is generated into
# $(LIBEVENT_CACHEDIR)/include, so keep it early in include search path.
INCLUDES += -I$(LIBEVENT_CACHEDIR)/include -I$(LIBEVENT_SRCDIR)/include

# Link libevent statically from our cached build, removing external dependency.
LDLIBS = $(LIBEVENT_LIBS) -lpthread -lm

# Cache SpeexDSP separately as a static archive.
SPEEXDSP_CACHEDIR ?= ../build/speexdsp/$(TOOLCHAIN)-$(FLOATABI)-$(CACHE_TAG)
SPEEXDSP_LIB = $(SPEEXDSP_CACHEDIR)/libspeexdsp.a

DIVINUS_OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SRC))
FAAC_OBJS = $(patsubst %.c,$(FAAC_CACHEDIR)/%.o,$(FAAC_SRC))
SMOLRTSP_OBJS = $(patsubst %.c,$(SMOLRTSP_CACHEDIR)/%.o,$(SMOLRTSP_SRC))
SMOLLE_OBJS = $(patsubst %.c,$(SMOLLE_CACHEDIR)/%.o,$(SMOLLE_SRC))
SPEEXDSP_OBJS = $(patsubst %.c,$(SPEEXDSP_CACHEDIR)/%.o,$(SPEEXDSP_SRC))
LIBFYAML_OBJS = $(patsubst %.c,$(LIBFYAML_CACHEDIR)/%.o,$(LIBFYAML_SRC))

# Default to a size-optimized release build unless OPT is provided by the caller.
# Per request: build everything with -Os, and only faac + speex with -O2.
OPT ?= -Os -pipe -s

# "Fast" optimization (used for faac and (optionally) speex sources).
FASTOPT ?= -O2 -pipe

# Avoid passing linker-only flags (like -s) to compile-only steps.
OPT_C := $(filter-out -s,$(OPT))
FASTOPT_C := $(filter-out -s,$(FASTOPT))

# Size optimizations:
# - Put each function/data item in its own section so the linker can garbage-collect
#   unused code from static libraries (libfyaml, faac, speexdsp, etc.).
# - Enable section GC on ELF linkers; use -dead_strip on Darwin.
SECTIONS_CFLAGS ?= -ffunction-sections -fdata-sections
CFLAGS += $(SECTIONS_CFLAGS)
ifneq (,$(findstring darwin,$(TOOLCHAIN)))
  LDFLAGS += -Wl,-dead_strip
else
  LDFLAGS += -Wl,--gc-sections
endif

# Export only the ISP compatibility symbols needed by vendor libs (e.g. Goke v4)
# instead of full -rdynamic to keep binary size small.
ISP_EXPORT_SYMS := \
	-Wl,--export-dynamic-symbol=isp_malloc \
	-Wl,--export-dynamic-symbol=isp_alg_register_acs \
	-Wl,--export-dynamic-symbol=isp_alg_register_dehaze \
	-Wl,--export-dynamic-symbol=ISP_AlgRegisterDehaze \
	-Wl,--export-dynamic-symbol=isp_alg_register_drc \
	-Wl,--export-dynamic-symbol=ISP_AlgRegisterDrc \
	-Wl,--export-dynamic-symbol=isp_alg_register_ldci \
	-Wl,--export-dynamic-symbol=ISP_AlgRegisterLdci \
	-Wl,--export-dynamic-symbol=isp_ir_auto_run_once \
	-Wl,--export-dynamic-symbol=MPI_ISP_IrAutoRunOnce
LDFLAGS += $(ISP_EXPORT_SYMS)
RDYNAMIC :=

# Extra flags for vendored libfyaml compilation only.
LIBFYAML_CFLAGS ?= -DNDEBUG

# Enable LTO by default.
LTO ?= 1
ifeq ($(LTO),1)
  LTOFLAGS := -flto
else
  LTOFLAGS :=
endif

# If any "speex" sources appear in the tree later, compile them with FASTOPT.
FAST_SRCPATTERNS := ../%/speex/% ../%/speexdsp/%
COPT_FOR = $(if $(filter $(FAST_SRCPATTERNS),$(1)),$(FASTOPT_C),$(OPT_C))

.PHONY: clean distclean faac-clean libfyaml-clean speex-clean smol-clean libevent-clean divinus
divinus: $(OBJ)

CPPFLAGS += -DDIVINUS_WITH_SPEEXDSP

$(OBJ): $(DIVINUS_OBJS) $(FAAC_LIB) $(SPEEXDSP_LIB) $(SMOLRTSP_LIB) $(SMOLLE_LIB) $(LIBFYAML_LIB) $(LIBEVENT_STAMP)
	$(CC) $(DIVINUS_OBJS) $(RDYNAMIC) $(OPT) $(LDFLAGS) $(FAAC_LIB) $(SPEEXDSP_LIB) $(SMOLLE_LIB) $(SMOLRTSP_LIB) $(LIBFYAML_LIB) $(LDLIBS) -o $@

$(OBJDIR)/%.o: ../%.c Makefile $(LIBEVENT_CONFIG_H)
	@mkdir -p $(dir $@)
	$(CC) -c $< $(call COPT_FOR,$<) $(LTOFLAGS) $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -DPACKAGE_VERSION=\"faac-bundled\" -o $@

ifeq ($(LTO),1)
AR ?= $(shell $(CC) -print-prog-name=gcc-ar 2>/dev/null || $(CC) -print-prog-name=ar 2>/dev/null || echo ar)
RANLIB ?= $(shell $(CC) -print-prog-name=gcc-ranlib 2>/dev/null || $(CC) -print-prog-name=ranlib 2>/dev/null || echo ranlib)
else
AR ?= $(shell $(CC) -print-prog-name=ar 2>/dev/null || echo ar)
RANLIB ?= $(shell $(CC) -print-prog-name=ranlib 2>/dev/null || echo ranlib)
endif

$(FAAC_CACHEDIR)/%.o: ../%.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $< $(FASTOPT_C) $(LTOFLAGS) $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -DPACKAGE_VERSION=\"faac-bundled\" -o $@

$(FAAC_LIB): $(FAAC_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^
	$(RANLIB) $@ >/dev/null 2>&1 || true

$(LIBFYAML_CACHEDIR)/%.o: ../%.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $< $(OPT_C) $(LTOFLAGS) $(CPPFLAGS) $(CFLAGS) $(LIBFYAML_CFLAGS) $(INCLUDES) $(LIBFYAML_INCLUDES) -o $@

$(LIBFYAML_LIB): $(LIBFYAML_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^
	$(RANLIB) $@ >/dev/null 2>&1 || true

$(SMOLRTSP_CACHEDIR)/%.o: ../%.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $< $(OPT_C) $(LTOFLAGS) $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -o $@

$(SMOLRTSP_LIB): $(SMOLRTSP_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^
	$(RANLIB) $@ >/dev/null 2>&1 || true

$(SMOLLE_CACHEDIR)/%.o: ../%.c Makefile $(LIBEVENT_CONFIG_H)
	@mkdir -p $(dir $@)
	$(CC) -c $< $(OPT_C) $(LTOFLAGS) $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -o $@

$(SMOLLE_LIB): $(SMOLLE_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^
	$(RANLIB) $@ >/dev/null 2>&1 || true

$(LIBEVENT_CONFIG_H): $(LIBEVENT_SRCDIR)/configure
	@mkdir -p $(LIBEVENT_CACHEDIR)
	@if [ ! -f "$(LIBEVENT_CACHEDIR)/Makefile" ]; then \
		echo "Configuring libevent in $(LIBEVENT_CACHEDIR)"; \
		cd "$(LIBEVENT_CACHEDIR)" && \
			"$(LIBEVENT_SRCDIR)/configure" \
				--host="$(TOOLCHAIN)" \
				--disable-shared \
				--enable-static \
				--disable-openssl \
				--disable-samples \
				--disable-libevent-regress \
				CC="$(CC)" \
				AR="$(AR)" \
				RANLIB="$(RANLIB)" \
				CFLAGS="$(OPT_C) $(LTOFLAGS) $(CFLAGS)" \
				CPPFLAGS="$(CPPFLAGS)"; \
	fi
	$(MAKE) -C "$(LIBEVENT_CACHEDIR)" include/event2/event-config.h

$(LIBEVENT_STAMP): $(LIBEVENT_CONFIG_H)
	# libevent's autotools build is occasionally racy under -j; build it serially.
	$(MAKE) -C "$(LIBEVENT_CACHEDIR)" -j1
	@touch $@

# SpeexDSP build flags (requested defaults):
# - --enable-neon
# - --enable-low-accuracy
SPEEXDSP_ENABLE_NEON ?= 1
SPEEXDSP_ENABLE_LOW_ACCURACY ?= 1

# NOTE:
# - AGC in SpeexDSP 1.2.1 preprocess is only available in FLOATING_POINT builds.
# - ARM4/ARM5E asm paths are only available in FIXED_POINT builds.
# So by default we keep FLOATING_POINT to support AGC (denoise/VAD/noise suppress/AGC).
SPEEXDSP_FIXED_POINT ?= 0

SPEEXDSP_CFLAGS =
ifeq ($(SPEEXDSP_FIXED_POINT),1)
  SPEEXDSP_CFLAGS += -DFIXED_POINT
else
  SPEEXDSP_CFLAGS += -DFLOATING_POINT
endif

# SpeexDSP's `fftwrap.c` requires selecting an FFT backend.
# We vendor `kiss_fft.c`/`kiss_fftr.c`, so use KISS FFT by default.
SPEEXDSP_CFLAGS += -DUSE_KISS_FFT

ifneq (,$(filter arm%,$(TOOLCHAIN)))
  ifeq ($(SPEEXDSP_ENABLE_NEON),1)
    # For AArch64, NEON is mandatory; don't force incompatible -mfpu/-march flags.
    ifneq (,$(findstring aarch64,$(TOOLCHAIN)))
      SPEEXDSP_CFLAGS += -DUSE_NEON
    else ifneq (,$(findstring arm64,$(TOOLCHAIN)))
      SPEEXDSP_CFLAGS += -DUSE_NEON
    else
      SPEEXDSP_CFLAGS += -DUSE_NEON -march=armv7-a -mfpu=neon
    endif
  endif
endif

ifeq ($(SPEEXDSP_ENABLE_LOW_ACCURACY),1)
  # Equivalent of --enable-low-accuracy: allow faster but less precise math.
  # Only meaningful for floating-point builds.
  ifneq ($(SPEEXDSP_FIXED_POINT),1)
    SPEEXDSP_CFLAGS += -ffast-math -fno-math-errno -funsafe-math-optimizations
  endif
endif

$(SPEEXDSP_CACHEDIR)/%.o: ../%.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $< $(OPT) $(CPPFLAGS) $(CFLAGS) $(SPEEXDSP_CFLAGS) $(INCLUDES) -o $@

$(SPEEXDSP_LIB): $(SPEEXDSP_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^
	$(RANLIB) $@ >/dev/null 2>&1 || true

clean:
	rm -rf $(OBJDIR) $(OBJ)

faac-clean:
	rm -rf $(FAAC_CACHEDIR)

libfyaml-clean:
	rm -rf $(LIBFYAML_CACHEDIR)

speex-clean:
	rm -rf $(SPEEXDSP_CACHEDIR)

smol-clean:
	rm -rf $(SMOLRTSP_CACHEDIR) $(SMOLLE_CACHEDIR)

libevent-clean:
	rm -rf $(LIBEVENT_CACHEDIR)

distclean: clean faac-clean libfyaml-clean speex-clean smol-clean libevent-clean
