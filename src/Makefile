SRC_LOCAL = $(shell find ./ -name '*.c')
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

OBJ = ../divinus
INCLUDES = -I../3dparty -I../3dparty/metalang99/include -I../3dparty/smolrtsp/include -I../3dparty/smolrtsp-libevent/include -I../3dparty/faac/include -I../3dparty/faac/libfaac
LDLIBS = -levent_core -levent_extra -levent_pthreads -lpthread -lm

# Put all objects into a per-toolchain/per-ABI directory to avoid mixing .o
# built with different float ABIs (soft/softfp/hard) across toolchains.
TOOLCHAIN ?= $(shell $(CC) -dumpmachine 2>/dev/null || echo unknown)
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
OBJDIR ?= ../build/$(TOOLCHAIN)-$(FLOATABI)

# Cache FAAC separately so switching/cleaning app objects doesn't rebuild it.
FAAC_CACHEDIR ?= ../build/faac/$(TOOLCHAIN)-$(FLOATABI)
FAAC_LIB = $(FAAC_CACHEDIR)/libfaac.a

# Cache smolrtsp + smolrtsp-libevent separately as static archives.
SMOLRTSP_CACHEDIR ?= ../build/smolrtsp/$(TOOLCHAIN)-$(FLOATABI)
SMOLRTSP_LIB = $(SMOLRTSP_CACHEDIR)/libsmolrtsp.a
SMOLLE_CACHEDIR ?= ../build/smolrtsp-libevent/$(TOOLCHAIN)-$(FLOATABI)
SMOLLE_LIB = $(SMOLLE_CACHEDIR)/libsmolrtsp-libevent.a

DIVINUS_OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SRC))
FAAC_OBJS = $(patsubst %.c,$(FAAC_CACHEDIR)/%.o,$(FAAC_SRC))
SMOLRTSP_OBJS = $(patsubst %.c,$(SMOLRTSP_CACHEDIR)/%.o,$(SMOLRTSP_SRC))
SMOLLE_OBJS = $(patsubst %.c,$(SMOLLE_CACHEDIR)/%.o,$(SMOLLE_SRC))

# Default to a size-optimized release build unless OPT is provided by the caller.
# Per request: build everything with -Os, and only faac + speex with -O2.
OPT ?= -Os -pipe -s

# "Fast" optimization (used for faac and (optionally) speex sources).
FASTOPT ?= -O2 -pipe

# Avoid passing linker-only flags (like -s) to compile-only steps.
OPT_C := $(filter-out -s,$(OPT))
FASTOPT_C := $(filter-out -s,$(FASTOPT))

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

.PHONY: clean distclean faac-clean smol-clean divinus
divinus: $(OBJ)

$(OBJ): $(DIVINUS_OBJS) $(FAAC_LIB) $(SMOLRTSP_LIB) $(SMOLLE_LIB)
	$(CC) $(DIVINUS_OBJS) -rdynamic $(OPT) $(LTOFLAGS) $(LDFLAGS) $(FAAC_LIB) $(SMOLLE_LIB) $(SMOLRTSP_LIB) $(LDLIBS) -o $@

$(OBJDIR)/%.o: ../%.c Makefile
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

$(SMOLRTSP_CACHEDIR)/%.o: ../%.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $< $(OPT_C) $(LTOFLAGS) $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -o $@

$(SMOLRTSP_LIB): $(SMOLRTSP_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^
	$(RANLIB) $@ >/dev/null 2>&1 || true

$(SMOLLE_CACHEDIR)/%.o: ../%.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $< $(OPT_C) $(LTOFLAGS) $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -o $@

$(SMOLLE_LIB): $(SMOLLE_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^
	$(RANLIB) $@ >/dev/null 2>&1 || true

clean:
	rm -rf $(OBJDIR) $(OBJ)

faac-clean:
	rm -rf $(FAAC_CACHEDIR)

smol-clean:
	rm -rf $(SMOLRTSP_CACHEDIR) $(SMOLLE_CACHEDIR)

distclean: clean faac-clean smol-clean
