SRC_LOCAL = $(shell find ./ -name '*.c')
SMOL_LOCAL = $(shell find ../3dparty/smolrtsp/src ../3dparty/smolrtsp-libevent/src -name '*.c')
FAAC_LOCAL = $(shell find ../3dparty/faac/libfaac -name '*.c')

# Store sources as paths relative to repo root (no leading ./ or ../).
# This lets us build all objects into a toolchain/ABI-specific OBJDIR without
# accidentally escaping it via ".." path segments.
SRC = $(patsubst ./%,src/%,$(SRC_LOCAL))
SMOL_SRC = $(patsubst ../%,%,$(SMOL_LOCAL))
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

DIVINUS_OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SRC) $(SMOL_SRC))
FAAC_OBJS = $(patsubst %.c,$(FAAC_CACHEDIR)/%.o,$(FAAC_SRC))

# Default to an optimized release build unless OPT is provided by the caller.
OPT ?= -O2 -pipe -s

.PHONY: clean distclean faac-clean divinus
divinus: $(OBJ)

$(OBJ): $(DIVINUS_OBJS) $(FAAC_LIB)
	$(CC) $(DIVINUS_OBJS) -rdynamic $(OPT) $(LDFLAGS) $(FAAC_LIB) $(LDLIBS) -o $@

$(OBJDIR)/%.o: ../%.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $< $(OPT) $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -DPACKAGE_VERSION=\"faac-bundled\" -o $@

AR ?= $(shell $(CC) -print-prog-name=ar 2>/dev/null || echo ar)
RANLIB ?= $(shell $(CC) -print-prog-name=ranlib 2>/dev/null || echo ranlib)

$(FAAC_CACHEDIR)/%.o: ../%.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $< $(OPT) $(CPPFLAGS) $(CFLAGS) $(INCLUDES) -DPACKAGE_VERSION=\"faac-bundled\" -o $@

$(FAAC_LIB): $(FAAC_OBJS)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^
	$(RANLIB) $@ >/dev/null 2>&1 || true

clean:
	rm -rf $(OBJDIR) $(OBJ)

faac-clean:
	rm -rf $(FAAC_CACHEDIR)

distclean: clean faac-clean
