SRC = $(shell find ./ -name '*.c')
SMOL_SRC = $(shell find ../3dparty/smolrtsp/src ../3dparty/smolrtsp-libevent/src -name '*.c')
FAAC_SRC = $(shell find ../3dparty/faac/libfaac -name '*.c')
OBJ = ../divinus
INCLUDES = -I../3dparty -I../3dparty/metalang99/include -I../3dparty/smolrtsp/include -I../3dparty/smolrtsp-libevent/include -I../3dparty/faac/include -I../3dparty/faac/libfaac
LDLIBS = -levent_core -levent_extra -levent_pthreads -lpthread -lm

# Default to an optimized release build unless OPT is provided by the caller.
OPT ?= -O2 -pipe -s

.PHONY: clean divinus
divinus: $(OBJ)

$(OBJ): $(SRC:%.c=%.o) $(SMOL_SRC:%.c=%.o) $(FAAC_SRC:%.c=%.o)
	$(CC) $^ -rdynamic $(OPT) $(LDLIBS) -o $@

%.o: %.c Makefile
	$(CC) -c $< $(OPT) $(INCLUDES) -DPACKAGE_VERSION=\"faac-bundled\" -o $@

clean:
	rm -rf $(SRC:%.c=%.o) $(SMOL_SRC:%.c=%.o) $(OBJ)
