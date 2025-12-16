SRC = $(shell find ./ -name '*.c')
SMOL_SRC = $(shell find ../3dparty/smolrtsp/src ../3dparty/smolrtsp-libevent/src -name '*.c')
OBJ = ../divinus
INCLUDES = -I../3dparty -I../3dparty/metalang99/include -I../3dparty/smolrtsp/include -I../3dparty/smolrtsp-libevent/include
LDLIBS = -levent -levent_pthreads -lpthread

.PHONY: clean divinus
divinus: $(OBJ)

$(OBJ): $(SRC:%.c=%.o) $(SMOL_SRC:%.c=%.o)
	$(CC) $^ -rdynamic $(OPT) $(LDLIBS) -o $@

%.o: %.c Makefile
	$(CC) -c $< $(OPT) $(INCLUDES) -o $@

clean:
	rm -rf $(SRC:%.c=%.o) $(SMOL_SRC:%.c=%.o) $(OBJ)
