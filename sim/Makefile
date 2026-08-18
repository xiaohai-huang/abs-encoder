# PC test build for the encoder firmware logic.
# Run from this directory:  mingw32-make run   (or make run)
# Links App/ logic against the simulated hardware instead of hal_stm32.c.

CC    = gcc   # override on the command line if your gcc lives elsewhere
CFLAGS = -std=c11 -Wall -Wextra -g -O0 -I../App

SRCS = sim_main.c hal_sim.c mt6701_slave_sim.c \
       ../App/mt6701.c

BIN = build/enc-sim

$(BIN): $(SRCS) ../App/hal.h ../App/mt6701.h mt6701_slave_sim.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SRCS) -o $@

run: $(BIN)
	./$(BIN)

clean:
	rm -rf build

.PHONY: run clean
