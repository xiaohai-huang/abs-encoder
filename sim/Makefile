# PC test build for the encoder firmware logic.
# Run from this directory:  mingw32-make run   (or make run)
# Links App/ logic against the simulated hardware instead of hal_stm32.c.

CC    = gcc   # override on the command line if your gcc lives elsewhere
CFLAGS = -std=c11 -Wall -Wextra -g -O0 -I../App

# Integration harness: App/ logic + simulated hardware.
SRCS = sim_main.c hal_sim.c mt6701_slave_sim.c \
       ../App/mt6701.c ../App/gear_decode.c ../App/i2c_pos.c

BIN = build/enc-sim

# Unit test: the gear_decode public API only (no simulated hardware).
SRCS_UNIT = test_gear_decode.c ../App/gear_decode.c

BIN_UNIT = build/test-gear-decode

$(BIN): $(SRCS) ../App/hal.h ../App/mt6701.h ../App/gear_decode.h ../App/gear_config.h ../App/i2c_pos.h mt6701_slave_sim.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SRCS) -o $@

$(BIN_UNIT): $(SRCS_UNIT) ../App/gear_decode.h ../App/gear_config.h ../App/mt6701.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SRCS_UNIT) -o $@

run: $(BIN) $(BIN_UNIT)
	./$(BIN)
	./$(BIN_UNIT)

clean:
	rm -rf build

.PHONY: run clean
