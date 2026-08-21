# PC test build for the encoder firmware logic (C++).
# Run from this directory:  mingw32-make run   (or make run)
# Links App/ logic against the simulated hardware instead of hal_stm32.cpp.

CC    = g++   # override on the command line if your g++ lives elsewhere
CFLAGS = -std=c++11 -fno-exceptions -fno-rtti -fno-threadsafe-statics \
         -Wall -Wextra -g -O0 -I../App

# Integration harness: App/ logic + simulated hardware.
SRCS = sim_main.cpp hal_sim.cpp mt6701_slave_sim.cpp \
       ../App/mt6701.cpp ../App/gear_decode.cpp ../App/i2c_pos.cpp \
       ../App/app_entry.cpp

BIN = build/enc-sim

# Unit test: the GearDecoder public API only (no simulated hardware).
SRCS_UNIT = test_gear_decode.cpp ../App/gear_decode.cpp

BIN_UNIT = build/test-gear-decode

$(BIN): $(SRCS) ../App/hal.h ../App/mt6701.h ../App/gear_decode.h ../App/gear_config.h ../App/i2c_pos.h ../App/app_entry.h mt6701_slave_sim.h
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