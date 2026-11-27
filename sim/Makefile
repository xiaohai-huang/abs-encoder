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

# Interactive console: same firmware logic and simulated hardware, driven
# from stdin instead of a script.
SRCS_SHELL = interactive_main.cpp hal_sim.cpp mt6701_slave_sim.cpp \
             ../App/mt6701.cpp ../App/gear_decode.cpp ../App/i2c_pos.cpp \
             ../App/app_entry.cpp

BIN_SHELL = build/enc-shell

# Web GUI: the same firmware logic behind a localhost HTTP server.
SRCS_GUI = gui_main.cpp hal_sim.cpp mt6701_slave_sim.cpp \
           ../App/mt6701.cpp ../App/gear_decode.cpp ../App/i2c_pos.cpp \
           ../App/app_entry.cpp

BIN_GUI = build/enc-gui

ifeq ($(OS),Windows_NT)
LDLIBS += -lws2_32
endif

$(BIN): $(SRCS) ../App/hal.h ../App/mt6701.h ../App/gear_decode.h ../App/gear_config.h ../App/i2c_pos.h ../App/app_entry.h mt6701_slave_sim.h gear_train_sim.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SRCS) -o $@

$(BIN_UNIT): $(SRCS_UNIT) ../App/gear_decode.h ../App/gear_config.h ../App/mt6701.h gear_train_sim.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SRCS_UNIT) -o $@

$(BIN_SHELL): $(SRCS_SHELL) ../App/hal.h ../App/mt6701.h ../App/gear_decode.h ../App/gear_config.h ../App/i2c_pos.h ../App/app_entry.h mt6701_slave_sim.h gear_train_sim.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SRCS_SHELL) -o $@

$(BIN_GUI): $(SRCS_GUI) ../App/hal.h ../App/mt6701.h ../App/gear_decode.h ../App/gear_config.h ../App/i2c_pos.h ../App/app_entry.h mt6701_slave_sim.h gear_train_sim.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(SRCS_GUI) -o $@ $(LDLIBS)

run: $(BIN) $(BIN_UNIT)
	./$(BIN)
	./$(BIN_UNIT)

interactive: $(BIN_SHELL)
	./$(BIN_SHELL)

gui: $(BIN_GUI)
	./$(BIN_GUI)

clean:
	rm -rf build

.PHONY: run interactive gui clean