# Obstacle-avoiding robot firmware — command-line build with Microchip XC8.
#
#   make            # -> build/obstacle-avoiding-robot.hex
#   make clean
#
# Requires XC8 v2.x (xc8-cc) on PATH.

CC     := xc8-cc
MCU    := 16F877A
BUILD  := build
TARGET := $(BUILD)/obstacle-avoiding-robot.hex
SRC    := src/main.c
CFLAGS := -mcpu=$(MCU) -O2 -std=c99 -Wall

$(TARGET): $(SRC) | $(BUILD)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC)

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)

.PHONY: clean
