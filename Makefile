CC      := gcc
CFLAGS  := -Wall -Wextra -O2 -std=c99
LDFLAGS := -lm
SRC     := openEndedC.c sim.c
TARGET  := drone

ifeq ($(OS),Windows_NT)
    TARGET := drone.exe
    RM     := del /Q
else
    RM     := rm -f
endif

.PHONY: all run clean test

all: $(TARGET)

$(TARGET): $(SRC) sim.h
	$(CC) $(CFLAGS) -o $@ openEndedC.c sim.c $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	-$(RM) $(TARGET) flight_log.csv

test:
	$(MAKE) -C tests test
