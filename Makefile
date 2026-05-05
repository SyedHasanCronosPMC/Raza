CC      := gcc
CFLAGS  := -Wall -Wextra -O2 -std=c99
LDFLAGS := -lm
SRC     := openEndedC.c
TARGET  := drone

ifeq ($(OS),Windows_NT)
    TARGET := drone.exe
    RM     := del /Q
else
    RM     := rm -f
endif

.PHONY: all run clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

run: $(TARGET)
	./$(TARGET)

clean:
	-$(RM) $(TARGET) flight_log.csv
