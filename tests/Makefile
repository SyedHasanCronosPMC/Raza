CC      := gcc
CFLAGS  := -Wall -Wextra -O0 -g -std=c99
LDFLAGS := -lm

.PHONY: test clean

test: sim_test
	./sim_test

sim_test: sim_test.c ../sim.c ../sim.h
	$(CC) $(CFLAGS) -o $@ sim_test.c ../sim.c $(LDFLAGS)

clean:
	-rm -f sim_test sim_test.exe
