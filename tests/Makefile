# host build of the motion planner test: make -C tests
CC ?= cc
CFLAGS = -std=c11 -O2 -Wall -Wextra -I../include -I../src

test: test_profile
	./test_profile

test_profile: test_profile.c ../src/profile.c ../src/profile.h ../include/config.h
	$(CC) $(CFLAGS) test_profile.c ../src/profile.c -lm -o $@

clean:
	rm -f test_profile

.PHONY: test clean
