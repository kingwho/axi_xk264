# xk264-app - userspace demo for the xk264 hardware encoder
#
# Host build (offline packaging tests, --sim-dir):
#     make
# Target build (Zynq-7000, PetaLinux 2020.2):
#     make CROSS=arm-linux-gnueabihf-

CROSS   ?=
CC      := $(CROSS)gcc
CFLAGS  ?= -O2 -Wall -Wextra

TARGETS := xk264-app xk264-v4l2-app

all: $(TARGETS)

xk264-app: xk264-app.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ xk264-app.c

xk264-v4l2-app: xk264-v4l2-app.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ xk264-v4l2-app.c

clean:
	rm -f $(TARGETS)

.PHONY: all clean
