# xk264_app - userspace demo for the xk264 hardware encoder
#
# Host build (offline packaging tests, --sim-dir):
#     make
# Target build (Zynq-7000, PetaLinux 2020.2):
#     make CROSS=arm-linux-gnueabihf-

CROSS   ?=
CC      := $(CROSS)gcc
CFLAGS  ?= -O2 -Wall -Wextra

TARGET  := xk264_app

all: $(TARGET)

$(TARGET): xk264_app.c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ xk264_app.c

clean:
	rm -f $(TARGET)

.PHONY: all clean
