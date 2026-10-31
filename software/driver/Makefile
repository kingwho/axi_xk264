# xk264 kernel driver build
#
# In-tree module build (PetaLinux 2020.2 / kernel 5.4):
#   make -C $(PETALINUX)/build/tmp/work-shared/plnx_arm/kernel-source \
#        M=$(PWD) ARCH=arm CROSS_COMPILE=arm-xilinx-linux-gnueabi- modules
#
# Or copy this directory to:
#   <petalinux-project>/project-spec/meta-user/recipes-modules/xk264/files/
# and enable it via petalinux-config -> rootfs.

obj-m += xk264.o

ccflags-y += -Wno-unused-function
