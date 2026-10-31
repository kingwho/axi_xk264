# Yocto recipe for the xk264 encoder kernel module.
#
# Install under:
#   <plnx-proj>/project-spec/meta-user/recipes-modules/xk264/
#     xk264.bb
#     files/xk264.c
#     files/xk264_regs.h
#     files/Makefile
#
# Enable with:
#   petalinux-config -c rootfs   ->  modules  ->  kernel-module-xk264
#
# The recipe assumes the module builds against the kernel source tree
# provided by the PetaLinux 2020.2 kernel 5.4 layer.

SUMMARY = "xk264 H.264 encoder AXI IP kernel module"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0;md5=801f80980d171dd6425610833a22dbe6"

inherit module

SRC_URI = "file://xk264.c \
           file://xk264_regs.h \
           file://Makefile \
           "

S = "${WORKDIR}"

# name the module kernel-module-xk264 automatically via module class
RDEPENDS_${PN} += "kernel-modules"
