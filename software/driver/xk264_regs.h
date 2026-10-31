/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Register offsets of the xk264 AXI wrapper (see hdl/wrapper/xk264_axi_top.v).
 * All registers are 32-bit, byte offsets as below.
 */
#ifndef _XK264_REGS_H
#define _XK264_REGS_H

#define XK264_REG_IDR		0x00	/* ro  0x02640001 */
#define XK264_REG_CTRL		0x04	/* [0] start (wo) [1] irq_en */
#define XK264_REG_STATUS	0x08	/* [0] busy [1] done [2] rd_err [3] wr_err */
#define XK264_REG_ISTAT		0x0C	/* rw1c [0] frame_done */
#define XK264_REG_CFG		0x10	/* [5:0] qp [6] intra [7] sys_mode */
#define XK264_REG_PIC		0x14	/* [15:0] mb_x_total [31:16] mb_y_total */
#define XK264_REG_YUV		0x18	/* input YUV buffer phys addr (MB-tiled) */
#define XK264_REG_BANK0		0x1C	/* reconstruction bank 0 phys addr */
#define XK264_REG_BANK1		0x20	/* reconstruction bank 1 phys addr */
#define XK264_REG_BSADDR	0x24	/* bitstream output buffer phys addr */
#define XK264_REG_BSLEN		0x28	/* ro  bytes written this frame */
#define XK264_REG_FRMCNT	0x2C	/* ro  frames encoded since reset */

#define XK264_CTRL_START	BIT(0)
#define XK264_CTRL_IRQ_EN	BIT(1)

#define XK264_STATUS_BUSY	BIT(0)
#define XK264_STATUS_DONE	BIT(1)
#define XK264_STATUS_RD_ERR	BIT(2)
#define XK264_STATUS_WR_ERR	BIT(3)

#define XK264_ISTAT_FRAME_DONE	BIT(0)

#define XK264_IDR_MAGIC		0x02640001u

#endif /* _XK264_REGS_H */
