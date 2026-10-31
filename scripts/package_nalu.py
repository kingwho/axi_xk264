#!/usr/bin/env python3
"""Package raw xk264 encoder output into a playable H.264 Annex-B stream.

The xk264 core emits only the macroblock-layer CAVLC bits of each frame
(plus rbsp trailing bits written by BitStream_packer). It produces no
slice headers, SPS/PPS, NAL unit headers or start codes, and no
emulation prevention bytes. This script adds all of the missing
surrounding syntax so the result decodes with a standard H.264 decoder:

  frame 0  ->  [start code][NAL type 5 IDR slice][slice hdr + core bits]
  frame N  ->  [start code][NAL type 1 non-IDR slice][slice hdr + core bits]

Usage:
  package_nalu.py --w-mbs 4 --h-mbs 4 --qp 28 \
      --bs bs_frame0.bin --bs bs_frame1.bin --out out.h264

Encoding assumptions the generated headers make about the core:
  - baseline profile, CAVLC, one slice per frame, frame_mbs_only
  - pic_order_cnt_type = 2, log2_max_frame_num = 4
  - one reference frame, no weighted prediction, no B slices
  - deblocking with default parameters (control header absent)
"""

import argparse
import sys


class BitWriter:
    def __init__(self):
        self.bits = []

    def u(self, n, v):
        for i in range(n - 1, -1, -1):
            self.bits.append((v >> i) & 1)

    def ue(self, v):
        v += 1
        n = v.bit_length()
        self.u(n - 1, 0)
        self.u(n, v)

    def se(self, v):
        if v <= 0:
            self.ue(-2 * v)
        else:
            self.ue(2 * v - 1)

    def trailing(self):
        self.bits.append(1)
        while len(self.bits) % 8:
            self.bits.append(0)

    def bytes(self):
        # The core padded its own output to its byte grid; after the
        # (unaligned) slice header the stop bit can sit mid-byte, so pad
        # with zeros to the grid of the assembled NAL.
        while len(self.bits) % 8:
            self.bits.append(0)
        out = bytearray()
        for i in range(0, len(self.bits), 8):
            b = 0
            for j in range(8):
                b = (b << 1) | self.bits[i + j]
            out.append(b)
        return bytes(out)

    def append_bytes(self, data):
        """Append raw bytes at the current (unaligned) bit position."""
        for b in data:
            self.u(8, b)


def sps(w_mbs, h_mbs, crop_bottom, level_idc):
    w = BitWriter()
    w.u(8, 66)            # profile_idc: baseline
    w.u(8, 0)             # constraint_set flags + reserved
    w.u(8, level_idc)
    w.ue(0)               # seq_parameter_set_id
    w.ue(0)               # log2_max_frame_num_minus4 -> 4 bits
    w.ue(2)               # pic_order_cnt_type = 2
    w.ue(1)               # max_num_ref_frames
    w.u(1, 0)             # gaps_in_frame_num_value_allowed
    w.ue(w_mbs - 1)       # pic_width_in_mbs_minus1
    w.ue(h_mbs - 1)       # pic_height_in_map_units_minus1
    w.u(1, 1)             # frame_mbs_only_flag
    w.u(1, 1)             # direct_8x8_inference_flag
    if crop_bottom:
        w.u(1, 1)         # frame_cropping_flag
        w.ue(0)           # left
        w.ue(0)           # right
        w.ue(0)           # top
        w.ue(crop_bottom)
    else:
        w.u(1, 0)
    w.u(1, 0)             # vui_parameters_present_flag
    w.trailing()
    return w.bytes()


def pps(qp):
    w = BitWriter()
    w.ue(0)               # pic_parameter_set_id
    w.ue(0)               # seq_parameter_set_id
    w.u(1, 0)             # entropy_coding_mode_flag: CAVLC
    w.u(1, 0)             # bottom_field_pic_order_in_frame_present
    w.ue(0)               # num_slice_groups_minus1
    w.ue(0)               # num_ref_idx_l0_default_active_minus1 -> 1
    w.ue(0)               # num_ref_idx_l1_default_active_minus1
    w.u(1, 0)             # weighted_pred_flag
    w.u(2, 0)             # weighted_bipred_idc
    w.se(qp - 26)         # pic_init_qp_minus26
    w.se(0)               # pic_init_qs_minus26
    w.se(0)               # chroma_qp_index_offset
    w.u(1, 0)             # deblocking_filter_control_present_flag
    w.u(1, 0)             # constrained_intra_pred_flag
    w.u(1, 0)             # redundant_pic_cnt_present_flag
    w.trailing()
    return w.bytes()


def slice_header(frame_idx, intra, qp):
    w = BitWriter()
    w.ue(0)               # first_mb_in_slice
    w.ue(7 if intra else 5)   # slice_type: all-I / all-P
    w.ue(0)               # pic_parameter_set_id
    w.u(4, frame_idx & 0xF)   # frame_num (log2_max_frame_num = 4)
    if intra:
        w.ue(0)           # idr_pic_id
    else:
        w.u(1, 0)         # num_ref_idx_active_override_flag
        w.u(1, 0)         # ref_pic_list_modification_flag_l0
    if intra:
        w.u(1, 0)         # no_output_of_prior_pics
        w.u(1, 0)         # long_term_reference_flag
    else:
        w.u(1, 0)         # adaptive_ref_pic_marking_mode_flag
    w.se(0)               # slice_qp_delta (pic_init_qp == core qp)
    # deblocking_filter_control_present_flag = 0 -> nothing more
    return w


def add_emulation_prevention(rbsp):
    out = bytearray()
    zeros = 0
    for b in rbsp:
        if zeros >= 2 and b <= 3:
            out.append(3)
            zeros = 0
        out.append(b)
        zeros = zeros + 1 if b == 0 else 0
    return bytes(out)


def nal(nal_ref_idc, nal_type, rbsp):
    hdr = bytes([(nal_ref_idc << 5) | nal_type])
    return b"\x00\x00\x00\x01" + hdr + add_emulation_prevention(rbsp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--w-mbs", type=int, required=True)
    ap.add_argument("--h-mbs", type=int, required=True)
    ap.add_argument("--qp", type=int, default=28)
    ap.add_argument("--level", type=int, default=40)
    ap.add_argument("--crop-bottom", type=int, default=0,
                    help="frame_crop_bottom_offset in chroma crop units")
    ap.add_argument("--bs", action="append", required=True,
                    help="raw core bitstream file, one per frame, frame 0 = IDR")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    out = bytearray()
    out += nal(3, 7, sps(args.w_mbs, args.h_mbs, args.crop_bottom, args.level))
    out += nal(3, 8, pps(args.qp))

    for i, path in enumerate(args.bs):
        with open(path, "rb") as f:
            core = f.read()
        intra = (i == 0)
        w = slice_header(i, intra, args.qp)
        w.append_bytes(core)   # core bits follow mid-byte; core already ends with rbsp stop bit
        out += nal(3 if intra else 2, 5 if intra else 1, w.bytes())
        print(f"frame {i}: {'IDR' if intra else 'P'} core={len(core)} B "
              f"total_so_far={len(out)} B")

    with open(args.out, "wb") as f:
        f.write(out)
    print(f"wrote {len(out)} bytes to {args.out}")


if __name__ == "__main__":
    sys.exit(main())
