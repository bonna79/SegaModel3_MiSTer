// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version. See LICENSE for the full text.
//
// ---------------------------------------------------------------------------
//
// m3_scan15_out (bonna79, macro M3_SCAN15): the 15 kHz picture for a CRT on Model 3's video output, with Model 2's
// scan converter (m2_scan15, its R898 / R939 / R983). The game keeps its native 24 kHz timing; with the OSD's
// "Video: 15kHz interlaced" the pins carry the converter's 480i (384 -> 480 lines by an exact 4:5 blend), else the
// native picture exactly as before.
//
// On clk_hps (70 MHz) with the rest of the framework's video side: the native picture is one sample per ce_in, 16/70
// of the clock on average (Model3.sv's vh_ce: each pixel seen once), 656 x 424 pixels a frame = 1,216,880 clocks. The
// converter's own pixel enable is a fraction of the same clock that puts two output fields in exactly two native
// frames: 547 (537, 525) lines of 656 pixels in 2,433,760 clocks = 547 / 3710 (537 / 3710, 525 / 3710), and DV
// (265 + 265 lines) a pixel every 7 clocks: Model 2's 547/5300 and 1/10 of its 100 MHz clock with 70 in place of
// 100. 3710 / 547 = 6.8 clocks a pixel at the most, above the converter's 5 (a pixel's two ring reads and blend).
//
// The ring: Model 2's 16 lines at its D_PX, of CW bits a channel. 15 lines, the least the 15.7 kHz rate can use
// steady, is not enough there once the pairing swaps the fields (an O field read half a line either side of D_PX):
// the bench's pairing runs at 15 lines read rows already overwritten. CW 6 in the build: 8192 x 18 is 16 M10K
// (2048 x 5 blocks, 4 wide and 4 deep; 20 at Model 2's 8 bits), and MiSTer's analogue VGA DAC takes six bits a
// channel. CW 5 is 12 M10K (3 wide). The bench (sim/video/tb_m2_scan15.sv, GLUE) checks this module's outputs
// against the converter driven by the bench's own enables, clock for clock.
//
// The OSD's bits come in raw (hps_io's status, on clk_sys) and are synchronised here; the image tag is the 3D frame
// on show (r3d_cmd's show_buf), which changes in the vertical blank.

`timescale 1ns/1ps

module m3_scan15_out #(
  parameter int unsigned CW   = 6,
  parameter int unsigned RING = 16
) (
  input  logic        clk,          // clk_hps
  input  logic        rst_n,        // memory reset, synchronised to clk
  input  logic        osd_15k,      // status[42]: 15 kHz interlaced
  input  logic [1:0]  osd_lines,    // status[61:60]: 0 273/274, 1 268/269, 2 262/263, 3 265 DV
  input  logic        osd_dvp,      // status[77]: DV as 240p
  input  logic        osd_pair_off, // status[67]: pairing off
  input  logic [1:0]  show,         // the 3D frame on show

  input  logic        ce_in,        // the native picture: one sample a pixel
  input  logic [23:0] rgb_in,
  input  logic        hs_in, vs_in, de_in, vb_in,

  output logic        ce_pix,       // CE_PIXEL
  output logic [23:0] rgb,
  output logic        hs, vs, de,
  output logic        f1            // VGA_F1: the converter's field at 15 kHz, 0 at 24 kHz
);
  logic [2:0] en_s, nl_s, ml_s, dv_s, dvp_s, pr_s;
  always_ff @(posedge clk) begin
    en_s  <= {en_s[1:0],  osd_15k};
    nl_s  <= {nl_s[1:0],  osd_lines == 2'd2};
    ml_s  <= {ml_s[1:0],  osd_lines == 2'd1};
    dv_s  <= {dv_s[1:0],  osd_lines == 2'd3};
    dvp_s <= {dvp_s[1:0], osd_dvp};
    pr_s  <= {pr_s[1:0],  ~osd_pair_off};
  end
  wire en = en_s[2], nl = nl_s[2], ml = ml_s[2], dv = dv_s[2], dvp = dvp_s[2];

  // the output pixel enable: free-running from reset, so its phase against ce_in is fixed for ever (m2_scan15's
  // alignment)
  wire  [12:0] num = dv ? 13'd1 : nl ? 13'd525 : ml ? 13'd537 : 13'd547;
  wire  [12:0] den = dv ? 13'd7 : 13'd3710;
  logic [12:0] acc = 13'd0;              // acc + num < 3710 + 547 < 2^13
  logic        ce_out = 1'b0;
  always_ff @(posedge clk) begin
    if (!rst_n || acc >= den) begin acc <= 13'd0; ce_out <= 1'b0; end      // reset, or a mode change: start clean
    else if (acc + num >= den) begin acc <= acc + num - den; ce_out <= 1'b1; end
    else begin acc <= acc + num; ce_out <= 1'b0; end
  end

  logic [3:0] tag;
  always_ff @(posedge clk) tag <= {2'b00, show};

  logic        sc_hs, sc_vs, sc_de, sc_fld;
  logic [23:0] sc_rgb;
  m2_scan15 #(.RING(RING), .CW(CW), .MAXD(2048)) u_conv (
    .clk, .rst_n, .en, .img_tag(tag), .pair_en(pr_s[2]),
    .ce_in, .de_in, .vb_in, .rgb_in,
    .ce_out, .nl, .ml, .dv, .dvp,
    .o_hs(sc_hs), .o_vs(sc_vs), .o_de(sc_de), .o_field(sc_fld), .o_rgb(sc_rgb));

  assign ce_pix = en ? ce_out : ce_in;
  assign {rgb, hs, vs, de} = en ? {sc_rgb, sc_hs, sc_vs, sc_de} : {rgb_in, hs_in, vs_in, de_in};
  assign f1 = en & sc_fld;
endmodule
