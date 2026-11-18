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
// (diagnostic) The CPU's idle time: over the last second, the clocks whose
// last retirement was in one of the game's wait loops (tools/mame-census/
// README.md: vblank and decrementer polls, count-downs). A small stand-in
// for m3_frame_busy where a game's frame never passes its vblank poll
// (Daytona 2's race). The count is in units of 16 clocks.

module m3_idle_mon #(
  parameter int CLK_HZ = 100_000_000
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        step2,          // Daytona 2's loops, else VF3's
  input  logic        rt_valid,       // the core's retirement port
  input  logic [31:0] rt_pc,
  output logic [23:0] o_idle,         // the last second's idle clocks / 16
  // (diagnostic) the CPU's events (m3_cpu_mon's ev: 0 instructions, 1 uops, 2 decode starved, 3 load-use,
  // 4 mul/div wait, 5 data memory wait, 6 redirects, 7 fetch waiting), counted only while the CPU is not in a
  // wait loop, so they split its busy time; the last second's counts
  input  logic [7:0]  ev,
  output logic [27:0] o_bcyc,         // busy clocks
  output logic [27:0] o_bins,         // instructions retired while busy
  output logic [27:0] o_bdw,          // data memory wait
  output logic [27:0] o_bfw,          // fetch waiting
  output logic [27:0] o_blu,          // load-use bubbles
  output logic [27:0] o_brd           // redirects
);

  // Daytona 2: 4C6CC-4C6D4, 4C73C-4C744, 4C754-4C760, 541C-5428;
  // VF3: 1E928-1E930, 1E97C-1E984, 1E9DC-1E9E4, 1E9FC-1EA04, 2518-251C
  function automatic logic wany(input logic [31:0] pc, input logic s2);
    if (pc[31:20] != 12'd0) wany = 1'b0;
    else if (!s2)
      wany = (pc[19:0] >= 20'h1E928 && pc[19:0] <= 20'h1E930) || (pc[19:0] >= 20'h1E97C && pc[19:0] <= 20'h1E984) ||
             (pc[19:0] >= 20'h1E9DC && pc[19:0] <= 20'h1E9E4) || (pc[19:0] >= 20'h1E9FC && pc[19:0] <= 20'h1EA04) ||
             (pc[19:0] >= 20'h02518 && pc[19:0] <= 20'h0251C);
    else
      wany = (pc[19:0] >= 20'h4C6CC && pc[19:0] <= 20'h4C6D4) || (pc[19:0] >= 20'h4C73C && pc[19:0] <= 20'h4C744) ||
             (pc[19:0] >= 20'h4C754 && pc[19:0] <= 20'h4C760) || (pc[19:0] >= 20'h0541C && pc[19:0] <= 20'h05428);
  endfunction

  logic        ia;
  logic [3:0]  pre;
  logic [23:0] cnt;
  logic [26:0] sec;
  logic        tick;
  always_ff @(posedge clk) begin
    if (rst) begin
      ia <= 1'b0; pre <= '0; cnt <= '0; sec <= '0; tick <= 1'b0; o_idle <= '0;
    end else begin
      tick <= (sec == 27'(CLK_HZ - 1));
      sec  <= tick ? 27'd0 : sec + 27'd1;
      if (rt_valid) ia <= wany(rt_pc, step2);
      if (tick) begin
        o_idle <= cnt; cnt <= '0; pre <= '0;
      end else if (ia) begin
        pre <= pre + 4'd1;
        if (pre == 4'hF) cnt <= cnt + 24'd1;
      end
    end
  end

  // the busy split: the events registered (as m3_cpu_mon's ev_q), counted while the last retirement was
  // outside the wait loops
  /* verilator lint_off UNUSEDSIGNAL */
  logic [7:0]  ev_q;                   // (uops, decode starved, mul/div wait not counted here)
  /* verilator lint_on UNUSEDSIGNAL */
  logic [27:0] k_cyc, k_ins, k_dw, k_fw, k_lu, k_rd;
  always_ff @(posedge clk) begin
    ev_q <= ev;
    if (rst) begin
      k_cyc <= '0; k_ins <= '0; k_dw <= '0; k_fw <= '0; k_lu <= '0; k_rd <= '0;
      o_bcyc <= '0; o_bins <= '0; o_bdw <= '0; o_bfw <= '0; o_blu <= '0; o_brd <= '0;
    end else if (tick) begin
      o_bcyc <= k_cyc; o_bins <= k_ins; o_bdw <= k_dw; o_bfw <= k_fw; o_blu <= k_lu; o_brd <= k_rd;
      k_cyc <= '0; k_ins <= '0; k_dw <= '0; k_fw <= '0; k_lu <= '0; k_rd <= '0;
    end else if (!ia) begin
      k_cyc <= k_cyc + 28'd1;
      k_ins <= k_ins + {27'd0, ev_q[0]};
      k_dw  <= k_dw  + {27'd0, ev_q[5]};
      k_fw  <= k_fw  + {27'd0, ev_q[7]};
      k_lu  <= k_lu  + {27'd0, ev_q[3]};
      k_rd  <= k_rd  + {27'd0, ev_q[6]};
    end
  end

endmodule
