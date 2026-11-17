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
  output logic [23:0] o_idle          // the last second's idle clocks / 16
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

endmodule
