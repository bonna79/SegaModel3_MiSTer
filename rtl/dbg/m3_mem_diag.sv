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
// m3_mem_diag (bonna79, board measurement only, macro M3_MEMDIAG with M3_RELEASE and M3_FBUSY_BUILD): where the
// CPU's time goes against the first SDRAM, counted over each second and held for the overlay (rows 3-5):
//   o[0]  instructions retired (ev 0)          o[1]  fetch waiting on memory (ev 7)
//   o[2]  data memory wait (ev 5)              o[3]  {DTLB misses taken, data micro-TLB replays / 16}
//   o[4]  CPU bus reads (m3_bus master 0)      o[5]  their clocks from request to first beat (sum)
//   o[6]  CPU bus writes                       o[7]  SDRAM port 0 (the bus) pending, not in flight
//   o[8]  port 0 pending while another port is in flight
//   o[9]  port 1 in flight (tile generator)    o[10] ports 2 + 3 in flight   o[11] ports 4 + 5 in flight
//   o[12] port 0 in flight
// Every input is registered once; nothing here feeds back into the design.
module m3_mem_diag #(
  parameter int CLK_HZ = 140_000_000
) (
  input  logic        clk,
  input  logic        rst,
  input  logic [7:0]  ev,
  input  logic [5:0]  ev_mmu,
  input  logic        m_req, m_ack, m_rvalid,
  input  logic [1:0]  m_cmd,
  input  logic [5:0]  sd_pend,
  input  logic [5:0]  sd_fly,
  output logic [12:0][31:0] o
);
  logic [7:0] ev_q; logic [5:0] mmu_q, pend_q, fly_q; logic req_q, ack_q, rv_q; logic [1:0] cmd_q;
  always_ff @(posedge clk) begin
    ev_q <= ev; mmu_q <= ev_mmu; req_q <= m_req; ack_q <= m_ack; rv_q <= m_rvalid; cmd_q <= m_cmd;
    pend_q <= sd_pend; fly_q <= sd_fly;
  end
  logic [31:0] sec;
  logic        tick;
  always_ff @(posedge clk) begin
    tick <= 1'b0;
    if (rst) sec <= '0;
    else if (sec == 32'(CLK_HZ - 1)) begin sec <= '0; tick <= 1'b1; end
    else sec <= sec + 32'd1;
  end
  // the CPU's bus transaction in progress (as tb_m3_system's cpu port count): a read from its request to its
  // first beat, a write to its acknowledge (ppc_cache_pkg: MC_LINE_RD 0, MC_LINE_WR 1, MC_RD 2, MC_WR 3)
  logic in_q, rd_q;
  wire  start = req_q && !ack_q && !in_q;
  wire  rd_s  = !cmd_q[0];
  always_ff @(posedge clk)
    if (rst) begin in_q <= 1'b0; rd_q <= 1'b0; end
    else if (start) begin in_q <= 1'b1; rd_q <= rd_s; end
    else if (in_q && (rd_q ? rv_q : ack_q)) in_q <= 1'b0;
  logic [12:0][31:0] c;
  wire [12:0] inc;
  assign inc[0]  = ev_q[0];
  assign inc[1]  = ev_q[7];
  assign inc[2]  = ev_q[5];
  assign inc[3]  = 1'b0;
  assign inc[4]  = start && rd_s;
  assign inc[5]  = in_q && rd_q;
  assign inc[6]  = start && !rd_s;
  assign inc[7]  = pend_q[0] && !fly_q[0];
  assign inc[8]  = pend_q[0] && !fly_q[0] && |fly_q[5:1];
  assign inc[9]  = fly_q[1];
  assign inc[10] = fly_q[2] || fly_q[3];
  assign inc[11] = fly_q[4] || fly_q[5];
  assign inc[12] = fly_q[0];
  logic [15:0] dtlb; logic [19:0] utlb;
  always_ff @(posedge clk) begin
    if (rst || tick) begin
      c <= '0; dtlb <= '0; utlb <= '0;
      if (tick) begin
        for (int i = 0; i < 13; i++) o[i] <= (i == 3) ? {dtlb, utlb[19:4]} : c[i];
      end
    end else begin
      for (int i = 0; i < 13; i++) if (inc[i]) c[i] <= c[i] + 32'd1;
      if (mmu_q[0] && dtlb != 16'hFFFF) dtlb <= dtlb + 16'd1;
      if (mmu_q[4] && utlb != 20'hFFFFF) utlb <= utlb + 20'd1;
    end
  end
  wire _unused = &{1'b0, ev_q[6], ev_q[4:1], mmu_q[5], mmu_q[3:1], cmd_q[1], pend_q[5:1]};
endmodule
