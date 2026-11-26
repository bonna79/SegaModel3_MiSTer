`timescale 1ns/1ps
// Bench top for m3_sdr_cal (our fork): the calibration, m2_sdram as m3_system
// builds it for the Daytona 2 core at 140 MHz, and m3_sdr_dps on its own scan
// clock. tb_sdr_cal.cpp models the board: the SDRAM at its pins, a read data
// window that moves with the PLL's phase steps, and the PLL's phase_done.
// After the calibration the bench drives the same ports as the loader and
// r3d_texup do (b_*), to test the memory at the chosen setting.
module tb_sdr_cal (
  input  logic        clk,
  input  logic        scanclk,
  input  logic        rst_n,
  input  logic        start,
  input  logic        man,
  input  logic [2:0]  man_sel,
  output logic        phase_en, updn,
  output logic [4:0]  cntsel,
  input  logic        phase_done,
  output logic        sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n,
  output logic [1:0]  sd_ba,
  output logic [12:0] sd_a,
  output logic [1:0]  sd_dqm,
  output logic [15:0] sd_dq_o,
  output logic        sd_dq_oe,
  input  logic [15:0] sd_dq_i,
  output logic        busy, done, ready,
  output logic [2:0]  rd_sel,
  output logic [15:0] stat,
  output logic [191:0] mask,
  // the bench's own accesses after the calibration
  input  logic        b_wreq,
  input  logic [26:1] b_waddr,
  input  logic [15:0] b_wdin,
  output logic        b_wack,
  input  logic        b_rreq,
  input  logic [26:1] b_raddr,
  output logic [63:0] b_rdout,
  output logic        b_rack
);
  localparam int AW = 26, NP = 6;
  logic cal_rst, cal_wreq, cal_rreq, wack, dps_req, dps_up, dps_ack, sdr_rst_n;
  logic [AW:1] cal_waddr, cal_raddr;
  logic [15:0] cal_wdin;
  logic [NP-1:0] p_req, p_we, p_long, p_lo, p_ack;
  logic [NP-1:0][AW:1] p_addr;
  logic [NP-1:0][15:0] p_din;
  logic [NP-1:0][1:0] p_be;
  logic [NP-1:0][63:0] p_dout;
  logic [5:0][31:0] mk;

  m3_sdr_cal #(.AW(AW)) u_cal (
    .clk, .rst_n, .start, .man, .man_sel, .sdr_ready(ready), .sdr_rst(cal_rst), .rd_sel,
    .busy, .done, .wr_req(cal_wreq), .wr_addr(cal_waddr), .wr_din(cal_wdin), .wr_ack(wack),
    .rd_req(cal_rreq), .rd_addr(cal_raddr), .rd_dout(p_dout[5]), .rd_ack(p_ack[5]),
    .dps_req, .dps_up, .dps_ack, .stat, .mask(mk));
  assign mask = mk;
  always_ff @(posedge clk) sdr_rst_n <= rst_n && !cal_rst;

  m3_sdr_dps #(.CNT(5'd2)) u_dps (.scanclk, .req(dps_req), .up(dps_up), .ack(dps_ack),
    .phase_en, .updn, .cntsel, .phase_done);

  always_comb begin
    p_req = '0; p_we = '0; p_long = '0; p_addr = '0; p_din = '0; p_be = '0;
    p_req[5]  = busy ? cal_rreq : b_rreq;
    p_addr[5] = busy ? cal_raddr : b_raddr;
  end
  assign b_rdout = p_dout[5];
  assign b_rack  = p_ack[5] && !busy;
  assign b_wack  = wack && !busy;

  m2_sdram #(.COL_BITS(10), .NP(NP), .T_REFI(1078), .T_RC(9), .T_RAS(6), .ACK_HOLD(1), .DUAL(1'b1), .MASK_ON_A(1'b1),
             .BXOR(1'b1), .BXC(1'b1), .WB(6'b000001), .SHD(6'b101111)) u_sdram (
    .clk, .rst_n(sdr_rst_n), .ready, .rd_lat_sel(rd_sel),
    .sd_cke, .sd_cs_n, .sd_ras_n, .sd_cas_n, .sd_we_n, .sd_ba, .sd_a, .sd_dqm, .sd_dq_o, .sd_dq_oe, .sd_dq_i,
    .wr_req(busy ? cal_wreq : b_wreq), .wr_addr(busy ? cal_waddr : b_waddr), .wr_din(busy ? cal_wdin : b_wdin),
    .wr_be(2'b11), .wr_ack(wack),
    .p_req, .p_we, .p_addr, .p_din, .p_be, .p_dout, .p_ack, .p_long, .p_lo,
    .wb_col(), .wb_din(16'd0), .wb_be(2'b11), .dbg_req(), .dbg_grant());
  wire _unused = &{1'b0, p_lo};
endmodule
