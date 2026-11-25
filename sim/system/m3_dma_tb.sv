// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA (our fork)
//
// Bench top for the Real3D DMA (Step 2, Daytona 2): m3_bus with its SDRAM target as the Daytona 2 core builds it
// (m3_sdr_port on m2_sdram, the SDRAM at its pins), m3_scripts as master 1 (R3DMA, step2), the CPU as master 0
// and the Real3D's host port driven by the C++ (sim/system/tb_m3_dma.cpp). DIR: m3_scripts DIR and m3_bus DDIR.
module m3_dma_tb
  import ppc_cache_pkg::*;
#(parameter bit DIR = 1'b0)
(
  input  logic        clk,
  input  logic        rst_n,
  input  logic        rst,
  output logic        ready,
  // the CPU (master 0)
  input  logic        c_req,
  input  logic [1:0]  c_cmd,
  input  logic [31:0] c_addr,
  input  logic [3:0]  c_be,
  input  logic [31:0] c_wdata,
  output logic        c_ack,
  output logic        c_wnext,
  output logic        c_rvalid,
  output logic [31:0] c_rdata,
  output logic        sn_req, sn_inv,
  output logic [31:5] sn_addr,
  input  logic        sn_ack,
  output logic        ext_int,
  output logic        dma_busy,
  // master 1 as the bus sees it, for the counters
  output logic        d_req, d_ack, d_wnext, d_rvalid,
  output logic [1:0]  d_cmd,
  output logic        sd_cke, sd_cs_n, sd_ras_n, sd_cas_n, sd_we_n,
  output logic [1:0]  sd_ba,
  output logic [12:0] sd_a,
  output logic [15:0] sd_dq_o,
  input  logic [15:0] sd_dq_i,
  // the Real3D's host port
  input  logic        h_rdy_i,
  output logic        h_go_o, h_we_o,
  output logic [31:0] h_addr_o, h_wdata_o
);
  logic [1:0]        m_req, m_ack, m_wnext, m_rvalid;
  logic [1:0][1:0]   m_cmd;
  logic [1:0][31:0]  m_addr, m_wdata;
  logic [1:0][3:0]   m_be, m_wbe;
  logic [31:0]       m_rdata;
  assign m_req[0] = c_req; assign m_cmd[0] = c_cmd; assign m_addr[0] = c_addr; assign m_be[0] = c_be;
  assign m_wdata[0] = c_wdata; assign m_wbe[0] = 4'hF; assign m_wbe[1] = 4'hF;
  assign c_ack = m_ack[0]; assign c_wnext = m_wnext[0]; assign c_rvalid = m_rvalid[0]; assign c_rdata = m_rdata;
  assign d_req = m_req[1]; assign d_ack = m_ack[1]; assign d_wnext = m_wnext[1]; assign d_rvalid = m_rvalid[1]; assign d_cmd = m_cmd[1];

  logic        sc_req, sc_we, sc_ack, sc_win_en;
  logic [6:2]  sc_addr;
  logic [3:0]  sc_be;
  logic [31:0] sc_wdata, sc_rdata;
  logic [31:8] sc_win_base;
  logic        cf_req, cf_we, cf_ack;
  logic [5:0]  cf_idx;
  logic [3:0]  cf_be;
  logic [31:0] cf_wdata, cf_rdata;
  logic        scsi_irq;
  logic        dh_v, dh_take;
  logic [31:0] dh_addr, dh_data;

  m3_scripts #(.R3DMA(1'b1), .LDV(1'b1), .DIR(DIR)) u_scsi (
    .clk, .rst, .step2(1'b1),
    .c_req(cf_req), .c_we(cf_we), .c_idx(cf_idx), .c_be(cf_be), .c_wdata(cf_wdata), .c_ack(cf_ack), .c_rdata(cf_rdata),
    .win_en(sc_win_en), .win_base(sc_win_base),
    .s_req(sc_req), .s_we(sc_we), .s_addr(sc_addr), .s_be(sc_be), .s_wdata(sc_wdata), .s_ack(sc_ack), .s_rdata(sc_rdata),
    .m_req(m_req[1]), .m_cmd(m_cmd[1]), .m_addr(m_addr[1]), .m_be(m_be[1]), .m_wdata(m_wdata[1]),
    .m_ack(m_ack[1]), .m_wnext(m_wnext[1]), .m_rvalid(m_rvalid[1]), .m_rdata,
    .irq(scsi_irq), .busy(dma_busy), .dh_v, .dh_addr, .dh_data, .dh_take);

  logic        s_req, s_swap, s_zero, s_ack, s_wnext, s_rvalid, s_idle;
  logic [1:0]  s_cmd;
  logic [26:0] s_sa;
  logic [3:0]  s_be;
  logic [31:0] s_wdata, s_rdata;
  logic [3:0]  s_wbe;
  logic        p_req, p_we, p_long, p_lo, p_ack;
  logic [26:1] p_addr;
  logic [15:0] p_din;
  logic [1:0]  p_be;
  logic        h_go, h_we, h_rv;
  logic [31:0] h_rdata;
  always_ff @(posedge clk) begin h_rv <= h_go && h_rdy_i && !h_we; h_rdata <= 32'hFDFF_FFFF; end
  assign h_go_o = h_go; assign h_we_o = h_we;
  logic [3:0]  wb_col;
  logic [15:0] wb_din;
  logic [1:0]  wb_be;
  logic [63:0] p_dout;

  m3_bus #(.AW(26), .MIDI_GAP(500), .FAST(1'b1), .R3P(1'b1), .TGT(1'b0), .OWIRE(1'b1), .WD_DEC(1'b1), .STEP2(1'b1), .DDIR(DIR)) u_bus (
    .clk, .rst, .sec_tick(1'b0),
    .step2(1'b1), .fixed8(1'b1), .crom_slot(4'd0), .crom_zero(1'b0), .crom_x(1'b0), .s_x(), .s_w2(),
    .sec_req(), .sec_we(), .sec_addr(), .sec_be(), .sec_wdata(), .sec_ack(1'b0), .sec_rdata(32'd0),
    .m_req, .m_cmd, .m_addr, .m_be, .m_wdata, .m_wbe, .m_ack, .m_wnext, .m_rvalid, .m_rdata,
    .sn_req, .sn_addr, .sn_inv, .sn_ack,
    .s_req, .s_cmd, .s_sa, .s_swap, .s_zero, .s_be, .s_wdata, .s_wbe, .s_ack, .s_wnext, .s_rvalid, .s_rdata,
    .sc_req, .sc_we, .sc_addr, .sc_be, .sc_wdata, .sc_ack, .sc_rdata, .sc_win_en, .sc_win_base,
    .cf_req, .cf_we, .cf_idx, .cf_be, .cf_wdata, .cf_ack, .cf_rdata, .scsi_irq,
    .io_sel(), .io_we(), .io_off(), .io_wbyte(), .io_rbyte(8'd0),
    .tg_reg_we(), .tg_reg_addr(), .tg_reg_wdata(), .tg_pal_we(), .tg_pal_addr(), .tg_pal_wdata(),
    .ln_start(1'b0), .ln(10'd0), .ln_vis(1'b0), .ext_int,
    .irq_state_o(), .irq_enable_o(), .crom_bank_o(),
    .n_submit(), .n_cull_lo(), .n_cull_hi(), .n_poly(), .n_tex(), .n_vrom(), .n_unmapped(), .last_unmapped(),
    .pingpong_o(), .pingpong_i(1'b0), .h_go, .h_rdy(h_rdy_i), .h_we, .h_addr(h_addr_o), .h_wdata(h_wdata_o), .h_rv, .h_rdata,
    .r3d_cfg0(), .r3d_cfg1(), .r3d_cfg2(),
    .jt_wr(), .jt_wdata(), .jt_busy(1'b0), .jt_tdo(1'b0), .jt_cfg_we(1'b0), .jt_cfg0('0), .jt_cfg1('0), .jt_cfg2('0),
    .snd_we(), .snd_data(), .snd_byte(),
    .tr_v(), .tr_master(), .tr_we(), .tr_addr(), .tr_be(), .tr_data(), .tr_nw(),
    .d_v(dh_v), .d_addr(dh_addr), .d_wdata(dh_data), .d_take(dh_take));

  // the Daytona 2 core's SDRAM side (Model3.sv: L16, LW1, PAIR, PW, FWD, FREQ, PW_DLY 8; BXOR, FDSP)
  m3_sdr_port #(.AW(26), .L16(1'b1), .LW1(1'b1), .PAIR(1'b1), .RA(1'b0), .PW(1'b1), .RA_CONF(1'b0), .FWD(1'b1), .FREQ(1'b1), .PW_DLY(8)) u_port (
    .clk, .rst(!rst_n), .ra_clr(1'b0),
    .s_req, .s_cmd, .s_sa, .s_swap, .s_zero, .s_be, .s_wdata, .s_wbe, .s_ack, .s_wnext, .s_rvalid, .s_rdata, .s_idle,
    .p_req, .p_we, .p_addr, .p_din, .p_be, .p_long, .p_dout, .p_lo, .p_ack, .wb_col, .wb_din, .wb_be);

  logic [1:0] pk, pl;
  logic [1:0][63:0] pd;
  assign p_ack = pk[0]; assign p_lo = pl[0]; assign p_dout = pd[0];
  m2_sdram #(.COL_BITS(10), .NP(2), .T_REFI(770), .ACK_HOLD(1), .DUAL(1'b1), .MASK_ON_A(1'b1), .INIT_NOP(200), .WB(2'b01), .L16(2'b01), .BXOR(1'b1), .PAIR(2'b01), .FDSP(1'b1), .HITRD(1'b0), .SHD(2'b11)) u_sdram (
    .clk, .rst_n, .ready, .rd_lat_sel(3'd2),
    .sd_cke, .sd_cs_n, .sd_ras_n, .sd_cas_n, .sd_we_n, .sd_ba, .sd_a, .sd_dqm(), .sd_dq_o, .sd_dq_oe(), .sd_dq_i,
    .wr_req(1'b0), .wr_addr('0), .wr_din('0), .wr_be('0), .wr_ack(),
    .p_req({1'b0, p_req}), .p_we({1'b0, p_we}), .p_addr({26'd0, p_addr}), .p_din({16'd0, p_din}), .p_be({2'b00, p_be}),
    .p_dout(pd), .p_ack(pk), .p_long({1'b0, p_long}), .p_lo(pl), .wb_col, .wb_din, .wb_be, .dbg_req(), .dbg_grant());
endmodule
