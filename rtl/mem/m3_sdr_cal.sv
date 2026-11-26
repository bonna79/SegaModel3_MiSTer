`timescale 1ns/1ps
// m3_sdr_cal: automatic SDRAM clock phase and read capture calibration, once
// at power-on (our fork, Daytona 2 core).
//
// WHY. The SDRAM's clock (clk_sdr, outclk_2) has a fixed phase chosen on one
// board, and the read capture point (m2_sdram rd_lat_sel, CL+n) a fixed
// default. At 140 MHz the margin is thin: on another board the same core
// froze a few seconds after start, CL+1 and CL+3 gave a black screen, CL+2
// mostly worked. A whole-cycle choice cannot fix that; the clock's phase can.
//
// WHAT. When the first ROM download starts (status, and so the OSD choice, is
// in by then; the HPS waits on ioctl_wait because mem_ready is held low for
// the loader until this is done), for each of NPH phase positions of the
// SDRAM clock, one PLL dynamic phase step apart (VCO / 8, 223 ps at 140 MHz,
// NPH = 32 steps = one clock period):
//   - the controller is reset, so it runs the SDRAM's bring-up again (a bad
//     phase can garble a command into a mode register write; every position
//     starts from a clean device);
//   - eight words are written, four at ADDR_A and four at ADDR_B (the other
//     end of the device: another bank and row), with a pattern that changes
//     at every position, so data left over from an earlier position never
//     passes;
//   - each block is read back REPS times at every capture depth CL+0..CL+5.
// A position/depth passes when every read returned the exact 64 bits.
//
// CHOICE. A later SDRAM clock delays the read data by the same amount, so a
// phase step k at depth n behaves as one point of a single axis x = k - NPH*n
// (one depth = one period = NPH steps). The passing points form one window
// on that axis; the result is the centre of the longest run of passes along
// it. The opposite ordering (x = k + NPH*n) is scanned as well and wins only
// with a strictly longer run, so a wrong assumption on the step's direction
// costs margin, not the boot.
//
// Then the clock is moved to the chosen phase (downward from the last
// position, which does not depend on how many steps make a period), the
// controller is reset again and the result verified VREPS times. A run
// shorter than MIN_RUN, a failed verify or a PLL that does not answer puts
// everything back as it was without calibration: phase step 0 and DEF_SEL.
//
// With the OSD on a manual capture value at download start, nothing runs:
// phase step 0 and the OSD's value, exactly as the core behaved before.
module m3_sdr_cal #(
  parameter int AW       = 24,
  parameter int NPH      = 32,         // phase positions swept (a power of two)
  parameter int NSEL     = 6,          // capture depths CL+0 .. CL+NSEL-1
  parameter int REPS     = 4,          // reads per block per depth in the sweep
  parameter int VREPS    = 16,         // reads per block in the final check
  parameter int MIN_RUN  = 3,
  parameter logic [2:0] DEF_SEL = 3'd2,
  parameter logic [AW:1] ADDR_A = '0,
  parameter logic [AW:1] ADDR_B = {{(AW-2){1'b1}}, 2'b00}
) (
  input  logic          clk,
  input  logic          rst_n,          // power-on (the memory reset): calibrate once after it
  input  logic          start,          // the first ROM download
  input  logic          man,            // OSD on a manual capture value (live)
  input  logic [2:0]    man_sel,        // its rd_lat_sel (live)

  input  logic          sdr_ready,
  output logic          sdr_rst,        // resets m2_sdram (its bring-up runs again)
  output logic [2:0]    rd_sel,         // to m2_sdram rd_lat_sel

  output logic          busy,           // owns the write port and the read port
  output logic          done,           // the loader may use the SDRAM
  output logic          wr_req,
  output logic [AW:1]   wr_addr,
  output logic [15:0]   wr_din,
  input  logic          wr_ack,
  output logic          rd_req,
  output logic [AW:1]   rd_addr,
  input  logic [63:0]   rd_dout,
  input  logic          rd_ack,

  output logic          dps_req,        // toggles: one phase step requested
  output logic          dps_up,         // its direction, stable before dps_req toggles
  input  logic          dps_ack,        // toggles when the step is done (other clock)

  output logic [15:0]   stat,           // {done, ok, err, sel[2:0], k[4:0], run[4:0]}
  output logic [NSEL-1:0][NPH-1:0] mask
);
  localparam int KW  = $clog2(NPH);
  localparam int NI  = NSEL * NPH;
  localparam int IW  = $clog2(NI + 1);
  localparam int TOW = 17;                                   // timeouts: 2^17 clocks

  initial if ((1 << KW) != NPH) $error("m3_sdr_cal: NPH must be a power of two");

  typedef enum logic [4:0] {
    S_OFF, S_WAIT, S_RST, S_RDY, S_WR, S_WR_W, S_SEL, S_RD, S_RD_W, S_CMP, S_NEXT,
    S_STEP, S_STEP_W, S_SCAN, S_PICK, S_MOVE, S_FAIL, S_DONE
  } st_t;
  st_t st, st_ret;

  // the pattern: eight words of opposite bit pairs, each byte XORed with a seed
  function automatic logic [15:0] pat(input logic [7:0] seed, input logic [2:0] w);
    logic [15:0] b;
    case (w)
      3'd0: b = 16'h5555; 3'd1: b = 16'hAAAA; 3'd2: b = 16'h3333; 3'd3: b = 16'hCCCC;
      3'd4: b = 16'h0F0F; 3'd5: b = 16'hF0F0; 3'd6: b = 16'h00FF; default: b = 16'hFF00;
    endcase
    pat = b ^ {seed, seed};
  endfunction

  logic [KW-1:0]   k;            // phase position (steps up from the static phase)
  logic [2:0]      n;            // capture depth under test
  logic [2:0]      w;            // word written
  logic            blk;          // block read
  logic [4:0]      rep;
  logic [7:0]      seed;
  logic            pass, verify, ok, err, derr, fl;
  logic [3:0]      rcnt;
  logic [TOW-1:0]  to;
  logic            ack_d, wack_d;
  logic [63:0]     exp_a, exp_b, got;
  logic [1:0]      ack_s;
  logic [KW:0]     mv;           // phase steps still to move
  logic [2:0]      sel_c;        // the chosen depth
  logic [KW-1:0]   k_c;          // the chosen phase position
  logic [7:0]      run_c;

  // the scan
  logic            ord;          // 0: x = k - NPH*n, 1: x = k + NPH*n
  logic [IW-1:0]   si, run, rstart, best, bstart;
  logic            bord;
  logic [2:0]      sn;
  logic [KW-1:0]   sk;
  logic [IW-1:0]   cidx;
  wire  [2:0]      cn = 3'(cidx[IW-1:KW]);   // the centre's depth index along the axis

  wire dps_done = (ack_s[1] == dps_req);

  always_ff @(posedge clk) begin
    ack_s <= {ack_s[0], dps_ack};
    ack_d <= rd_ack;
    wack_d <= wr_ack;
    if (!rst_n) begin
      st <= S_WAIT; st_ret <= S_WAIT;
      sdr_rst <= 1'b0; busy <= 1'b0; done <= 1'b0;
      wr_req <= 1'b0; rd_req <= 1'b0; dps_req <= 1'b0; dps_up <= 1'b1;
      k <= '0; n <= '0; w <= '0; blk <= 1'b0; rep <= '0; seed <= '0;
      pass <= 1'b0; verify <= 1'b0; ok <= 1'b0; err <= 1'b0; derr <= 1'b0; fl <= 1'b0; rcnt <= '0; to <= '0;
      mask <= '0; sel_c <= DEF_SEL; k_c <= '0; run_c <= '0; mv <= '0;
      ord <= 1'b0; si <= '0; run <= '0; rstart <= '0; best <= '0; bstart <= '0; bord <= 1'b0;
      sn <= '0; sk <= '0; cidx <= '0;
    end else begin
      wr_req <= 1'b0;
      rd_req <= 1'b0;
      to <= to + 1'b1;
      case (st)
        S_WAIT: if (start && sdr_ready) begin
          to <= '0;
          if (man) st <= S_DONE;                         // manual: as before
          else begin busy <= 1'b1; k <= '0; seed <= 8'h00; st <= S_RST; st_ret <= S_WR; end
        end

        // reset the controller; its bring-up runs again
        S_RST: begin
          sdr_rst <= 1'b1; rcnt <= rcnt + 1'b1;
          if (rcnt == 4'hF) begin sdr_rst <= 1'b0; rcnt <= '0; to <= '0; st <= S_RDY; end
        end
        S_RDY: if (sdr_ready && !sdr_rst) begin
          w <= '0; to <= '0; st <= st_ret;
        end else if (to[TOW-1]) begin err <= 1'b1; st <= S_FAIL; end

        // eight words, one at a time on the loader's port
        S_WR: begin
          wr_req  <= 1'b1;
          wr_addr <= (w[2] ? ADDR_B : ADDR_A) + (AW)'(w[1:0]);
          wr_din  <= pat(seed, w);
          to <= '0;
          st <= S_WR_W;
        end
        S_WR_W: if (wr_ack && !wack_d) begin
          w <= w + 1'b1;
          if (w == 3'd7) begin
            exp_a <= {pat(seed, 3'd3), pat(seed, 3'd2), pat(seed, 3'd1), pat(seed, 3'd0)};
            exp_b <= {pat(seed, 3'd7), pat(seed, 3'd6), pat(seed, 3'd5), pat(seed, 3'd4)};
            n <= verify ? sel_c : 3'd0; pass <= 1'b1; rcnt <= '0;
            st <= S_SEL;
          end else st <= S_WR;
        end else if (to[TOW-1]) begin err <= 1'b1; st <= S_FAIL; end

        // a capture depth: let m2_sdram register it, then read both blocks
        S_SEL: begin
          rcnt <= rcnt + 1'b1;
          if (rcnt == 4'd7) begin rcnt <= '0; blk <= 1'b0; rep <= '0; st <= S_RD; end
        end
        S_RD: begin
          rd_req  <= 1'b1;
          rd_addr <= blk ? ADDR_B : ADDR_A;
          to <= '0;
          st <= S_RD_W;
        end
        S_RD_W: if (rd_ack && !ack_d) begin
          got <= rd_dout; st <= S_CMP;
        end else if (to[TOW-1]) begin err <= 1'b1; st <= S_FAIL; end
        S_CMP: begin
          if (got != (blk ? exp_b : exp_a)) pass <= 1'b0;
          blk <= !blk;
          if (blk) rep <= rep + 1'b1;
          if (blk && rep == 5'((verify ? VREPS : REPS) - 1)) st <= S_NEXT;
          else st <= S_RD;
        end
        S_NEXT: begin
          if (verify) begin
            if (pass) begin ok <= 1'b1; st <= S_DONE; end
            else st <= S_FAIL;
          end else begin
            mask[n][k] <= pass;
            pass <= 1'b1;
            if (n != 3'(NSEL - 1)) begin n <= n + 1'b1; st <= S_SEL; end
            else if (k != KW'(NPH - 1)) begin                 // the next phase position
              dps_up <= 1'b1; st <= S_STEP; st_ret <= S_WR;
              mv <= 1; k <= k + 1'b1; seed <= seed + 8'h25;
            end else begin                                    // the sweep is over
              ord <= 1'b0; si <= '0; sn <= 3'(NSEL - 1); sk <= '0;
              run <= '0; best <= '0; bstart <= '0; bord <= 1'b0;
              st <= S_SCAN;
            end
          end
        end

        // phase steps: mv of them in direction dps_up, then reset the
        // controller and go on at st_ret
        S_STEP: if (mv == 0) begin
          rcnt <= '0; st <= S_RST;
        end else begin
          dps_req <= !dps_req; to <= '0; st <= S_STEP_W;
        end
        S_STEP_W: if (dps_done && to > 3) begin
          mv <= mv - 1'b1; st <= S_STEP;
        end else if (to[TOW-1]) begin
          // no answer: the PLL has no dynamic phase shift. The phase is where
          // it was before the unanswered request (k steps up).
          err <= 1'b1; derr <= 1'b1; dps_req <= ack_s[1]; st <= S_FAIL;
        end

        // the longest run of passes along the axis, in both orderings
        S_SCAN: begin
          if (mask[sn][sk]) begin
            if (run == 0) rstart <= si;
            if (run + 1'b1 > best) begin
              best <= run + 1'b1; bstart <= (run == 0) ? si : rstart; bord <= ord;
            end
            run <= run + 1'b1;
          end else run <= '0;
          si <= si + 1'b1;
          if (sk == KW'(NPH - 1)) begin
            sk <= '0;
            sn <= ord ? sn + 1'b1 : sn - 1'b1;
          end else sk <= sk + 1'b1;
          if (si == IW'(NI - 1)) begin
            run <= '0; si <= '0;
            if (!ord) begin ord <= 1'b1; sn <= '0; sk <= '0; end
            else st <= S_PICK;
          end
        end
        S_PICK: begin
          // the scan's tie rule: the second ordering replaced the first only
          // when strictly longer (run + 1 > best)
          cidx  <= bstart + ((best - 1'b1) >> 1);
          run_c <= 8'(best);
          st <= S_MOVE;
        end
        S_MOVE: begin
          if (best < IW'(MIN_RUN)) st <= S_FAIL;
          else begin
            k_c   <= cidx[KW-1:0];
            sel_c <= bord ? cn : 3'(NSEL - 1) - cn;
            // from position NPH-1 down to the chosen one
            dps_up <= 1'b0; mv <= (KW+1)'(NPH - 1) - (KW+1)'(cidx[KW-1:0]);
            k <= cidx[KW-1:0];
            verify <= 1'b1; seed <= 8'hC3;
            st <= S_STEP; st_ret <= S_WR;
          end
        end

        // back to the static phase (down k steps) and the default depth; a
        // second failure on the way (fl) gives up and hands the SDRAM over
        S_FAIL: begin
          ok <= 1'b0; verify <= 1'b0; sel_c <= DEF_SEL; k_c <= '0; fl <= 1'b1;
          if (fl) st <= S_DONE;
          else if (k != 0 && !derr) begin
            dps_up <= 1'b0; mv <= (KW+1)'(k); k <= '0;
            st <= S_STEP; st_ret <= S_DONE;
          end else begin
            // at the static phase already, or the PLL stopped answering
            rcnt <= '0; st <= S_RST; st_ret <= S_DONE;
          end
        end

        S_DONE: begin busy <= 1'b0; done <= 1'b1; end
        default: st <= S_DONE;
      endcase
    end
  end

  // during the calibration the depth under test; then the result, the OSD's
  // manual value, or the default when the calibration failed
  always_ff @(posedge clk)
    rd_sel <= busy ? n : man ? man_sel : ok ? sel_c : DEF_SEL;

  assign stat = {done, ok, err, sel_c, k_c[KW-1 -: 5], (run_c > 8'd31) ? 5'd31 : run_c[4:0]};
endmodule
