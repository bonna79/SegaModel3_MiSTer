`timescale 1ns/1ps
// m3_sdr_dps: one PLL dynamic phase step for each toggle of req (our fork).
//
// Runs on the PLL's scan clock (CLK_50M; Cyclone V's scanclk is limited to
// 100 MHz, the system clock is above that). req and up come from the system
// clock: up is set before req toggles and held until ack answers, so it is
// read only after req's synchroniser has seen the toggle.
//
// The step, as in Intel's AN 661 and the PC-FX core's controller: phase_en
// high until phase_done goes low (or 15 clocks), then low, then wait for
// phase_done high; ack toggles to say the step is done.
module m3_sdr_dps #(
  parameter logic [4:0] CNT = 5'd2              // the counter stepped: outclk_2
) (
  input  logic       scanclk,
  input  logic       req,                        // toggles (system clock)
  input  logic       up,                         // 1: later, 0: earlier (system clock, stable)
  output logic       ack,                        // toggles (scan clock)
  output logic       phase_en,
  output logic       updn,
  output logic [4:0] cntsel,
  input  logic       phase_done
);
  (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *) logic [2:0] dps_rq_s = 3'b000;
  (* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *) logic [1:0] dps_dn_s = 2'b11;
  logic [1:0] st = 2'd0;
  logic [3:0] t  = 4'd0;
  initial begin ack = 1'b0; phase_en = 1'b0; updn = 1'b1; end

  assign cntsel = CNT;

  always_ff @(posedge scanclk) begin
    dps_rq_s <= {dps_rq_s[1:0], req};
    dps_dn_s <= {dps_dn_s[0], phase_done};
    case (st)
      2'd0: if (dps_rq_s[2] != ack) begin        // a new request (seen for a clock already)
        updn <= up; t <= '0; st <= 2'd1;
      end
      2'd1: begin                                 // up settled for a clock: enable
        phase_en <= 1'b1; t <= '0; st <= 2'd2;
      end
      2'd2: begin
        t <= t + 1'b1;
        if (!dps_dn_s[1] || t == 4'd15) begin phase_en <= 1'b0; t <= '0; st <= 2'd3; end
      end
      default: begin
        t <= t + 1'b1;
        if (dps_dn_s[1] && t >= 4'd2) begin ack <= dps_rq_s[2]; st <= 2'd0; end
      end
    endcase
  end
endmodule
