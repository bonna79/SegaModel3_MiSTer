// tb_sdr_cal.cpp: bench for m3_sdr_cal (our fork), on a modelled board.
//
// The board model: the SDRAM at its pins (sim/system/sdram_dev.h, whose lat 2
// is "rd_lat_sel 2 captures it" at the PLL's own phase) and a read window that
// moves with the SDRAM clock's phase. One clock is T = 224 units; one phase
// step is T / P units. A word the model presents for edge j is valid at the
// controller's pins from j*T + off - W/2 to j*T + off + W/2, off = OFF0 + k*T/P
// for phase position k; an edge outside every window reads noise. Writes made
// with the phase in [cbad_lo, cbad_hi) (mod T) are corrupted (command/data
// timing at the chip). The PLL: phase_en raises, phase_done falls 2 scan
// clocks later and rises 4 later with the step made (none with --dead).
//
// options: --P n --off0 u --w u --cbad lo hi --man sel --dead --rev --ops n --seed s
// Prints the calibration's result and the memory test after it; exit 1 on a
// failure (no calibration when one was possible, a margin below what the
// sweep allowed, a memory test error).
#include "Vtb_sdr_cal.h"
#include "verilated.h"
#include "../system/sdram_dev.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <deque>
#include <cmath>

int main(int argc, char **argv) {
  int P = 32, off0 = 80, W = 130, cbad_lo = -1, cbad_hi = -1, man = -1, ops = 3000, seed = 1;
  bool dead = false, rev = false;
  for (int i = 1; i < argc; i++) {
    auto a = [&](const char *s) { return !std::strcmp(argv[i], s) && i + 1 < argc; };
    if (a("--P")) P = std::atoi(argv[++i]);
    else if (a("--off0")) off0 = std::atoi(argv[++i]);
    else if (a("--w")) W = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--cbad") && i + 2 < argc) { cbad_lo = std::atoi(argv[++i]); cbad_hi = std::atoi(argv[++i]); }
    else if (a("--man")) man = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--dead")) dead = true;
    else if (!std::strcmp(argv[i], "--rev")) rev = true;   // the step moves the data the other way (another counter)
    else if (a("--ops")) ops = std::atoi(argv[++i]);
    else if (a("--seed")) seed = std::atoi(argv[++i]);
  }
  const int T = 224;
  const double S = double(T) / P;                  // units a step
  const long CLK = T, SCAN = 627;                  // 140 MHz and 50 MHz in units
  std::mt19937 rng(seed);

  auto *t = new Vtb_sdr_cal;
  sdram_dev sd; sd.bxor = true; sd.bxc = true;
  std::vector<uint16_t> idq;                       // the model's word for each edge
  long k = 0;                                      // the PLL's phase position (steps)
  int dps_st = 0, dps_t = 0, steps = 0, bad_seq = 0;
  bool pen_d = false;
  uint64_t cyc = 0;
  long tclk = 0, tscan = 0;

  t->rst_n = 0; t->start = 0; t->man = man >= 0; t->man_sel = man >= 0 ? man : 2;
  t->phase_done = 1; t->b_wreq = 0; t->b_rreq = 0;
  t->clk = 0; t->scanclk = 0; t->eval();

  auto off_now = [&]() { return double(off0) + k * S; };
  auto cbad = [&]() {
    if (cbad_lo < 0) return false;
    double ph = std::fmod(off_now(), double(T)); if (ph < 0) ph += T;
    return ph >= cbad_lo && ph < cbad_hi;
  };
  auto clk_edge = [&]() {
    // as tb_m3_system's tick: the model's word for this edge, then the command
    // the controller registered at the previous edge, then the edge
    idq.push_back(sd.dq(cyc));                     // idq[cyc]: what lat 2 presents for this edge
    double tt = double(cyc) * T - off_now();
    long j = long(std::floor((tt + T / 2.0) / T));
    double r = tt - double(j) * T;
    uint16_t d;
    if (std::fabs(r) <= W / 2.0 && j >= 0 && j < long(idq.size())) d = idq[j];
    else if (std::fabs(r) <= W / 2.0) d = 0;
    else d = uint16_t(rng());
    t->sd_dq_i = d;
    uint16_t dqo = t->sd_dq_o;
    if (cbad()) dqo ^= 0x00FF;
    sd.cmd(cyc, t->sd_cke, t->sd_cs_n, t->sd_ras_n, t->sd_cas_n, t->sd_we_n, t->sd_ba, t->sd_a, dqo);
    t->clk = 1; t->eval();
    cyc++;
    t->clk = 0; t->eval();
  };
  auto scan_edge = [&]() {
    bool pen = t->phase_en;
    t->scanclk = 1; t->eval();
    // the PLL, at this scan edge, seeing phase_en as it was before it
    if (dps_st == 0 && pen && !pen_d) {
      if (t->cntsel != 2) { std::printf("FAIL cntsel %d\n", t->cntsel); std::exit(1); }
      dps_st = 1; dps_t = 0;
    } else if (dps_st == 1 && ++dps_t == 2) { if (!dead) t->phase_done = 0; dps_st = 2; dps_t = 0; }
    else if (dps_st == 2 && ++dps_t == 4) {
      if (!dead) { k += (t->updn != rev) ? 1 : -1; t->phase_done = 1; }
      if (pen && !dead) bad_seq++;                 // phase_en still high when done
      steps++; dps_st = 0;
    }
    pen_d = pen;
    t->scanclk = 0; t->eval();
  };
  auto run_to = [&](long until_units) {
    while (tclk < until_units || tscan < until_units) {
      if (tclk <= tscan) { clk_edge(); tclk += CLK; }
      else { scan_edge(); tscan += SCAN; }
    }
  };
  auto run_cycles = [&](long n) { run_to(tclk + n * CLK); };

  run_cycles(20);
  t->rst_n = 1;
  run_cycles(12000);                               // bring-up
  t->start = 1;                                    // the ROM download begins
  long c0 = cyc;
  while (!t->done && cyc - c0 < 20000000) run_cycles(100);
  if (!t->done) { std::printf("FAIL: not done\n"); return 1; }
  long ccal = cyc - c0;

  unsigned st = t->stat;
  int ok = (st >> 14) & 1, err = (st >> 13) & 1, sel = (st >> 10) & 7, kc = (st >> 5) & 31, run = st & 31;
  std::printf("P %d off0 %d W %d cbad %d-%d man %d dead %d: %ld cycles (%.2f ms at 140 MHz), steps %d, phase now %ld\n",
              P, off0, W, cbad_lo, cbad_hi, man, int(dead), ccal, ccal / 140e3, steps, k);
  std::printf("  stat ok %d err %d CL+%d phase step %d run %d%s; rd_sel %d\n", ok, err, sel, kc, run,
              run == 31 ? "+" : "", int(t->rd_sel));
  for (int n = 5; n >= 0; n--) {
    std::printf("  CL+%d ", n);
    for (int kk = 0; kk < 32; kk++) std::printf("%c", (t->mask[(n * 32 + kk) / 32] >> (kk % 32)) & 1 ? '#' : '.');
    std::printf("\n");
  }
  if (bad_seq) { std::printf("FAIL: phase_en held through a step %d times\n", bad_seq); return 1; }

  // the expected best: the margin of every (n, k) the sweep could choose, in the model
  // the margin of a setting: the distance to the nearest failure, either the
  // read window's edge or the command/write region's (in phase)
  auto margin = [&](int n, long kk) {
    double off = double(off0) + kk * S - (n - 2) * double(T);
    double m = W / 2.0 - std::fabs(off);
    if (cbad_lo >= 0) {
      double ph = std::fmod(double(off0) + kk * S, double(T)); if (ph < 0) ph += T;
      if (ph >= cbad_lo && ph < cbad_hi) return -1.0;
      double d1 = std::fmod(cbad_lo - ph + 2.0 * T, double(T)), d2 = std::fmod(ph - cbad_hi + 2.0 * T, double(T));
      m = std::min(m, std::min(d1, d2));
    }
    return m;
  };
  double best = -1e9;
  if (!dead) for (int n = 0; n < 6; n++) for (int kk = 0; kk < 32; kk++) { double m = margin(n, kk); if (m > best) best = m; }
  double got = dead ? 0 : margin(int(t->rd_sel), k);
  std::printf("  margin now %.1f units (%.0f ps), best possible %.1f (%.0f ps)\n", got, got * 7143 / T, best, best * 7143 / T);
  bool fail = false;
  if (man >= 0) {
    if (k != 0 || t->rd_sel != man || steps != 0) { std::printf("FAIL: manual mode moved something\n"); fail = true; }
  } else if (!dead) {
    if (best > S && !ok) { std::printf("FAIL: no calibration though a window exists\n"); fail = true; }
    if (ok && got < best - 2 * S) { std::printf("FAIL: margin %.1f below best %.1f - 2 steps\n", got, best); fail = true; }
    if (!ok && k != 0) { std::printf("FAIL: fallback not at phase 0\n"); fail = true; }
  }

  // the memory after it: random words written then read back in blocks of four
  std::vector<std::pair<uint32_t, uint64_t>> blk;
  int errs = 0;
  for (int o = 0; o < ops; o++) {
    uint32_t a = (rng() & 0x3FFFFFC) | 0;          // word address, a 4-word block
    a &= (1u << 26) - 4;
    uint64_t v = (uint64_t(rng()) << 32) | rng();
    for (int w = 0; w < 4; w++) {
      t->b_waddr = a + w; t->b_wdin = uint16_t(v >> (16 * w)); t->b_wreq = 1;
      run_cycles(1); t->b_wreq = 0;
      long c1 = cyc; while (!t->b_wack && cyc - c1 < 5000) run_cycles(1);
      if (!t->b_wack) { std::printf("FAIL: write ack\n"); return 1; }
      run_cycles(1);
    }
    t->b_raddr = a; t->b_rreq = 1; run_cycles(1); t->b_rreq = 0;
    long c1 = cyc; while (!t->b_rack && cyc - c1 < 5000) run_cycles(1);
    if (!t->b_rack) { std::printf("FAIL: read ack\n"); return 1; }
    if (uint64_t(t->b_rdout) != v) errs++;
    run_cycles(1);
  }
  std::printf("  memory test: %d of %d blocks wrong\n", errs, ops);
  // errors are expected only where the setting in use has no margin in the model
  bool expect_ok = dead ? margin(2, 0) > 0 : margin(int(t->rd_sel), k) > 0;
  if (expect_ok && errs) { std::printf("FAIL: memory errors\n"); fail = true; }
  std::printf(fail ? "RESULT FAIL\n" : "RESULT PASS\n");
  delete t;
  return fail ? 1 : 0;
}
