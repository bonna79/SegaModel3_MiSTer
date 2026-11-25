// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA (our fork)
//
// The Real3D DMA (Step 2, Daytona 2) on m3_bus as the Daytona 2 core builds it (m3_dma_tb.sv): m3_scripts' move
// engine as master 1, a CPU model as master 0, the SDRAM at its pins and the Real3D's host port as r3d_cmd's
// posted FIFO (32 words, drained at a chosen rate, with flip holds). The CPU programs moves as Daytona 2 does
// (C2000000 source, 04 destination, 0C configuration, 08 length starts; it waits for the interrupt, then
// acknowledges it), between them writes the command port and reads the status, and all along makes its own
// memory traffic (line fills, cast-outs, single accesses) in its own region, checked against a byte image.
// Checked: every host-port write, in order (the moves' words from the source as it was, byte-reversed when the
// configuration says, at the destination's addresses; the CPU's own writes between moves); the CPU's reads;
// the protocol. Measured: the cycles, the DMA's busy clocks a word, the CPU's transactions and their latency.
//
//   Vm3_dma_tb [--moves N] [--seed S] [--drain LO HI] [--hold EVERY LEN] [--cpu PCT] [--lens L,L,..]
//              [--snoop LO HI] [--wait PCT] [--quiet]
#include "Vm3_dma_tb.h"
#include "verilated.h"
#include "sdram_dev.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <random>
#include <string>
#include <vector>

static Vm3_dma_tb *t;
static sdram_dev sd;
static uint64_t cyc = 0;
static std::mt19937 rng, mv_rng;   // mv_rng: the moves alone, the same list whatever the timing
static long bad = 0, proto = 0;
static int rr(int lo, int hi) { return hi <= lo ? lo : lo + int(rng() % unsigned(hi - lo + 1)); }
static uint32_t bswap(uint32_t v) { return (v >> 24) | ((v >> 8) & 0xff00) | ((v << 8) & 0xff0000) | (v << 24); }

// work RAM as the bus reads it (word form: SDRAM 0x6400000 + address, little-endian bytes of the big-endian word)
static std::vector<uint8_t> ref;
static uint32_t const SDR_RAM = 0x6400000;
static void ref_write(uint32_t a, uint32_t v, uint32_t be) {
  uint32_t const s = SDR_RAM + (a & 0x7FFFFC);
  for (int k = 0; k < 4; k++) if ((be >> (3 - k)) & 1) ref[s + 3 - k] = uint8_t(v >> (24 - 8 * k));
}
static uint32_t ref_word(uint32_t a) {
  uint32_t const b = SDR_RAM + (a & 0x7FFFFC);
  return (uint32_t(ref[b + 3]) << 24) | (uint32_t(ref[b + 2]) << 16) | (uint32_t(ref[b + 1]) << 8) | ref[b];
}

struct Txn { int cmd; uint32_t addr, be; uint32_t wd[8]; std::vector<uint32_t> exp; bool check = true; int kind = 0; uint64_t t0 = 0; };
enum { K_MEM, K_REG, K_BUSY, K_ST, K_88 };

static std::deque<std::pair<uint32_t, uint32_t>> h_exp;   // host-port writes owed, in order
static long h_n = 0, h_bad = 0;
struct Mv { uint32_t src, dst, len; };
static std::vector<Mv> mvf;
static uint32_t h_form(uint32_t a) { return (a & 0xFF000000u) | (a & 0x003FFFFCu); }

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  long moves = 200; unsigned seed = 1; int dr_lo = 4, dr_hi = 12, hold_every = 0, hold_len = 0;
  int sn_lo = 4, sn_hi = 36, wait_pct = 50, cpu_pct = 30; bool quiet = false;
  std::vector<int> lens = {8, 24, 64, 128, 256, 512, 1024, 2048};
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto nx = [&]() { return std::atoi(argv[++i]); };
    if (a == "--moves") moves = nx();
    else if (a == "--seed") seed = unsigned(nx());
    else if (a == "--drain") { dr_lo = nx(); dr_hi = nx(); }
    else if (a == "--hold") { hold_every = nx(); hold_len = nx(); }
    else if (a == "--snoop") { sn_lo = nx(); sn_hi = nx(); }
    else if (a == "--wait") wait_pct = nx();
    else if (a == "--cpu") cpu_pct = nx();
    else if (a == "--lens") { lens.clear(); std::string l = argv[++i]; for (char *p = std::strtok(l.data(), ","); p; p = std::strtok(nullptr, ",")) lens.push_back(std::atoi(p)); }
    else if (a == "--quiet") quiet = true;
    else if (a == "--mvfile") {   // Supermodel's move log (frame pc src dst len a line): the moves in order, the source mapped into the bench's region
      FILE *f = std::fopen(argv[++i], "r"); unsigned fr, pc, sa, da, ln;
      while (f && std::fscanf(f, "%u %x %x %x %u", &fr, &pc, &sa, &da, &ln) == 5) mvf.push_back({sa, da, ln});
      if (f) std::fclose(f);
    }
  }
  rng.seed(seed); mv_rng.seed(seed * 7919u + 1u);
  sd.bxor = true;
  t = new Vm3_dma_tb;
  ref.assign(size_t(1) << 27, 0);
  // the DMA's sources 0x400000-0x47FFFF and the CPU's region 0x100000-0x10FFFF: random
  for (uint32_t base : {0x400000u, 0x100000u})
    for (uint32_t a = 0; a < (base == 0x400000u ? 0x80000u : 0x10000u); a += 2) {
      uint16_t v = uint16_t(rng()); uint32_t s = SDR_RAM + base + a;
      sd.m[s >> 1] = v; ref[s] = uint8_t(v); ref[s + 1] = uint8_t(v >> 8);
    }

  t->rst_n = 0; t->rst = 1; t->c_req = 0; t->sn_ack = 0; t->h_rdy_i = 1;
  // r3d_cmd's FIFO
  int pf_n = 0, pf_dly = 0; long hold_left = 0, hold_cnt = 0;
  auto tick = [&]() {
    t->sd_dq_i = sd.dq(cyc);
    // flip holds (sw_wait): the port closed for hold_len clocks every hold_every
    if (hold_every && ++hold_cnt >= hold_every) { hold_cnt = 0; hold_left = hold_len; }
    bool const held = hold_left > 0; if (held) hold_left--;
    t->h_rdy_i = !held && pf_n < 32;
    t->clk = 0; t->eval();
    if (t->h_go_o && t->h_rdy_i) {
      if (t->h_we_o) {
        h_n++; pf_n++;
        if (h_exp.empty() || h_exp.front().first != t->h_addr_o || h_exp.front().second != t->h_wdata_o) {
          if (h_bad++ < 8) std::printf("cycle %lu: host port write %08X %08X, expected %08X %08X\n", (unsigned long)cyc, t->h_addr_o, t->h_wdata_o,
                                       h_exp.empty() ? 0 : h_exp.front().first, h_exp.empty() ? 0 : h_exp.front().second);
        }
        if (!h_exp.empty()) h_exp.pop_front();
      }
    }
    if (pf_n > 0) { if (pf_dly <= 0) { pf_n--; pf_dly = rr(dr_lo, dr_hi) - 1; } else pf_dly--; }
    sd.cmd(cyc, t->sd_cke, t->sd_cs_n, t->sd_ras_n, t->sd_cas_n, t->sd_we_n, t->sd_ba, t->sd_a, t->sd_dq_o);
    t->clk = 1; t->eval();
    cyc++;
  };
  for (int i = 0; i < 4; i++) tick();
  t->rst_n = 1; t->rst = 0;
  while (!t->ready) tick();
  uint64_t const c_start = cyc;

  // the CPU
  struct { bool busy = false, acked = false; Txn x; int beats = 0, wbeat = 0; } M;
  std::vector<std::pair<uint32_t, uint32_t>> mv_words;   // the move's words, owed from its start
  std::deque<Txn> ctl;                  // the DMA program's accesses, in order, before any of the CPU's own
  enum { P_GAP, P_RUN, P_ACK } ph = P_GAP;
  long mv_done = 0, words = 0, c_done = 0, c_lat = 0, c_lat_n = 0, polls = 0, gap_left = 0;
  bool own_turn = false;
  int sn_st = 0, sn_dly = 0;
  long dbusy = 0, dsn = 0, dgr = 0, drd = 0, dwr = 0, lines = 0; int dr_n = -1, dw_n = -1;
  auto reg_w = [&](uint32_t off, uint32_t v, uint32_t be) { Txn x; x.cmd = 3; x.addr = 0xC2000000u + off; x.be = be; x.wd[0] = v; x.kind = K_REG; x.check = false; ctl.push_back(x); };
  uint64_t guard = 0;
  while (mv_done < moves || M.busy || ph != P_GAP) {
    if (++guard > 400000000ull) { std::printf("FAIL: timeout\n"); return 1; }
    // the DMA program
    if (ph == P_GAP && ctl.empty() && !M.busy && mv_done < moves) {
      if (gap_left-- <= 0) {
        // now and then the CPU's own Real3D accesses between moves: a command port write, a status read
        if (mv_rng() % 4 == 0) { Txn x; x.cmd = 3; x.addr = 0x88000000u; x.be = 15; x.wd[0] = mv_rng(); x.kind = K_88; x.check = false; ctl.push_back(x); }
        if (mv_rng() % 4 == 0) { Txn x; x.cmd = 2; x.addr = 0x84000000u; x.be = 15; x.kind = K_ST; x.check = false; ctl.push_back(x); }
        size_t const mi = size_t(mv_done) % (mvf.empty() ? 1 : mvf.size());
        int const len = mvf.empty() ? lens[mv_rng() % lens.size()] : int(mvf[mi].len);
        bool const swp = mv_rng() & 1;
        uint32_t const src = 0x400000u + ((mv_rng() % (0x80000u / 4 - uint32_t(len))) << 2);
        uint32_t const win = (mv_rng() % 4 == 0) ? 0x8C000000u : (mv_rng() % 3 == 0) ? 0x8E000000u : 0x98000000u;
        uint32_t const dst = win + ((mv_rng() % (0x100000u / 4 - uint32_t(len))) << 2);
        // the line alignment as the game has it, mostly: aligned source and destination
        uint32_t s2 = (mv_rng() % 4) ? (src & ~31u) : src, d2 = (mv_rng() % 4) ? (dst & ~31u) : dst;
        if (!mvf.empty()) {   // the logged destination; the source at its offset in the region, wrapped to fit
          d2 = mvf[mi].dst;
          uint32_t const off = ((mvf[mi].src & 0x7FFE0u) % (0x80000u - 4u * uint32_t(len) - 32u)) & ~31u;
          s2 = 0x400000u + off + (mvf[mi].src & 0x1Cu);
        }
        reg_w(0x00, bswap(s2), 15); reg_w(0x04, bswap(d2), 15); reg_w(0x0C, swp ? 0x8100u : 0x0100u, 0x2);
        reg_w(0x08, bswap(uint32_t(len)), 15);
        mv_words.clear();
        for (int k = 0; k < len; k++) { uint32_t w = ref_word(s2 + 4 * k); mv_words.push_back({h_form(d2 + 4 * k), bswap(swp ? bswap(w) : w)}); }
        words += len;
        ph = P_RUN;
      }
    }
    if (ph == P_RUN && ctl.empty() && !M.busy && t->ext_int) { reg_w(0x0C, 0x10000u, 0x4); ph = P_ACK; }
    if (ph == P_ACK && ctl.empty() && !M.busy) { ph = P_GAP; gap_left = rr(0, 200); mv_done++; }

    // the snoop: answered when the CPU is not mid-transaction, after a delay (the D-cache's lookup and its own work)
    t->sn_ack = 0;
    if (sn_st == 0 && t->sn_req) { sn_st = 1; sn_dly = rr(sn_lo, sn_hi); }
    if (sn_st == 1 && !M.busy && sn_dly-- <= 0) { t->sn_ack = 1; sn_st = 0; }

    // the CPU's next transaction
    if (!M.busy && sn_st == 0) {
      if (!ctl.empty()) { M.x = ctl.front(); ctl.pop_front(); M.busy = true; }
      else if (ph == P_RUN && (int(rng() % 100) < wait_pct)) {
        // waiting on the DMA: now and then a busy poll, as Daytona 2 does before its commands
        if (rng() % 64 == 0) { Txn x; x.cmd = 2; x.addr = 0xC200000Cu; x.be = 15; x.kind = K_BUSY; x.check = false; M.x = x; M.busy = true; polls++; }
      } else if (int(rng() % 100) < cpu_pct) {
        Txn x; x.kind = K_MEM; x.be = 15; int c = int(rng() % 4); x.cmd = c;
        x.addr = 0x100000u + ((rng() % 0x4000u) << 2);
        if (c <= 1) x.addr &= ~31u;
        if (c == 3) x.be = 1 + rng() % 15;
        for (auto &w : x.wd) w = rng();
        M.x = x; M.busy = true; own_turn = true;
      }
      if (M.busy) { M.acked = false; M.beats = 0; M.wbeat = 0; M.x.t0 = cyc; }
    }
    t->c_req = M.busy && !M.acked; t->c_cmd = uint8_t(M.x.cmd); t->c_addr = M.x.addr; t->c_be = uint8_t(M.x.be);
    t->c_wdata = M.x.wd[M.wbeat < 8 ? M.wbeat : 7];

    // counters, from master 1 as the bus sees it
    if (t->dma_busy) dbusy++;
    if (t->sn_req && !t->sn_ack) dsn++;
    if (t->d_req && !t->d_ack) dgr++;
    // read and write outstanding, as the diagnostic build counts them (acknowledge to the last beat)
    if (dr_n >= 0) { drd++; if (t->d_rvalid && --dr_n < 0) {} }
    if (dw_n >= 0) { dwr++; if (t->d_wnext && --dw_n < 0) {} }
    if (t->d_req && t->d_ack) { if (t->d_cmd == 0) { dr_n = 7; lines++; } else if (t->d_cmd == 2) dr_n = 0; else if (t->d_cmd == 1) dw_n = 7; }

    // outputs of the last edge
    if (t->c_ack) {
      if (!M.busy || M.acked) { if (proto++ < 5) std::printf("cycle %lu: CPU acknowledged with nothing pending\n", (unsigned long)cyc); }
      else {
        M.acked = true;
        Txn &x = M.x;
        if (x.kind == K_MEM) {
          if (x.cmd == 3) ref_write(x.addr, x.wd[0], x.be);
          else if (x.cmd == 1) for (int k = 0; k < 8; k++) ref_write(x.addr + 4 * k, x.wd[k], 15);
          else if (x.cmd == 2) x.exp = {ref_word(x.addr)};
          else for (int k = 0; k < 8; k++) x.exp.push_back(ref_word((x.addr & ~31u) + 4 * k));
        } else if (x.kind == K_88) h_exp.push_back({0x88000000u, bswap(x.wd[0])});
        else if (x.kind == K_REG && x.addr == 0xC2000008u) for (auto &w : mv_words) h_exp.push_back(w);
      }
    }
    if (t->c_rvalid) {
      if (!M.busy || !M.acked) { if (proto++ < 5) std::printf("cycle %lu: CPU read beat before acknowledge\n", (unsigned long)cyc); }
      else {
        if (M.x.check && M.beats < int(M.x.exp.size()) && t->c_rdata != M.x.exp[M.beats])
          if (bad++ < 8) std::printf("cycle %lu: CPU %s %08X beat %d: %08X, expected %08X\n", (unsigned long)cyc, M.x.cmd == 0 ? "LINE_RD" : "RD", M.x.addr, M.beats, t->c_rdata, M.x.exp[M.beats]);
        M.beats++;
      }
    }
    if (t->c_wnext) M.wbeat++;
    if (M.busy && M.acked) {
      bool fin = (M.x.cmd == 3) || (M.x.cmd == 2 && M.beats == 1) || (M.x.cmd == 0 && M.beats == 8) || (M.x.cmd == 1 && M.wbeat == 8);
      if (fin) {
        M.busy = false;
        if (own_turn) { c_done++; c_lat += long(cyc - M.x.t0); c_lat_n++; own_turn = false; }
      }
    }
    tick();
  }
  for (int i = 0; i < 2000 && (pf_n || !h_exp.empty()); i++) tick();
  uint64_t const cycles = cyc - c_start;
  bool const ok = !bad && !proto && !h_bad && h_exp.empty();
  std::printf("per line read: grant %.1f (snoop %.1f) read %.1f write %.1f\n", lines ? double(dgr) / lines : 0.0, lines ? double(dsn) / lines : 0.0, lines ? double(drd) / lines : 0.0, lines ? double(dwr) / lines : 0.0);
  std::printf("moves %ld words %ld cycles %lu: DMA busy %ld (%.2f clk/word), snoop wait %ld, grant wait %ld; host writes %ld; "
              "CPU %ld own transactions (latency %.1f), %ld polls; %ld read errors, %ld protocol, %ld host-port wrong, %zu owed  %s\n",
              mv_done, words, (unsigned long)cycles, dbusy, words ? double(dbusy) / words : 0.0, dsn, dgr, h_n, c_done,
              c_lat_n ? double(c_lat) / c_lat_n : 0.0, polls, bad, proto, h_bad, h_exp.size(), ok ? "PASS" : "FAIL");
  (void)quiet;
  delete t;
  return ok ? 0 : 1;
}
