# bonna79's options: what they do, how they were tested, and two game configurations

These are changes from our fork (bonna79/SegaModel3_MiSTer). On the board we tested them in two
**separate, game-specific cores** built from your main of 2026-11-19 (eccf1a0) on a single-SDRAM
MiSTer: `model3daytona2` (Daytona USA 2) and `model3rally2` (Sega Rally 2). We know you want one
core, so in this branch every change sits behind a Verilog macro and **is off by default**. With
no macro set, the core builds as your main does. The plan is to find which options are good for
every game and turn those on in the single core.

Ported onto your current main (1bf55dd). Two of our changes are left out because your main now
covers the same ground:
- our one-cycle `stfs` (FSF): you have F1S;
- our polygon-RAM DDR3 write bursts (R3PB): you have your own Real3D write bursts.

The combination with your new CPI items, PRX/RPX and write bursts has been checked on the benches
below but **not yet on the board**.

## The options

| Macro | Where | What it does |
|---|---|---|
| `M3_BXC` | m2_sdram `BXC` (m3_system `SDR_BXC`) | the chip bit XOR the row's bit 3 (byte bit 14): each region spread over both chips' eight banks |
| `M3_SND_WD15` | m3_system → m3_sndbridge `WD_BITS 15` | the bridge's watchdog at 743 ms instead of 93 ms (see "Audio") |
| `M3_SND_D384` | m3_system → m3_sndbridge `D 384` | play delay 384 samples (8.7 ms) instead of 256 |
| `M3_DT4` | ppc_core `DT4` (m3_system `CPU_DT4`) | a 4-way, 128-entry data TLB (the 603e's is 2-way, 64) |
| `M3_DUN8` | ppc_core `DUN 8` (m3_system `CPU_DUN`) | 8-entry data micro-TLB (default 4) |
| `M3_IPF` | ppc_icache `IPF` (m3_system `CPU_IPF`) | I-cache next-line prefetch: while a line streams and the next line of the same page misses, it is filled in the background |
| `M3_DMADIR` | m3_scripts `DIR`, m3_bus `DDIR` | a Step 2 Real3D DMA move into the host port (88/8C/8E/90/94/98) feeds r3d_cmd straight from the move engine's FIFO, not as line writes over the bus |
| `M3_R3LATE` | r3d_cmd | late start: a frame the Real3D cannot take at its flip waits (host held, as for any draw) and starts as soon as it can, instead of being dropped |
| `M3_R3EMPTY` | r3d_geo `EMPTY_DROP` | a triangle that covers no pixel (the rasteriser's own rounding) is not set up or sent |
| `M3_NO_VGA` | sys_top | no analog VGA path (HDMI only), to make room in test builds |

### DT4 on your main: one fix needed

Your new RPX path took the pending request's page and WIMG from TLB ways 0/1 only
(`d_xpa`, `d_xwimg`). With DT4 the hit can be in ways 2/3, so `test_cpu --dt4` failed at seed 1. In
this branch those two now pick the hit's way of four, as the micro-TLB fill already does
(`DT4 = 0` keeps your expression).

### EMPTY_DROP: why and the rule

Far cars and objects are drawn at full detail. In Sega Rally 2's snow stage (Supermodel attract
dumps) the heavy frames send 18-22k triangles, and 50-60% of them cover no pixel (median height
0.6 px). The geometry still runs the set-up kernel for each one, and the rasteriser stores,
replays, converts and walks each one.

The rasteriser draws:
- rows from `round(y_min)` to `round(y_max) - 1`, within 0..383;
- in each row, columns from `round(x_left)` to `round(x_right) - 1`, within 0..495.

Here "round" means two steps, as in rtl_tri: the vertex is rounded to 16.16 (nearest, ties away),
then rounded to an integer by poly.h's rule (half rounds down). Every edge's x at a row centre lies
between the triangle's x_min and x_max.

So when either range is empty after clamping to the screen (`round(y_min) >= round(y_max)` or
`round(x_min) >= round(x_max)`), the triangle draws nothing. The set-up port drops it after its
staging words, as it drops a non-finite one, before the set-up kernel. The test runs in a 3-stage
pipeline beside the gathered words and adds no cycles.

## Benches (all on this branch unless noted)

- **CPU**:
  - `test_cpu`, 200 seeds: PASS with the defaults;
  - DT4 = 1 (`--dt4`): PASS, with and without DUN 8;
  - DT4 = 1 with RPX = 0: PASS (this run is how we found the RPX interaction);
  - more seeds and `--xpage`: see the PR text.
- **r3d_geo** (`tb_r3d_geo --setup sm --cull --lod --setup-out --step 20 --lat 25 35`, new `--empty`):
  - SR2 race dumps (400 frames): every triangle field matches the model, with EMPTY_DROP and without.
    448k of 2.02M records are dropped, and cycles per frame fall at p99 by 4.6% and at max by 6%.
  - Snow dumps (326 frames): 325 frames match. Frame 10290 has a different count with and without
    EMPTY_DROP, so it is a property of that dump, not of the change. 267k records are dropped.
  - Worst snow frames (9760-9830): geometry -7.4%.
- **r3d_rast** (MIP = 1, the board's records against EMPTY_DROP's), worst snow frames: 0 pixels wrong,
  the same shaded pixels, replay cycles -14 to -16%, binning -44 to -48%.
- **DMA** (`sim/system/m3_dma_tb.sv` + `tb_m3_dma.cpp`, new): bus + SDRAM port + m2_sdram as the cores
  build them, the CPU and snoop models, and an r3d_cmd FIFO with drain and holds. Every host-port
  word is checked in order, and so is every CPU read.
  - The base reproduces the board's 14.2 clk/word.
  - With Supermodel's logged race moves (`--mvfile`): base 14.1-15.5 clk/word, DMADIR 9.5-13.2.
  - A planted fault (`M3_DMADIR_FAULT`) is caught.
- **Late start**: r3d_host_tb traces, every frame drawn, all checks pass (on our fork's base).

## The two game configurations (as tested on the board)

Both are built from eccf1a0 with these macros added to Model3.qsf, plus `.SDR_BXC(1'b1)` (`M3_BXC`
here), FSF (your F1S now) and R3PB (your bursts now).

**Daytona 2** (`model3daytona2`, 135 MHz): `M3_BXC M3_SND_WD15 M3_SND_D384 M3_DT4 M3_DUN8 M3_IPF
M3_DMADIR M3_NO_VGA`. Being built now: `+ M3_R3LATE M3_R3EMPTY`.

**Sega Rally 2** (`model3rally2`, 140 MHz): `M3_BXC M3_SND_WD15 M3_SND_D384 M3_IPF M3_R3LATE
M3_NO_VGA`. Being built now: `+ M3_R3EMPTY`. No DT4: SR2 hardly uses the TLB.

**Clocks.** In rtl/pll/pll_0002.v, outputs 1-3 are set to 135 or 140 MHz and `SYS_HZ` in Model3.sv
is changed to match. The SDRAM clocks keep the same lead in ns as at 130 MHz. The phase must be a
multiple of the VCO's 1/8 step (VCO = 4 × f), otherwise the fitter rejects it.

| Clock | phase_shift2 | phase_shift3 |
|---|---|---|
| 130 MHz (yours) | 3125 ps | 2885 ps |
| 135 MHz | 3009 ps | 2778 ps |
| 140 MHz | 2902 ps | 2679 ps |

Fits:
- D2 at 135, seed 4: holds positive, setup -4.245 (the 130 MHz builds sit at -3.4 to -4.2 on the
  same report).
- SR2 at 140, seed 3: holds positive, setup -3.850.

You saw wrong results in Daytona 2's attract at 135 MHz. We have not seen graphics errors at 135 in
Daytona 2 or at 140 in Sega Rally 2 so far, but it is one board.

## Board results (one MiSTer, HDMI, overlay OCR every second, two passes)

**Daytona 2** race mean fps, by build:

| Build | Race mean fps |
|---|---|
| your main of 2026-11-19 + BXC | fixed 29 in the race |
| + DT4, DUN 8 | 32.9 |
| + IPF | 32.6 (fewer dips, laps 2% faster) |
| + R3PB | 39.9 |
| + DMADIR, at 135 MHz | **55.0** (76% of race seconds at 56-58) |

What is left at 135 MHz:
- Once a lap, at the same place, it drops to 42-48: all drops "geometry still running" → the late
  start being built now.
- A few seconds go to 29-44 with no drops: the CPU's frame at 0.97-1.02 vblank, of which DMA wait
  is 0.04-0.07.

**Sega Rally 2**:
- Late start at 130 MHz: Real3D drops went from 278 to 0, puddle minimum 43 → 50, spectators
  29 → 36, race mean 56.0.
- At 140 MHz: race mean 56.4, 88% of race seconds at 56-58, stage starts at 50-51 (38-39 and 30 at
  130). The only dip left is the snow stage's spectators (40-46), where both geometry (about 1.25
  vblank a frame) and rasteriser (about 1.12) are over budget → EMPTY_DROP, being built now.

**Audio** (`M3_SND_WD15`): before it, the bridge's 93 ms watchdog fired when the ARM server fell
behind at a load peak. It reset the bridge, the server rebooted the sound board, and the music the
game had started never came back. With 743 ms there were zero resets in races of both games.
