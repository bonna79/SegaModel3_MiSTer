# (our fork) a synthetic Real3D host trace and Supermodel-style dumps for tb_r3d_host (the board form: build it with
# +define+M3_DMAFAST=1 +define+M3_R3B=1 [+define+M3_R3PB=1]). usage: gen_r3d_synth.py OUTDIR SEED NFRAMES (D2=1: line-aligned polygon moves)
import sys, struct, random, os
out, seed, nf = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
rng = random.Random(seed)
os.makedirs(out + '/dump', exist_ok=True)
recs = []
def rec(frame, addr, kind, size, data=0): recs.append(struct.pack('<IIIBBBBQ', frame, 0, addr, kind, size, 0, 0, data))
def bsw(v): return int.from_bytes(v.to_bytes(4, 'little'), 'big')
m8c = bytearray(0x400000); m8e = bytearray(0x100000); mp = bytearray(0x400000)
def wr(frame, hi, a, v):
    rec(frame, (hi << 24) | (a << 2), 1, 4, bsw(v))   # bus value; tb byte-swaps back to v
    m = {0x8C: m8c, 0x8E: m8e, 0x98: mp}[hi]
    m[4*a:4*a+4] = v.to_bytes(4, 'little')
FLIP = 368
rec(0, 0xF1180008, 1, 4, bsw(FLIP))
# a few fixed polygon areas rewritten every frame (as games do), plus random runs
areas = [(rng.randrange(0, 0xF0000) & ~7, rng.randrange(64, 1200)) for _ in range(6)]
lst = []
for f in range(1, nf + 1):
    rec(f, 0, 2, 3)                                           # frame start (line 384)
    def burst(no8c=False):
        k = rng.random()
        if no8c and 0.75 <= k < 0.85: k = 0.9
        if k < 0.45:                                          # polygon RAM run (aligned area, or random)
            if rng.random() < 0.6: st, n = rng.choice(areas); st += rng.randrange(0, 4)*8
            else: st, n = rng.randrange(0, 0xFFC00), rng.randrange(1, 900)
            if os.environ.get('D2'): st &= ~7; n = max(8, n & ~7)
            elif rng.random() < 0.3: st += rng.randrange(1, 8)  # unaligned start
            for i in range(n): wr(f, 0x98, (st + i) & 0xFFFFF, rng.getrandbits(32))
        elif k < 0.75:                                        # 8E ten-word moves (SR2) or longer
            st = rng.randrange(0, 0x3FF00); n = 10 if rng.random() < 0.7 else rng.randrange(1, 200)
            for i in range(n): wr(f, 0x8E, (st + i) & 0x3FFFF, rng.getrandbits(32))
        elif k < 0.85:                                        # 8C singles (above Supermodel's update buffers)
            for i in range(rng.randrange(1, 20)): wr(f, 0x8C, rng.randrange(0x20000, 0x100000), rng.getrandbits(32))
        else:                                                 # interleaved single polygon / 8E words
            for i in range(rng.randrange(1, 40)):
                if rng.random() < 0.5: wr(f, 0x98, rng.randrange(0, 0x100000), rng.getrandbits(32))
                else: wr(f, 0x8E, rng.randrange(0, 0x40000), rng.getrandbits(32))
    for _ in range(rng.randrange(4, 14)): burst()
    rec(f, 0x88000000, 1, 4, 0)                               # the command port
    rec(f, 0xF118000C, 1, 4, 0)                               # tile register 0C
    rec(f, FLIP, 2, 2)                                        # the flip
    if f % 2 == 0:
        lst.append(f)
        with open(f'{out}/dump/r3d_{f:06d}.bin', 'wb') as fh:
            fh.write(m8c); fh.write(m8e); fh.write(mp); fh.write(b'\0'*0x800000)
    for _ in range(rng.randrange(0, 3)): burst(True)          # next frame's first writes, after the flip (no 8C: it is checked at the frame's end)
rec(nf + 1, 0, 2, 3)
open(out + '/trace.bin', 'wb').write(b''.join(recs))
open(out + '/list.txt', 'w').write(','.join(map(str, lst)))
print(len(recs), 'records', len(lst), 'frames listed')
