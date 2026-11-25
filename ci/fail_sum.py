# Group the failing endpoints by the module path of their destination (and source)
import sys,re,collections
def mod(n,d=4):
    n=re.sub(r'\[\d+\]','[]',n); parts=n.split('|')[:-1]
    parts=[p.split(':')[-1] for p in parts]
    return '|'.join(parts[:d]) or '(top)'
for kind in ('setup','recovery'):
    try: L=[l.rstrip('\n').split('\t') for l in open(f'fail_{kind}.txt') if l.strip()]
    except OSError: print(kind,'no report'); continue
    L=[(float(s),f,t) for s,f,t in L]
    print(f'== {kind}: {len(L)} failing endpoints, TNS {sum(s for s,_,_ in L):.1f} ns, worst {min([s for s,_,_ in L] or [0]):.3f}')
    for depth in (3,5):
        g=collections.defaultdict(list)
        for s,f,t in L: g[(mod(t,depth),mod(f,depth))].append(s)
        print(f'-- by (to, from) module, depth {depth}: endpoints, worst, sum')
        for k,v in sorted(g.items(),key=lambda kv:min(kv[1]))[:40]: print(f'{len(v):6d} {min(v):7.3f} {sum(v):9.1f}  to {k[0]}  from {k[1]}')
    print('-- worst 25 endpoints')
    for s,f,t in sorted(L)[:25]: print(f'{s:7.3f}  {f}  ->  {t}')
