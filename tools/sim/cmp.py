import sys
a=open(sys.argv[1],'rb').read(); b=open(sys.argv[2],'rb').read()
n=min(len(a),len(b))//8
for i in range(n):
    if a[8*i:8*i+8]!=b[8*i:8*i+8]:
        print(f"DIVERGE at tick {i} (t={i/16000:.5f}s) of {n}"); sys.exit(1)
print(f"IDENTICAL {n} ticks ({n/16000:.2f}s)" + ("" if len(a)==len(b) else " LENGTH DIFFERS"))
