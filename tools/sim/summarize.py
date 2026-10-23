import csv,sys
def summ(fn):
    rows=list(csv.DictReader(open(fn)))
    # first time run state reaches 4 after each start command
    out=[]
    prev=None
    for r in rows:
        st=(r['main'],r['run'])
        if st!=prev: out.append((float(r['t']),st,float(r['true_rpm'])))
        prev=st
    return out
for fn in sys.argv[1:]:
    tr=summ(fn)
    print(fn, ' '.join(f"{t:.2f}:{m}/{s}({v:.0f})" for t,(m,s),v in tr[:30]))
