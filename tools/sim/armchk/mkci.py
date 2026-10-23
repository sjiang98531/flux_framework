
import os,re,sys
n,out=sys.argv[1],sys.argv[2]
dirs=['FW/Inc','MCI/Inc','User/Inc','Board/Inc','Main/Inc','User/HeadFiles','CMSIS','Device/inc','Device/src']
have={}
for d in dirs:
    p=os.path.join(n,d)
    if os.path.isdir(p):
        for f in os.listdir(p): have.setdefault(f.lower(),os.path.join(p,f))
inc=set()
for root,_,fs in os.walk(n):
    for f in fs:
        if f.endswith(('.c','.h')):
            try: s=open(os.path.join(root,f),encoding='latin-1').read()
            except: continue
            inc.update(re.findall(r'#\s*include\s*"([^"]+)"',s))
for x in inc:
    exact=any(os.path.exists(os.path.join(n,d,x)) for d in dirs)
    if not exact and x.lower() in have:
        os.symlink(have[x.lower()],os.path.join(out,x))
