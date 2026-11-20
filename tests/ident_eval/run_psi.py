# ψ 扰动起动测试：10 类电机 × 磁链 ×{0.5,0.7,1.4,2.0} × 4 个初始角，AUTO 辨识 + 起动，60 s，允许故障后重启。
# 用法：先 run_eval.build_suite(ref, work) 构建，再 python3 run_psi.py <work>
import os,subprocess,sys,json,collections
import concurrent.futures as cf
CFG={"fan":"1100","fan48":"1100","bigL":"1100","mid16":"3000","mid48":"3000","sm120":"5000","sm48":"5000","vac16":"5000","vac48":"5000","ipm2":"1100"}
def one(a):
    wd,k,kp,th=a
    e=dict(os.environ); e.update({"SIM_PWMDLY":"1","SIM_NOISE":"2","SIM_DSAT":"0","SIM_PK_RS":"1","SIM_TH0_DEG":str(th),"SIM_KPSI":str(kp),"SIM_FAULT_CONTINUE":"1"})
    o=subprocess.run([wd+"/exe_"+k,"60","0",CFG[k]],env=e,capture_output=True,text=True).stdout.splitlines()
    S=[l for l in o if l.startswith("SENS ")]
    if not S: return k,kp,th,"noout"
    kv=dict(x.split("=") for x in S[-1].split()[1:] if "=" in x)
    ok=kv["fault"]=="0x0" and kv["run"]=="4" and abs(float(kv["rpm"]))>0.2*float(CFG[k])
    return k,kp,th,("ok" if ok else f"run{kv['run']}/rpm{float(kv['rpm']):.0f}/nF{kv['nFault']}")
for wd in sys.argv[1:]:
    jobs=[(wd,k,kp,th) for k in CFG for kp in (0.5,0.7,1.4,2.0) for th in (0,90,180,270)]
    res=list(cf.ThreadPoolExecutor(int(os.environ.get("NJ","4"))).map(one,jobs))
    json.dump(res,open(wd+"_psi.json","w"))
    print("==",wd,"total ok",sum(r[3]=="ok" for r in res),"/",len(res))
    for kp in (0.5,0.7,1.4,2.0):
        print(f"   ψ×{kp}: "+"  ".join(f"{k}:{sum(1 for r in res if r[0]==k and r[1]==kp and r[3]=='ok')}/4" for k in CFG))
