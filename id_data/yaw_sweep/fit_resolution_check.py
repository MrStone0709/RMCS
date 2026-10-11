import csv, math, sys
import numpy as np
FS=1000.0
def load(p):
    rows=list(csv.DictReader(open(p,encoding='utf-8-sig')))
    g=lambda k: np.array([float(r[k]) for r in rows])
    return g('/gimbal/yaw/control_torque'), g('/gimbal/yaw/velocity')
def welch(u,y,fs,nfft):
    step,win=nfft//2,np.hanning(nfft); wp=np.sum(win**2)
    suu=suy=syy=None; n=0
    for s in range(0,len(u)-nfft,step):
        us=(u[s:s+nfft]-u[s:s+nfft].mean())*win
        ys=(y[s:s+nfft]-y[s:s+nfft].mean())*win
        U,Y=np.fft.rfft(us),np.fft.rfft(ys)
        a,b,c=np.abs(U)**2,np.conj(U)*Y,np.abs(Y)**2
        if suu is None: suu,suy,syy=a,b,c
        else: suu+=a; suy+=b; syy+=c
        n+=1
    suu/=n*wp; suy/=n*wp; syy/=n*wp
    with np.errstate(divide='ignore',invalid='ignore'):
        return np.fft.rfftfreq(nfft,1/fs), suy/suu, np.abs(suy)**2/(suu*syy), n
def fit(f,H,coh,lo,hi):
    m=(f>=lo)&(f<=hi)&(coh>0.4)&np.isfinite(H); idx=np.where(m)[0]
    if len(idx)<3: return None
    w=2*math.pi*f[idx]; Yv=1.0/H[idx]
    sw=np.sqrt(coh[idx]**2)
    A=np.vstack([np.column_stack([np.ones_like(w),np.zeros_like(w)]),
                 np.column_stack([np.zeros_like(w),w])])
    b=np.concatenate([Yv.real,Yv.imag]); sw2=np.concatenate([sw,sw])
    sol,*_=np.linalg.lstsq(A*sw2[:,None], b*sw2, rcond=None)
    B,J=float(sol[0]),float(sol[1])
    pred=1.0/(B+1j*J*w)
    res=np.median(np.abs(pred-H[idx])/np.abs(H[idx]))
    return B,J,res,len(idx)
for NFFT in [8192,16384,32768]:
    Bs,Js,rs,ns=[],[],[],[]
    for p in ['run1.csv','run2.csv','run3.csv','run4.csv','run5.csv']:
        u,y=load('/tmp/iddata/'+p); n0=int(1.0*FS)
        f,H,coh,nseg=welch(u[n0:],y[n0:],FS,NFFT)
        r=fit(f,H,coh,0.3,3.0)
        if r: Bs.append(r[0]); Js.append(r[1]); rs.append(r[2]); ns.append(r[3])
    Bs,Js,rs=np.array(Bs),np.array(Js),np.array(rs)
    print(f"NFFT={NFFT:6d} ({NFFT/FS:5.2f}s, {FS/NFFT:.4f}Hz, {nseg} 段, {int(np.mean(ns))} 点): "
          f"B={Bs.mean():.4f}+/-{Bs.std(ddof=1):.4f} (CV{100*Bs.std(ddof=1)/Bs.mean():4.1f}%)  "
          f"J={Js.mean():.5f}+/-{Js.std(ddof=1):.5f} (CV{100*Js.std(ddof=1)/Js.mean():4.1f}%)  "
          f"残差{rs.mean()*100:.1f}%")
