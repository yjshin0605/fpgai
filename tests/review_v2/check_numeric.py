"""Deterministic arithmetic/feature checks; run from repo root with --rtl ROOT.
Requires iverilog/vvp in PATH (or IVL/IVL_BASE/VVP environment overrides).
Models the documented integer operations, not classifier accuracy or FPGA timing.
"""
import os, random, subprocess, tempfile, argparse
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--rtl',default='.');args=ap.parse_args();root=Path(args.rtl).resolve()
rng=random.Random(605)
checks=0
with tempfile.TemporaryDirectory() as td:
 td=Path(td)
 def run(name,decl,body,ports):
  global checks
  src='`timescale 1ns/1ps\nmodule tb; reg clk=0; always #5 clk=~clk; reg rstn=0;\n'+decl+'\ninitial begin repeat(3) @(negedge clk); rstn=1;\n'+body+'\n$display("PASS '+name+'"); $finish; end\ninitial begin #100000000; $fatal(1,"timeout"); end endmodule\n'
  (td/'tb.v').write_text(src)
  cmd=[os.getenv('IVL','iverilog')]
  if os.getenv('IVL_BASE'):cmd+=['-B',os.environ['IVL_BASE']]
  subprocess.run(cmd+['-g2012','-s','tb','-o',str(td/'a.vvp'),str(td/'tb.v')]+[str(root/(p+'.v')) for p in ports],check=True)
  subprocess.run([os.getenv('VVP','vvp'),str(td/'a.vvp')],check=True)
 def eq(expr,value):
  global checks
  checks+=1
  return f'if ({expr} !== 32\'d{value}) $fatal(1,"check {checks}: {expr} got=%0d expected={value}",{expr});\n'
 # All norm_sh values, signed extremes, truncation boundaries and random IQ.
 body=''
 for sh in range(-16,16):
  for i,q in [(0,0),(-131072,-131072),(131071,-131072),(1,3),(950,350)]+[(rng.randrange(-131072,131072),rng.randrange(-131072,131072)) for _ in range(20)]:
   hi,lo=sorted([abs(i),abs(q)],reverse=True);v=hi+(lo>>2)+(lo>>3);v=min(131071,v<<sh if sh>=0 else v>>-sh)
   body+=f'@(negedge clk); si={i}; sq={q}; sh={sh}; sv=1; @(negedge clk); sv=0; wait(ov); #1;\n'+eq('ovalue',v)+'@(negedge clk);\n'
 run('env_detect 800 vectors / all 32 shifts','reg signed[17:0] si=0,sq=0; reg signed[4:0] sh=0; reg sv=0; wire ov; wire[16:0] ovalue; env_detect d(.clk(clk),.rstn(rstn),.s_i(si),.s_q(sq),.s_valid(sv),.norm_sh(sh),.e_tdata(ovalue),.e_tvalid(ov));',body,['env_detect'])
 body=''
 vals=[(0,0),(-16777216,-16777216),(16777215,16777215),(0,-16777216),(3000000,0),(6000000,0)]+[(rng.randrange(-16777216,16777216),rng.randrange(-16777216,16777216)) for _ in range(200)]
 for re,im in vals:
  x,y=abs(re),im
  for k in range(16):x,y=(x+(y>>k),y-(x>>k)) if y>=0 else (x-(y>>k),y+(x>>k))
  v=min(262143,max(0,x>>4))
  body+=f'@(negedge clk); re={re}; im={im}; sv=1; @(negedge clk); sv=0; wait(ov); #1;\n'+eq('ovalue',v)+'@(negedge clk);\n'
 run('cordic_mag 206 signed and saturation vectors','reg signed[24:0] re=0,im=0;reg sv=0;wire ov;wire[17:0] ovalue; cordic_mag d(.clk(clk),.rstn(rstn),.s_re(re),.s_im(im),.s_tvalid(sv),.m_tvalid(ov),.m_tdata(ovalue));',body,['cordic_mag'])
 body=''
 frames=[[0]*256,[262143]*256,[3]*256,[0]*255+[1000],[1000]+[0]*127+[1000]+[0]*127]+[[rng.randrange(262144) for _ in range(256)] for _ in range(20)]
 for vals in frames:
  mx=max(vals);peak=min((i+128)%256 for i,v in enumerate(vals) if v==mx);bw=sum(v>=mx//4 for v in vals)
  for k,v in enumerate(vals):body+=f'@(negedge clk); sv=1; v={v}; bin={k}; last={int(k==255)};\n'
  body+='@(negedge clk);sv=0;last=0;wait(ov);#1;'+eq('p',peak)+eq('m',mx)+eq('c',bw)+'@(negedge clk);\n'
 run('traj_frame 25 full frames: tie/last/zero/256 width/random','reg sv=0,last=0;reg[17:0] v=0;reg[7:0] bin=0;wire ov;wire[7:0] p;wire[17:0] m;wire[8:0] c;traj_frame d(.clk(clk),.rstn(rstn),.s_tdata(v),.s_bin(bin),.s_invalid(1\'b0),.s_frame(8\'d0),.s_tlast(last),.s_tvalid(sv),.fr_valid(ov),.fr_p(p),.fr_m(m),.fr_c(c));',body,['traj_frame'])
 body=''
 names='n_active n_pairs n_zero n_small n_jump mono curv_sum n_triples rep_max rep_lag hmax nocc bw_sum'.split()
 records=[]
 for kind in range(24):
  p=[42]*127 if kind<2 else ([i for i in range(127)] if kind==2 else ([255 if i%2 else 0 for i in range(127)] if kind==3 else [rng.randrange(256) for _ in range(127)]))
  invalid=[kind==0 or (kind>=4 and rng.randrange(5)==0) for i in range(127)]; skip=0 if kind<4 else [0,24,126,127,255][kind%5];c=[256 if kind<4 else rng.randrange(257) for _ in p];a=[not invalid[i] and i>=skip for i in range(127)]
  diffs=[p[i]-p[i-1] for i in range(1,127) if a[i] and a[i-1]];curv=[abs(p[i]-2*p[i-1]+p[i-2]) for i in range(2,127) if a[i] and a[i-1] and a[i-2]]
  reps=[sum(a[i] and a[i-l] and abs(p[i]-p[i-l])<=1 for i in range(l,127)) for l in range(2,64)];best=max(reps);lag=reps.index(best)+2 if best else 0;h=[sum(a[i] and p[i]==k for i in range(127)) for k in range(256)]
  exp=[sum(a),len(diffs),sum(d==0 for d in diffs),sum(1<=abs(d)<=4 for d in diffs),sum(abs(d)>4 for d in diffs),abs(sum((1 if d>0 else -1) for d in diffs if 1<=abs(d)<=4)),sum(curv),len(curv),best,lag,max(h),sum(v>=2 for v in h),sum(c[i] for i in range(127) if a[i])]
  body+=f'skip={skip};\n'
  for i in range(127):body+=f'@(negedge clk); sv=1;p={p[i]};c={c[i]};inv={int(invalid[i])};fr={i}; @(negedge clk);sv=0;\n'
  body+='wait(ov);#1;'+''.join(eq(n,v) for n,v in zip(names,exp))+'@(negedge clk);\n'
 decl='reg sv=0,inv=0;reg[7:0] p=0,fr=0,skip=0;reg[8:0] c=0;wire ov;wire[31:0] '+','.join(names)+';traj_rec d(.clk(clk),.rstn(rstn),.fr_valid(sv),.fr_p(p),.fr_c(c),.fr_invalid(inv),.fr_frame(fr),.skip_frames(skip),.regs_valid(ov),'+','.join('.'+n+'('+n+')' for n in names)+');'
 run('traj_rec 24 records / all 13 features',decl,body,['traj_rec'])
 body=''
 for kind in range(5):
  bins=([0]*16384 if kind==0 else [31]*16384 if kind==1 else [0]*8192+[31]*8192 if kind==2 else [i%32 for i in range(16384)] if kind==3 else [rng.randrange(32) for _ in range(16384)])
  h=[bins.count(i) for i in range(32)];s=sum(bins);ss=sum(b*b for b in bins);exp=[sum(h[i]>=256 and h[i]>(h[i-1] if i else 0) and h[i]>=(h[i+1] if i<31 else 0) for i in range(32)),sum(v>=256 for v in h),s>>6,(16384*ss-s*s)>>20]
  for b in bins:body+=f'@(negedge clk);sv=1;v={b*1024};\n'
  body+='@(negedge clk);sv=0;wait(ov);#1;'+''.join(eq(n,v) for n,v in zip(['nm','occ','meanq','varq'],exp))+'@(negedge clk);\n'
 run('env_hist 5 full records / 4 features / max variance','reg sv=0;reg[16:0] v=0;wire ov;wire[31:0] nm,occ,meanq,varq;env_hist d(.clk(clk),.rstn(rstn),.e_tvalid(sv),.e_tdata(v),.regs_valid(ov),.n_modes(nm),.occ(occ),.mean_q8(meanq),.var_q8(varq));',body,['env_hist'])
 print('PASS numeric checks:',checks)
