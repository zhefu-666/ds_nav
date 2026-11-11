"""Offline, timestamp-aligned IMU/chassis diagnostic metrics; no ROS runtime."""
import argparse,json,math
from pathlib import Path
import numpy as np

def rotate(q,v):
    a=np.array(q[:3]);v=np.array(v);return v+2*q[3]*np.cross(a,v)+2*np.cross(a,np.cross(a,v))
def multiply(a,b):
    av=np.array(a[:3]);bv=np.array(b[:3]);return [*list(a[3]*bv+b[3]*av+np.cross(av,bv)),a[3]*b[3]-np.dot(av,bv)]
def yaw(q):return math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
p=argparse.ArgumentParser();p.add_argument('run');args=p.parse_args();root=Path(args.run)
summary=json.loads((root/'summary.json').read_text());rows=[json.loads(l) for l in (root/'samples.jsonl').read_text().splitlines()]
a=[r for r in rows if r['topic']=='/aft_mapped_to_init'];b=[r for r in rows if r['topic']=='/sim/ground_truth_odometry']
tf=summary['map_to_camera_init']
if tf is None or not a or not b:raise SystemExit('No accepted map alignment or samples; cannot calculate accuracy')
bt=np.array([r['t'] for r in b]);bp=np.array([r['p'] for r in b]);by=np.unwrap([yaw(r['q']) for r in b]);errors=[];angles=[];times=[]
for r in a:
    t=r['t']
    if not bt[0]<=t<=bt[-1]:continue
    actual=np.array([np.interp(t,bt,bp[:,i]) for i in range(3)])
    estimated=rotate(tf['q'],r['p'])+tf['p'];errors.append(estimated-actual);times.append(t)
    dy=yaw(multiply(tf['q'],r['q']))-np.interp(t,bt,by);angles.append(math.atan2(math.sin(dy),math.cos(dy)))
e=np.array(errors);times=np.array(times);angles=np.array(angles)
events=summary['events'];begin=next((v['t'] for v in events if v['phase']=='translate'),float('inf'));end=next((v['t'] for v in events if v['phase']=='stop_rotation'),float('inf'));mask=(times>=begin)&(times<=end)
result={'completed':summary['completed'],'failure':summary['failure'],'reference':'IMU/body versus chassis; same current rigid origin; map alignment fixed before motion','matched_samples':len(e),'max_position_error_m':float(np.linalg.norm(e,axis=1).max()),'motion_samples':int(mask.sum())}
if mask.any():result.update(motion_position_rmse_m=float(np.sqrt((e[mask]**2).sum(axis=1).mean())),motion_yaw_rmse_deg=float(np.degrees(np.sqrt((angles[mask]**2).mean()))))
result['segments']={}
for i,event in enumerate(events):
    stop=events[i+1]['t'] if i+1<len(events) else bt[-1];sel=(bt>=event['t'])&(bt<=stop)
    pts=bp[sel]
    if len(pts)>1:result['segments'][event['phase']]={'displacement_m':(pts[-1]-pts[0]).tolist(),'duration_sim_s':float(bt[sel][-1]-bt[sel][0])}
(root/'metrics.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
