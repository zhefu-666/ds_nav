#!/usr/bin/env python3
"""Independent full mission endpoint, complete CAD, evidence and shutdown gates.

Truth and joint measurements are assessment-only. Missing envelope samples fail;
no gaps are dropped from the denominator. Historical outputs are never replaced.
"""
import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from sim_evidence_gate import evaluate, check_stream, shutdown_gate
from sim_trajectory_evaluate import interpolate_pose, chassis_to_base, yaw
from sim_tunnel_passage import envelope, clearance, RESOURCES


def passage_sequence_reasons(passes, expected, center_x):
    reasons=[]
    if len(passes) != expected:reasons.append('required_pass_count_not_met')
    targets=[row.get('target',[]) for row in passes]
    if any(len(target)!=3 or not np.isfinite(target).all() or abs(abs(target[0]-center_x)-1.7)>1e-6 for target in targets):
        reasons.append('passage_endpoint_not_at_opposite_staging_pose')
    elif any((a[0]-center_x)*(b[0]-center_x)>=0 for a,b in zip(targets,targets[1:])):
        reasons.append('passage_direction_not_alternating')
    stamps=[row.get('stamp') for row in passes]
    if any(not isinstance(t,(int,float)) or not math.isfinite(t) for t in stamps) or any(b<=a for a,b in zip(stamps,stamps[1:])):
        reasons.append('passage_time_not_increasing')
    return reasons


def assess(p, expected_passes=None):
    mission=p/'mission'; result=json.loads((mission/'result.json').read_text())
    events=[json.loads(line) for line in (p/'trajectory.jsonl').read_text().splitlines()]
    truth=list({e['stamp_ns']:e for e in events if e.get('kind')=='truth'}.values())
    stamps=[e['stamp_ns'] for e in truth]
    if len(stamps)<2:return dict(passed=False,reasons=['missing_truth'])
    report=evaluate(events,min(stamps),max(stamps))
    reasons=report['reasons']
    launch_log=(p/'simulation.log').read_text()
    reasons.extend(shutdown_gate(json.loads((p/'shutdown.json').read_text()),launch_log))
    search_failures=[line for line in launch_log.splitlines()
                     if '[SMAC 2D]' in line and 'open set exhausted' in line]
    report['smac_open_set_exhausted_count']=len(search_failures)
    if search_failures:reasons.append('smac_search_failure_during_mission')
    if not result.get('completed') or not result.get('disabled_confirmed') or result.get('failure'):
        reasons.append('mission_not_completed_and_disabled')
    config_path=p/'run_config.json'
    config=json.loads(config_path.read_text()) if config_path.exists() else {}
    post_count=config.get('post_navigation_goals',0)
    report['expected_post_navigation_goals']=post_count
    post_endpoints=[]
    if post_count:
        post_path=p/'post_navigation_result.json'
        post=json.loads(post_path.read_text()) if post_path.exists() else {}
        run_path=p/'run_result.json'
        run=json.loads(run_path.read_text()) if run_path.exists() else {}
        targets_path=p/'post_navigation_targets.json'
        targets=json.loads(targets_path.read_text()) if targets_path.exists() else []
        goals=post.get('goals',[])
        if (post.get('completed') is not True or post.get('failure')
                or run.get('post_navigation_exit') != 0 or post.get('cleanup_errors')
                or post.get('final_control_status') != 'disabled'
                or post.get('final_control_fresh') is not True):
            reasons.append('post_navigation_not_completed')
        if len(goals) != post_count or len(targets) != post_count:
            reasons.append('post_navigation_count_not_met')
        for index,goal in enumerate(goals):
            row=dict(number=index+1,target=goal.get('target'),passed=False)
            try:
                gt=chassis_to_base(interpolate_pose(truth,stamps,goal['sim_end_ns'],150_000_000))
                target=targets[index]
                xy=float(np.linalg.norm(np.array(gt['position'])[:2]-target[:2]))
                angle=abs(math.remainder(yaw(gt['orientation'])-target[2],2*math.pi))
                row.update(xy_error_m=xy,yaw_error_rad=angle,
                           passed=goal.get('action_status') == 4 and goal.get('target') == target
                           and xy < .15 and angle < math.radians(3))
            except (ValueError,TypeError,KeyError,IndexError) as exc:row['failure']=str(exc)
            post_endpoints.append(row)
        if not post_endpoints or not all(row['passed'] for row in post_endpoints):
            reasons.append('post_navigation_endpoint_failure')
    report['post_navigation_endpoints']=post_endpoints
    expected_stages = ['goal']
    if result.get('route'):
        expected_stages = ['approach','entrance','passage','departure',
                           'relocalization','navigation_refresh','goal']
        for label in ('relocalization','navigation_refresh'):
            path = mission/(label+'.json')
            data = json.loads(path.read_text()) if path.exists() else {}
            if data.get('completed') is not True or data.get('failure'):
                reasons.append(label+'_not_proven')
            if label == 'relocalization' and data.get('parameter_restored') is not True:
                reasons.append('gicp_parameter_restore_not_proven')
            if label == 'navigation_refresh' and data.get('commands') != [3,0]:
                reasons.append('navigation_reset_startup_not_proven')
    stages = {row['stage']: row for row in result.get('stages', [])}
    for label in expected_stages:
        row = stages.get(label, {})
        success = row.get('exit_code') == 0 if label in ('approach','passage','goal') else row.get('completed') is True
        if not success:
            reasons.append(label+'_stage_not_completed')
    ends=[]
    for label in ['approach','entrance','passage','departure','goal']:
        path=mission/(label+'.json')
        if not path.exists():continue
        data=json.loads(path.read_text())
        goals=data.get('goals',[]) or ([dict(target=a['target'],sim_end_ns=round(a['stamp']*1e9)) for a in data.get('passes',[])] if label=='passage' else [dict(target=data.get('target'),sim_end_ns=data.get('sim_end_ns'))])
        for goal in goals:
            row=dict(stage=label,target=goal.get('target'),passed=False)
            try:
                gt=chassis_to_base(interpolate_pose(truth,stamps,goal['sim_end_ns'],150_000_000))
                target=goal['target'];xy=float(np.linalg.norm(np.array(gt['position'])[:2]-target[:2]))
                angle=abs(math.remainder(yaw(gt['orientation'])-target[2],2*math.pi))
                row.update(xy_error_m=xy,yaw_error_rad=angle,passed=xy<(.05 if label in ['entrance','passage','departure'] else .15) and angle<(.035 if label=='passage' else math.radians(3)))
            except (ValueError,TypeError,KeyError) as exc:row['failure']=str(exc)
            ends.append(row)
    if not ends or not all(e['passed'] for e in ends):reasons.append('endpoint_failure')
    roofs=[];sides=[];rejected=0;scored=0
    if result.get('route'):
        passage_path=mission/'passage.json'
        passage=json.loads(passage_path.read_text()) if passage_path.exists() else {}
        if not passage_path.exists():reasons.append('missing_passage_evidence')
        if expected_passes is not None:
            center_x={'west':-2.5,'east':2.5}[result['route']['tunnel']]
            reasons.extend(passage_sequence_reasons(passage.get('passes',[]),expected_passes,center_x))
        if len(ends)!=4+len(passage.get('passes',[])):reasons.append('missing_mission_stage_endpoints')
        joints=[e for e in events if e.get('kind')=='joint']
        joint_report=check_stream(events,'joint',min(stamps),max(stamps),150_000_000)
        if not joint_report['passed']:reasons.extend('joint:'+r for r in joint_report['reasons'])
        jt=[e['stamp_ns'] for e in joints]
        cfg=json.loads((RESOURCES/'config/model.json').read_text())
        support={k:np.array(v) for k,v in json.loads((RESOURCES/'config/cad_support_vertices.json').read_text()).items()}
        cx,cy={'west':(-2.5,4.46),'east':(2.5,-2.46)}[result['route']['tunnel']]
        wheels={wheel['joint'] for wheel in cfg['wheels']}
        required={joint['name'] for joint in cfg['joints']
                  if joint['type'] != 'fixed' and joint['name'] not in wheels}
        for row in truth:
            index=bisect_left(jt,row['stamp_ns'])
            candidates=[i for i in (index-1,index) if 0<=i<len(jt)]
            if not candidates:rejected+=1;continue
            index=min(candidates,key=lambda i:abs(jt[i]-row['stamp_ns']))
            if abs(jt[index]-row['stamp_ns'])>75_000_000:rejected+=1;continue
            try:
                rot=Rotation.from_quat(row['pose']['orientation'])
                position=np.array(row['pose']['position'])+rot.apply([0,0,-.1])
                measured=joints[index]['joints']
                if not required.issubset(measured) or not all(math.isfinite(measured[key]) for key in required):
                    raise ValueError('missing_or_invalid_measured_joint')
                pts=envelope(cfg,support,measured,rot.as_matrix())
                if not np.isfinite(pts).all():raise ValueError('nonfinite_envelope')
            except (KeyError,ValueError):rejected+=1;continue
            scored+=1
            if position[0]+pts[:,0].max()>cx-1 and position[0]+pts[:,0].min()<cx+1:
                sides.append(clearance(pts,position[1],cy))
            if position[0]+pts[:,0].max()>cx-.175 and position[0]+pts[:,0].min()<cx+.175:
                roofs.append(.33-float(position[2]+pts[:,2].max()))
        if rejected:reasons.append('incomplete_cad_sample_coverage')
        if not roofs or not sides or min(roofs)<=0 or min(sides)<=0:reasons.append('cad_clearance_not_proven')
    report.update(passed=not reasons,endpoints=ends,cad_scored_samples=scored,cad_rejected_samples=rejected,
                  minimum_roof_margin_m=min(roofs) if roofs else None,minimum_side_margin_m=min(sides) if sides else None)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory',type=Path);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--expected-passes',type=int,choices=[1,3,5,7,9])
    args=parser.parse_args();report=assess(args.run_directory,args.expected_passes)
    report['expected_passes']=args.expected_passes
    with args.out.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps(report,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
