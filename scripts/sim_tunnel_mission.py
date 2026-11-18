#!/usr/bin/env python3
"""Explicit simulation mission: Nav2 approach, surveyed portal transfer, tunnel, Nav2 exit.

No world truth inputs. Ordinary Nav2 and its collision parameters are unchanged.
This entry is opt-in and requires the audited prior map and tunnel wheel profile.
"""
import argparse
import hashlib
import yaml
from types import SimpleNamespace
import json
import math
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformException
from geometry_msgs.msg import Twist
from sim_tunnel_passage import (Passage, ROOT, RESOURCES, envelope, passage_margin,
                                fit_wall, lateral_kalman, limit_velocity,
                                interpolated_command,observed_floor_base_z,confirm_disabled)


def route(start, goal, requested='auto'):
    """Only route surveyed cross-corridor goals; reject goals inside the narrow structure."""
    if len(start) != 3 or len(goal) != 3 or not np.isfinite([*start, *goal]).all():
        raise ValueError('finite_start_and_goal_required')
    candidates=[]
    for name,cx,cy in [('west',-2.5,4.46),('east',2.5,-2.46)]:
        lane=cy-.07928764148265385
        if abs(goal[0]-cx)<1.5 and abs(goal[1]-lane)<.85:
            raise ValueError('goal_inside_tunnel_requires_external_exit_goal')
        opposite=(start[0]<cx-1.5 and goal[0]>cx+1.5) or (goal[0]<cx-1.5 and start[0]>cx+1.5)
        if not opposite:continue
        y=start[1]+(goal[1]-start[1])*(cx-start[0])/(goal[0]-start[0])
        if requested==name or (requested=='auto' and abs(y-lane)<1.):
            direction=1 if start[0]<cx else -1
            safe_y=3.3 if name=='west' else -1.3
            candidates.append(dict(tunnel=name,approach=[cx-direction*1.7,safe_y,0.],
                                   entrance=[cx-direction*1.7,lane,0.],
                                   exit=[cx+direction*1.7,lane,0.],
                                   departure=[cx+direction*1.7,safe_y,0.]))
    if len(candidates)>1:raise ValueError('ambiguous_tunnel_route')
    if requested!='auto' and not candidates:raise ValueError('requested_tunnel_not_between_start_and_goal')
    return candidates[0] if candidates else None


def fit_portal_wall(points, seed_y, outer_wall_y, seed_x):
    """Fit the surveyed outer boundary while the robot remains outside the beam.

    End-face/roof aliases can fit a 2D line, but cannot explain a portal
    heading within the existing 0.15 rad alignment limit. The field boundary
    remains visible when the short tunnel side wall is occluded by its end.
    """
    candidates=[]
    for wall_y in (outer_wall_y,):
        expected=wall_y-seed_y
        span=2.5 if outer_wall_y>0 else 1.8
        visible=(abs(points[:,1]-expected)<.20)&(points[:,2]>.025)&(points[:,2]<.40)&(abs(points[:,0])<span)&((points[:,0]>.46)|(points[:,0]<-.36))
        if wall_y==5.:
            # The rendered Y=4.86 tunnel panel and corner box are parallel to
            # Y=5, but are not that landmark. Use only the audited full-height
            # boundary segments, with a .30m guard around their X endpoints.
            x=points[:,0]+seed_x
            full_height=((x>-5.)&(x<-3.8))|(x>-1.2)
            # Above the .33m tunnel/panel, the real Y=5 boundary remains
            # visible across the tunnel span. Base floor is .08m; >.26m
            # relative height excludes that panel and the low roof returns.
            high_boundary=(x>-5.)&(points[:,2]>.26)
            visible &= full_height|high_boundary
        try:
            slope,intercept,count=fit_wall(points[visible,:2])
        except ValueError:
            continue
        if abs(math.atan(slope))>.15:
            continue
        measured=wall_y-intercept*math.cos(math.atan(slope))
        candidates.append((count,measured,slope,intercept,wall_y))
    if not candidates:
        raise ValueError('portal_parallel_wall_not_observable')
    count,measured,slope,intercept,wall_y=max(candidates,key=lambda v:v[0])
    return measured,slope,intercept,count,wall_y


class Portal(Passage):
    """Same observed outer wall over the wider, fully external transfer strip."""
    def pose(self):
        try:
            self.portal_pose_source='surveyed_wall'
            return super().pose()
        except ValueError as exc:
            initial_wall_missing=(str(exc)=='side_wall_not_fresh' and self.wall_fit is None)
            initial_x_missing=(str(exc)=='end_wall_not_observable' and self.x_anchor is None)
            if not (initial_wall_missing or initial_x_missing):
                raise
            # Initial external transfer only: select the existing LIO pose path.
            # The caller enforces full-body external clearance and final wall
            # uncertainty, so this cannot complete alignment or enter the beam.
            # spin_once is single threaded; no callback runs during this call.
            wall=self.wall_y
            self.wall_y=None
            try:
                pose=super().pose()
            finally:
                self.wall_y=wall
            lane=(4.46 if self.wall_side<0 else -2.46)-.07928764148265385
            if abs(pose[0][1]-lane)<.03:raise ValueError('side_wall_not_fresh')
            self.x_info=dict(method='external_lio_standoff',uncertainty_m=.05,lateral_uncertainty_m=.12)
            self.portal_pose_source='initial_external_lio'
            return pose

    def publish(self,command=None,active=False):
        super().publish(command,active)
        if self.zero_requested:
            if getattr(self,'portal_zero_since',None) is None:
                self.portal_zero_since=self.get_clock().now().nanoseconds*1e-9
        else:
            self.portal_zero_since=None

    def on_imu(self,msg):
        # The X velocity from the last visible end face can be stale during a
        # stationary portal hold. Relevel only after two simulated seconds of
        # explicit zero, plus measured lateral/gyro rest and stable gravity.
        # This never changes the odometry or the longitudinal bridge budget.
        if msg.header.frame_id=='body':
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            w=np.array([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
            a=np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
            since=getattr(self,'portal_zero_since',None)
            resting=(since is not None and stamp-since>2. and abs(self.wall_velocity)<.002 and
                     np.linalg.norm(w)<.003 and np.isfinite(a).all() and abs(np.linalg.norm(a)-9.81)<.15)
            from collections import deque
            if not hasattr(self,'portal_gravity'):self.portal_gravity=deque()
            if resting:
                self.portal_gravity.append((stamp,a))
                while self.portal_gravity and stamp-self.portal_gravity[0][0]>.6:self.portal_gravity.popleft()
                if (self.local_attitude is not None and stamp-self.portal_gravity[0][0]>=.5 and
                    np.max(np.std([v for _,v in self.portal_gravity],axis=0))<.02):
                    g=np.mean([v for _,v in self.portal_gravity],axis=0)
                    heading=self.local_attitude.as_euler('xyz')[2]
                    self.local_attitude=Rotation.from_euler('xyz',[math.atan2(g[1],g[2]),math.atan2(-g[0],math.hypot(g[1],g[2])),heading])
            else:self.portal_gravity.clear()
        super().on_imu(msg)

    def on_cloud(self,msg):
        if msg.header.frame_id!='navigation_laser' or self.odom is None:return
        data=point_cloud2.read_points(msg,field_names=['x','y','z'],skip_nans=False)
        points=np.column_stack([data[k] for k in ['x','y','z']]).astype(float)
        points += [.0557597190051302,-.163594690492484,.198643863403111]
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.raw_scan=(stamp,points)
        points=points[np.isfinite(points).all(1)]
        if self.local_attitude is not None:
            roll,pitch,_=self.local_attitude.as_euler('xyz')
            points=Rotation.from_euler('xyz',[roll,pitch,0.]).apply(points)
        # Association seeded by fresh map-frame LIO, not command or world truth.
        try:
            t=self.tf.lookup_transform('map','camera_init',rclpy.time.Time.from_msg(self.odom.header.stamp))
            q=t.transform.rotation
            mapped=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply([
                self.odom.pose.pose.position.x,self.odom.pose.pose.position.y,self.odom.pose.pose.position.z])
            seed=mapped[1]+t.transform.translation.y
            seed_x=mapped[0]+t.transform.translation.x
            measured,slope,intercept,count,wall_y=fit_portal_wall(points,seed,self.outer_wall_y,seed_x)
            self.update_wall_filter(stamp,measured,wall_y)
            self.wall_fit=(stamp,slope,intercept)
            self.active_wall_y=wall_y
        except (ValueError,TransformException):
            # pose() enforces freshness and the caller disables on stale observations.
            return


def transfer(tunnel,target,out,stopping):
    cx,cy={'west':(-2.5,4.46),'east':(2.5,-2.46)}[tunnel]
    side=-1 if tunnel=='west' else 1
    node=Portal(cy+side*.4,side)
    cfg=json.loads((RESOURCES/'config/model.json').read_text())
    profile=json.loads((RESOURCES/'config/tunnel_profile.json').read_text())
    support={k:np.asarray(v) for k,v in json.loads((RESOURCES/'config/cad_support_vertices.json').read_text()).items()}
    result=dict(completed=False,truth_used_for_control=False,target=target,trace=[],failure=None)
    previous=np.zeros(3);ramp=np.zeros(3);last=-math.inf;settled=None;begin=time.monotonic();ready=False;ever_ready=False;missing_since=None
    try:
        # This is a wall-time execution cap, not a motion/acceptance tolerance.
        # Slow headless simulation must still finish the unchanged measured
        # 10mm alignment and .75 simulated-second rest window. Match passage's
        # bounded 600s cap; stale sensors and cancellation still stop immediately.
        while not stopping() and time.monotonic()-begin<600:
            rclpy.spin_once(node,timeout_sec=.01)
            if node.contacts:raise ValueError('body_contact_stop')
            stamp=node.get_clock().now().nanoseconds*1e-9
            if stamp-last<.05:
                node.publish(interpolated_command(previous,ramp,(stamp-last)/.05),active=ready)
                continue
            try:
                # Bootstrap must stay at zero until measured gravity is ready.
                # Otherwise LIO-only startup can move before the wall branch
                # requires tilt, preventing its stationary initialization.
                if node.local_attitude is None:
                    raise ValueError('stationary_gravity_initialization_or_gyro_not_ready')
                p,rot,stamp=node.pose()
            except (ValueError,TransformException):
                node.publish();ready=False;previous=np.zeros(3);ramp=np.zeros(3)
                missing_since=missing_since or time.monotonic()
                if time.monotonic()-missing_since>(15. if ever_ready else 45.):raise
                continue
            ever_ready=True;missing_since=None
            if abs(p[0]-target[0])>.18:raise ValueError('portal_x_outside_external_transfer_strip')
            pts=envelope(cfg,support,node.joints,rot.as_matrix())
            margin,inside=passage_margin(pts,p,cx,cy,node.x_info.get('uncertainty_m',.005))
            margin-=node.x_info.get('lateral_uncertainty_m',0.)
            if inside or margin<.003:raise ValueError('portal_transfer_not_fully_outside_structure')
            for key,value in [('pitch1_drive_joint',profile['fold_rad']),('yaw1_joint',0.),('yaw2_joint',0.),('pitch2_joint',profile['aim_rad'])]:
                if abs(node.joints.get(key,math.inf)-value)>profile['joint_tolerance_rad']:raise ValueError('portal_not_folded')
            yaw=rot.as_euler('xyz')[2];err=math.remainder(-yaw,2*math.pi)
            if abs(err)>.15:raise ValueError('portal_heading_not_aligned')
            dy=target[1]-p[1]
            vy=np.clip(2.5*dy-1.45*node.wall_velocity,-.08,.08)
            vx=np.clip(.8*(target[0]-p[0])-.6*node.x_velocity,-.03,.03)
            desired=np.array([math.cos(yaw)*vx+math.sin(yaw)*vy,-math.sin(yaw)*vx+math.cos(yaw)*vy,np.clip(4.4*err-.8*node.imu_history[-1][1][2],-.15,.15)])
            current=np.array([*previous]) if not ready else np.array([*ramp])
            command=limit_velocity(current,desired,.05)
            if settled is None and abs(target[0]-p[0])<.025 and abs(dy)<.010 and abs(err)<.02:
                settled=stamp;settled_position=p.copy()
            if settled is not None:
                command=np.zeros(3)
                if stamp-settled>.75:
                    if (abs(target[0]-p[0])<.03 and abs(dy)<.018 and abs(err)<.03 and
                        abs(node.wall_velocity)<.01 and abs(node.imu_history[-1][1][2])<.02 and
                        np.linalg.norm(p[:2]-settled_position[:2])<.01 and
                        node.x_info.get('uncertainty_m',math.inf)<=.03):
                        floor_z=observed_floor_base_z(rot.apply(node.raw_scan[1][np.isfinite(node.raw_scan[1]).all(1)]))
                        m=node.odom;lp=m.pose.pose.position;lq=m.pose.pose.orientation
                        result['observed_pose']={'position':[float(p[0]),float(p[1]),floor_z],'orientation':rot.as_quat().tolist()}
                        result['lio_pose']={'position':[lp.x,lp.y,lp.z],'orientation':[lq.x,lq.y,lq.z,lq.w]}
                        result['completed']=True;result['sim_end_ns']=int(stamp*1e9)
                        result['endpoint_longitudinal']=node.x_info.copy();break
                    settled=None
            previous=current;ramp=command;last=stamp;ready=True
            node.publish(interpolated_command(previous,ramp,0),active=True)
            result['trace'].append(dict(stamp=stamp,xy=p[:2].tolist(),yaw=yaw,position=p.tolist(),rpy=rot.as_euler('xyz').tolist(),longitudinal=node.x_info.copy(),side_margin_m=margin,command=command.tolist(),joints=node.joints.copy()))
        if not result['completed']:raise ValueError('portal_transfer_timeout_or_interrupted')
    except Exception as exc:result['failure']=str(exc)
    finally:
        result['sim_stop_request_ns']=node.get_clock().now().nanoseconds
        result['disabled_confirmed']=confirm_disabled(node)
        result['sim_cleanup_ns']=node.get_clock().now().nanoseconds
        if not result['disabled_confirmed']:
            result['completed']=False
            result['failure']=result['failure'] or 'disabled_confirmation_timeout'
        if node.raw_scan is not None:np.savez_compressed(out.with_suffix('.scan.npz'),stamp=node.raw_scan[0],points=node.raw_scan[1])
        node.destroy_node();out.write_text(json.dumps(result,indent=2))
    return result['completed']


def refresh_navigation_after_relocalization(out,stopping):
    """Recreate the simulation navigation maps in the newly confirmed gauge.

    ROGMap intentionally fixes its prior projection transform at construction.
    Clearing a Nav2 costmap does not recreate that projection or its field.
    Use the existing simulation lifecycle manager only after stopped transfer
    and successful GICP; ordinary navigate_to_pose never enters this path.
    """
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from nav2_msgs.srv import ManageLifecycleNodes
    from std_msgs.msg import String
    node=Node('sim_tunnel_navigation_refresh',parameter_overrides=[Parameter('use_sim_time',value=True)])
    mode=node.create_publisher(String,'/sim/control_mode',10)
    states=[]
    node.create_subscription(String,'/sim/control_status',lambda m:states.append((time.monotonic(),m.data)),10)
    client=node.create_client(ManageLifecycleNodes,'/lifecycle_manager_sim_navigation/manage_nodes')
    report=dict(completed=False,commands=[],failure=None)
    def wait(predicate,budget):
        deadline=time.monotonic()+budget
        while not stopping() and time.monotonic()<deadline:
            mode.publish(String(data='disabled'));rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return True
        return False
    try:
        if not wait(lambda:states and states[-1][1]=='disabled' and time.monotonic()-states[-1][0]<.5,10):
            raise ValueError('navigation_refresh_not_disabled')
        if not wait(client.service_is_ready,10):raise ValueError('navigation_refresh_service_unavailable')
        for command in [ManageLifecycleNodes.Request.RESET,ManageLifecycleNodes.Request.STARTUP]:
            request=ManageLifecycleNodes.Request();request.command=command
            future=client.call_async(request)
            if not wait(future.done,60):raise ValueError('navigation_refresh_timeout')
            if not future.result().success:raise ValueError('navigation_refresh_rejected')
            report['commands'].append(int(command))
        report['completed']=True
    except Exception as exc:report['failure']=str(exc)
    finally:
        mode.publish(String(data='disabled'));node.destroy_node()
        out.write_text(json.dumps(report,indent=2))
    return report['completed']


def relocalization_seed(observed,local):
    rotation=Rotation.from_quat(observed['orientation'])*Rotation.from_quat(local['orientation']).inv()
    translation=np.asarray(observed['position'])-rotation.apply(local['position'])
    return [*map(float,translation),*map(float,rotation.as_euler('xyz'))]


def relocalize(departure,out,stopping):
    """Seed, then require real GICP convergence and verify the global handover."""
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import GetParameters,SetParameters
    from std_srvs.srv import Trigger
    from std_msgs.msg import String,Bool
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from tf2_ros import Buffer,TransformListener
    node=Node('sim_tunnel_relocalization',parameter_overrides=[Parameter('use_sim_time',value=True)])
    mode=node.create_publisher(String,'/sim/control_mode',10)
    states=[];valid=[];odoms=[];buffer=Buffer();listener=TransformListener(buffer,node)
    node.create_subscription(String,'/gicp_relocalization_node/localization_state',lambda m:states.append((time.monotonic(),m.data)),10)
    node.create_subscription(Bool,'/sim/localization_valid',lambda m:valid.append((time.monotonic(),m.data)),10)
    node.create_subscription(Odometry,'/sim/base_odom',lambda m:odoms.append(m),qos_profile_sensor_data)
    get=node.create_client(GetParameters,'/gicp_relocalization_node/get_parameters')
    setp=node.create_client(SetParameters,'/gicp_relocalization_node/set_parameters')
    recall=node.create_client(Trigger,'/gicp_recall');original=None
    report=dict(completed=False,truth_used_for_control=False,failure=None)
    def wait(predicate,budget):
        deadline=time.monotonic()+budget
        while not stopping() and time.monotonic()<deadline:
            mode.publish(String(data='disabled'));rclpy.spin_once(node,timeout_sec=.02)
            if predicate():return True
        return False
    def call(client,request,attempts=1):
        if not wait(client.service_is_ready,10):raise ValueError('relocalization_service_unavailable')
        for attempt in range(attempts):
            future=client.call_async(request)
            if wait(future.done,10):return future.result()
            client.remove_pending_request(future);future.cancel()
            report.setdefault('service_timeouts',[]).append(
                dict(service=client.srv_name,attempt=attempt+1))
            if stopping() or attempt+1==attempts:
                raise ValueError('relocalization_service_timeout')
    try:
        req=GetParameters.Request();req.names=['initial_pose','recall_use_parameter_seed'];old=call(get,req,attempts=2)
        if len(old.values)!=2 or not old.values[1].bool_value:raise ValueError('seeded_recall_not_enabled')
        original=old.values[0];seed=relocalization_seed(departure['observed_pose'],departure['lio_pose']);report['seed']=seed
        req=SetParameters.Request();req.parameters=[Parameter('initial_pose',value=seed).to_parameter_msg()]
        if not all(r.successful for r in call(setp,req).results):raise ValueError('relocalization_seed_rejected')
        since=time.monotonic()
        if not call(recall,Trigger.Request()).success:raise ValueError('relocalization_recall_rejected')
        def converged():
            history=[s for t,s in states if t>=since]
            return (any(s!='localized' for s in history) and history and history[-1]=='localized' and
                    valid and valid[-1][1] and time.monotonic()-valid[-1][0]<.5)
        if not wait(converged,90):raise ValueError('relocalization_convergence_timeout')
        def verified():
            if not odoms:return False
            m=odoms[-1]
            try:
                tf=buffer.lookup_transform('map','camera_init',rclpy.time.Time.from_msg(m.header.stamp))
                q=tf.transform.rotation;r=Rotation.from_quat([q.x,q.y,q.z,q.w]);p=m.pose.pose.position
                position=r.apply([p.x,p.y,p.z])+[tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z]
                q=m.pose.pose.orientation;body=r*Rotation.from_quat([q.x,q.y,q.z,q.w])
                error=float(np.linalg.norm(position[:2]-departure['observed_pose']['position'][:2]));angle=abs((body.inv()*Rotation.from_quat(departure['observed_pose']['orientation'])).magnitude())
                report.update(global_position=position.tolist(),position_disagreement_m=error,angle_disagreement_rad=angle)
                return error<.05 and angle<.05
            except TransformException:return False
        if not wait(verified,10):raise ValueError('relocalization_handover_disagreement')
        report['completed']=True
    except Exception as exc:report['failure']=str(exc)
    finally:
        if original is not None:
            from rcl_interfaces.msg import Parameter as ParameterMsg
            req=SetParameters.Request();req.parameters=[ParameterMsg(name='initial_pose',value=original)]
            # Restore even after cancellation while publishing disabled.
            future=setp.call_async(req);deadline=time.monotonic()+3
            while not future.done() and time.monotonic()<deadline:
                mode.publish(String(data='disabled'));rclpy.spin_once(node,timeout_sec=.02)
            restored=future.done() and all(r.successful for r in future.result().results)
            report['parameter_restored']=bool(restored)
            if not restored:report['completed']=False;report['failure']='relocalization_parameter_restore_failed'
        node.destroy_node();out.write_text(json.dumps(report,indent=2))
    return report['completed']


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal',nargs=3,type=float,required=True)
    parser.add_argument('--passes',type=int,default=1,help='odd 1..9 tunnel traversals for repeatability verification')
    parser.add_argument('--tunnel',choices=['auto','west','east'],default='auto')
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--cancel-after',type=float,help='wall seconds after mission start, for safety regression')
    args=parser.parse_args()
    if args.passes not in [1,3,5,7,9]:parser.error('passes must be odd 1..9 to finish on the goal side')
    if args.out.exists():parser.error('output directory already exists')
    if not np.isfinite(args.goal).all() or (args.cancel_after is not None and (not math.isfinite(args.cancel_after) or args.cancel_after<=0)):parser.error('finite goal and positive cancel-after required')
    args.out.mkdir(parents=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stopped=False;begin=time.monotonic();child=None
    def stop(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    def stopping():return stopped or (args.cancel_after is not None and time.monotonic()-begin>=args.cancel_after)
    result=dict(completed=False,truth_used_for_control=False,goal=args.goal,stages=[],failure=None,cancelled=False)
    def run(label,cmd):
        nonlocal child
        with (args.out/(label+'.log')).open('w') as stream:
            child=subprocess.Popen([sys.executable,'scripts/sim_supervisor.py','--',sys.executable,*cmd],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
            while child.poll() is None:
                if stopping():child.send_signal(signal.SIGINT);break
                time.sleep(.05)
            try:code=child.wait(timeout=20)
            except subprocess.TimeoutExpired:child.terminate();code=child.wait(timeout=5)
        result['stages'].append({'stage':label,'exit_code':code})
        if stopping():raise ValueError('mission_interrupted')
        if code:raise ValueError(label+'_failed')
        if label=='passage':
            stage=json.loads((args.out/'passage.json').read_text())
            if stage.get('completed') is not True or stage.get('disabled_confirmed') is not True:
                raise ValueError('passage_stop_not_confirmed')
    def nav(label,goal):run(label,['scripts/sim_nav_trial.py','--goal',*map(str,goal),'--goal-timeout','300','--out',str(args.out/(label+'.json'))])
    try:
        # Bootstrap with the standard trial observer, without publishing a mode.
        from sim_nav_trial import Trial
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer,TransformListener
        observer=Trial();odoms=[];buffer=Buffer();listener=TransformListener(buffer,observer)
        # Reject saved-local/rotated maps before sending any surveyed waypoint.
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile,DurabilityPolicy
        maps=ROOT/'src/simulation_integration/huananhu_sim_bringup/maps'
        meta=yaml.safe_load((maps/'rmul_2027.yaml').read_text())
        with (maps/meta['image']).open('rb') as stream:
            if stream.readline().strip()!=b'P5':raise ValueError('audited_map_format')
            width,height=map(int,stream.readline().split())
            if stream.readline().strip()!=b'255':raise ValueError('audited_map_depth')
            pixels=np.flipud(np.frombuffer(stream.read(),dtype=np.uint8).reshape(height,width))
        cells=np.where(pixels==0,100,np.where(pixels==254,0,-1)).astype(np.int8)
        contract=SimpleNamespace(map_shape=(width,height),map_meta=meta,
                                 map_hash=hashlib.sha256(cells.tobytes()).hexdigest(),map_ok=False)
        def on_map(msg):
            Passage.on_map(contract,msg)
            q=msg.info.origin.orientation
            contract.map_ok=contract.map_ok and abs(msg.info.origin.position.z)<1e-7 and np.allclose([q.x,q.y,q.z,q.w],[0.,0.,0.,1.],atol=1e-7)
        observer.create_subscription(OccupancyGrid,'/map',on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        observer.create_subscription(Odometry,'/sim/base_odom',lambda m:odoms.append((time.monotonic(),m)),qos_profile_sensor_data)
        deadline=time.monotonic()+60;start=None
        while not stopping() and time.monotonic()<deadline:
            rclpy.spin_once(observer,timeout_sec=.02)
            if not contract.map_ok or not observer.healthy() or not odoms or time.monotonic()-odoms[-1][0]>.5:continue
            m=odoms[-1][1]
            if m.header.frame_id!='camera_init' or m.child_frame_id!='base_footprint':continue
            try:
                t=buffer.lookup_transform('map','camera_init',rclpy.time.Time.from_msg(m.header.stamp));q=t.transform.rotation
                rot=Rotation.from_quat([q.x,q.y,q.z,q.w]);p=rot.apply([m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z])+[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
                a=m.pose.pose.orientation;heading=(rot*Rotation.from_quat([a.x,a.y,a.z,a.w])).as_euler('xyz')[2]
                start=[p[0],p[1],heading];break
            except (ValueError,TransformException):continue
        observer.destroy_node()
        if start is None:raise ValueError('mission_audited_map_or_start_pose_not_ready')
        selected=route(start,args.goal,args.tunnel);result['route']=selected;result['start']=start
        if selected:
            nav('approach',selected['approach'])
            for label,target in [('entrance',selected['entrance'])]:
                if not transfer(selected['tunnel'],target,args.out/(label+'.json'),stopping):raise ValueError(label+'_failed')
                result['stages'].append({'stage':label,'completed':True})
            run('passage',['scripts/sim_tunnel_passage.py','--tunnel',selected['tunnel'],'--passes',str(args.passes),'--speed','.12','--out',str(args.out/'passage.json')])
            if not transfer(selected['tunnel'],selected['departure'],args.out/'departure.json',stopping):raise ValueError('departure_failed')
            result['stages'].append({'stage':'departure','completed':True})
            departure=json.loads((args.out/'departure.json').read_text())
            if not relocalize(departure,args.out/'relocalization.json',stopping):raise ValueError('global_relocalization_failed')
            result['stages'].append({'stage':'relocalization','completed':True})
            if not refresh_navigation_after_relocalization(args.out/'navigation_refresh.json',stopping):
                raise ValueError('navigation_refresh_failed')
            result['stages'].append({'stage':'navigation_refresh','completed':True})
        nav('goal',args.goal)
        result['completed']=True
    except Exception as exc:
        result['failure']=str(exc)
        result['cancelled']=bool(stopping())
    finally:
        # Every child/transfer owns final zero + disabled; also assert disabled here.
        # The existing supervisor watches this parent, so an orphaned action or
        # passage is interrupted if this mission process disappears.
        from std_msgs.msg import String
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        n=Node('sim_tunnel_mission_cleanup',parameter_overrides=[Parameter('use_sim_time',value=True)])
        pub=n.create_publisher(String,'/sim/control_mode',10)
        status=[]
        n.create_subscription(String,'/sim/control_status',lambda m:status.append((time.monotonic(),m.data)),10)
        deadline=time.monotonic()+2.
        while time.monotonic()<deadline:
            pub.publish(String(data='disabled'));rclpy.spin_once(n,timeout_sec=.02)
            stamp=n.get_clock().now().nanoseconds
            if stamp>0:result.setdefault('sim_stop_request_ns',stamp)
        result['sim_cleanup_ns']=n.get_clock().now().nanoseconds
        result['disabled_confirmed']=bool(status and status[-1][1]=='disabled' and time.monotonic()-status[-1][0]<.5)
        n.destroy_node();rclpy.shutdown()
        (args.out/'result.json').write_text(json.dumps(result,indent=2))
    return 0 if result['completed'] and result.get('disabled_confirmed') else 1

if __name__=='__main__':raise SystemExit(main())
