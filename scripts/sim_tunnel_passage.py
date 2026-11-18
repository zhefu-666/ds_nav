#!/usr/bin/env python3
"""Experimental tunnel segment; autonomous passage acceptance is still incomplete.

Feedback uses localization, LiDAR and raw gyro, never Gazebo truth.

Ordinary Nav2/MINCO remains responsible for open-space approach/departure. This
segment must start at an aligned staging pose outside the metal roof. The
circumscribed-circle planner is not used to claim a path through an 800mm gap.
"""
import argparse
import json
import hashlib
import yaml
import math
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from ros_gz_interfaces.msg import Contacts
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
from sensor_msgs.msg import JointState, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from collections import deque
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parents[1]
RESOURCES = ROOT / 'src/simulation_integration/huananhu_sim_resources'
sys.path.insert(0, str(RESOURCES / 'scripts'))
from kinematic_math import forward_kinematics  # noqa: E402


def passage_settle_candidate(dx, dy, yaw_error):
    # Reuse the motion-alignment yaw margin before commanding a zero window.
    # Capturing near the final .03rad limit repeatedly interrupts correction.
    return abs(dx)<.020 and abs(dy)<.010 and abs(yaw_error)<.01


def envelope(config, support, joints, rotation):
    # Wheel phase is not an articulation of the body envelope. Passive roller
    # phase is bounded separately by a 1mm support-height allowance, not shrunk.
    q = dict(joints)
    for wheel in config['wheels']:
        q[wheel['joint']] = 0.
    fk = forward_kinematics(config, q)
    points = np.concatenate([v @ fk[k][:3, :3].T + fk[k][:3, 3]
                             for k, v in support.items()])
    return (points + [0., 0., .062]) @ rotation.T


def clearance(points, y, center_y, opening_width=.8):
    return min(y + points[:, 1].min() - (center_y - opening_width / 2),
               center_y + opening_width / 2 - y - points[:, 1].max())



def passage_margin(points, position, center_x, center_y, x_uncertainty=0.):
    # The 2m end beams extend well beyond the 350mm metal roof. Keep the
    # complete body inside the opening until it has cleared those beams.
    in_corridor=(position[0]+points[:,0].max()+x_uncertainty>center_x-1. and
                 position[0]+points[:,0].min()-x_uncertainty<center_x+1.)
    if in_corridor:
        return clearance(points,position[1],center_y),True
    return min(position[1]+points[:,1].min()+3.,5.-position[1]-points[:,1].max()),False


def overlaps_metal_roof(points, position_x, center_x, x_uncertainty=0.):
    # Audited metal roof is 350mm long; side beams remain guarded over 2m.
    return bool(position_x+points[:,0].max()+x_uncertainty>=center_x-.175 and
                position_x+points[:,0].min()-x_uncertainty<=center_x+.175)


def interpolated_command(start, target, fraction):
    velocity=np.asarray(start)+(np.asarray(target)-np.asarray(start))*min(1.,max(0.,fraction))
    command=Twist()
    command.linear.x,command.linear.y,command.angular.z=map(float,velocity)
    return command


def fit_wall(points):
    if len(points) < 10:
        raise ValueError('insufficient_wall_points')
    best = None
    rng = np.random.default_rng(0)
    for _ in range(48):
        a, b = points[rng.choice(len(points), 2, replace=False)]
        if abs(b[0]-a[0]) < .12:
            continue
        slope = (b[1]-a[1])/(b[0]-a[0])
        if abs(slope) > .25:
            continue
        mask = np.abs(points[:,1] - slope*points[:,0] - (a[1]-slope*a[0])) < .006
        if mask.sum() >= 10 and np.ptp(points[mask,0]) > .15:
            if best is None or mask.sum() > best.sum():
                best = mask
    if best is None:
        raise ValueError('wall_line_not_observable')
    slope, intercept = np.linalg.lstsq(np.column_stack([points[best,0],np.ones(best.sum())]),points[best,1],rcond=None)[0]
    return float(slope), float(intercept), int(best.sum())


def lateral_kalman(position, velocity, covariance, measurement, command, dt):
    """Wall-position update with the measured local command response model."""
    if position is None:
        return measurement,0.,np.diag([.001**2,.03**2])
    if not 0<dt<=.5:
        raise ValueError('side_wall_sample_interval_invalid')
    alpha=.98**(dt/.05)
    beta=.30*(1-alpha)/.02
    F=np.array([[1.,dt*alpha],[0.,alpha]])
    state=F@np.array([position,velocity])+np.array([dt*beta,beta])*command
    residual=measurement-state[0]
    if abs(residual)>.02+.15*dt:
        raise ValueError('side_wall_measurement_jump')
    covariance=F@covariance@F.T+np.diag([1e-8,.04**2])*dt/.05
    gain=covariance[:,0]/(covariance[0,0]+.0005**2)
    state+=gain*residual
    covariance=(np.eye(2)-np.outer(gain,[1.,0.]))@covariance
    return float(state[0]),float(state[1]),covariance


def limit_velocity(previous, target, dt):
    # Limits use SIMULATION elapsed time; the ordinary command gate and its
    # wall-clock watchdog/ramp are unchanged. Safety zeros bypass this filter.
    return previous+np.clip(target-previous,-np.array([.15,.60,1.5])*dt,np.array([.15,.60,1.5])*dt)


def constrained_control(points, y, center_y, velocity, omega, previous, nominal, dt=.05):
    """Coupled 0.3s body sweep; no geometry shrinking or world-truth input.

    Enumerate bounded lateral/yaw controls including the nominal proposal.
    The first prediction step retains the old input to cover interpolation and
    actuation delay. A separate coast check covers measured momentum during
    that interval. An infeasible sweep requests an immediate disabled brake.
    """
    lower=np.maximum([-.08,-.25],previous-np.array([.60,1.5])*dt)
    upper=np.minimum([.08,.25],previous+np.array([.60,1.5])*dt)
    nominal=np.clip(nominal,lower,upper)
    grid=np.meshgrid(np.linspace(lower[0],upper[0],9),np.linspace(lower[1],upper[1],9))
    candidates=np.vstack([nominal,np.column_stack([grid[0].ravel(),grid[1].ravel()])])
    polygon=points[ConvexHull(points[:,:2]).vertices,:2]
    lo=min(center_y-.4+.008,y+polygon[:,1].min())
    hi=max(center_y+.4-.008,y+polygon[:,1].max())
    py=np.full(len(candidates),y);vy=np.full(len(candidates),velocity)
    theta=np.zeros(len(candidates));w=np.full(len(candidates),omega)
    feasible=np.ones(len(candidates),dtype=bool)
    # Cover measured momentum throughout the same 0.3 s prediction horizon.
    # A one-step coast check allowed a late brake when roller/PI response took
    # several steps to shed lateral/yaw momentum. Do not credit unconfirmed
    # deceleration with clearance that has not actually been observed yet.
    for step in range(1,7):
        elapsed=step*dt
        coast_y=y+elapsed*velocity+math.cos(elapsed*omega)*polygon[:,1]+math.sin(elapsed*omega)*polygon[:,0]
        if coast_y.min()<lo-1e-7 or coast_y.max()>hi+1e-7:
            return None
    for step in range(6):
        control=previous[None,:] if step==0 else candidates
        vy=.98*vy+.30*control[:,0]
        w=.76*w+.49*control[:,1]
        py+=dt*vy;theta+=dt*w
        swept_y=py[:,None]+np.cos(theta[:,None])*polygon[:,1]+np.sin(theta[:,None])*polygon[:,0]
        feasible &= (swept_y.min(1)>=lo-1e-7)&(swept_y.max(1)<=hi+1e-7)
    if not feasible.any():
        return None
    cost=np.sum(((candidates-nominal)/[.08,.25])**2,axis=1)
    cost[~feasible]=np.inf
    return candidates[int(np.argmin(cost))]


def fit_end_wall(points, seed_x, y, wall_x, verify_normal=False, association_window=.25):
    """Absolute X from an audited transverse wall; points are in map axes.

    Reject parallel walls, floor-only clusters, sparse/ambiguous returns, and
    corrections outside the bounded local association window.
    """
    selected=points[np.isfinite(points).all(1) & (np.abs(points[:,0]+seed_x-wall_x)<association_window) &
                    (points[:,1]+y>-2.95) & (points[:,1]+y<4.95) &
                    (points[:,2]>.005) & (points[:,2]<.419) &
                    ((points[:,0]>.50)|(points[:,0]<-.40))]
    if len(selected)<4:
        raise ValueError('end_wall_not_observable')
    order=np.argsort(selected[:,0]);a=selected[order]
    right=np.searchsorted(a[:,0],a[:,0]+.006,side='right')
    if not verify_normal:
        left=int(np.argmax(right-np.arange(len(a))));inliers=a[left:right[left]]
        # A high transverse line still observes X when the 16-line scanner sees
        # only one elevation row. Low/floor-only rings remain unobservable.
        vertical_support=len(inliers)>=4 and np.ptp(inliers[:,1])>=.03 and np.ptp(inliers[:,2])>=.08
        high_line=len(inliers)>=6 and np.ptp(inliers[:,1])>=.15 and inliers[:,2].min()>.22
        if not (vertical_support or high_line):
            raise ValueError('end_wall_not_observable')
        alternatives=(right-np.arange(len(a))>=max(4,.8*len(inliers))) & (np.abs(a[:,0]-np.median(inliers[:,0]))>.02)
        if alternatives.any():
            raise ValueError('end_wall_ambiguous_clusters')
    else:
        sizes=right-np.arange(len(a))
        candidates=[]
        examined=[]
        for left in np.argsort(sizes)[::-1]:
            inliers=a[left:right[left]]
            vertical_support=len(inliers)>=4 and np.ptp(inliers[:,1])>=.03 and np.ptp(inliers[:,2])>=.08
            high_line=len(inliers)>=6 and np.ptp(inliers[:,1])>=.15 and inliers[:,2].min()>.22
            if not (vertical_support or high_line):continue
            center=float(np.median(inliers[:,0]))
            if any(abs(center-other)<=.02 for other in examined):continue
            examined.append(center)
            # Fit each distinct cluster before comparing their support. A
            # different surface beyond the existing 20mm ambiguity separation
            # cannot supply or destroy this candidate's observable normal.
            neighbors=selected[np.abs(selected[:,0]-center)<.02]
            values,vectors=np.linalg.eigh(np.cov(neighbors.T))
            if values[1]<2e-7 or values[1]<5*max(values[0],1e-12) or abs(vectors[0,0])<.9:
                continue
            candidates.append((center,inliers))
        if not candidates:
            raise ValueError('transverse_face_normal_unobservable')
        _,inliers=candidates[0]
        if any(len(other)>=max(4,.8*len(inliers)) for _,other in candidates[1:]):
            raise ValueError('end_wall_ambiguous_clusters')
    x=wall_x-float(np.median(inliers[:,0]))
    if abs(x-seed_x)>association_window:
        raise ValueError('end_wall_association_out_of_bounds')
    return x,len(inliers),float(np.std(inliers[:,0]))


def confirm_absolute_fix(pending, plane, stamp, x):
    """Five distinct stable scans; duplicate callbacks cannot confirm a jump."""
    if pending is None or pending[0]!=plane:
        pending=(plane,deque(maxlen=5))
    samples=pending[1]
    if not samples or stamp>samples[-1][0]:samples.append((stamp,x))
    confirmed=(len(samples)==5 and samples[-1][0]-samples[0][0]>=.099 and
               np.ptp([v for _,v in samples])<.015)
    return pending,confirmed


def observed_floor_base_z(points):
    """Height used ONLY for landmark association, from the observed local floor.

    The surveyed portal floor panels are at .080.. .083m. LIO Z can drift;
    do not clip or overwrite that state or use it to reject a visible roof face.
    Require an extended horizontal patch and return its measured height offset.
    """
    p=points[np.isfinite(points).all(1)&(np.abs(points[:,2])<.03)&
             (np.abs(points[:,0])>.5)&(np.abs(points[:,0])<2.5)&(np.abs(points[:,1])<2.5)]
    if len(p)<30:raise ValueError('portal_floor_not_observable')
    p=p[np.argsort(p[:,2])]
    right=np.searchsorted(p[:,2],p[:,2]+.006,side='right')
    i=int(np.argmax(right-np.arange(len(p))));patch=p[i:right[i]]
    if len(patch)<30 or np.ptp(patch[:,0])<.25 or np.ptp(patch[:,1])<.25:
        raise ValueError('portal_floor_not_observable')
    values,vectors=np.linalg.eigh(np.cov(patch.T))
    if values[1]<.001 or abs(vectors[2,0])<.98:
        raise ValueError('portal_floor_not_horizontal')
    return .0815-float(np.median(patch[:,2]))


def landmark_points(points, plane_x, center_x, center_y, base_z):
    """Restrict local landmark fits to their surveyed face extent, not just X.

    Field end walls retain the full existing height/Y test. Beam caps and the
    20mm roof edge are distinct faces; accepting arbitrary nearby surfaces as
    either face can create a false absolute longitudinal correction.
    """
    if abs(abs(plane_x)-6.)<1e-6:
        # Exclude the surveyed corner boxes that share the outer wall height.
        return points[(points[:,1]+center_y>-2.45)&(points[:,1]+center_y<4.45)]
    y=points[:,1]+center_y
    # center_y here is measured base Y; the surveyed tunnel center is supplied
    # separately by center_x's fixed mirrored field geometry.
    tunnel_y=4.46 if center_x<0 else -2.46
    z=points[:,2]+base_z
    if abs(abs(plane_x-center_x)-1.)<1e-6:
        side=-1 if center_x<0 else 1
        a,b=sorted([tunnel_y+side*.4,tunnel_y+side*.48])
        keep=(y>=a-.003)&(y<=b+.003)&(z>=.085)&(z<=.283)
        if center_x<0 and abs(plane_x-(center_x+1.))<1e-6:
            # Rendered west support face at X=-1.5: actual STL triangles
            # span Y=2.7..3.98, Z=.08.. .28. Exclude its low floor seam.
            # This is a separate surveyed transverse face, not a wider
            # association window or an extension of the LIO bridge budget.
            keep |= (y>=2.7)&(y<=3.98)&(z>=.09)&(z<=.28)
    else:
        keep=(y>=tunnel_y-.483)&(y<=tunnel_y+.483)&(z>=.327)&(z<=.353)
        # The opposite plate edge is hidden behind the near edge/underside.
        # Its identical Y/Z extent must not create a 350mm longitudinal alias.
        keep &= points[:,0]*(center_x-plane_x)>0.
    return points[keep]


def propagate_gyro(rotation, history, begin, end):
    samples = list(history)
    if not samples or samples[0][0] > begin or end-samples[-1][0] > .025:
        raise ValueError('gyro_history_not_fresh_or_not_bracketed')
    before = [s for s in samples if s[0] <= begin]
    previous_t, previous_w = begin, before[-1][1]
    for stamp, w in samples:
        if stamp <= begin:
            continue
        if stamp > end:
            break
        if stamp-previous_t > .025:
            raise ValueError('gyro_history_gap')
        rotation = rotation * Rotation.from_rotvec((previous_w+w)*.5*(stamp-previous_t))
        previous_t, previous_w = stamp, w
    return rotation * Rotation.from_rotvec(previous_w*(end-previous_t))


class Passage(rclpy.node.Node):
    def __init__(self, wall_y=None, wall_side=-1):
        super().__init__('sim_tunnel_passage', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.valid = self.ready = False
        self.valid_at = self.ready_at = self.odom_at = self.joints_at = -math.inf
        self.odom = None
        self.joints = {}
        self.contacts = []
        self.map_ok = False
        maps = ROOT / 'src/simulation_integration/huananhu_sim_bringup/maps'
        meta = yaml.safe_load((maps / 'rmul_2027.yaml').read_text())
        with (maps / meta['image']).open('rb') as stream:
            if stream.readline().strip() != b'P5':
                raise ValueError('unsupported_audited_map_image')
            width, height = map(int, stream.readline().split())
            if stream.readline().strip() != b'255':
                raise ValueError('unsupported_map_depth')
            pixels = np.flipud(np.frombuffer(stream.read(), dtype=np.uint8).reshape(height, width))
        cells = np.where(pixels == 0, 100, np.where(pixels == 254, 0, -1)).astype(np.int8)
        self.map_shape, self.map_meta = (width, height), meta
        self.map_hash = hashlib.sha256(cells.tobytes()).hexdigest()
        self.control = ''
        self.control_received_at = 0.
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.cmd = self.create_publisher(Twist, '/sim/manual_cmd_vel', 10)
        self.imu_history = deque()
        self.gravity_samples = deque()
        self.local_attitude = None
        self.attitude_stamp = None
        self.wall_y, self.wall_fit = wall_y, None
        self.active_wall_y = wall_y
        self.wall_side = wall_side
        self.outer_wall_y = None if wall_y is None else wall_y-wall_side*.94
        self.wall_velocity = 0.
        self.wall_position = None
        self.lateral_covariance=np.diag([.001**2,.03**2])
        self.command_y_world=0.
        self.command_heading=0.
        self.zero_requested=True
        self.cloud_info = {}
        self.raw_scan = None
        self.end_wall_x = -6. if wall_side<0 else 6.
        self.x_fix = None
        self.x_anchor = None
        cx=-2.5 if wall_side<0 else 2.5
        self.x_planes=[self.end_wall_x,-self.end_wall_x,cx-1.,cx+1.,cx-.175,cx+.175]
        self.x_velocity = 0.
        self.pending_absolute = None
        self.x_info = {}
        self.mode = self.create_publisher(String, '/sim/control_mode', 10)
        self.posture = self.create_publisher(String, '/huananhu/posture', 10)
        self.create_subscription(PointCloud2, '/lidar/points', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, '/sim/localization_valid', self.on_valid, 10)
        self.create_subscription(Bool, '/sim/posture_ready', self.on_ready, 10)
        self.create_subscription(String, '/sim/control_status', self.on_control, 10)
        self.create_subscription(Odometry, '/sim/base_odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joint, qos_profile_sensor_data)
        self.create_subscription(Contacts, '/assessment/body_contacts', self.on_contact, qos_profile_sensor_data)

    def update_wall_filter(self, stamp, measured, known_y):
        dt=stamp-self.wall_fit[0] if self.wall_fit is not None else .025
        if dt>.5:
            # Never integrate across an unobserved interval. Recover only at
            # explicit zero with independently sampled lateral and gyro rest.
            self.pending_wall,confirmed=confirm_absolute_fix(
                getattr(self,'pending_wall',None),known_y,stamp,measured)
            samples=self.pending_wall[1]
            if (not confirmed or not self.zero_requested or not self.imu_history or
                np.linalg.norm(self.imu_history[-1][1])>=.003 or
                np.ptp([v for _,v in samples])>=.001):
                raise ValueError('side_wall_reacquisition_pending')
            self.wall_position,self.wall_velocity,self.lateral_covariance=lateral_kalman(
                None,0.,self.lateral_covariance,measured,0.,.025)
            self.pending_wall=None
        else:
            self.wall_position,self.wall_velocity,self.lateral_covariance=lateral_kalman(
                self.wall_position,self.wall_velocity,getattr(self,'lateral_covariance',np.diag([.001**2,.03**2])),
                measured,getattr(self,'command_y_world',0.),dt)

    def on_cloud(self, msg):
        # Use the existing instantaneous 40Hz raw acquisition for local walls.
        # Point-LIO keeps its independent 10Hz composite scan unchanged. No
        # synthetic timestamps, world pose, or additional sensor publication.
        if msg.header.frame_id != 'navigation_laser':
            return
        data = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'], skip_nans=False)
        points = np.column_stack([data[k] for k in ['x', 'y', 'z']]).astype(float)
        points += [.0557597190051302, -.163594690492484, .198643863403111]
        self.raw_scan=(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,points)
        points=points[np.isfinite(points).all(1)]
        if getattr(self,'local_attitude',None) is not None:
            roll,pitch,_=self.local_attitude.as_euler('xyz')
            points=Rotation.from_euler('xyz',[roll,pitch,0.]).apply(points)
        inner_band = ((points[:,1] > -.50) & (points[:,1] < -.285) if self.wall_side < 0 else
                      (points[:,1] > .44) & (points[:,1] < .65))
        outer_band = ((points[:,1] > .56) & (points[:,1] < .68) if self.wall_side < 0 else
                      (points[:,1] > -.52) & (points[:,1] < -.40))
        exterior = (np.isfinite(points).all(1) & (points[:,2] > .025) & (points[:,2] < .40) &
                    (np.abs(points[:,0]) < 1.8) & ((points[:,0] > .46) | (points[:,0] < -.36)))
        self.cloud_info = dict(source='/lidar/points')
        # Either audited continuous side wall may be occluded by the folded
        # body near an exit. Switch only after a full observable line fit; each
        # line retains its own surveyed wall coordinate, including velocity.
        for label, band, known_y in [('outer',outer_band,self.outer_wall_y),
                                     ('inner',inner_band,self.wall_y)]:
            recovering=(self.wall_fit is not None and self.raw_scan[0]-self.wall_fit[0]>.5)
            if label=='outer' and recovering and self.zero_requested:
                # The last narrow band can be left during physical braking.
                # Reassociate only the continuous surveyed boundary within a
                # bounded window; the five-scan stationary guard still applies.
                band=abs(points[:,1]-(known_y-self.wall_position))<.20
            visible=exterior & band
            if label=='inner':
                visible &= points[:,2]<.185
            selected=points[visible]
            self.cloud_info[label+'_candidates']=len(selected)
            try:
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                slope, intercept, count = fit_wall(selected[:,:2])
                measured_y=known_y-intercept*math.cos(math.atan(slope))
                if recovering and abs(math.atan(slope))>.15:
                    continue
                self.update_wall_filter(stamp,measured_y,known_y)
                self.wall_fit = stamp, slope, intercept
                self.active_wall_y=known_y
                self.cloud_info.update(wall=label,slope=slope,intercept=intercept,inliers=count,stamp=stamp)
                break
            except ValueError:
                continue

    def on_imu(self, msg):
        if msg.header.frame_id != 'body':
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        w = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        if not np.all(np.isfinite(w)) or (self.imu_history and stamp <= self.imu_history[-1][0]):
            return
        if self.local_attitude is None:
            a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            if np.isfinite(a).all() and abs(np.linalg.norm(a)-9.81)<.15 and np.linalg.norm(w)<.01:
                self.gravity_samples.append((stamp,a))
                while self.gravity_samples and stamp-self.gravity_samples[0][0]>1.05:
                    self.gravity_samples.popleft()
                if stamp-self.gravity_samples[0][0]>=1. and np.max(np.std([a for _,a in self.gravity_samples],axis=0))<.03:
                    g=np.mean([a for _,a in self.gravity_samples],axis=0)
                    self.local_attitude=Rotation.from_euler('xyz',[math.atan2(g[1],g[2]),math.atan2(-g[0],math.hypot(g[1],g[2])),0.])
                    self.attitude_stamp=stamp
            else:
                self.gravity_samples.clear()
        elif self.imu_history:
            # Tilt starts from measured stationary gravity, then follows the
            # high-rate gyro. Delayed LIO tilt must not create false millimetre
            # roof violations. No IMU absolute orientation is consumed.
            dt=stamp-self.attitude_stamp
            if dt>.025:
                self.local_attitude=None
                self.gravity_samples.clear()
            else:
                self.local_attitude=self.local_attitude*Rotation.from_rotvec((self.imu_history[-1][1]+w)*.5*dt)
                self.attitude_stamp=stamp
                # Relevel only under explicit zero command and measured rest.
                # This keeps long repeated passes from accumulating tilt error;
                # tight measured-rest bounds reject small residual braking motion.
                a=np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
                resting=(getattr(self,'zero_requested',False) and abs(getattr(self,'x_velocity',1.))<.005 and
                         abs(getattr(self,'wall_velocity',1.))<.002 and np.linalg.norm(w)<.003)
                if resting and np.isfinite(a).all() and abs(np.linalg.norm(a)-9.81)<.15:
                    self.gravity_samples.append((stamp,a))
                    while self.gravity_samples and stamp-self.gravity_samples[0][0]>1.05:
                        self.gravity_samples.popleft()
                    if stamp-self.gravity_samples[0][0]>=.5 and np.max(np.std([a for _,a in self.gravity_samples],axis=0))<.03:
                        g=np.mean([a for _,a in self.gravity_samples],axis=0)
                        yaw=self.local_attitude.as_euler('xyz')[2]
                        self.local_attitude=Rotation.from_euler('xyz',[math.atan2(g[1],g[2]),math.atan2(-g[0],math.hypot(g[1],g[2])),yaw])
                else:
                    self.gravity_samples.clear()
        self.imu_history.append((stamp, w))
        while self.imu_history and stamp - self.imu_history[0][0] > 1.5:
            self.imu_history.popleft()

    def on_map(self, msg):
        # Saved-local maps also use the name map. Check actual map content,
        # rather than accidentally applying whole-field portals in another gauge.
        info = msg.info
        self.map_ok = bool(msg.header.frame_id == 'map' and
            (info.width, info.height) == self.map_shape and
            abs(info.resolution - self.map_meta['resolution']) < 1e-7 and
            abs(info.origin.position.x - self.map_meta['origin'][0]) < 1e-7 and
            abs(info.origin.position.y - self.map_meta['origin'][1]) < 1e-7 and
            hashlib.sha256(np.asarray(msg.data, dtype=np.int8).tobytes()).hexdigest() == self.map_hash)

    def on_valid(self, msg):
        self.valid, self.valid_at = msg.data, time.monotonic()

    def on_ready(self, msg):
        self.ready, self.ready_at = msg.data, time.monotonic()

    def on_control(self, msg):
        self.control = msg.data
        self.control_received_at = time.monotonic()

    def on_odom(self, msg):
        self.odom, self.odom_at = msg, time.monotonic()

    def on_joint(self, msg):
        self.joints, self.joints_at = dict(zip(msg.name, msg.position)), time.monotonic()

    def on_contact(self, msg):
        self.contacts.extend([c.collision1.name, c.collision2.name] for c in msg.contacts)

    def pose(self):
        now = time.monotonic()
        if (not self.map_ok or not self.valid or not self.ready or self.odom is None or
                any(now - t > .5 for t in [self.valid_at, self.ready_at, self.odom_at, self.joints_at])):
            self.health_diagnostics=dict(map_ok=self.map_ok,localization_valid=self.valid,posture_ready=self.ready,odom_present=self.odom is not None,receipt_ages_s={key:float(now-value) if math.isfinite(value) else None for key,value in [('valid',self.valid_at),('posture',self.ready_at),('odom',self.odom_at),('joint',self.joints_at)]})
            raise ValueError('localization_posture_or_joint_not_fresh')
        m = self.odom
        if m.header.frame_id != 'camera_init' or m.child_frame_id != 'base_footprint':
            raise ValueError('unexpected_odometry_frame')
        age = (self.get_clock().now().nanoseconds -
               m.header.stamp.sec * 10**9 - m.header.stamp.nanosec) * 1e-9
        if not 0 <= age <= .5:
            raise ValueError('measurement_time_not_fresh')
        t = self.tf.lookup_transform('map', 'camera_init', rclpy.time.Time.from_msg(m.header.stamp))
        q, a = m.pose.pose.orientation, t.transform.rotation
        rotation = Rotation.from_quat([a.x, a.y, a.z, a.w])
        position = rotation.apply([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        position += [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        lio_measured_x=float(position[0])
        body = rotation * Rotation.from_quat([q.x, q.y, q.z, q.w])
        # LIO is delayed by scan accumulation. Predict only within its existing
        # .5s validity window, using measured body twist, never Gazebo truth.
        v, w = m.twist.twist.linear, m.twist.twist.angular
        position += body.apply([v.x, v.y, v.z]) * age
        # Propagate attitude across the scan delay with raw measured gyro.
        # No IMU orientation/world heading or Gazebo pose is consumed.
        end = self.get_clock().now().nanoseconds * 1e-9
        begin = end - age
        body = propagate_gyro(body, self.imu_history, begin, end)
        if self.wall_y is not None:
            if self.wall_fit is None or not 0 <= end-self.wall_fit[0] <= .5:
                raise ValueError('side_wall_not_fresh')
            wall_stamp, slope, intercept = self.wall_fit
            wall_heading = -math.atan(slope)
            lio_heading=body.as_euler('xyz')[2]
            if abs(math.remainder(lio_heading-wall_heading,2*math.pi))>.25:
                raise ValueError('tunnel_heading_branch_not_aligned')
            current_heading = propagate_gyro(Rotation.from_euler('z', wall_heading),
                                            self.imu_history, wall_stamp, end).as_euler('xyz')[2]
            if self.local_attitude is None or not 0<=end-self.attitude_stamp<=.025:
                raise ValueError('stationary_gravity_initialization_or_gyro_not_ready')
            tilt=self.local_attitude*Rotation.from_rotvec(self.imu_history[-1][1]*(end-self.attitude_stamp))
            roll, pitch, _ = tilt.as_euler('xyz')
            position[1] = self.wall_position + self.wall_velocity*(end-wall_stamp)
            body = Rotation.from_euler('xyz', [roll, pitch, current_heading])
            self.command_heading=current_heading
            if self.raw_scan is None or not 0<=end-self.raw_scan[0]<=.1:
                raise ValueError('end_wall_not_fresh')
            scan_stamp,scan_points=self.raw_scan
            relative_x=None
            if self.x_anchor is not None:
                anchor_time,anchor_x,anchor_lio,anchor_age=self.x_anchor
                relative_x=anchor_x+lio_measured_x-anchor_lio+self.x_velocity*(age-anchor_age)
            if self.x_fix is None or scan_stamp>self.x_fix[0]:
                delta=propagate_gyro(Rotation.identity(),self.imu_history,scan_stamp,end)
                scan_rotation=body*delta.inv()
                world_axes=scan_rotation.apply(scan_points[np.isfinite(scan_points).all(1)])
                seed_x=position[0] if relative_x is None else relative_x
                try:floor_base_z=observed_floor_base_z(world_axes)
                except ValueError:floor_base_z=None
                accepted=None
                reacquiring=False
                self.x_diagnostics=dict(scan_stamp=scan_stamp,seed_x=float(seed_x),
                    relative_x=None if relative_x is None else float(relative_x),
                    rotation_xyzw=scan_rotation.as_quat().tolist(),floor_base_z=floor_base_z,
                    last_fix=self.x_fix,zero_requested=self.zero_requested,attempts=[])
                for plane in sorted(self.x_planes,key=lambda x:(abs(abs(x)-6.)>1e-6,abs(x-seed_x))):
                    roof_recovered=False
                    attempt=dict(plane=plane)
                    self.x_diagnostics['attempts'].append(attempt)
                    try:
                        if abs(abs(plane)-6.)>1e-6 and floor_base_z is None:
                            attempt['rejected']='floor_not_observable'
                            continue
                        face=landmark_points(world_axes,plane,-2.5 if self.wall_side<0 else 2.5,position[1],floor_base_z if floor_base_z is not None else position[2])
                        field=abs(abs(plane)-6.)<1e-6
                        roof=abs(abs(plane-(-2.5 if self.wall_side<0 else 2.5))-.175)<1e-6
                        window=.6 if field or (roof and self.zero_requested and relative_x is not None) else .25
                        x,count,residual=fit_end_wall(face,seed_x,position[1],plane,verify_normal=True,association_window=window)
                        attempt.update(x=x,count=count,residual_m=residual,window_m=window)
                        if relative_x is not None and abs(x-relative_x)>.02+self.x_info.get('uncertainty_m',.005):
                            if not (field or roof) or count<12:
                                attempt['rejected']='jump_requires_supported_absolute_face'
                                continue
                            self.pending_absolute,confirmed=confirm_absolute_fix(
                                self.pending_absolute,plane,scan_stamp,x)
                            if roof:
                                # Moving roof association stays at .25m; the
                                # stopped .6m search still needs five stationary
                                # observations before it may replace the anchor.
                                confirmed=(confirmed and self.zero_requested and
                                           np.ptp([v for _,v in self.pending_absolute[1]])<.002 and
                                           np.linalg.norm(self.imu_history[-1][1])<.003)
                            if (not confirmed or abs(self.wall_velocity)>.01 or
                                abs(self.imu_history[-1][1][2])>.02):
                                attempt.update(rejected='absolute_confirmation_pending',
                                    confirmation_samples=list(self.pending_absolute[1]),
                                    wall_velocity=float(self.wall_velocity),
                                    gyro=self.imu_history[-1][1].tolist())
                                reacquiring=True
                                break
                            roof_recovered=roof
                        if self.x_fix is not None:
                            elapsed=scan_stamp-self.x_fix[0]
                            if abs(x-self.x_fix[1])>.015+.3*elapsed:
                                attempt['rejected']='absolute_velocity_discontinuity'
                                continue
                        attempt['accepted']=True
                        accepted=(x,count,residual,plane)
                        break
                    except ValueError as exc:
                        attempt['rejected']=str(exc)
                        continue
                if reacquiring:
                    raise ValueError('longitudinal_reacquisition_pending')
                if accepted is not None:
                    self.pending_absolute=None
                    x,count,residual,plane=accepted
                    if roof_recovered:
                        self.x_velocity=0.
                    elif self.x_fix is not None:
                        elapsed=scan_stamp-self.x_fix[0]
                        self.x_velocity=.7*self.x_velocity+.3*(x-self.x_fix[1])/elapsed
                    self.x_fix=scan_stamp,x
                    current_x=x+self.x_velocity*(end-scan_stamp)
                    self.x_anchor=(end,current_x,lio_measured_x,age)
                    self.x_info=dict(method='surveyed_plane',stamp=scan_stamp,x=x,inliers=count,
                                     residual_m=residual,wall_x=plane,uncertainty_m=.005,floor_base_z_m=floor_base_z)
                    relative_x=None
                elif self.x_anchor is None:
                    raise ValueError('end_wall_not_observable')
                else:
                    elapsed=end-self.x_anchor[0];travel=abs(relative_x-self.x_anchor[1])
                    if elapsed>5. or travel>.5:
                        raise ValueError('longitudinal_bridge_exhausted')
                    self.x_info=dict(method='bounded_relative_lio',stamp=end,x=relative_x,
                                     anchor_age_s=elapsed,relative_travel_m=travel,
                                     uncertainty_m=.01+.20*travel)
            if self.x_info.get('method')=='bounded_relative_lio':
                position[0]=relative_x
            else:
                position[0]=self.x_fix[1]+self.x_velocity*(end-self.x_fix[0])
        return position, body, self.get_clock().now().nanoseconds * 1e-9

    def publish(self, command=None, active=False):
        c=command if command is not None and active else Twist()
        self.command_y_world=math.sin(self.command_heading)*c.linear.x+math.cos(self.command_heading)*c.linear.y
        self.zero_requested=abs(c.linear.x)+abs(c.linear.y)+abs(c.angular.z)<1e-12
        self.posture.publish(String(data='tunnel'))
        self.mode.publish(String(data='manual' if active else 'disabled'))
        self.cmd.publish(command if command is not None else Twist())


def confirm_disabled(node, timeout=3.):
    """Publish zero/disabled until a fresh post-request status is received.

    Wall time bounds cleanup even with a paused simulation clock. Callback
    count cannot bound this wait: queued IMU/cloud callbacks return immediately.
    """
    requested = time.monotonic()
    deadline = requested + timeout
    last_publish = -math.inf
    while rclpy.ok() and time.monotonic() < deadline:
        now = time.monotonic()
        if now-last_publish >= .1:
            node.publish()
            last_publish = now
        rclpy.spin_once(node, timeout_sec=.02)
        now = time.monotonic()
        if (node.control == 'disabled' and
                requested <= node.control_received_at <= now and
                now-node.control_received_at <= .5):
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tunnel', choices=['west', 'east'], required=True)
    parser.add_argument('--passes', type=int, default=5)
    parser.add_argument('--speed', type=float, default=.08)
    parser.add_argument('--clearance', type=float, default=.2495,
                        help='may only reduce the audited clearance, e.g. .235 negative test')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.passes <= 10 or not .02 <= args.speed <= .12 or not .1 <= args.clearance <= .2495:
        parser.error('passes 1..10, speed .02.. .12, clearance .1.. .2495 required')
    if args.out.exists():
        parser.error('output already exists')
    sys.path.insert(0, str(ROOT / 'src/simulation_integration/huananhu_sim_bringup/scripts'))
    from audit_prior_geometry import audit
    audit(RESOURCES, ROOT / 'src/simulation_integration/huananhu_sim_bringup/maps')
    maps=ROOT/'src/simulation_integration/huananhu_sim_bringup/maps'
    manifest=json.loads((maps/'prior_map_manifest.json').read_text())
    for source in manifest['visual_sources']:
        model_path=RESOURCES/'models'/source['uri'].removeprefix('model://')
        if hashlib.sha256(model_path.read_bytes()).hexdigest()!=source['sha256']:
            raise ValueError('longitudinal_landmark_prior_source_changed')
    config = json.loads((RESOURCES / 'config/model.json').read_text())
    profile = json.loads((RESOURCES / 'config/tunnel_profile.json').read_text())
    support = {k: np.asarray(v) for k, v in json.loads((RESOURCES / 'config/cad_support_vertices.json').read_text()).items()}
    cx, cy = {'west': (-2.5, 4.46), 'east': (2.5, -2.46)}[args.tunnel]
    lane = cy + profile['lane_body_center_y_m']
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    wall_side = -1 if args.tunnel == 'west' else 1
    node = Passage(cy + wall_side*.4, wall_side)
    stopping = False
    def stop(*_args):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    result = dict(completed=False, truth_used_for_control=False, tunnel=args.tunnel,
                  controller='bounded_LIO_X_LiDAR_wall_Y_heading_raw_gyro_tunnel_segment', passes=[], failure=None)
    trace = []
    holds=[]
    safety_brakes=[]
    last_hold=None
    readiness_error=None
    try:
        if profile['cad_height_m']+.001>=args.clearance:
            raise ValueError('nominal_folded_profile_clearance_rejected')
        deadline = time.monotonic() + 90
        while not stopping and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            node.publish()
            try:
                position, rotation, stamp = node.pose()
                break
            except Exception as exc:
                readiness_error=str(exc)
                continue
        else:
            raise ValueError('readiness_timeout_or_interrupted:'+str(readiness_error))
        if abs(position[1] - lane) > .1 or min(abs(position[0] - (cx - 1.7)), abs(position[0] - (cx + 1.7))) > .15:
            raise ValueError('must_start_at_portal_staging_pose')
        target = cx + 1.7 if position[0] < cx else cx - 1.7
        for number in range(args.passes):
            deadline = time.monotonic() + 600
            aligned = False
            settled_at = None
            previous = None
            last_control = -math.inf
            last_command = Twist()
            last_active = False
            ramp_start=np.zeros(3)
            ramp_target=np.zeros(3)
            brake_until=-math.inf
            wf = 0.
            while not stopping and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                if node.contacts:
                    raise ValueError('body_contact_stop')
                stamp = node.get_clock().now().nanoseconds * 1e-9
                if stamp-last_control < .05:
                    # Keep callback service lightweight between the existing
                    # 20Hz control updates; repeated CAD FK on every IMU
                    # callback otherwise starves this node's gyro subscription.
                    last_command=interpolated_command(ramp_start,ramp_target,(stamp-last_control)/.05)
                    node.publish(last_command, active=last_active)
                    continue
                try:
                    position, rotation, stamp = node.pose()
                except ValueError as exc:
                    if str(exc) not in ('side_wall_not_fresh', 'stationary_gravity_initialization_or_gyro_not_ready', 'end_wall_not_observable', 'end_wall_not_fresh', 'longitudinal_bridge_exhausted', 'longitudinal_reacquisition_pending'):
                        raise
                    # A dropped scan stops motion; only recomputed controls may
                    # resume after fresh wall data and a new lane alignment.
                    node.publish()
                    if str(exc)!=last_hold:
                        hold=dict(stamp=stamp,reason=str(exc),longitudinal=getattr(node,'x_diagnostics',None))
                        if node.raw_scan is not None and len(holds)<20:
                            snapshot=args.out.with_name(args.out.stem+'.hold_%03d.npz'%len(holds))
                            np.savez_compressed(snapshot,stamp=node.raw_scan[0],points=node.raw_scan[1])
                            hold['scan_snapshot']=str(snapshot)
                        holds.append(hold)
                    last_hold=str(exc)
                    # A zero-command longitudinal confirmation does not revoke
                    # an already measured lane alignment. Wall/gyro loss and
                    # bridge exhaustion still require fresh alignment; a brake
                    # below also clears it independently.
                    if str(exc)!='longitudinal_reacquisition_pending':
                        aligned=False
                    last_control, last_command = stamp, Twist()
                    last_active = False
                    ramp_start=np.zeros(3)
                    ramp_target=np.zeros(3)
                    previous = None
                    continue
                last_hold=None
                if node.contacts:
                    raise ValueError('body_contact_stop')
                for key, desired in [('pitch1_drive_joint', profile['fold_rad']), ('yaw1_joint', 0.),
                                     ('yaw2_joint', 0.), ('pitch2_joint', profile['aim_rad'])]:
                    if key not in node.joints or abs(node.joints[key] - desired) > profile['joint_tolerance_rad']:
                        raise ValueError('not_in_audited_folded_posture')
                points = envelope(config, support, node.joints, rotation.as_matrix())
                side,in_corridor = passage_margin(points,position,cx,cy,node.x_info.get('uncertainty_m',.005))
                height = float(np.ptp(points[:, 2]))
                under_roof=overlaps_metal_roof(points,position[0],cx,node.x_info.get('uncertainty_m',.005))
                result['last_geometry'] = dict(side_margin_m=side, height_m=height, position=position.tolist(),
                    rpy=rotation.as_euler('xyz').tolist(), in_corridor=in_corridor, under_roof=under_roof, joints=node.joints)
                if side < .002 or (under_roof and height + .001 >= args.clearance):
                    raise ValueError('folded_envelope_clearance_rejected')
                last_control = stamp
                yaw = math.atan2(rotation.as_matrix()[1, 0], rotation.as_matrix()[0, 0])
                dx, dy, err = target - position[0], lane - position[1], math.remainder(-yaw, 2 * math.pi)
                dt=.05 if previous is None else max(.001,stamp-previous[0])
                if previous is not None and stamp > previous[0]:
                    wf = .4*wf + .6*node.imu_history[-1][1][2]
                previous = stamp, position[:2].copy(), yaw
                # Align before motion, then keep continuous lateral feedback.
                # Repeated axis stops excite low-speed roller stick-slip; the
                # full-envelope and contact guards above remain unconditional.
                if not aligned and side > .018 and abs(dy) < .005 and abs(err) < .01:
                    aligned = True
                vx = max(-args.speed, min(args.speed, 1.2 * dx)) if aligned else 0.
                # Gains match the recorded local response (y a=.98,b=.30;
                # yaw a=.76,b=.49 per 50ms), with stronger lateral damping.
                vy = max(-.08, min(.08, 4.7 * dy - 1.45 * node.wall_velocity))
                w = max(-.25, min(.25, 4.4 * err - .8 * wf))
                command = Twist()
                command.linear.x = math.cos(yaw) * vx + math.sin(yaw) * vy
                command.linear.y = -math.sin(yaw) * vx + math.cos(yaw) * vy
                command.angular.z = w
                prior=np.array([last_command.linear.x,last_command.linear.y,last_command.angular.z])
                target_velocity=np.array([command.linear.x,command.linear.y,command.angular.z])
                limited=limit_velocity(prior,target_velocity,.05 if previous is None else min(dt,.05))
                command.linear.x,command.linear.y,command.angular.z=map(float,limited)
                brake_active=stamp<brake_until
                # Enforce the narrow lane from the aligned staging pose onward.
                # Relative LIO X can stall before landmark reacquisition; using
                # that X to enable lateral safety can activate it too late.
                if not brake_active:
                    previous_control=np.array([math.sin(yaw)*prior[0]+math.cos(yaw)*prior[1],prior[2]])
                    nominal=np.array([math.sin(yaw)*limited[0]+math.cos(yaw)*limited[1],limited[2]])
                    safe=constrained_control(points,position[1],cy,node.wall_velocity,
                                             node.imu_history[-1][1][2],previous_control,nominal)
                    if safe is None:
                        brake_until=stamp+.3
                        brake_active=True
                        aligned=False
                        safety_brakes.append(dict(stamp=stamp,side_margin_m=side,xy=position[:2].tolist()))
                    else:
                        world_x=math.cos(yaw)*limited[0]-math.sin(yaw)*limited[1]
                        command.linear.x=float(math.cos(yaw)*world_x+math.sin(yaw)*safe[0])
                        command.linear.y=float(-math.sin(yaw)*world_x+math.cos(yaw)*safe[0])
                        command.angular.z=float(safe[1])
                if brake_active:
                    command=Twist()
                    settled_at=None
                if settled_at is None and passage_settle_candidate(dx,dy,err):
                    settled_at, settled_position = stamp, position.copy()
                if settled_at is not None:
                    # Reaching a pose must command a real zero before testing
                    # rest; tiny nonzero corrections keep wheel PI propulsion
                    # alive and never allow a stationary confirmation window.
                    command=Twist()
                    if stamp-settled_at>.75:
                        if (node.x_info.get('method')=='surveyed_plane' and
                            abs(dx)<.03 and abs(dy)<.012 and abs(err)<.03 and
                            np.linalg.norm(position[:2]-settled_position[:2])<.02 and
                            abs(node.imu_history[-1][1][2])<.02 and abs(node.wall_velocity)<.01 and abs(node.x_velocity)<.02):
                            if in_corridor:
                                raise ValueError('goal_not_outside_full_corridor')
                            node.publish()
                            result['passes'].append(dict(number=number+1,target=[target,lane,0.],
                                                         stamp=stamp,estimate=position.tolist(),yaw=yaw))
                            print(json.dumps({'pass_completed':number+1,'stamp':stamp,'target_x':target}),flush=True)
                            break
                        settled_at=None
                target_vector=np.array([command.linear.x,command.linear.y,command.angular.z])
                if np.linalg.norm(target_vector)<1e-12:
                    ramp_start=np.zeros(3)
                    ramp_target=np.zeros(3)
                    last_command=Twist()
                else:
                    ramp_start=np.array([last_command.linear.x,last_command.linear.y,last_command.angular.z])
                    ramp_target=target_vector
                    last_command=interpolated_command(ramp_start,ramp_target,0.)
                last_active = not brake_active
                node.publish(last_command, active=last_active)
                if not trace or stamp - trace[-1]['stamp'] >= .05:
                    trace.append(dict(stamp=stamp, pass_number=number + 1, xy=position[:2].tolist(),
                                      yaw=yaw, in_corridor=in_corridor, side_margin_m=side, height_m=height,
                                      joints={j['name']:node.joints.get(j['name'], 0.) for j in config['joints']},
                                      longitudinal=node.x_info, command=[command.linear.x,command.linear.y,command.angular.z], published_command=[last_command.linear.x,last_command.linear.y,last_command.angular.z], wall_fit=node.wall_fit, wall_y=node.active_wall_y, wall_velocity=node.wall_velocity))
            else:
                raise ValueError('segment_timeout_or_interrupted')
            target = 2 * cx - target
        result['completed'] = True
    except Exception as exc:
        result['failure'] = type(exc).__name__ + ':' + str(exc)
    finally:
        # Record the actual stop/cleanup boundary; estimator trace samples are
        # not cancellation timestamps and may share a clock tick with commands.
        result['cancelled'] = stopping
        result['sim_stop_request_ns'] = node.get_clock().now().nanoseconds
        result['disabled_confirmed'] = confirm_disabled(node)
        result['sim_cleanup_ns'] = node.get_clock().now().nanoseconds
        if not result['disabled_confirmed']:
            result['completed'] = False
            result['failure'] = result['failure'] or 'disabled_confirmation_timeout'
        result['body_contacts'] = node.contacts
        result['last_cloud_fit'] = node.cloud_info
        result['last_longitudinal_fix'] = node.x_info
        result['last_longitudinal_diagnostics'] = getattr(node,'x_diagnostics',None)
        result['last_health_rejection'] = getattr(node,'health_diagnostics',None)
        result['trace'] = trace
        result['sensor_holds'] = holds
        result['predictive_safety_brakes'] = safety_brakes
        args.out.parent.mkdir(parents=True, exist_ok=True)
        if node.raw_scan is not None:
            np.savez_compressed(args.out.with_suffix('.scan.npz'),stamp=node.raw_scan[0],points=node.raw_scan[1])
        args.out.write_text(json.dumps(result, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps({k:v for k,v in result.items() if k!='trace'}), flush=True)
    return 0 if result['completed'] and result['disabled_confirmed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
