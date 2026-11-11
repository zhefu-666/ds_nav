"""Measurement-time rigid sensor/base recovery; no ROS or truth dependencies."""
import bisect
import math

MOUNT = (.0557597190051302, .106643863403111, .163594690492484)


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def neg(a):
    return tuple(-x for x in a)


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def mul(a, b):
    x,y,z,w = a; X,Y,Z,W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def inv(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate(q, v):
    return mul(mul(q, (*v, 0.0)), inv(q))[:3]


def axis_q(axis, angle):
    return (*[v*math.sin(angle/2) for v in axis], math.cos(angle/2))


def mount_state(angle, rate):
    """T_base_sensor, angular and linear joint velocity expressed in base.

    base_footprint->chassis z=.10, chassis->CAD base z=-.038,
    CAD->ROS Rx(pi/2), CAD yaw1 origin y=.03 and local axis -y.
    Optical origin is an explicitly uncalibrated CAD entity assumption.
    """
    cad = axis_q((1,0,0), math.pi/2)
    yaw = axis_q((0,-1,0), angle)
    lever = rotate(cad, rotate(yaw, MOUNT))
    p = add((0,0,.062+.03), lever)
    q = mul(mul(cad, yaw), axis_q((1,0,0), -math.pi/2))
    joint_w = rotate(cad, (0,-rate,0))
    return p, q, joint_w, cross(joint_w, lever)


def recover(p_world_sensor, q_world_sensor, v_sensor, w_sensor, state):
    """Return world base pose and base-frame twist from sensor-frame twist."""
    r, q_bs, joint_w, joint_v = state
    q_wb = mul(q_world_sensor, inv(q_bs))
    p_wb = add(p_world_sensor, neg(rotate(q_wb, r)))
    w_base = add(rotate(q_bs, w_sensor), neg(joint_w))
    v_base = add(add(rotate(q_bs, v_sensor), neg(cross(w_base, r))), neg(joint_v))
    return p_wb, q_wb, v_base, w_base


class JointHistory:
    def __init__(self, max_gap=.1, duration=3.):
        self.samples = []
        self.max_gap = max_gap
        self.duration = duration

    def append(self, t, angle):
        if not math.isfinite(t) or not math.isfinite(angle):
            raise ValueError('invalid_joint_sample')
        if self.samples and t <= self.samples[-1][0]:
            raise ValueError('non_monotonic_joint_time')
        self.samples.append((t, angle))
        self.samples = [s for s in self.samples if s[0] >= t-self.duration]

    def at(self, t):
        i = bisect.bisect_left([s[0] for s in self.samples], t)
        if i == 0 or i >= len(self.samples):
            raise ValueError('joint_history_not_bracketed')
        a,b = self.samples[i-1],self.samples[i]
        dt = b[0]-a[0]
        if dt > self.max_gap:
            raise ValueError('joint_history_gap')
        # yaw1 mechanical limits are +/-pi; interpolate measured coordinates,
        # not wrapped heading (crossing the limit is not a short rotation).
        rate = (b[1]-a[1])/dt
        return a[1]+(t-a[0])*rate,rate


class TrackingGuard:
    """Latch loss after first valid publication; estimator restart is recovery.

    Times are simulation measurement times. A stopped simulation clock cannot
    consume freshness allowance. Limits apply to recovered base, not moving IMU.
    """
    def __init__(self, stale_after=.5, max_speed=.8, max_yaw_rate=1.5,
                 position_slack=.03, yaw_slack=.03):
        limits=(stale_after,max_speed,max_yaw_rate,position_slack,yaw_slack)
        if not all(math.isfinite(x) and x>0 for x in limits):
            raise ValueError('invalid_tracking_guard_limits')
        self.stale_after,self.max_speed,self.max_yaw_rate,self.position_slack,self.yaw_slack=limits
        self.was_valid=False
        self.reason=None
        self.previous=None

    def mark_valid(self):
        if self.reason is None:
            self.was_valid=True

    def latch(self, reason):
        if self.was_valid and self.reason is None:
            self.reason=reason
        return self.reason

    def check_time(self, now):
        if self.was_valid and self.previous is not None and now-self.previous[0]>self.stale_after:
            self.latch('measurement_stale')
        return self.reason

    def observe(self, stamp, position, quaternion, velocity, angular):
        if self.reason:
            return self.reason
        if not all(math.isfinite(x) for x in (stamp,*position,*quaternion,*velocity,*angular)):
            return self.latch('nonfinite_base_state') or 'nonfinite_base_state'
        if math.sqrt(sum(v*v for v in velocity))>self.max_speed:
            return self.latch('base_speed_limit') or 'base_speed_limit'
        if abs(angular[2])>self.max_yaw_rate:
            return self.latch('base_yaw_rate_limit') or 'base_yaw_rate_limit'
        x,y,z,w=quaternion
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        if self.previous is not None:
            old_stamp,old_position,old_yaw=self.previous
            dt=stamp-old_stamp
            if dt<0:
                return self.latch('nonmonotonic_base_time') or 'nonmonotonic_base_time'
            if dt==0:
                return None
            if self.was_valid and dt>self.stale_after:
                return self.latch('measurement_gap')
            distance=math.sqrt(sum((a-b)**2 for a,b in zip(position,old_position)))
            yaw_delta=abs(math.atan2(math.sin(yaw-old_yaw),math.cos(yaw-old_yaw)))
            if distance>self.max_speed*dt+self.position_slack:
                return self.latch('base_pose_jump') or 'base_pose_jump'
            if yaw_delta>self.max_yaw_rate*dt+self.yaw_slack:
                return self.latch('base_yaw_jump') or 'base_yaw_jump'
        self.previous=(stamp,tuple(position),yaw)
        return None
