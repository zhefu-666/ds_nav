"""Actual CAD envelope, gyro propagation and map provenance regressions; no simulator or build."""
import importlib.util
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location('tunnel_segment', Path(__file__).with_name('sim_tunnel_passage.py'))
t = importlib.util.module_from_spec(spec)
spec.loader.exec_module(t)
from kinematic_math import joint_positions


def geometry(fold=0., yaw=0.):
    cfg = json.loads((t.RESOURCES/'config/model.json').read_text())
    profile = json.loads((t.RESOURCES/'config/tunnel_profile.json').read_text())
    support = {k:np.asarray(v) for k,v in json.loads((t.RESOURCES/'config/cad_support_vertices.json').read_text()).items()}
    q = joint_positions(cfg,fold,0.,0.,profile['aim_rad'] if fold==0. else 0.)
    return t.envelope(cfg,support,q,Rotation.from_euler('z',yaw).as_matrix()), profile


def test_real_folded_envelope_and_235_negative():
    p,_ = geometry()
    assert abs(np.ptp(p[:,2])-.2444692201864279)<1e-8
    assert np.ptp(p[:,2])+.001<.2495
    assert np.ptp(p[:,2])>.235


def test_standing_cannot_enter_250mm():
    p,_=geometry(np.deg2rad(81.))
    assert np.ptp(p[:,2])>.42


def test_actual_lateral_envelope_and_misalignment_rejection():
    p,profile=geometry()
    assert .023<t.clearance(p,profile['lane_body_center_y_m'],0.)<.024
    turned,_=geometry(yaw=np.deg2rad(12.))
    assert t.clearance(turned,profile['lane_body_center_y_m'],0.)<0.


def test_runtime_map_must_match_audited_field_content_and_frame():
    import hashlib
    import types
    from nav_msgs.msg import OccupancyGrid
    node=types.SimpleNamespace(map_shape=(3,1),map_meta={'resolution':.01,'origin':[-1.,-2.,0.]},
        map_hash=hashlib.sha256(np.array([0,100,-1],dtype=np.int8).tobytes()).hexdigest())
    m=OccupancyGrid();m.header.frame_id='map';m.info.width=3;m.info.height=1
    m.info.resolution=.01;m.info.origin.position.x=-1.;m.info.origin.position.y=-2.
    m.data=[0,100,-1]
    t.Passage.on_map(node,m);assert node.map_ok
    m.data=[0,0,-1];t.Passage.on_map(node,m);assert not node.map_ok
    m.data=[0,100,-1];m.header.frame_id='camera_init'
    t.Passage.on_map(node,m);assert not node.map_ok


def test_raw_gyro_propagation_matches_known_rotation():
    history=[(float(stamp),np.array([0.,0.,.5])) for stamp in np.arange(0.,.301,.005)]
    q=t.propagate_gyro(Rotation.from_euler('z',.7),history,.05,.25)
    assert abs(q.as_euler('xyz')[2]-.8)<1e-10


def test_gyro_stale_gap_and_unbracketed_history_rejected():
    import pytest
    for history,begin,end in [([(0.,np.zeros(3))],.0,.1),
                              ([(0.,np.zeros(3)),(.1,np.zeros(3))],0.,.1),
                              ([(.1,np.zeros(3))],0.,.1)]:
        with pytest.raises(ValueError):t.propagate_gyro(Rotation.identity(),history,begin,end)


def test_wall_fit_recovers_surface_with_clutter_and_rejects_degeneracy():
    import pytest
    rng=np.random.default_rng(12)
    x=np.linspace(-.9,.9,60)
    points=np.column_stack([x,.02*x-.3207+rng.normal(0.,.0005,len(x))])
    clutter=np.column_stack([rng.uniform(-.9,.9,60),rng.uniform(-.5,-.285,60)])
    slope,intercept,count=t.fit_wall(np.vstack([points,clutter]))
    assert abs(slope-.02)<.004 and abs(intercept+.3207)<.002 and count>=60
    with pytest.raises(ValueError):t.fit_wall(np.array([[.1,-.32]]*20))


def test_tilt_initializes_from_stationary_gravity_and_tracks_gyro():
    import types
    from collections import deque
    from sensor_msgs.msg import Imu
    n=types.SimpleNamespace(imu_history=deque(),gravity_samples=deque(),local_attitude=None,attitude_stamp=None)
    expected=Rotation.from_euler('xyz',[.02,-.01,0.])
    g=expected.inv().apply([0.,0.,9.81])
    for i in range(220):
        m=Imu();m.header.frame_id='body';m.header.stamp.sec=1+i//200;m.header.stamp.nanosec=(i%200)*5000000
        m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z=map(float,g)
        # An arbitrary absolute orientation must not influence this estimator.
        m.orientation.x=.5;m.orientation.y=.5;m.orientation.z=.5;m.orientation.w=.5
        t.Passage.on_imu(n,m)
    assert (n.local_attitude.inv()*expected).magnitude()<1e-10
    initial=n.attitude_stamp
    for i in range(1,21):
        m=Imu();m.header.frame_id='body';stamp=initial+i*.005;m.header.stamp.sec=int(stamp);m.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
        m.angular_velocity.z=.1;t.Passage.on_imu(n,m)
    assert (n.local_attitude.inv()*(expected*Rotation.from_euler('z',.00975))).magnitude()<1e-8
    m.header.stamp.nanosec+=100000000;t.Passage.on_imu(n,m)
    assert n.local_attitude is None


def test_moving_imu_cannot_initialize_stationary_tilt():
    import types
    from collections import deque
    from sensor_msgs.msg import Imu
    n=types.SimpleNamespace(imu_history=deque(),gravity_samples=deque(),local_attitude=None,attitude_stamp=None)
    for i in range(250):
        m=Imu();m.header.frame_id='body';m.header.stamp.sec=1+i//200;m.header.stamp.nanosec=(i%200)*5000000
        m.linear_acceleration.z=9.81;m.angular_velocity.z=.1;t.Passage.on_imu(n,m)
    assert n.local_attitude is None


def test_instantaneous_raw_wall_scan_uses_actual_stamp_and_excludes_body():
    import types
    from std_msgs.msg import Header
    n=types.SimpleNamespace(wall_side=-1,wall_y=4.06,outer_wall_y=5.,active_wall_y=4.06,wall_fit=None,wall_velocity=0.,wall_position=None,cloud_info={})
    n.update_wall_filter=types.MethodType(t.Passage.update_wall_filter,n)
    x=np.r_[np.linspace(-1.7,-1.1,30),np.linspace(1.1,1.7,30)]
    body=np.column_stack([x,.01*x-.3207,np.full(len(x),.10)])
    self_hits=np.tile([.1,-.32,.1],(100,1))
    sensor=np.vstack([body,self_hits])-[.0557597190051302,-.163594690492484,.198643863403111]
    header=Header();header.frame_id='navigation_laser';header.stamp.sec=3;header.stamp.nanosec=25000000
    m=t.point_cloud2.create_cloud_xyz32(header,sensor)
    t.Passage.on_cloud(n,m)
    assert n.wall_fit[0]==3.025 and abs(n.wall_fit[1]-.01)<1e-7
    assert abs(n.wall_fit[2]+.3207)<1e-7 and n.cloud_info['inner_candidates']==60
    saved=n.wall_fit;m.header.frame_id='unexpected';t.Passage.on_cloud(n,m)
    assert n.wall_fit==saved


def test_switching_audited_wall_does_not_create_false_velocity():
    import types
    from std_msgs.msg import Header
    n=types.SimpleNamespace(wall_side=-1,wall_y=4.06,outer_wall_y=5.,active_wall_y=4.06,wall_fit=None,wall_velocity=0.,wall_position=None,cloud_info={})
    n.update_wall_filter=types.MethodType(t.Passage.update_wall_filter,n)
    for stamp,y in [(3.,-.3207),(3.025,.6193)]:
        x=np.linspace(-1.5,-.5,40)
        points=np.column_stack([x,np.full(len(x),y),np.full(len(x),.1)])-[.0557597190051302,-.163594690492484,.198643863403111]
        h=Header();h.frame_id='navigation_laser';h.stamp.sec=int(stamp);h.stamp.nanosec=round((stamp-int(stamp))*1e9)
        t.Passage.on_cloud(n,t.point_cloud2.create_cloud_xyz32(h,points))
    assert n.cloud_info['wall']=='outer' and n.active_wall_y==5.
    assert abs(n.wall_velocity)<1e-5


def test_transverse_wall_recovers_x_with_lio_bias_and_rejects_degenerate_returns():
    import pytest
    rng=np.random.default_rng(41)
    x=-2.1;wall=-6.
    points=np.column_stack([np.full(20,wall-x)+rng.normal(0.,.0003,20),
                            np.linspace(-.4,.3,20),np.tile([.04,.36],10)])
    fixed,count,residual=t.fit_end_wall(points,x-.12,4.38,wall)
    assert abs(fixed-x)<.001 and count==20 and residual<.001
    for bad in [points[:3],points*np.array([1,1,0])+[0,0,.03],
                points+np.array([0.,20.,0.])]:
        with pytest.raises(ValueError):t.fit_end_wall(bad,x,4.38,wall)
    with pytest.raises(ValueError):t.fit_end_wall(points,x-.4,4.38,wall)
    with pytest.raises(ValueError):t.fit_end_wall(np.vstack([points,points+[.08,0.,0.]]),x,4.38,wall)


def test_lateral_observer_reduces_differentiated_measurement_noise():
    rng=np.random.default_rng(9);position=None;velocity=0.;cov=np.diag([.001**2,.03**2]);errors=[];vel=[];raw=[]
    for i in range(400):
        truth=4.38+.02*i*.025;measurement=truth+rng.normal(0.,.0007)
        position,velocity,cov=t.lateral_kalman(position,velocity,cov,measurement,(1-.98)*.02/.30,.025)
        if i>30:errors.append(position-truth);vel.append(velocity-.02);raw.append(measurement)
    assert np.sqrt(np.mean(np.square(errors)))<.001
    assert np.std(vel)<.025 and np.std(vel)<np.std(np.diff(raw)/.025)/2


def test_local_acceleration_limits_are_independent_of_callback_frequency():
    target=np.array([.12,.06,.3]);ends=[]
    for dt in [.025,.05]:
        command=np.zeros(3)
        for _ in range(round(1./dt)):
            previous=command.copy();command=t.limit_velocity(command,target,dt)
            assert np.all(np.abs((command-previous)/dt)<=np.array([.15,.60,1.5])+1e-9)
            assert np.all(command<=target+1e-9)
        ends.append(command)
    assert np.allclose(ends[0],ends[1]) and np.allclose(ends[0],target)


def test_east_transverse_wall_uses_mirrored_geometry():
    points=np.column_stack([np.full(12,3.9),np.linspace(-.35,.3,12),np.tile([.04,.36],6)])
    x,count,residual=t.fit_end_wall(points,2.25,-2.539,6.)
    assert abs(x-2.1)<1e-9 and count==12 and residual<1e-9


def test_far_end_wall_low_returns_and_clutter_remain_observable():
    # Far end-wall rays straddle the low and high visible rows. Keeping only
    # z>.02 loses the low row and makes an otherwise observable exit disappear.
    x=-1.62
    wall=np.array([[ -6-x + .0004*(i%3),y,z]for i,(y,z)in enumerate(
        [(y,z)for y in [-.30,-.16,-.02]for z in [.017,.379]])])
    other=wall[:2]+[.23,.1,0.]
    fixed,count,residual=t.fit_end_wall(np.vstack([wall,other]),x,4.38,-6.)
    assert abs(fixed-x)<.002 and count==6 and residual<.001



def test_lateral_predictor_rejects_jumps_and_keeps_valid_covariance():
    import pytest
    position=None;velocity=0.;cov=np.eye(2)*.001
    for dt in [.01,.025,.05,.1]*30:
        position,velocity,cov=t.lateral_kalman(position,velocity,cov,4.38,0.,dt)
        assert np.isfinite(cov).all() and np.allclose(cov,cov.T)
        assert np.linalg.eigvalsh(cov).min()>0.
    for dt in [0.,.6]:
        with pytest.raises(ValueError):t.lateral_kalman(position,velocity,cov,4.38,0.,dt)
    with pytest.raises(ValueError):t.lateral_kalman(position,velocity,cov,4.50,0.,.025)


def test_staging_and_stopping_clear_the_end_beams_not_just_the_roof():
    p,profile=geometry();lane=4.46+profile['lane_body_center_y_m']
    for x in [-4.2,-.8]:
        margin,inside=t.passage_margin(p,np.array([x,lane,.08]),-2.5,4.46)
        assert not inside and margin>.10
    margin,inside=t.passage_margin(p,np.array([-1.6,lane,.08]),-2.5,4.46)
    assert inside and margin<.025  # old endpoint still overlaps the beam
    margin,inside=t.passage_margin(p,np.array([-1.6,lane+.03,.08]),-2.5,4.46)
    assert inside and margin<0.
    margin,inside=t.passage_margin(p,np.array([-.8,4.9,.08]),-2.5,4.46)
    assert not inside and margin<0.  # outer field wall remains protected


def test_command_interpolation_is_continuous_bounded_and_zeroable():
    start=np.array([.01,-.02,.03]);target=np.array([.02,.01,-.03])
    values=[]
    for f in np.linspace(0.,1.,11):
        c=t.interpolated_command(start,target,float(f));values.append([c.linear.x,c.linear.y,c.angular.z])
    assert np.allclose(values[0],start) and np.allclose(values[-1],target)
    assert np.allclose(np.diff(values,axis=0),(target-start)/10)
    c=t.interpolated_command(np.zeros(3),np.zeros(3),.7)
    assert c.linear.x==c.linear.y==c.angular.z==0.


def test_single_high_transverse_row_observes_x_but_floor_ring_does_not():
    import pytest
    points=np.column_stack([np.full(12,-4.5),np.linspace(-.6,.3,12),np.full(12,.38)])
    x,count,_=t.fit_end_wall(points,-1.5,4.38,-6.)
    assert x==-1.5 and count==12
    points[:,2]=.03
    with pytest.raises(ValueError):t.fit_end_wall(points,-1.5,4.38,-6.)


def test_coupled_filter_keeps_centered_motion_and_brakes_unavoidable_sweep():
    points,profile=geometry();y=profile['lane_body_center_y_m']
    safe=t.constrained_control(points,y,0.,0.,0.,np.zeros(2),np.zeros(2))
    assert safe is not None and np.allclose(safe,0.)
    # Sideways momentum cannot be made safe by a small rate-limited command.
    safe=t.constrained_control(points,y+.012,0.,.25,.3,np.array([.05,.2]),np.array([-.08,-.25]))
    assert safe is None


def test_coupled_filter_reduces_an_unsafe_nominal_yaw_lateral_combination():
    points,profile=geometry();y=profile['lane_body_center_y_m']
    previous=np.array([.02,.05]);nominal=np.array([.05,.125])
    safe=t.constrained_control(points,y,0.,.015,.03,previous,nominal)
    assert safe is not None and not np.allclose(safe,nominal)
    assert np.all(np.abs(safe)<=[.08,.25])


def test_optional_wheel_profile_preserves_default_geometry_and_rejects_bad_gain(tmp_path,monkeypatch):
    import runpy,tempfile,functools
    import xml.etree.ElementTree as ET
    from launch import LaunchContext
    module=runpy.run_path(str(t.ROOT/'src/simulation_integration/huananhu_sim_bringup/launch/spawn_robot.launch.py'))
    original=tempfile.NamedTemporaryFile
    monkeypatch.setattr(tempfile,'NamedTemporaryFile',functools.partial(original,dir=tmp_path))
    def spawn(gain,proportional="0.30"):
        context=LaunchContext();context.launch_configurations.update(dict(robot_model='huananhu',drive_model='rollers',robot_name='huananhu',spawn_pose='',sampling_mode='snapshot',wheel_integral_gain=gain,wheel_proportional_gain=proportional))
        before=set(tmp_path.iterdir());module['spawn'](context)
        return ET.parse(next(iter(set(tmp_path.iterdir())-before))).getroot()
    baseline=spawn('1.0');tunnel=spawn('0.0')
    plugin=tunnel.find(".//plugin[@name='kaohe::WheelTractionDrive']")
    assert baseline.find('.//integral_gain') is None
    assert plugin.findtext('integral_gain')=='0.0'
    plugin.remove(plugin.find('integral_gain'))
    assert ET.tostring(baseline)==ET.tostring(tunnel)
    stiff=spawn('0.2','1.0');plugin=stiff.find(".//plugin[@name='kaohe::WheelTractionDrive']")
    assert plugin.findtext('proportional_gain')=='1.0' and plugin.findtext('integral_gain')=='0.2'
    plugin.remove(plugin.find('integral_gain'));plugin.remove(plugin.find('proportional_gain'))
    assert ET.tostring(stiff)==ET.tostring(baseline)
    import pytest
    for gain in ['nan','-0.1','2.0']:
        with pytest.raises(ValueError):spawn(gain)
    for proportional in ['nan','0','2.5']:
        with pytest.raises(ValueError):spawn('1.0',proportional)


def test_sparse_transverse_patch_and_beam_cap_are_observable():
    # Four independent returns across a small vertical end face constrain X;
    # the cap need not have the 150mm width of the remote field wall.
    p=np.array([[-.7,y,z]for y in [-.9,-.85]for z in [.04,.16]])
    x,count,_=t.fit_end_wall(p,-.8,4.38,-1.5)
    assert abs(x+.8)<1e-12 and count==4
    import pytest
    with pytest.raises(ValueError):t.fit_end_wall(p[:3],-.8,4.38,-1.5)


def test_longitudinal_uncertainty_keeps_corridor_guard_until_body_clear():
    points,profile=geometry();lane=4.46+profile['lane_body_center_y_m']
    pos=np.array([-1.19,lane,.08])
    assert not t.passage_margin(points,pos,-2.5,4.46)[1]
    assert t.passage_margin(points,pos,-2.5,4.46,.08)[1]


def test_gravity_relevel_requires_zero_command_and_measured_rest():
    import types
    from collections import deque
    from sensor_msgs.msg import Imu
    for zero,wall_velocity in [(True,0.),(False,0.),(True,.006)]:
        n=types.SimpleNamespace(imu_history=deque([(1.,np.zeros(3))]),gravity_samples=deque(),
            local_attitude=Rotation.from_euler('xyz',[.02,0.,.1]),attitude_stamp=1.,
            zero_requested=zero,x_velocity=0.,wall_velocity=wall_velocity)
        for i in range(1,130):
            m=Imu();m.header.frame_id='body';stamp=1+i*.005;m.header.stamp.sec=int(stamp);m.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
            m.linear_acceleration.z=9.8;t.Passage.on_imu(n,m)
        roll,pitch,yaw=n.local_attitude.as_euler('xyz')
        assert abs(roll-(0. if zero and wall_velocity==0. else .02))<1e-10 and abs(yaw-.1)<1e-10


def test_longitudinal_landmarks_reject_other_face_extents():
    # A dense nearby surface cannot be relabelled as the 20mm roof edge.
    points=np.column_stack([np.ones(20),np.linspace(-.2,.3,20),np.full(20,.12)])
    assert len(t.landmark_points(points,2.325,2.5,-2.54,.08))==0
    roof=points.copy();roof[:,2]=.26
    assert len(t.landmark_points(roof,2.325,2.5,-2.54,.08))==20
    assert 0<len(t.landmark_points(roof,6.,2.5,-2.54,.08))<20
    cap=points.copy();cap[:,1]=np.linspace(.48,.55,20)
    assert len(t.landmark_points(cap,1.5,2.5,-2.54,.08))==20
    cap[:,1]=-.1
    assert len(t.landmark_points(cap,1.5,2.5,-2.54,.08))==0


def test_horizontal_roof_arc_is_not_a_transverse_face():
    import pytest
    # Adjacent horizontal returns must defeat an artificial dense 6mm X bin.
    y=np.linspace(-.45,.45,80)
    horizontal=np.column_stack([1.+.04*y*y,y,np.full(len(y),.26)])
    with pytest.raises(ValueError,match='normal_unobservable'):
        t.fit_end_wall(horizontal,1.325,-2.54,2.325,verify_normal=True)
    # The same elevation ring intersecting a genuine vertical face is curved
    # in Z and constant in X, so its observable normal points along X.
    vertical=np.column_stack([np.ones(len(y)),y,.25+.04*y*y])
    x,_,_=t.fit_end_wall(vertical,1.325,-2.54,2.325,verify_normal=True)
    assert x==pytest.approx(1.325)


def test_observed_floor_reference_is_independent_of_lio_z():
    import pytest
    x,y=np.meshgrid(np.linspace(.6,2.,12),np.linspace(-1.,1.,12))
    floor=np.column_stack([x.ravel(),y.ravel(),np.full(x.size,-.002)])
    assert t.observed_floor_base_z(floor)==pytest.approx(.0835)
    with pytest.raises(ValueError):t.observed_floor_base_z(floor[:4])
    vertical=floor.copy();vertical[:,2]=np.linspace(-.025,.025,len(vertical));vertical[:,1]=.1
    with pytest.raises(ValueError):t.observed_floor_base_z(vertical)


def test_absolute_reacquisition_needs_distinct_stationary_scans():
    pending=None
    for _ in range(20):pending,ready=t.confirm_absolute_fix(pending,6.,1.,1.65)
    assert not ready
    for i in range(1,5):pending,ready=t.confirm_absolute_fix(pending,6.,1.+i*.025,1.65+.0001*i)
    assert ready
    pending,ready=t.confirm_absolute_fix(pending,-6.,1.2,1.65)
    assert not ready and len(pending[1])==1
    pending=None
    for i in range(8):pending,ready=t.confirm_absolute_fix(pending,6.,i*.025,1.65+i*.02)
    assert not ready


def test_stopped_outer_wall_recovery_covers_braking_outside_old_band():
    from std_msgs.msg import Header
    n=t.Passage.__new__(t.Passage)
    n.wall_side=1;n.wall_y=-2.06;n.outer_wall_y=-3.;n.wall_fit=(0.,0.,-.46)
    n.wall_position=-2.54;n.wall_velocity=0.;n.lateral_covariance=np.eye(2)
    n.zero_requested=False;n.imu_history=[(1.,np.zeros(3))]
    x=np.linspace(.5,1.7,50)
    points=np.column_stack([x,np.full_like(x,-.524),np.full_like(x,.1)])-[.0557597190051302,-.163594690492484,.198643863403111]
    def scan(stamp):
        h=Header();h.frame_id='navigation_laser';h.stamp.sec=int(stamp);h.stamp.nanosec=round((stamp-int(stamp))*1e9)
        return t.point_cloud2.create_cloud_xyz32(h,points)
    n.on_cloud(scan(1.));assert n.wall_fit[0]==0.
    n.zero_requested=True
    for stamp in [1.,1.025,1.05,1.075]:
        n.on_cloud(scan(stamp));assert n.wall_fit[0]==0.
    n.on_cloud(scan(1.1))
    assert abs(n.wall_position-(-2.476))<1e-6
    assert n.wall_velocity==0. and n.wall_fit[0]==1.1


def test_west_exit_uses_surveyed_support_face_without_accepting_floor_or_other_end():
    import pytest
    position=np.array([-.85,4.38,.08])
    y,z=np.meshgrid(np.linspace(2.8,3.9,20),[.11,.17,.25])
    absolute=np.column_stack([np.full(y.size,-1.5),y.ravel(),z.ravel()])
    face=t.landmark_points(absolute-position,-1.5,-2.5,position[1],position[2])
    x,count,_=t.fit_end_wall(face,-.86,position[1],-1.5,verify_normal=True)
    assert abs(x-position[0])<1e-9 and count==60
    floor=absolute.copy();floor[:,2]=.083
    assert not len(t.landmark_points(floor-position,-1.5,-2.5,position[1],position[2]))
    # The support extent belongs only to the audited west X=-1.5 face.
    assert not len(t.landmark_points(absolute-position,-3.5,-2.5,position[1],position[2]))
    with pytest.raises(ValueError):
        t.fit_end_wall(face,-1.3,position[1],-1.5,verify_normal=True)


def test_reverse_endpoint_does_not_recapture_near_final_yaw_limit():
    from sim_tunnel_passage import passage_settle_candidate
    # west_repeat5_diagnostic_20261102_02 stopped correcting at this endpoint.
    assert not passage_settle_candidate(-.012702244174285, -.003913091085845, .02531978445951542)
    assert not passage_settle_candidate(-.012707794020477, -.003538766967855, .024576833339439014)
    assert passage_settle_candidate(-.0127, -.0039, .009)


def test_passage_capture_requires_position_and_alignment():
    for dx,dy,yaw in [(.021,0.,0.),(0.,.011,0.),(0.,0.,.011),(float('nan'),0.,0.)]:
        assert not t.passage_settle_candidate(dx,dy,yaw)


def test_roof_overlap_uses_complete_body_and_uncertainty_in_both_directions():
    points=np.array([[-.30,0.,0.],[.40,0.,.245]])
    # Actual third-pass failure had its front 275mm before the metal roof.
    assert not t.overlaps_metal_roof(points,-3.342128196923222,-2.5,.005)
    assert t.overlaps_metal_roof(points,-3.07,-2.5,.005)
    assert not t.overlaps_metal_roof(points,-3.09,-2.5,.005)
    assert t.overlaps_metal_roof(points,-2.02,-2.5,.01)
    assert not t.overlaps_metal_roof(points,-2.00,-2.5,.005)
    assert t.overlaps_metal_roof(points,-2.5,-2.5,.005)


def test_roof_diagnostic_is_json_serializable_python_bool():
    points=np.array([[-.30,0.,0.],[.40,0.,.245]])
    for position_x in [-3.342128196923222,-2.5,-2.0]:
        value=t.overlaps_metal_roof(points,position_x,-2.5,.005)
        assert type(value) is bool
        assert json.loads(json.dumps({'under_roof':value}))['under_roof'] is value


def test_roof_far_edge_cannot_relabel_visible_near_edge():
    y=np.linspace(-.4,.4,20)
    points=np.column_stack((np.full(20,1.54031),y,np.full(20,.26)))
    assert len(t.landmark_points(points,2.325,2.5,-2.54,.08))==20
    assert len(t.landmark_points(points,2.675,2.5,-2.54,.08))==0
    points[:,0]=-1.54031
    assert len(t.landmark_points(points,2.675,2.5,-2.54,.08))==20
    assert len(t.landmark_points(points,2.325,2.5,-2.54,.08))==0


def test_visible_roof_recovery_requires_zero_and_five_new_stable_scans():
    import types,time,pytest
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.time import Time
    for seed,zero,gyro in [(1.01636,False,0.),(1.01636,True,.004),(1.01636,True,0.),(1.24354,False,0.),(1.24354,True,.004),(1.24354,True,0.)]:
        n=t.Passage.__new__(t.Passage)
        n.map_ok=n.valid=n.ready=True
        n.valid_at=n.ready_at=n.odom_at=n.joints_at=time.monotonic()
        n.odom=Odometry();n.odom.header.frame_id='camera_init';n.odom.child_frame_id='base_footprint'
        n.odom.pose.pose.position.x=seed;n.odom.pose.pose.position.y=-2.54;n.odom.pose.pose.position.z=.0815;n.odom.pose.pose.orientation.w=1.
        tf=TransformStamped();tf.transform.rotation.w=1.
        n.tf=types.SimpleNamespace(lookup_transform=lambda *args:tf)
        n.wall_y=-2.06;n.wall_side=1;n.wall_position=-2.54;n.wall_velocity=0.
        n.local_attitude=Rotation.identity();n.x_velocity=0.;n.x_planes=[2.325,2.675]
        n.x_fix=(6.,1.3897);n.x_anchor=(6.,1.3,1.3,.05)
        n.x_info={'uncertainty_m':.085};n.pending_absolute=None;n.zero_requested=zero
        x,y=np.meshgrid(np.linspace(.6,2.,12),np.linspace(-1.,1.,12))
        floor=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))
        y,z=np.meshgrid(np.linspace(-.35,.35,15),[.25,.26,.27])
        face=np.column_stack((np.full(y.size,1.54031),y.ravel(),z.ravel()))
        points=np.vstack((floor,face))
        now=[10.]
        n.get_clock=lambda:types.SimpleNamespace(now=lambda:Time(seconds=now[0]))
        for i in range(5):
            now[0]=10.+i*.025
            n.odom.header.stamp=Time(seconds=now[0]-.05).to_msg()
            n.wall_fit=(now[0]-.025,0.,-.46);n.attitude_stamp=now[0]
            n.imu_history=[(now[0]-.1+j*.01,np.array([0.,0.,gyro])) for j in range(11)]
            n.raw_scan=(now[0]-.025,points)
            if seed>1.1 and not zero:
                position,_,_=n.pose()
                assert abs(position[0]-seed)<1e-8
                assert n.x_info['method']=='bounded_relative_lio'
            elif i<4 or not zero or gyro:
                with pytest.raises(ValueError,match='longitudinal_reacquisition_pending'):n.pose()
            else:
                position,_,_=n.pose()
                assert abs(position[0]-.78469)<1e-8
                assert n.x_info['method']=='surveyed_plane'
                assert n.x_info['wall_x']==2.325
                assert n.x_velocity==0.
