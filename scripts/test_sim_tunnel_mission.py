"""Route ownership and external handover constraints; no simulator required."""
import pytest
from sim_tunnel_mission import route

@pytest.mark.parametrize('side,start,goal,cx',[
 ('west',[-4.5,3.8,0],[-.5,4.,0],-2.5),
 ('east',[.5,-2.,0],[4.5,-2.,0],2.5),
 ('west',[-.5,4.,0],[-4.5,3.8,0],-2.5),
 ('east',[4.5,-2.,0],[.5,-2.,0],2.5)])
def test_auto_routes_both_sides_and_directions(side,start,goal,cx):
    r=route(start,goal)
    assert r['tunnel']==side
    assert abs(r['entrance'][0]-cx)==pytest.approx(1.7)
    assert abs(r['exit'][0]-cx)==pytest.approx(1.7)
    assert r['entrance'][0]!=r['exit'][0]
    assert r['approach'][1]==r['departure'][1]
    assert abs(r['approach'][1]-r['entrance'][1])>1.

def test_ordinary_goal_retains_nav2():
    assert route([-4,0,0],[-2,1,0]) is None
    assert route([-4,0,0],[4,0,0]) is None

def test_no_ambiguous_or_interior_goal():
    with pytest.raises(ValueError,match='inside_tunnel'):route([-4.5,3.8,0],[-2.5,4.3,0])
    with pytest.raises(ValueError,match='not_between'):route([-4,0,0],[-3,0,0],'east')
    with pytest.raises(ValueError,match='finite'):route([float('nan'),0,0],[0,0,0])


def test_portal_gravity_requires_long_explicit_zero_and_lateral_rest():
    from sim_tunnel_mission import Portal
    from sensor_msgs.msg import Imu
    from scipy.spatial.transform import Rotation
    from collections import deque
    import numpy as np
    for duration,velocity,expected in [(3.,0.,0.),(.7,0.,.02),(3.,.006,.02)]:
        node=Portal.__new__(Portal)
        node.imu_history=deque([(1.,np.zeros(3))]);node.gravity_samples=deque()
        node.local_attitude=Rotation.from_euler('xyz',[.02,0.,.1]);node.attitude_stamp=1.
        node.zero_requested=True;node.portal_zero_since=1.;node.x_velocity=.1;node.wall_velocity=velocity
        for i in range(1,round(duration/.005)+1):
            msg=Imu();msg.header.frame_id='body';stamp=1+i*.005
            msg.header.stamp.sec=int(stamp);msg.header.stamp.nanosec=round((stamp-int(stamp))*1e9)
            msg.linear_acceleration.z=9.81;node.on_imu(msg)
        angles=node.local_attitude.as_euler('xyz')
        assert angles[0]==pytest.approx(expected,abs=1e-10)
        assert angles[2]==pytest.approx(.1,abs=1e-10)


def test_external_lio_bootstrap_cannot_replace_a_lost_wall():
    from unittest.mock import patch
    from sim_tunnel_mission import Portal,Passage
    node=Portal.__new__(Portal);node.wall_y=4.06;node.wall_fit=None;node.wall_side=-1
    with patch.object(Passage,'pose',side_effect=[ValueError('side_wall_not_fresh'),([-4.2,3.3,.08],'rotation',1.)]):
        assert node.pose()==([-4.2,3.3,.08],'rotation',1.)
    assert node.wall_y==4.06 and node.x_info['uncertainty_m']>.03
    node.wall_fit=(1.,0.,.6)
    with patch.object(Passage,'pose',side_effect=[ValueError('side_wall_not_fresh'),([-4.2,4.37,.08],'rotation',1.)]):
        with pytest.raises(ValueError):node.pose()
    node.wall_fit=None
    with patch.object(Passage,'pose',side_effect=[ValueError('side_wall_not_fresh'),ValueError('tf_not_ready')]):
        with pytest.raises(ValueError):node.pose()
    assert node.wall_y==4.06


def test_seed_reconstructs_map_camera_init_without_truth():
    import numpy as np
    from scipy.spatial.transform import Rotation
    from sim_tunnel_mission import relocalization_seed
    local={'position':[1.,-.2,-.19],'orientation':Rotation.from_euler('xyz',[.01,-.02,.3]).as_quat()}
    transform=Rotation.from_euler('xyz',[.03,.01,-.4]);translation=np.array([2.,-1.,.28])
    observed={'position':transform.apply(local['position'])+translation,'orientation':(transform*Rotation.from_quat(local['orientation'])).as_quat()}
    seed=relocalization_seed(observed,local)
    assert np.allclose(seed[:3],translation)
    assert np.allclose(Rotation.from_euler('xyz',seed[3:]).as_matrix(),transform.as_matrix())


def test_portal_uses_visible_boundary_when_short_wall_is_occluded():
    import numpy as np
    from sim_tunnel_mission import fit_portal_wall
    x=np.linspace(.5,1.7,50)
    # A slanted end/roof alias has enough points to fit the old wall query.
    alias=np.column_stack([x,.227*x-.518,np.full_like(x,.17)])
    boundary=np.column_stack([x,.0045*x+.646,np.full_like(x,.17)])
    measured,slope,_,count,wall=fit_portal_wall(np.concatenate([alias,boundary]),4.3362,5.,-.8)
    assert wall==5. and count==50
    assert measured==pytest.approx(5.-.646*np.cos(np.arctan(.0045)))
    assert slope==pytest.approx(.0045)
    with pytest.raises(ValueError,match='not_observable'):
        fit_portal_wall(alias,4.3362,5.,-.8)


def test_portal_stale_filter_requires_distinct_stationary_scans_to_recover():
    import numpy as np
    from types import SimpleNamespace as S
    from unittest.mock import patch
    from sim_tunnel_mission import Portal
    node=Portal.__new__(Portal)
    from builtin_interfaces.msg import Time
    node.odom=S(header=S(stamp=Time()),pose=S(pose=S(position=S(x=0.,y=0.,z=0.))))
    node.local_attitude=None;node.outer_wall_y=5.
    node.tf=S(lookup_transform=lambda *args:S(transform=S(rotation=S(x=0.,y=0.,z=0.,w=1.),translation=S(x=-.8,y=4.35))))
    node.wall_fit=(0.,0.,.1);node.wall_position=3.9;node.wall_velocity=.1
    node.lateral_covariance=np.eye(2);node.command_y_world=0.
    node.zero_requested=True;node.imu_history=[(1.,np.zeros(3))]
    x=np.linspace(.5,1.7,50)
    raw=np.column_stack([x,np.full_like(x,.65),np.full_like(x,.17)])-[.0557597190051302,-.163594690492484,.198643863403111]
    data=np.array([tuple(r) for r in raw],dtype=[('x',float),('y',float),('z',float)])
    def scan(stamp):
        return S(header=S(frame_id='navigation_laser',stamp=S(sec=int(stamp),nanosec=round((stamp-int(stamp))*1e9))))
    with patch('sim_tunnel_mission.point_cloud2.read_points',return_value=data):
        for _ in range(6):node.on_cloud(scan(1.))
        assert node.wall_fit[0]==0.  # duplicate scans cannot confirm
        for t in [1.025,1.05,1.075]:node.on_cloud(scan(t))
        assert node.wall_fit[0]==0.
        node.zero_requested=False;node.on_cloud(scan(1.1))
        assert node.wall_fit[0]==0.  # never reanchor during commanded motion
        node.zero_requested=True;node.on_cloud(scan(1.125))
        assert node.wall_fit[0]==pytest.approx(1.125)
        assert node.wall_position==pytest.approx(4.35)
        assert node.wall_velocity==0.


@pytest.mark.parametrize('reset_success,expected_commands,completed',[(True,[3,0],True),(False,[3],False)])
def test_navigation_refresh_requires_disabled_and_successful_lifecycle_reset(tmp_path,reset_success,expected_commands,completed):
    from types import SimpleNamespace as S
    from unittest.mock import patch
    from sim_tunnel_mission import refresh_navigation_after_relocalization
    commands=[];modes=[]
    def call(request):
        commands.append(request.command)
        return S(done=lambda:True,result=lambda:S(success=reset_success or request.command==0))
    fake=S(create_publisher=lambda *a:S(publish=lambda m:modes.append(m.data)),
           create_subscription=lambda typ,topic,callback,qos:callback(S(data='disabled')),
           create_client=lambda *a:S(service_is_ready=lambda:True,call_async=call),
           destroy_node=lambda:None)
    with patch('rclpy.node.Node',return_value=fake),patch('rclpy.spin_once'):
        assert refresh_navigation_after_relocalization(tmp_path/'report.json',lambda:False)==completed
    assert commands==expected_commands
    assert modes and set(modes)=={'disabled'}
    commands.clear()
    with patch('rclpy.node.Node',return_value=fake),patch('rclpy.spin_once'):
        assert not refresh_navigation_after_relocalization(tmp_path/'cancelled.json',lambda:True)
    assert not commands


def test_external_wall_uses_visible_span_beyond_near_body_occlusion():
    import numpy as np
    from sim_tunnel_mission import fit_portal_wall
    x=np.tile([1.6,1.65,1.7,1.75,1.9,2.,2.1,2.2,2.3,2.4],2)
    points=np.column_stack([x,1.37-.01*x,np.repeat([.1,.29],10)])
    measured,_,_,count,wall=fit_portal_wall(points,3.63,5.,-.8)
    assert wall==5. and count==20
    assert measured==pytest.approx(5-1.37*np.cos(np.arctan(-.01)))
    # Association remains bounded: far returns cannot replace the wall.
    points[:,0]+=2.
    with pytest.raises(ValueError):fit_portal_wall(points,3.63,5.,-.8)


def test_west_boundary_rejects_parallel_tunnel_panel_inside_its_surveyed_span():
    import numpy as np
    from sim_tunnel_mission import fit_portal_wall
    x=np.linspace(.9,2.4,70)
    panel=np.column_stack([x,.64+.014*x,np.full_like(x,.25)])
    high_x=np.linspace(.9,1.5,14)
    boundary=np.column_stack([high_x,.776+.014*high_x,np.full_like(high_x,.28)])
    measured,_,_,count,wall=fit_portal_wall(np.concatenate([panel,boundary]),4.36,5.,-4.2)
    assert wall==5. and count==14
    assert measured==pytest.approx(5-.776*np.cos(np.arctan(.014)))
    with pytest.raises(ValueError):fit_portal_wall(panel,4.36,5.,-4.2)


def test_nav_readiness_retries_a_lost_service_reply_without_faking_active():
    from types import SimpleNamespace as S
    from unittest.mock import Mock,patch
    from sim_nav_trial import Trial
    first=Mock();first.done.return_value=False
    second=Mock();second.done.return_value=False
    client=Mock();client.service_is_ready.return_value=True;client.call_async.side_effect=[first,second,Mock()]
    node=S(lifecycle_request=None,lifecycle_request_at=-float('inf'),navigator_active=False,lifecycle=client)
    with patch('sim_nav_trial.time.monotonic',return_value=1.):
        assert not Trial.active(node)
    with patch('sim_nav_trial.time.monotonic',return_value=3.1):
        assert not Trial.active(node)
    client.remove_pending_request.assert_called_once_with(first)
    first.cancel.assert_called_once()
    assert node.lifecycle_request is second
    second.done.return_value=True;second.result.return_value=S(current_state=S(id=3))
    with patch('sim_nav_trial.time.monotonic',return_value=3.2):
        assert Trial.active(node)


def test_portal_initial_lio_bootstrap_when_wall_precedes_first_longitudinal_fix():
    from unittest.mock import patch
    from sim_tunnel_mission import Portal,Passage
    node=Portal.__new__(Portal);node.wall_y=-2.06;node.wall_fit=(1.,0.,-1.7);node.wall_side=1;node.x_anchor=None
    with patch.object(Passage,'pose',side_effect=[ValueError('end_wall_not_observable'),([.8,-1.3,.08],'rotation',1.)]):
        assert node.pose()==([.8,-1.3,.08],'rotation',1.)
    assert node.wall_y==-2.06 and node.x_info['uncertainty_m']>.03
    node.x_anchor=(1.,.8,.8,0.)
    with patch.object(Passage,'pose',side_effect=ValueError('end_wall_not_observable')):
        with pytest.raises(ValueError):node.pose()
    node.x_anchor=None
    with patch.object(Passage,'pose',side_effect=[ValueError('end_wall_not_observable'),([.8,-2.53,.08],'rotation',1.)]):
        with pytest.raises(ValueError):node.pose()

@pytest.mark.parametrize('responses,attempts,cancelled,expected_calls,success',[
    (['ready'],2,False,1,True),
    ([None,'ready'],2,False,2,True),
    ([None,None],2,False,2,False),
    ([None,'ready'],1,False,1,False),
    ([None,'ready'],2,True,1,False),
])
def test_relocalization_read_retry_is_bounded_and_cleans_timed_out_future(
        responses,attempts,cancelled,expected_calls,success):
    import ast
    import inspect
    from concurrent.futures import Future
    from sim_tunnel_mission import relocalize
    tree=ast.parse(inspect.getsource(relocalize))
    function=next(n for n in tree.body[0].body if isinstance(n,ast.FunctionDef) and n.name=='call')
    class Client:
        srv_name='/gicp_relocalization_node/get_parameters'
        def __init__(self):self.sent=[];self.removed=[]
        def service_is_ready(self):return True
        def call_async(self,request):
            future=Future();value=responses[len(self.sent)];self.sent.append(future)
            if value is not None:future.set_result(value)
            return future
        def remove_pending_request(self,future):self.removed.append(future)
    client=Client();report={};budgets=[]
    def wait(predicate,budget):budgets.append(budget);return predicate()
    namespace=dict(wait=wait,stopping=lambda:cancelled,report=report)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function],type_ignores=[])),
                 'relocalization_call','exec'),namespace)
    if success:assert namespace['call'](client,object(),attempts=attempts)=='ready'
    else:
        with pytest.raises(ValueError,match='relocalization_service_timeout'):
            namespace['call'](client,object(),attempts=attempts)
    assert len(client.sent)==expected_calls
    assert all(f.cancelled() for f in client.removed)
    assert len(report.get('service_timeouts',[]))==len(client.removed)
    assert all(b==10 for b in budgets)
    # Only the initial read opts into retry; parameter writes and recall keep
    # the one-attempt default even when their response is lost.
    retry_calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call)
                 and isinstance(n.func,ast.Name) and n.func.id=='call' and n.keywords]
    assert len(retry_calls)==1 and retry_calls[0].args[0].id=='get'
    assert ast.literal_eval(retry_calls[0].keywords[0].value)==2


@pytest.mark.parametrize('report,rejected',[
    ({'completed':True,'disabled_confirmed':True},False),
    ({'completed':True,'disabled_confirmed':False},True),
    ({'completed':True},True),
    ({'completed':False,'disabled_confirmed':True},True),
])
def test_mission_rejects_success_exit_without_passage_stop_evidence(tmp_path,report,rejected):
    """Exercise the actual mission child runner despite a nominal zero exit."""
    import ast,json,sys
    from pathlib import Path
    from types import SimpleNamespace
    import sim_tunnel_mission as mission
    source=ast.parse(Path(mission.__file__).read_text())
    main=next(n for n in source.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    run=next(n for n in main.body if isinstance(n,ast.FunctionDef) and n.name=='run')
    wrapper=ast.parse("def wrapper():\n child=None\n").body[0]
    wrapper.body.extend([run,ast.parse("run('passage', [])").body[0]])
    module=ast.fix_missing_locations(ast.Module(body=[wrapper],type_ignores=[]))
    child=SimpleNamespace(poll=lambda:0,wait=lambda timeout:0)
    (tmp_path/'passage.json').write_text(json.dumps(report))
    ns=dict(args=SimpleNamespace(out=tmp_path),result={'stages':[]},sys=sys,
            subprocess=SimpleNamespace(Popen=lambda *a,**kw:child,STDOUT=-2),ROOT=tmp_path,
            stopping=lambda:False,json=json)
    exec(compile(module,'<mission child runner>','exec'),ns)
    if rejected:
        with pytest.raises(ValueError,match='passage_stop_not_confirmed'):ns['wrapper']()
    else:ns['wrapper']()
