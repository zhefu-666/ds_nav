"""Reference-point and false-PASS regressions; no ROS graph required."""
import math
import pytest
from sim_replay_evaluate import score_rows
from sim_tf_validate import TfAudit
from sim_trajectory_evaluate import compose, inverse, chassis_to_base
from sensor_state_math import mount_state, axis_q


def fixture(moving_joint=True, translate=True):
    # Tilted initial gauge and moving yaw1 make yaw-only/fixed-mount fixes fail.
    gauge = {'position':[2.,-3.,.4], 'orientation':axis_q((1,0,0), .2)}
    estimates, truth, joints = [], [], []
    for i in range(1,13):
        t=i*.05
        joints.append((round(t*1e9), .8*t if moving_joint else 0.))
    for i in range(2,12):
        t=i*.05;stamp=round(t*1e9)
        chassis={'position':[.6*t if translate else 0.,0.,.18],
                 'orientation':axis_q((0,0,1), .7*t)}
        base=chassis_to_base(chassis)
        p,q,_,_=mount_state(.8*t if moving_joint else 0., .8 if moving_joint else 0.)
        sensor=compose(compose(inverse(gauge),base),{'position':list(p),'orientation':list(q)})
        estimates.append({'kind':'estimate','stamp_ns':stamp,'parent_frame':'camera_init',
                          'child_frame':'body','pose':sensor})
        truth.append({'kind':'truth','stamp_ns':stamp,'parent_frame':'world',
                      'child_frame':'robot/chassis','pose':chassis})
    return estimates,truth,joints


def test_tilted_gauge_moving_joint_same_base():
    result=score_rows(*fixture())
    assert result['status']=='SCORED'
    assert result['metrics']['position_rmse_m'] < 1e-9
    assert result['metrics']['yaw_rmse_deg'] < 1e-8


def test_missing_joint_not_silently_dropped_from_coverage():
    est,truth,joints=fixture()
    result=score_rows(est,truth,joints[:4])
    assert result['status']=='FAIL'
    assert result['coverage']<.9
    assert result['estimate_samples']==len(est)
    assert result['recovery_rejected']


def test_joint_rollback_rejected():
    est,truth,joints=fixture()
    with pytest.raises(ValueError, match='non_monotonic_joint_time'):
        score_rows(est,truth,joints+[joints[0]])


def test_truth_clock_rollback_not_sorted_away():
    est,truth,joints=fixture()
    result=score_rows(est,truth+[truth[0]],joints)
    assert result['status']=='FAIL'
    assert 'clock_rollback_in_odometry' in result['reasons']


def test_yaw_only_does_not_become_translation_pass():
    result=score_rows(*fixture(translate=False))
    assert result['metrics']['position_rmse_m']<1e-9
    assert result['status']=='FAIL'
    assert result['motion_status']=='INSUFFICIENT_MOTION'


def test_wrong_sensor_frame_rejected():
    est,truth,joints=fixture()
    for row in est:row['child_frame']='already_base'
    result=score_rows(est,truth,joints)
    assert result['status']=='FAIL'
    assert result['recovery_rejected']['unexpected_sensor_odometry_frame']==len(est)


def test_tf_second_parent_detected():
    a=TfAudit();a.observe('map','base',1,'/tf');a.observe('odom','base',2,'/tf')
    assert a.report(['base'])['duplicate_parent_frames']==['base']


def test_tf_same_edge_multiple_publishers_detected():
    a=TfAudit();a.observe('map','base',1,'/tf','one');a.observe('map','base',2,'/tf','two')
    assert a.report(['base'])['multiple_publisher_frames']==['base']


def test_tf_dynamic_static_conflict_detected():
    a=TfAudit();a.observe('map','base',1,'/tf');a.observe('map','base',0,'/tf_static')
    assert a.report(['base'])['status']=='FAIL'


def test_tf_missing_evidence_fails():
    assert TfAudit().report(['base'])['status']=='FAIL'


def test_tf_dynamic_rollback_detected():
    a=TfAudit();a.observe('map','base',2,'/tf');a.observe('map','base',1,'/tf')
    assert 'dynamic_stamp_rollback:base' in a.report(['base'])['reasons']


def test_tf_valid_repeated_edge_and_static_zero():
    a=TfAudit();a.observe('map','odom',0,'/tf_static');a.observe('odom','base',1,'/tf')
    a.observe('odom','base',2,'/tf')
    assert a.report(['base','odom'])['status']=='PASS'


def test_tf_cycle_cannot_pass():
    a=TfAudit();a.observe('b','a',1,'/tf');a.observe('a','b',1,'/tf')
    assert 'tf_cycle' in a.report(['a','b'])['reasons']


def test_tf_required_frames_in_different_trees_fail():
    a=TfAudit();a.observe('root1','base',1,'/tf');a.observe('root2','camera',1,'/tf')
    assert 'required_frames_disconnected' in a.report(['base','camera'])['reasons']
