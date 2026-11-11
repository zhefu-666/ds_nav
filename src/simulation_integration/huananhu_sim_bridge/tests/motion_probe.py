#!/usr/bin/env python3
"""Bounded physics diagnostic, not navigation acceptance. Truth is recorded only."""
import argparse,json,math,os,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool,String
from livox_ros_driver2.msg import CustomMsg
from tf2_ros import Buffer,TransformListener
from rclpy.signals import SignalHandlerOptions


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='87' or not os.environ.get('IGN_PARTITION','').startswith('ds_nav_motion_'):
        raise SystemExit('This diagnostic requires the isolated ds_nav motion simulation')
    out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=rclpy.create_node('bounded_motion_diagnostic');buf=Buffer();listener=TransformListener(buf,n)
    pub=n.create_publisher(Twist,'/sim/manual_cmd_vel',10)
    mode_pub=n.create_publisher(String,'/sim/control_mode',10)
    latest={};valid=False;state='';rows=[]
    stream=(out/'samples.jsonl').open('w')
    def record(topic,m):
        nonlocal valid,state
        row={'topic':topic,'wall':time.monotonic()}
        if isinstance(m,Odometry):
            p=m.pose.pose.position;q=m.pose.pose.orientation
            row.update(t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w],v=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z])
            latest[topic]=row
        elif isinstance(m,Bool):valid=m.data;row['value']=valid
        elif isinstance(m,String):state=m.data;row['value']=state
        else:
            offsets=sorted(set(p.offset_time for p in m.points));row.update(offsets=offsets,points=m.point_num)
        stream.write(json.dumps(row)+'\n');rows.append(row)
    for t,typ in [('/aft_mapped_to_init',Odometry),('/sim/ground_truth_odometry',Odometry),('/sim/localization_valid',Bool),('/sim/localization_status',String),('/livox/lidar',CustomMsg)]:
        n.create_subscription(typ,t,lambda m,t=t:record(t,m),qos_profile_sensor_data)
    start=time.monotonic();phase='wait';phase_t=None;events=[];failure=None;transform=None
    # Inputs are velocity only; no pose/force teleportation, no truth feedback.
    segments=[('settle',2.,0.,0.),('translate',3.,.15,0.),('stop_translation',2.,0.,0.),('rotate',2.,0.,.2),('stop_rotation',3.,0.,0.)]
    index=0
    try:
        while time.monotonic()-start<180:
            rclpy.spin_once(n,timeout_sec=.02)
            truth=latest.get('/sim/ground_truth_odometry')
            if truth is None:continue
            sim=truth['t']
            if phase=='wait':
                if not valid:
                    if time.monotonic()-start>60:failure='localization_readiness_timeout';break
                    continue
                try:
                    tf=buf.lookup_transform('map','camera_init',rclpy.time.Time())
                    transform={'p':[tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z],'q':[tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w]}
                except Exception:continue
                mode_pub.publish(String(data='manual'))
                phase_t=sim;phase=segments[0][0];events.append({'phase':phase,'t':sim})
            if not valid:
                failure='localization_invalid:'+state;break
            name,duration,v,w=segments[index]
            if sim-phase_t>=duration:
                index+=1
                if index==len(segments):break
                phase_t=sim;name,duration,v,w=segments[index];phase=name;events.append({'phase':phase,'t':sim})
            cmd=Twist();cmd.linear.x=v;cmd.angular.z=w;pub.publish(cmd)
            stream.write(json.dumps({'topic':'command','t':sim,'v':v,'w':w})+'\n')
        else:failure='wall_timeout'
    except KeyboardInterrupt:
        failure='diagnostic_interrupted'
    except Exception as e:
        failure=f'diagnostic_error:{type(e).__name__}:{e}'
    finally:
        for _ in range(15):pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.02)
        mode_pub.publish(String(data='disabled'))
        stream.close()
        summary={'diagnostic_only':True,'failure':failure,'completed':index==len(segments),'events':events,'map_to_camera_init':transform,'wall_seconds':time.monotonic()-start,'final_status':state}
        (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
        n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
