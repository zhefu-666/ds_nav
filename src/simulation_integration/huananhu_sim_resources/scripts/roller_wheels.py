"""Explicit passive cylinder rollers; motor forces act only on hub joints."""
from pathlib import Path
import json,math
import xml.etree.ElementTree as E
import numpy as np


def add_rollers(xml,share,friction=.8,payload_kg=0.):
    root=E.fromstring(xml);model=root.find('model')
    cfg=json.loads((Path(share)/'config/model.json').read_text())
    names={w['joint'] for w in cfg['wheels']}
    for p in list(model.findall('plugin')):
        if p.get('filename')=='MecanumDrive2' or p.findtext('joint_name') in names:
            model.remove(p)
    for link in list(model.findall('link')):
        if link.get('name') in ['front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel']:
            model.remove(link)
    for j in list(model.findall('joint')):
        if j.get('name') in ['front_left_wheel_joint','front_right_wheel_joint','rear_left_wheel_joint','rear_right_wheel_joint']:
            model.remove(j)
    drive=E.SubElement(model,'plugin',filename='WheelTractionDrive',name='kaohe::WheelTractionDrive')
    E.SubElement(drive,'max_torque').text='1.2'
    joints={j['name']:j for j in cfg['joints']}
    up=np.array([0.,1.,0.])
    for wheel in cfg['wheels']:
        spec=joints[wheel['joint']];hub=spec['child'];axle=np.array(spec['axis']);roll=np.cross(axle,up)
        body_axis=np.array(wheel['axis_ros']);center=np.array(wheel['center_ros']);rolling=np.cross(body_axis,[0,0,1]);r=wheel['radius']
        row=[rolling[0]/r,rolling[1]/r,float(rolling@np.cross([0,0,1],center))/r]
        w=E.SubElement(drive,'wheel');E.SubElement(w,'joint').text=wheel['joint'];E.SubElement(w,'jacobian').text=' '.join(map(str,row))
        hub_link=model.find(f"link[@name='{hub}']")
        # The hub's full-wheel box would fill the passive roller gaps and
        # bypass roller traction. Wheel contact remains on the 64 cylinders.
        for collision in list(hub_link.findall('collision')): hub_link.remove(collision)
        for sensor in list(hub_link.findall('sensor')):
            if sensor.get('type') == 'contact': hub_link.remove(sensor)
        inert=hub_link.find('inertial');old=float(inert.findtext('mass'));inert.find('mass').text=str(old-.006*16)
        for entry in inert.find('inertia'):entry.text=str(float(entry.text)*(old-.006*16)/old)
        for i in range(16):
            theta=2*math.pi*i/16;radial=up*math.cos(theta)+roll*math.sin(theta);tangent=-up*math.sin(theta)+roll*math.cos(theta)
            center_local=(r-.009)*radial
            rotation=np.column_stack([axle,np.cross(tangent,axle),tangent])
            rpy=[math.atan2(rotation[2,1],rotation[2,2]),math.asin(-rotation[2,0]),math.atan2(rotation[1,0],rotation[0,0])]
            name=f'roller_{hub}_{i}';link=E.SubElement(model,'link',name=name)
            E.SubElement(link,'pose',relative_to=hub).text=' '.join(map(str,list(center_local)+rpy))
            inert=E.SubElement(link,'inertial');E.SubElement(inert,'mass').text='.006';I=E.SubElement(inert,'inertia')
            for k,v in {'ixx':.006*(3*.009**2+.021**2)/12,'iyy':.006*(3*.009**2+.021**2)/12,'izz':.006*.009**2/2,'ixy':0,'ixz':0,'iyz':0}.items():E.SubElement(I,k).text=str(v)
            coll=E.SubElement(link,'collision',name='tread');shape=E.SubElement(E.SubElement(coll,'geometry'),'cylinder');E.SubElement(shape,'radius').text='.009';E.SubElement(shape,'length').text='.021'
            ode=E.SubElement(E.SubElement(E.SubElement(coll,'surface'),'friction'),'ode');E.SubElement(ode,'mu').text=str(friction);E.SubElement(ode,'mu2').text=str(friction)
            if i%4==0:
                sensor=E.SubElement(link,'sensor',name='roller_contact',type='contact');E.SubElement(sensor,'always_on').text='true';E.SubElement(sensor,'update_rate').text='20';c=E.SubElement(sensor,'contact');E.SubElement(c,'collision').text='tread';E.SubElement(c,'topic').text='/assessment/wheel_contacts'
            joint=E.SubElement(model,'joint',name=name+'_joint',type='revolute');E.SubElement(joint,'parent').text=hub;E.SubElement(joint,'child').text=name;E.SubElement(joint,'pose',relative_to=name).text='0 0 0 0 0 0';axis=E.SubElement(joint,'axis');E.SubElement(axis,'xyz').text='0 0 1';lim=E.SubElement(axis,'limit');E.SubElement(lim,'lower').text='-1e16';E.SubElement(lim,'upper').text='1e16';E.SubElement(lim,'effort').text='10';E.SubElement(lim,'velocity').text='1000';d=E.SubElement(axis,'dynamics');E.SubElement(d,'damping').text='.00001'
    if payload_kg > 0:
        link=E.SubElement(model,'link',name='test_payload')
        E.SubElement(link,'pose',relative_to='chassis').text='.10 0 .18 0 0 0'
        inert=E.SubElement(link,'inertial');E.SubElement(inert,'mass').text=str(payload_kg);I=E.SubElement(inert,'inertia')
        for k,v in {'ixx':payload_kg*(.2**2+.15**2)/12,'iyy':payload_kg*(.2**2+.15**2)/12,'izz':payload_kg*(.2**2+.2**2)/12,'ixy':0,'ixz':0,'iyz':0}.items():E.SubElement(I,k).text=str(v)
        j=E.SubElement(model,'joint',name='test_payload_joint',type='fixed');E.SubElement(j,'parent').text='chassis';E.SubElement(j,'child').text='test_payload';E.SubElement(j,'pose',relative_to='chassis').text='.10 0 .18 0 0 0'
    return E.tostring(root,encoding='unicode')


def append_roller_urdf(urdf,sdf):
    root=E.fromstring(urdf);model=E.fromstring(sdf).find('model')
    # Remove the carrier's four invisible fixed support branches.
    for child in list(root):
        name=child.get('name','')
        if name in ['front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel','front_left_wheel_joint','front_right_wheel_joint','rear_left_wheel_joint','rear_right_wheel_joint']:root.remove(child)
    for link in root.findall('link'):
        if link.get('name') in ['wheel_1', 'wheel_2', 'wheel_3', 'wheel_4']:
            for collision in list(link.findall('collision')): link.remove(collision)
    for link in model.findall('link'):
        name=link.get('name','')
        if name=='test_payload':
            E.SubElement(root,'link',name=name);j=E.SubElement(root,'joint',name='test_payload_joint',type='fixed');E.SubElement(j,'parent',link='chassis');E.SubElement(j,'child',link=name);E.SubElement(j,'origin',xyz='.10 0 .18',rpy='0 0 0');continue
        if not name.startswith('roller_'):continue
        urdf_link=E.SubElement(root,'link',name=name)
        collision=E.SubElement(urdf_link,'collision',name='tread')
        geometry=E.SubElement(collision,'geometry')
        E.SubElement(geometry,'cylinder',radius='.009',length='.021')
        j=E.SubElement(root,'joint',name=name+'_joint',type='continuous');E.SubElement(j,'parent',link=link.find('pose').get('relative_to'));E.SubElement(j,'child',link=name);p=link.findtext('pose').split();E.SubElement(j,'origin',xyz=' '.join(p[:3]),rpy=' '.join(p[3:]));E.SubElement(j,'axis',xyz='0 0 1')
    return E.tostring(root,encoding='unicode')
