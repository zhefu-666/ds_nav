"""Compose articulated CAD on the existing RMUL force-driven carrier.

Link-local CAD convex hulls follow the actual joints;
this does not implement real omni-wheel traction or a mechanical digital twin.
"""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET


def compose(share, carrier_xml, robot_name):
    share = Path(share)
    root = ET.fromstring(carrier_xml)
    root.set('version', '1.9')
    model = root.find('model')
    model.set('name', robot_name)
    for link in model.findall('link'):
        for visual in list(link.findall('visual')):
            link.remove(visual)
    chassis = model.find("link[@name='chassis']")
    # The CAD links carry their own local collision boxes. A chassis-wide
    # swept volume would continue blocking the lowered robot as if upright.
    for collision in list(chassis.findall('collision')):
        chassis.remove(collision)
    for sensor in list(chassis.findall('sensor')):
        if sensor.get('type') == 'contact':
            chassis.remove(sensor)
    # Low-friction supports are enlarged only in their placement, not radius.
    for link in model.findall('link'):
        if link.get('name', '').endswith('_wheel'):
            xyz = link.find('pose').text.split()
            xyz[0] = '.25' if float(xyz[0]) > 0 else '-.25'
            xyz[1] = '.25' if float(xyz[1]) > 0 else '-.25'
            link.find('pose').text = ' '.join(xyz)
    for joint in model.findall('joint'):
        if joint.get('name', '').endswith('_wheel_joint'):
            xyz = joint.find('pose').text.split()
            xyz[0] = '.25' if float(xyz[0]) > 0 else '-.25'
            xyz[1] = '.25' if float(xyz[1]) > 0 else '-.25'
            joint.find('pose').text = ' '.join(xyz)
    # Use the sensor's default visibility mask: CAD geometry must occlude
    # environmental returns. Excluding robot visuals lets rays pass through
    # the body and is incompatible with the raw measurement contract.
    carrier_only = ET.tostring(root, encoding='unicode')

    cad = ET.parse(share / 'models/huananhu_demo/model.sdf').getroot().find('model')
    for child in list(cad):
        if child.tag == 'link':
            hull_path = share / 'models/huananhu_demo/collision' / (child.get('name') + '_hull.obj')
            if child.get('name') != 'base_link' and not child.get('name').startswith('wheel_'):
                if not hull_path.is_file():
                    raise FileNotFoundError('Rebuild huananhu_kinematic_demo to install CAD hull: ' + str(hull_path))
                # A rotated group box invents high corners absent from CAD.
                # Each convex hull contains the unscaled CAD visual vertices.
                for collision in list(child.findall('collision')):
                    child.remove(collision)
                collision = ET.SubElement(child, 'collision', name='cad_convex_hull')
                mesh = ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh')
                ET.SubElement(mesh, 'uri').text = str(hull_path)
            # Gazebo transforms every local collider with its actual link,
            # including intermediate joint angles.
            for index, collision in enumerate(child.findall('collision')):
                sensor = ET.SubElement(child, 'sensor',
                                       name=f'body_contact_{index}', type='contact')
                ET.SubElement(sensor, 'always_on').text = 'true'
                ET.SubElement(sensor, 'update_rate').text = '20'
                contact = ET.SubElement(sensor, 'contact')
                ET.SubElement(contact, 'collision').text = collision.get('name')
                ET.SubElement(contact, 'topic').text = '/assessment/body_contacts'
            gravity = child.find('gravity')
            if gravity is None:
                gravity = ET.SubElement(child, 'gravity')
            gravity.text = 'true'
            pose = child.find('pose')
            if pose is None:
                pose = ET.SubElement(child, 'pose')
                pose.text = '0 0 0 0 0 0'
            assert not pose.get('relative_to'), pose.attrib
            values = [float(x) for x in pose.text.split()]
            values[2] += .062
            pose.text = ' '.join(str(x) for x in values)
            for visual in child.findall('visual'):
                # 1 is IGN_VISIBILITY_GUI, not the normal geometry mask.
                # Keep the renderer default so sensor rays see the real CAD.
                uri = visual.find('geometry/mesh/uri')
                if uri is not None:
                    uri.text = str(share / 'models/huananhu_demo/meshes' / uri.text.rsplit('/', 1)[1])
            model.append(child)
        elif child.tag == 'joint':
            model.append(child)
        elif child.tag == 'plugin' and child.get('name', '').endswith('JointPositionController'):
            model.append(child)
    mount = ET.SubElement(model, 'joint', name='huananhu_mount', type='fixed')
    ET.SubElement(mount, 'parent').text = 'chassis'
    ET.SubElement(mount, 'child').text = 'base_link'
    ET.SubElement(mount, 'pose', relative_to='chassis').text = '0 0 -.038 0 0 0'
    # SIMULATION_ASSUMPTION: visual_1555 entity origin, not calibrated optical
    # centre. Co-located IMU/lidar are one rigid measurement frame on yaw1.
    yaw1 = model.find("link[@name='yaw1']")
    for name in ('navigation_lidar3d', 'chassis_imu'):
        sensor = chassis.find(f"sensor[@name='{name}']")
        if sensor is None:
            raise ValueError(f'Missing carrier sensor {name}')
        chassis.remove(sensor)
        sensor.find('pose').text = '.0557597190051302 .106643863403111 .163594690492484 -1.5707963267948966 0 0'
        yaw1.append(sensor)
    return ET.tostring(root, encoding='unicode'), carrier_only


def compose_urdf(share, carrier_urdf):
    root = ET.fromstring(carrier_urdf)
    cad = ET.parse(Path(share) / 'urdf/huananhu.urdf').getroot()
    for link in cad.findall('link'):
        hull_path = Path(share) / 'models/huananhu_demo/collision' / (link.get('name') + '_hull.obj')
        if link.get('name') == 'base_link' or link.get('name').startswith('wheel_'):
            continue
        if not hull_path.is_file():
            raise FileNotFoundError(str(hull_path))
        for collision in list(link.findall('collision')):
            link.remove(collision)
        collision = ET.SubElement(link, 'collision')
        ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh', filename=str(hull_path))
    for child in cad:
        root.append(deepcopy(child))
    mount = ET.SubElement(root, 'joint', name='huananhu_mount', type='fixed')
    ET.SubElement(mount, 'parent', link='chassis')
    ET.SubElement(mount, 'child', link='base_link')
    ET.SubElement(mount, 'origin', xyz='0 0 -.038', rpy='0 0 0')
    for joint in list(root.findall('joint')):
        if joint.find('child').get('link') in ('body', 'navigation_laser'):
            root.remove(joint)
    for frame in ('body', 'navigation_laser'):
        joint = ET.SubElement(root, 'joint', name='yaw1_to_' + frame, type='fixed')
        ET.SubElement(joint, 'parent', link='yaw1')
        ET.SubElement(joint, 'child', link=frame)
        ET.SubElement(joint, 'origin', xyz='.0557597190051302 .106643863403111 .163594690492484', rpy='-1.5707963267948966 0 0')
    return ET.tostring(root, encoding='unicode')
