"""Configuration files must agree with galbot_s1_control/robot_config.py."""
import os
import xml.etree.ElementTree as ET

import yaml

from galbot_s1_control import robot_config as rc
from galbot_s1_control.scene import Scene

CONTROLLED = {j for _, (_, js) in rc.GROUPS.items() for j in js} | set(rc.STEERING_JOINTS) | set(rc.WHEEL_JOINTS)


def test_initial_positions_yaml(src):
    with open(os.path.join(src, 'galbot_s1_gazebo', 'config', 'initial_positions.yaml')) as f:
        data = yaml.safe_load(f)
    assert data == rc.INITIAL_POSITIONS


def test_ros2_control_tag(gz_urdf):
    root = ET.fromstring(gz_urdf)
    rc_tag = root.find('ros2_control')
    assert rc_tag is not None
    joints = {j.get('name'): j for j in rc_tag.findall('joint')}
    assert set(joints) == CONTROLLED
    for name, value in rc.INITIAL_POSITIONS.items():
        init = [p for s in joints[name].findall('state_interface') if s.get('name') == 'position'
                for p in s.findall('param') if p.get('name') == 'initial_value']
        assert init and abs(float(init[0].text) - value) < 1e-6, name
    # mimic joints must be commanded explicitly in simulation
    assert root.findall('.//mimic') == []


def test_classic_transmissions(classic_urdf):
    root = ET.fromstring(classic_urdf)
    joints = {t.find('joint').get('name') for t in root.findall('transmission')}
    assert joints == CONTROLLED
    assert root.find('ros2_control') is None


def test_description_has_mimic(description_urdf):
    root = ET.fromstring(description_urdf)
    assert len(root.findall('.//mimic')) == 4


def test_controller_files(src):
    cfg = os.path.join(src, 'galbot_s1_gazebo', 'config')
    with open(os.path.join(cfg, 'ros1_controllers.yaml')) as f:
        r1 = yaml.safe_load(f)
    with open(os.path.join(cfg, 'ros2_controllers.yaml')) as f:
        r2 = yaml.safe_load(f)
    expected = {c: js for _, (c, js) in rc.GROUPS.items()}
    expected['steering_controller'] = rc.STEERING_JOINTS
    expected['wheel_controller'] = rc.WHEEL_JOINTS
    for name, joints in expected.items():
        assert r1[name]['joints'] == joints
        assert r2[name]['ros__parameters']['joints'] == joints
        assert name in r2['controller_manager']['ros__parameters']


def test_worlds_match_scene(src):
    scene = Scene.from_file(os.path.join(src, 'galbot_s1_gazebo', 'config', 'scene.yaml'))
    for fname in ('table_scene.world', 'table_scene.sdf'):
        root = ET.parse(os.path.join(src, 'galbot_s1_gazebo', 'worlds', fname)).getroot()
        models = {m.get('name'): m for m in root.iter('model')}
        for obj in scene.objects:
            assert obj.name in models, (fname, obj.name)
            pose = [float(v) for v in models[obj.name].find('pose').text.split()]
            assert abs(pose[0] - obj.position[0]) < 1e-6 and abs(pose[1] - obj.position[1]) < 1e-6
        assert scene.table_name in models
