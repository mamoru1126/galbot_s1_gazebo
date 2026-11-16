#!/usr/bin/env python3
"""Generate the Gazebo Classic and Gazebo (gz-sim) worlds from config/scene.yaml.

    python3 scripts/generate_worlds.py            # writes worlds/table_scene.{world,sdf}

Only needs PyYAML. Run it again after editing config/scene.yaml.
"""
import math
import os

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def f(v):
    return ('%.5f' % v).rstrip('0').rstrip('.') if isinstance(v, float) else str(v)


def vec(*vals):
    return ' '.join(f(float(v)) for v in vals)


def box_inertia(m, x, y, z):
    return (m * (y * y + z * z) / 12.0, m * (x * x + z * z) / 12.0, m * (x * x + y * y) / 12.0)


def cyl_inertia(m, r, h):
    ixx = m * (3 * r * r + h * h) / 12.0
    return (ixx, ixx, m * r * r / 2.0)


def material(color, gz):
    r, g, b = color
    amb = vec(r * 0.7, g * 0.7, b * 0.7, 1)
    dif = vec(r, g, b, 1)
    return ('<material><ambient>%s</ambient><diffuse>%s</diffuse>'
            '<specular>0.2 0.2 0.2 1</specular></material>' % (amb, dif))


def geometry(shape, dims):
    if shape == 'box':
        return '<box><size>%s</size></box>' % vec(*dims)
    return '<cylinder><radius>%s</radius><length>%s</length></cylinder>' % (f(float(dims[0])), f(float(dims[1])))


def static_box_model(name, pose, parts, color, gz):
    """parts: list of (part_name, xyz, size)."""
    out = ['    <model name="%s">' % name, '      <static>true</static>',
           '      <pose>%s</pose>' % vec(*pose), '      <link name="link">']
    for pname, xyz, size in parts:
        out += ['        <collision name="%s_collision">' % pname,
                '          <pose>%s 0 0 0</pose>' % vec(*xyz),
                '          <geometry>%s</geometry>' % geometry('box', size),
                '          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>',
                '        </collision>',
                '        <visual name="%s_visual">' % pname,
                '          <pose>%s 0 0 0</pose>' % vec(*xyz),
                '          <geometry>%s</geometry>' % geometry('box', size),
                '          ' + material(color, gz),
                '        </visual>']
    out += ['      </link>', '    </model>']
    return out


def object_model(o, z, gz):
    name = o['name']
    m = float(o.get('mass', 0.2))
    if o['shape'] == 'cylinder':
        dims = (float(o['radius']), float(o['length']))
        I = cyl_inertia(m, *dims)
    else:
        dims = tuple(float(v) for v in o['size'])
        I = box_inertia(m, *dims)
    x, y = o['position']
    out = ['    <model name="%s">' % name,
           '      <pose>%s</pose>' % vec(x, y, z, 0, 0, o.get('yaw', 0.0)),
           '      <link name="link">',
           '        <inertial>',
           '          <mass>%s</mass>' % f(m),
           '          <inertia><ixx>%.3e</ixx><ixy>0</ixy><ixz>0</ixz><iyy>%.3e</iyy><iyz>0</iyz><izz>%.3e</izz></inertia>' % I,
           '        </inertial>',
           '        <collision name="collision">',
           '          <geometry>%s</geometry>' % geometry(o['shape'], dims),
           '          <surface>',
           '            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>',
           '            <contact><ode><kp>1e5</kp><kd>10</kd><max_vel>0.1</max_vel><min_depth>0.001</min_depth></ode></contact>',
           '          </surface>',
           '        </collision>',
           '        <visual name="visual">',
           '          <geometry>%s</geometry>' % geometry(o['shape'], dims),
           '          ' + material(o.get('color', (0.8, 0.8, 0.8)), gz),
           '        </visual>',
           '      </link>']
    if gz:
        out += ['      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">',
                '        <odom_frame>odom</odom_frame>',
                '        <robot_base_frame>%s</robot_base_frame>' % name,
                '        <odom_topic>/galbot/objects/%s/odom</odom_topic>' % name,
                '        <tf_topic>/galbot/objects/%s/tf</tf_topic>' % name,
                '        <odom_publish_frequency>20</odom_publish_frequency>',
                '        <dimensions>3</dimensions>',
                '      </plugin>']
    out.append('    </model>')
    return out


def overview_camera(cam, gz):
    if not cam:
        return []
    out = ['    <model name="overview_camera">', '      <static>true</static>',
           '      <pose>%s</pose>' % vec(*cam['pose']), '      <link name="link">',
           '        <sensor name="overview_camera" type="camera">',
           '          <always_on>true</always_on>',
           '          <update_rate>10</update_rate>',
           '          <visualize>false</visualize>']
    if gz:
        out.append('          <topic>/galbot/overview_camera/image_raw</topic>')
    out += ['          <camera>',
            '            <horizontal_fov>%s</horizontal_fov>' % f(float(cam.get('hfov', 1.1))),
            '            <image><width>%d</width><height>%d</height><format>R8G8B8</format></image>' % (
                int(cam.get('width', 640)), int(cam.get('height', 480))),
            '            <clip><near>0.05</near><far>30</far></clip>',
            '          </camera>']
    if not gz:
        out += ['          <plugin name="overview_camera_plugin" filename="libgazebo_ros_camera.so">',
                '            <alwaysOn>true</alwaysOn>',
                '            <updateRate>0.0</updateRate>',
                '            <cameraName>galbot/overview_camera</cameraName>',
                '            <imageTopicName>image_raw</imageTopicName>',
                '            <cameraInfoTopicName>camera_info</cameraInfoTopicName>',
                '            <frameName>overview_camera</frameName>',
                '          </plugin>']
    out += ['        </sensor>', '      </link>', '    </model>']
    return out


def table_parts(t):
    dx, dy, h = t['size']
    th = float(t.get('top_thickness', 0.04))
    leg = 0.05
    parts = [('top', (0, 0, h - th / 2.0), (dx, dy, th))]
    lx, ly = dx / 2.0 - leg, dy / 2.0 - leg
    for i, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        parts.append(('leg%d' % i, (sx * lx, sy * ly, (h - th) / 2.0), (leg, leg, h - th)))
    return parts


def world(scene, gz):
    t = scene['table']
    out = ['<?xml version="1.0"?>',
           '<!-- GENERATED by scripts/generate_worlds.py from config/scene.yaml - do not edit by hand -->',
           '<sdf version="%s">' % ('1.9' if gz else '1.6'),
           '  <world name="%s">' % (scene.get('world_name', 'galbot_world') if gz else 'default')]
    if gz:
        out += ['    <physics name="1ms" type="ignored">',
                '      <max_step_size>0.001</max_step_size>',
                '      <real_time_factor>1.0</real_time_factor>',
                '    </physics>',
                '    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>',
                '    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>',
                '    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>',
                '    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">',
                '      <render_engine>ogre2</render_engine>',
                '    </plugin>',
                '    <!-- attach / detach grasped objects, reset object poses (galbot_s1_gazebo/src/gz) -->',
                '    <plugin filename="galbot_sim_helper_gz" name="galbot_s1_gazebo::SimHelper"/>',
                '    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.8 0.85 0.9 1</background>'
                '<shadows>true</shadows></scene>']
    else:
        out += ['    <physics type="ode">',
                '      <max_step_size>0.001</max_step_size>',
                '      <real_time_factor>1</real_time_factor>',
                '      <real_time_update_rate>1000</real_time_update_rate>',
                '      <ode><solver><type>quick</type><iters>50</iters><sor>1.3</sor></solver>',
                '        <constraints><cfm>0</cfm><erp>0.2</erp><contact_max_correcting_vel>100</contact_max_correcting_vel>'
                '<contact_surface_layer>0.001</contact_surface_layer></constraints></ode>',
                '    </physics>',
                '    <!-- attach / detach grasped objects, object poses, reset (galbot_s1_gazebo/src/classic) -->',
                '    <plugin name="galbot_sim_helper" filename="libgalbot_sim_helper_classic.so">',
                '      <publish_models>%s</publish_models>' % ' '.join(o['name'] for o in scene['objects']),
                '      <publish_rate>20</publish_rate>',
                '    </plugin>',
                '    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.8 0.85 0.9 1</background>'
                '<shadows>true</shadows></scene>',
                '    <gui><camera name="user_camera"><pose>-1.2 -2.6 2.4 0 0.42 0.95</pose></camera></gui>']
    out += ['    <light type="directional" name="sun">',
            '      <cast_shadows>true</cast_shadows>',
            '      <pose>0 0 10 0 0 0</pose>',
            '      <diffuse>0.85 0.85 0.85 1</diffuse>',
            '      <specular>0.2 0.2 0.2 1</specular>',
            '      <direction>-0.4 0.3 -0.9</direction>',
            '    </light>',
            '    <model name="ground_plane">',
            '      <static>true</static>',
            '      <link name="link">',
            '        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>',
            '          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>',
            '        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>',
            '          <material><ambient>0.55 0.55 0.52 1</ambient><diffuse>0.7 0.7 0.67 1</diffuse>'
            '<specular>0.1 0.1 0.1 1</specular></material></visual>',
            '      </link>',
            '    </model>']
    cx, cy = t['center']
    out += static_box_model(t.get('name', 'table'), (cx, cy, 0, 0, 0, 0), table_parts(t),
                            (0.62, 0.45, 0.3), gz)
    h = float(t['size'][2])
    tray = scene.get('tray')
    if tray:
        sx, sy, sz = tray['size']
        tx, ty = tray['center']
        out += static_box_model(tray.get('name', 'tray'), (tx, ty, h, 0, 0, 0),
                                [('plate', (0, 0, sz / 2.0), (sx, sy, sz))],
                                tray.get('color', (0.15, 0.15, 0.18)), gz)
    for o in scene['objects']:
        height = float(o['length']) if o['shape'] == 'cylinder' else float(o['size'][2])
        out += object_model(o, h + height / 2.0 + 0.001, gz)
    out += overview_camera(scene.get('overview_camera'), gz)
    out += ['  </world>', '</sdf>']
    return '\n'.join(out) + '\n'


def main():
    with open(os.path.join(PKG, 'config', 'scene.yaml')) as fh:
        scene = yaml.safe_load(fh)
    os.makedirs(os.path.join(PKG, 'worlds'), exist_ok=True)
    for gz, fname in ((False, 'table_scene.world'), (True, 'table_scene.sdf')):
        path = os.path.join(PKG, 'worlds', fname)
        with open(path, 'w') as fh:
            fh.write(world(scene, gz))
        print('wrote', path)


if __name__ == '__main__':
    main()
