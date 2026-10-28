from pathlib import Path
import json
import math
import os
import signal
import subprocess
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import Costmap
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.srv import GetParameters
from tf2_ros import StaticTransformBroadcaster

backup = Path('/home/ubuntu/rb3302_final_proj_code/backups/local_costmap_height_fix_20261020_170218_SGT')
assert os.environ.get('ROS_DOMAIN_ID') == '114'
assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
rclpy.init()
node = Node('height_filter_isolated_verifier')
tf = StaticTransformBroadcaster(node)
transforms = []
for parent, child, height in [('odom', 'base_link', 0.0), ('base_link', 'laser', 0.09)]:
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.z = height
    t.transform.rotation.w = 1.0
    transforms.append(t)
tf.sendTransform(transforms)
pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        msg = LaserScan()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'laser'
        msg.angle_min = -math.pi
        msg.angle_increment = 2.0 * math.pi / 360
        msg.angle_max = msg.angle_min + 359 * msg.angle_increment
        msg.range_min = 0.15
        msg.range_max = 12.0
        msg.scan_time = 0.1
        msg.ranges = [1.0] * 360
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)

def request(service_type, name, req):
    client = node.create_client(service_type, name)
    try:
        assert client.wait_for_service(timeout_sec=10), 'Service unavailable: ' + name
        future = client.call_async(req)
        end = time.monotonic() + 12
        while not future.done() and time.monotonic() < end:
            pump(0.1)
        assert future.done(), 'Service timed out: ' + name
        return future.result()
    finally:
        node.destroy_client(client)

results = {}
try:
    for label, filename in [('before', 'nav2_params.yaml.before'), ('after', 'nav2_params.yaml.after')]:
        params = yaml.safe_load((backup / filename).read_text())['local_costmap']['local_costmap']['ros__parameters']
        paramfile = Path('/tmp/rb3302_height_verify_' + label + '.yaml')
        paramfile.write_text(yaml.safe_dump({'/local_costmap/local_costmap': {'ros__parameters': params}}))
        stats = {'messages': 0, 'max_nonzero_cells': 0, 'max_lethal_cells': 0, 'max_inflation_cells': 0}
        def callback(msg):
            stats['messages'] += 1
            stats['max_nonzero_cells'] = max(stats['max_nonzero_cells'], sum(v > 0 for v in msg.data))
            stats['max_lethal_cells'] = max(stats['max_lethal_cells'], sum(v == 254 for v in msg.data))
            stats['max_inflation_cells'] = max(stats['max_inflation_cells'], sum(0 < v < 254 for v in msg.data))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        sub = node.create_subscription(Costmap, '/local_costmap/costmap_raw', callback, qos)
        logfile = (backup / ('isolated_' + label + '.log')).open('w')
        process = subprocess.Popen(['/opt/ros/jazzy/lib/nav2_costmap_2d/nav2_costmap_2d', '--ros-args', '-r', '__node:=local_costmap', '-r', '__ns:=/local_costmap', '--params-file', str(paramfile)], stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            pump(1)
            req = ChangeState.Request()
            req.transition.id = 1
            assert request(ChangeState, '/local_costmap/local_costmap/change_state', req).success, 'Configure failed'
            req.transition.id = 3
            assert request(ChangeState, '/local_costmap/local_costmap/change_state', req).success, 'Activate failed'
            reqp = GetParameters.Request()
            reqp.names = ['obstacle_layer.scan.max_obstacle_height']
            stats['effective_scan_max_obstacle_height'] = request(GetParameters, '/local_costmap/local_costmap/get_parameters', reqp).values[0].double_value
            pump(4.5)
            assert process.poll() is None, 'Costmap process exited unexpectedly'
            assert stats['messages'] >= 2, 'Insufficient costmap observations'
            results[label] = dict(stats)
            print(label, json.dumps(stats), flush=True)
        finally:
            if process.poll() is None:
                for transition in (2, 4):
                    try:
                        req = ChangeState.Request()
                        req.transition.id = transition
                        request(ChangeState, '/local_costmap/local_costmap/change_state', req)
                    except Exception as exc:
                        print('Test cleanup:', str(exc), flush=True)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=2)
            logfile.close()
            node.destroy_subscription(sub)
            paramfile.unlink(missing_ok=True)
            pump(0.5)
    assert results['before']['max_nonzero_cells'] == 0
    assert results['after']['max_lethal_cells'] > 0
    assert results['after']['max_inflation_cells'] > 0
    results['validation'] = 'PASS: installed Nav2 costmap, isolated synthetic LaserScan at z=0.09 m; not a physical avoidance test'
    results['domain'] = 114
    results['commands_published'] = False
    (backup / 'verification.json').write_text(json.dumps(results, indent=2) + '\n')
    print(json.dumps(results, indent=2), flush=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
