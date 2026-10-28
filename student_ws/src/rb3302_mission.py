#!/usr/bin/env python3
"""RB3302 graded run: drive S0 -> S1 -> S2 -> S3 -> S4 by landmark letter.

The course is a 4 x 6 grid of 0.6 m cubes labelled A..X:

        N
    A B C D E F
    G H I J K L
    M N O P Q R
    S T U V W X
        S

Nav2 wants metres in the map frame; the graded run is given to you in letters.
This bridges the two with a two-point calibration, then sends the four
landmarks to /follow_waypoints. The 3 s stop at each landmark comes from
WaitAtWaypoint in nav2_params.yaml, not from here.

Usage
-----
Once per map, tell it where the grid sits:

    python3 rb3302_mission.py calibrate

Check the table without moving the robot:

    python3 rb3302_mission.py run --start S --bearing N --seq H J Q T --dry-run

The graded run:

    python3 rb3302_mission.py run --start S --bearing N --seq H J Q T
"""
import argparse
import math
import os
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints

LETTERS = 'ABCDEFGHIJKLMNOPQRSTUVWX'
NROWS, NCOLS = 4, 6
CALIB_PATH = os.path.expanduser(
    '~/rb3302_final_proj_code/student_ws/course_calib.yaml')

# Bearing -> (north component, east component).
BEARINGS = {
    'N': (1, 0), 'NE': (1, 1), 'E': (0, 1), 'SE': (-1, 1),
    'S': (-1, 0), 'SW': (-1, -1), 'W': (0, -1), 'NW': (1, -1),
}


class Course:
    """Maps a landmark letter to a pose in the map frame.

    The grid is a regular lattice, so it is fully described by the centre of A
    and two per-cell vectors: east and south. Those come from the centres of
    two opposite corners, A (row 0, col 0) and X (row 3, col 5):

        X = A + 5*east + 3*south,  south = east rotated -90 degrees

    which is a 2x2 solve with determinant 34.
    """

    def __init__(self, a_xy, x_xy):
        self.a = a_xy
        dx = x_xy[0] - a_xy[0]
        dy = x_xy[1] - a_xy[1]
        ex = (5.0 * dx - 3.0 * dy) / 34.0
        ey = (3.0 * dx + 5.0 * dy) / 34.0
        self.east = (ex, ey)
        self.south = (ey, -ex)
        self.north = (-ey, ex)

    @property
    def cell_size(self):
        return math.hypot(*self.east)

    def centre(self, letter):
        letter = letter.upper()
        if letter not in LETTERS:
            raise ValueError(f"'{letter}' is not a landmark A-X")
        i = LETTERS.index(letter)
        row, col = divmod(i, NCOLS)
        return (self.a[0] + col * self.east[0] + row * self.south[0],
                self.a[1] + col * self.east[1] + row * self.south[1])

    def yaw(self, bearing):
        bearing = bearing.upper()
        if bearing not in BEARINGS:
            raise ValueError(
                f"'{bearing}' is not one of {', '.join(BEARINGS)}")
        n, e = BEARINGS[bearing]
        vx = n * self.north[0] + e * self.east[0]
        vy = n * self.north[1] + e * self.east[1]
        return math.atan2(vy, vx)

    def save(self, path):
        with open(path, 'w') as fh:
            fh.write('# RB3302 course calibration (map-frame metres).\n')
            fh.write('# Centres of the two opposite corner squares.\n')
            fh.write(f'a_x: {self.a[0]:.4f}\na_y: {self.a[1]:.4f}\n')
            fh.write(f'x_x: {self.a[0] + 5 * self.east[0] + 3 * self.south[0]:.4f}\n')
            fh.write(f'x_y: {self.a[1] + 5 * self.east[1] + 3 * self.south[1]:.4f}\n')

    @classmethod
    def load(cls, path):
        if not os.path.exists(path):
            sys.exit(f"No calibration at {path}\n"
                     "Run:  python3 rb3302_mission.py calibrate")
        vals = {}
        with open(path) as fh:
            for line in fh:
                line = line.split('#', 1)[0].strip()
                if ':' in line:
                    k, v = line.split(':', 1)
                    vals[k.strip()] = float(v)
        return cls((vals['a_x'], vals['a_y']), (vals['x_x'], vals['x_y']))


def quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_pose(node, x, y, yaw, frame='map'):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z, msg.pose.orientation.w = quat(yaw)
    return msg


def spin_for(node, seconds):
    end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def cmd_calibrate(args):
    rclpy.init()
    node = Node('rb3302_calibrate')
    clicks = []

    def on_click(msg):
        clicks.append((msg.point.x, msg.point.y))
        print(f'  got ({msg.point.x:.3f}, {msg.point.y:.3f})')

    node.create_subscription(PointStamped, '/clicked_point', on_click, 10)

    print('\nIn RViz: pick the "Publish Point" tool, then click the centre of')
    print('the two opposite corner squares of the course on the map.\n')
    for want, label in ((1, 'A  (north-west corner)'),
                        (2, 'X  (south-east corner)')):
        print(f'Click the centre of square {label} ...')
        while len(clicks) < want:
            rclpy.spin_once(node, timeout_sec=0.2)

    course = Course(clicks[0], clicks[1])
    size = course.cell_size
    print(f'\nDerived cell size: {size:.3f} m  (expected ~0.60)')
    if not 0.50 <= size <= 0.70:
        print('That is a long way off 0.6 m — you probably clicked the wrong')
        print('squares, or clicked them in the wrong order. Not saving.')
        return 1

    heading = math.degrees(math.atan2(course.north[1], course.north[0]))
    print(f'Course north points {heading:.1f} deg in the map frame.')
    course.save(CALIB_PATH)
    #args = ap.parse_args()
    print(f'Saved {CALIB_PATH}\n')
    print_table(course)
    node.destroy_node()
    rclpy.shutdown()
    return 0


def print_table(course):
    print('Landmark centres (map frame):')
    for row in range(NROWS):
        cells = []
        for col in range(NCOLS):
            letter = LETTERS[row * NCOLS + col]
            x, y = course.centre(letter)
            cells.append(f'{letter} ({x:6.2f},{y:6.2f})')
        print('  ' + '  '.join(cells))


def seed_amcl(node, x, y, yaw):
    qos = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = quat(yaw)
    # The rules only promise the bearing to within +/-45 deg, so give AMCL a
    # yaw spread that actually covers that rather than stacking every particle
    # on one heading.
    msg.pose.covariance[0] = 0.10
    msg.pose.covariance[7] = 0.10
    msg.pose.covariance[35] = 0.25
    deadline = node.get_clock().now().nanoseconds + 5_000_000_000
    while pub.get_subscription_count() == 0:
        if node.get_clock().now().nanoseconds > deadline:
            node.get_logger().warn('nothing subscribed to /initialpose — '
                                   'is amcl up?')
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    pub.publish(msg)
    node.get_logger().info('seeded AMCL, letting particles settle')
    spin_for(node, 3.0)


def cmd_run(args):
    course = Course.load(CALIB_PATH)

    for letter in [args.start] + args.seq:
        if letter.upper() not in LETTERS:
            sys.exit(f"'{letter}' is not a landmark A-X")
    if args.bearing.upper() not in BEARINGS:
        sys.exit(f"'{args.bearing}' is not one of {', '.join(BEARINGS)}")

    sx, sy = course.centre(args.start)
    syaw = course.yaw(args.bearing)
    targets = [(L.upper(), *course.centre(L)) for L in args.seq]

    print(f'\nStart  {args.start.upper()} facing {args.bearing.upper()}  '
          f'-> ({sx:.2f}, {sy:.2f}) yaw {math.degrees(syaw):.0f} deg')
    for n, (letter, x, y) in enumerate(targets, 1):
        print(f'  S{n}  {letter}  -> ({x:.2f}, {y:.2f})')
    print()

    if args.dry_run:
        print_table(course)
        return 0

    rclpy.init()
    node = Node('rb3302_mission')

    if not args.no_seed:
        seed_amcl(node, sx, sy, syaw)

    goal = FollowWaypoints.Goal()
    goal.poses = [make_pose(node, x, y, syaw) for _, x, y in targets]

    client = ActionClient(node, FollowWaypoints, 'follow_waypoints')
    node.get_logger().info('waiting for /follow_waypoints ...')
    if not client.wait_for_server(timeout_sec=20.0):
        node.get_logger().error(
            '/follow_waypoints never appeared. Is planning.launch.py up and '
            'did lifecycle_manager print "Nav2 is ready for use"?')
        return 1

    last = {'i': None}

    def on_feedback(fb):
        i = fb.feedback.current_waypoint
        if i != last['i']:
            last['i'] = i
            if i < len(targets):
                node.get_logger().info(
                    f'heading to S{i + 1} = {targets[i][0]}')

    send = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send)
    handle = send.result()
    if handle is None or not handle.accepted:
        node.get_logger().error('goal rejected')
        return 1

    result_future = handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future)
    except KeyboardInterrupt:
        node.get_logger().warn('interrupted — cancelling')
        rclpy.spin_until_future_complete(node, handle.cancel_goal_async())
        return 130

    result = result_future.result()
    missed = list(getattr(result.result, 'missed_waypoints', []))
    if missed:
        names = ', '.join(f'S{i + 1}={targets[i][0]}' for i in missed)
        node.get_logger().warn(f'did not reach: {names}')
    else:
        node.get_logger().info('reached all four landmarks')
    node.get_logger().info(f'status={result.status} (4 = succeeded)')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if result.status == 4 else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)

    sub.add_parser('calibrate', help='click A and X in RViz to locate the grid')

    run = sub.add_parser('run', help='seed the start pose and drive the sequence')
    run.add_argument('--start', required=True, metavar='S0',
                     help='starting square, A-X')
    run.add_argument('--bearing', required=True, metavar='B0',
                     help='N, NE, E, SE, S, SW, W or NW')
    run.add_argument('--seq', required=True, nargs=4, metavar='S',
                     help='the four landmarks in order')
    run.add_argument('--dry-run', action='store_true',
                     help='print the coordinates and exit without moving')
    run.add_argument('--no-seed', action='store_true',
                     help='skip /initialpose (you set it in RViz yourself)')

    args = ap.parse_args()
    return cmd_calibrate(args) if args.cmd == 'calibrate' else cmd_run(args)


if __name__ == '__main__':
    sys.exit(main())
