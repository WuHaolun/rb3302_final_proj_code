import ast
import math
import pathlib
import sys
import threading
from types import SimpleNamespace as NS

source = pathlib.Path(sys.argv[1])
tree = ast.parse(source.read_text())
driver = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'WaveRoverDriver')
method = next(n for n in driver.body if isinstance(n, ast.FunctionDef) and n.name == '_publish_odom')

class Time:
    def __init__(self, ns): self.nanoseconds = ns
    def __sub__(self, other): return Time(self.nanoseconds - other.nanoseconds)
    def to_msg(self): return self.nanoseconds

def message():
    return NS(header=NS(), child_frame_id='',
              pose=NS(pose=NS(position=NS(), orientation=NS())),
              twist=NS(twist=NS(linear=NS(), angular=NS())))

namespace = {'math': math, 'Odometry': message}
exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)

class Driver:
    _publish_odom = namespace['_publish_odom']
    def __init__(self, yaw=0.0):
        self.feedback_lock = threading.Lock()
        self.dist_L = self.dist_R = 10.0
        self.dist_centre = 100.0
        self.yaw_odom = yaw
        self.prev_dist_L = self.prev_dist_R = None
        self.prev_odom_time = Time(0)
        self.odom_x = self.odom_y = 0.0
        self.t = 0
        self.current_angular = 0.0
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        self.publish_odom_tf = False
        self.messages = []
        self.odom_pub = NS(publish=self.messages.append)
    def get_clock(self): return NS(now=lambda: Time(self.t))
    def _yaw_radians(self, value): return value
    def step(self, left, right, centre):
        self.t += 1_000_000_000
        self.dist_L, self.dist_R, self.dist_centre = left, right, centre
        self._publish_odom()
        return self.messages[-1]

def near(actual, expected):
    assert math.isclose(actual, expected, abs_tol=1e-9), (actual, expected)

d = Driver(); d._publish_odom()
near(d.odom_x, 0.0)
m = d.step(9.9, 9.9, 101.0)
near(m.twist.twist.linear.x, -0.1); near(d.odom_x, -0.1)
m = d.step(10.1, 10.1, 102.0)
near(m.twist.twist.linear.x, 0.2); near(d.odom_x, 0.1)
m = d.step(10.0, 10.2, 103.0)
near(m.twist.twist.linear.x, 0.0); near(d.odom_x, 0.1)
m = d.step(10.3, 10.3, 104.0)
near(m.twist.twist.linear.x, 0.2); near(d.odom_x, 0.3)
m = d.step(10.3, 10.3, 105.0)
near(m.twist.twist.linear.x, 0.0); near(d.odom_x, 0.3)
d = Driver(math.pi / 2); d._publish_odom()
m = d.step(9.9, 9.9, 101.0)
near(d.odom_x, 0.0); near(d.odom_y, -0.1); near(m.twist.twist.linear.x, -0.1)
print('PASS: initial baseline, reverse, forward, in-place turn, unequal wheel travel, stationary, and reverse at 90-degree heading')
