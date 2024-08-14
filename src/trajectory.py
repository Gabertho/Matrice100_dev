#!/usr/bin/python3
import rospy
import math
import sys

#
# Take current position as atart.  button0 is greeb manual, yellow=3 auto
#

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, Vector3Stamped, Vector3, Point, Quaternion
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64, Header, ColorRGBA, Empty
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Joy, LaserScan, BatteryState

from goto_trajectory import GotoTrajectory
from spline_trajectory import SplineTrajectory
from eight_shape_trajectory import EightShapeTrajectory  


from optparse import OptionParser
from threading import Lock, Event

inside_timer_callback = False
current_pose = PoseStamped()
current_pose.pose.position.x = 0.0
current_pose.pose.position.y = 0.0
current_pose.pose.position.z = 2.5
have_current_pose = False
counter = 0
set_initial_position_flag = True
afull_trajectory_flag = False

parser = OptionParser()
parser.add_option("", "--dt", action="store", dest="dt", type="float", default=0.02, help='Perions, default 0.02')
parser.add_option("", "--trajectory_type", action="store", dest="trajectory_type", type="string", help="Trajectory type (e.g., sinusoidal, step_z, step_xyz, spline, long_spline, rectangle, hexagon)", default="go_to_point")
(options, args) = parser.parse_args()

#Joystick callback: enable/disable trajectory tracking and move target with joystick.
def joy_callback(data):
    global set_initial_position_flag, full_trajectory_flag

    if not data.buttons[9] and not data.buttons[10]:
        trajectory.have_changed_index = False

    if data.buttons[9]:
        trajectory.previous_target()

    if data.buttons[10]:
        trajectory.next_target()

    if data.buttons[2]:
        full_trajectory_flag = False
   
    if data.buttons[3]:
        print("Full trejectory")
        full_trajectory_flag = True

    if data.buttons[6]:
        trajectory.enable()
    else:
        trajectory.disable()
        # trajectory.reset(current_pose.pose.position.x, current_pose.pose.position.z, current_pose.pose.position.z)
        trajectory.have_initial_position = False
        set_initial_position_flag = True

    factor = 8.0
    trajectory.move_target(-data.axes[3]/factor, data.axes[4]/factor, data.axes[1]*data.buttons[4]/factor)

def notify_pose_callback(data):
    trajectory.notify_position(data.pose.position.x, data.pose.position.y, data.pose.position.z)
    
# Pose callback: gets x,y,z position and set trajectory initial position (if flag enabled) to it.
def pose_callback(data):
    global current_pose
    global set_initial_position_flag
    # print("pose_callback:", data)
    current_pose = data
    if set_initial_position_flag:
        trajectory.set_initial_position(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z)
        set_initial_position_flag = False

def set_target_callback(data):
    trajectory.set_target(data.x, data.y, data.z)

def get_sphere_marker(id, x, y, z, current_target=False):
    m = Marker()
    m.header.frame_id = "world"
    m.header.stamp = rospy.Time.now()
    m.ns = "targets"
    m.id = id
    m.type = m.SPHERE
    m.action = 0
    m.lifetime = rospy.Duration(0)
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.pose.orientation.x = 0.0
    m.pose.orientation.y = 0.0
    m.pose.orientation.z = 0.0
    m.pose.orientation.w = 1.0
    if current_target:
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
    else:
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
    m.color.a = 1.0
    m.scale.x = 0.3
    m.scale.y = 0.3
    m.scale.z = 0.3
    return m

def display_targets():
    try:
        targets = trajectory.targets
        id = 1
        target_index = trajectory.target_index
        ma = MarkerArray()
        for index, target in enumerate(targets):
            m = get_sphere_marker(id, target[0], target[1], target[2], target_index == index)
            ma.markers.append(m)
            id += 1
        marker_array_pub.publish(ma)
    except Exception as e:
        msg2 = trajectory.get_target_point_stamped()
        target_pub.publish(msg2)

def publish_full_trajectory(points):
    m = Marker()
    m.header.frame_id = "world"
    m.header.stamp = rospy.Time.now()
    m.ns = "spline_trajetory"
    m.id = 1
    m.type = m.LINE_STRIP
    m.action = 0
    m.lifetime = rospy.Duration(0)
    m.points = points
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 0.0
    m.color.a = 1.0
    m.scale.x = 0.05
    marker_pub.publish(m)
        
def display_path():
    try:
        points = trajectory.get_path_points()
        # print("POINTS:", points)
        publish_full_trajectory(points)
    except Exception as e:
        # print("EXCEPTION:", e)
        pass
    
# Timer callback: Calls trajectory updates or full path, and publish them.
def timer_callback(event): 
    global inside_timer_callback, counter

    ### print("trajectory timer_callback:", full_trajectory_flag, counter)


    if inside_timer_callback:
        return
    inside_timer_callback = True

    # Move target with joystick.
    trajectory.move_tick()

    display_targets()
    display_path()

    # If we want the full path:
    if full_trajectory_flag:
        if counter >= 2.0/options.dt:
            #print("SEND BIG PATH/TRAJECTORY")
            msg = trajectory.get_path(options.dt)
            #print("GOT BIG PATH/TRAJECTORY MSG from tracetoryclass")
            counter = 0
            print_speed(msg, options.dt)
            full_trajectory_pub.publish(msg)
    # If we want to use the trajectory position in each iteration:
    else:
        trajectory.tick(options.dt)

        msg = trajectory.get_point_stamped()
        trajectory_pub.publish(msg)

        if trajectory.enabled():
            msg3 = trajectory.get_target_yaw()
            yaw_pub.publish(msg3)

    inside_timer_callback = False
    counter += 1

def print_speed(pathmsg, dt):
    speeds = []
    for i in range(1, len(pathmsg.poses)):
        p0 = pathmsg.poses[i-1].pose.position
        p1 = pathmsg.poses[i].pose.position
        dx = p1.x-p0.x
        dy = p1.y-p0.y
        dz = p1.z-p0.z
        speeds.append(round(math.sqrt(dx*dx+dy*dy+dz*dz)/dt, 2))
    print("SPEED FROM PATHMSG:", speeds)


def generate_figure_eight_trajectory(radius=5.0, height=2.5, points_per_loop=50, loops=1):
    trajectory_points = []
    t = np.linspace(0, 2 * np.pi, points_per_loop)
    
    for loop in range(loops):
        for i in range(points_per_loop):
            x = radius * np.sin(t[i])
            y = radius * np.sin(t[i]) * np.cos(t[i])
            z = height * np.sin(t[i] / 2) + loop * height  # Opcional: múltiplos loops
            trajectory_points.append([x, y, z])
    
    return trajectory_points

    
if __name__ == "__main__":
    rospy.init_node ("trajectory")
    ns = rospy.get_namespace ().rstrip("/")

    # If don't specify desired target (x,y,z) and speed, it will be (3,0,2.5) and 3 m/s.
    x = rospy.get_param("~x", 3.0)
    y = rospy.get_param("~y", 0.0)
    z = rospy.get_param("~z", 2.5)
    speed = rospy.get_param("~speed", 3.0)
    control_mode = rospy.get_param("control_mode", "velocity")
    spline_flag =  rospy.get_param("~spline", False)
    eight_shape_flag = rospy.get_param("~eight_shape", False)  

    full_trajectory_flag = rospy.get_param("full_trajectory", False)

    print("MODE X Y Z SPEED - trajectory:", control_mode, x, y, z, speed, full_trajectory_flag)

    if spline_flag:
        fixed_targets = generate_figure_eight_trajectory(radius=5.0, height=2.5, points_per_loop=100, loops=2)
        trajectory = SplineTrajectory(x, y, z, speed, fixed_targets=fixed_targets)
    elif eight_shape_flag:
        trajectory = EightShapeTrajectory(x, y, z, speed, num_laps=3)
    else:
        trajectory = GotoTrajectory(x, y, z, speed)
    
    full_trajectory_pub = rospy.Publisher("full_trajectory", Path, latch=False, queue_size=10)
    trajectory_pub = rospy.Publisher("trajectory", PointStamped, latch=False, queue_size=10)
    yaw_pub = rospy.Publisher("yaw_trajectory", Float64, latch=False, queue_size=10)
    target_pub = rospy.Publisher("target", PointStamped, latch=False, queue_size=10)
    marker_array_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, latch=False, queue_size=1000)
    marker_pub = rospy.Publisher("/visualization_marker", Marker, latch=False, queue_size=1000)   # queue_size ....
    
    joy_sub = rospy.Subscriber("/drone/joy", Joy, joy_callback)       #/dji_sdk/local_position
    target_sub = rospy.Subscriber("target/pose", PoseStamped, pose_callback)       # from controller
    set_target_sub = rospy.Subscriber("set_target", Vector3, set_target_callback)       # new to set the target
    pose_sub = rospy.Subscriber("pose", PoseStamped, notify_pose_callback)       #/dji_sdk/local_position
    pose2_sub = rospy.Subscriber("/mat2/pose", PoseStamped, notify_pose_callback)       #/dji_sdk/local_position

    rospy.Timer(rospy.Duration(options.dt), timer_callback)
    
    print("Spinning trajectory node")
    
    rospy.spin()
    
