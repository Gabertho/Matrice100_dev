#!/usr/bin/python3
import rospy
import math
import sys

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, Vector3Stamped, Vector3, Point, Quaternion
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64, Header, ColorRGBA, Empty
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Joy, LaserScan, BatteryState

from goto_trajectory import GotoTrajectory

from optparse import OptionParser
from threading import Lock, Event

inside_timer_callback = False
enable_flag = False
current_pose = None

parser = OptionParser()
parser.add_option("", "--dt", action="store", dest="dt", type="float", default=0.1, help='Perions, default 0.1')
parser.add_option("", "--trajectory_type", action="store", dest="trajectory_type", type="string", help="Trajectory type (e.g., sinusoidal, step_z, step_xyz, spline, long_spline, rectangle, hexagon)", default="go_to_point")
(options, args) = parser.parse_args()

def joy_callback(data):
    global enable_flag
    if data.buttons[6]:
        enable_flag = True
    else:
        enable_flag = False
        trajectory.reset()

def pose_callback(data):
    global current_pose
    # print("pose_callback:", data)
    current_pose = data

def timer_callback(event): 
    # print("trajectory timer_callback")

    global inside_timer_callback

    if inside_timer_callback:
        return
    inside_timer_callback = True

    if enable_flag:
        trajectory.tick(options.dt)
    msg = trajectory.get_point_stamped()
    trajectory_pub.publish(msg)

    inside_timer_callback = False


if __name__ == "__main__":
    rospy.init_node ("pycontroller")
    ns = rospy.get_namespace ().rstrip("/")

    x = rospy.get_param("~x", 30.0)
    y = rospy.get_param("~y", 0.0)
    z = rospy.get_param("~z", 0.0)
    speed = rospy.get_param("~speed", 1.0)
    control_mode = rospy.get_param("control_mode", "velocity")

    print("MODE X Y Z SPEED:", control_mode, x, y, z, speed)

    trajectory = GotoTrajectory(x, y, z, speed)

    trajectory_pub = rospy.Publisher("trajectory", PointStamped, latch=False, queue_size=10)

    joy_sub = rospy.Subscriber("/drone/joy", Joy, joy_callback)       #/dji_sdk/local_position
    pose_sub = rospy.Subscriber("pose", PoseStamped, pose_callback)       #/dji_sdk/local_position

    rospy.Timer(rospy.Duration(options.dt), timer_callback)
    
    print("Spinning pycontroller node")
    
    rospy.spin()
    
