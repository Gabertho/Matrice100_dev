#!/usr/bin/python3
import rospy
import math
import sys

#
# Take current position as atart.  button0 is greeb manual, yellow=3 auto
#

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
current_pose = PoseStamped()
current_pose.pose.position.x = 0.0
current_pose.pose.position.y = 0.0
current_pose.pose.position.z = 2.5
set_initial_position_flag = True

parser = OptionParser()
parser.add_option("", "--dt", action="store", dest="dt", type="float", default=0.1, help='Perions, default 0.1')
parser.add_option("", "--trajectory_type", action="store", dest="trajectory_type", type="string", help="Trajectory type (e.g., sinusoidal, step_z, step_xyz, spline, long_spline, rectangle, hexagon)", default="go_to_point")
(options, args) = parser.parse_args()

def joy_callback(data):
    global enable_flag
    global set_initial_position_flag
    #if data.axes[6] > 0.5:
    #    trajectory.set_target0()
    #if data.axes[6] < -0.5:
    #    trajectory.set_target1()

    if data.buttons[6]:
        trajectory.enable()
    else:
        trajectory.disable()
        # trajectory.reset(current_pose.pose.position.x, current_pose.pose.position.z, current_pose.pose.position.z)
        trajectory.have_initial_position = False
        set_initial_position_flag = True

    trajectory.move_target(-data.axes[3]/2.0, data.axes[4]/2.0, data.axes[1]*data.buttons[4])
        
        
def pose_callback(data):
    global current_pose
    global set_initial_position_flag
    # print("pose_callback:", data)
    current_pose = data
    if set_initial_position_flag:
        trajectory.set_initial_position(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z)
        set_initial_position_flag = False

def timer_callback(event): 
    # print("trajectory timer_callback")

    global inside_timer_callback

    if inside_timer_callback:
        return
    inside_timer_callback = True

    trajectory.tick(options.dt)

    msg2 = trajectory.get_target_point_stamped()
    target_pub.publish(msg2)

    msg = trajectory.get_point_stamped()
    trajectory_pub.publish(msg)

    inside_timer_callback = False


if __name__ == "__main__":
    rospy.init_node ("pycontroller")
    ns = rospy.get_namespace ().rstrip("/")

    x = rospy.get_param("~x", 3.0)
    y = rospy.get_param("~y", 0.0)
    z = rospy.get_param("~z", 2.5)
    speed = rospy.get_param("~speed", 3.0)
    control_mode = rospy.get_param("control_mode", "velocity")

    print("MODE X Y Z SPEED:", control_mode, x, y, z, speed)

    trajectory = GotoTrajectory(x, y, z, speed)

    trajectory_pub = rospy.Publisher("trajectory", PointStamped, latch=False, queue_size=10)
    target_pub = rospy.Publisher("target", PointStamped, latch=False, queue_size=10)

    joy_sub = rospy.Subscriber("/drone/joy", Joy, joy_callback)       #/dji_sdk/local_position
    pose_sub = rospy.Subscriber("pose", PoseStamped, pose_callback)       #/dji_sdk/local_position
    pose2_sub = rospy.Subscriber("/mat2/pose", PoseStamped, pose_callback)       #/dji_sdk/local_position

    rospy.Timer(rospy.Duration(options.dt), timer_callback)
    
    print("Spinning pycontroller node")
    
    rospy.spin()
    
