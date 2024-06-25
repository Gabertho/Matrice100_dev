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

from optparse import OptionParser
from threading import Lock, Event

inside_timer_callback = False
current_pose = PoseStamped()
current_pose.pose.position.x = 0.0
current_pose.pose.position.y = 0.0
current_pose.pose.position.z = 2.5
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
        
# Pose callback: gets x,y,z position and set trajectory initial position (if flag enabled) to it.
def pose_callback(data):
    global current_pose
    global set_initial_position_flag
    # print("pose_callback:", data)
    current_pose = data
    if set_initial_position_flag:
        trajectory.set_initial_position(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z)
        set_initial_position_flag = False

# Timer callback: Calls trajectory updates or full path, and publish them.
def timer_callback(event): 
    global inside_timer_callback, counter

    ### print("trajectory timer_callback:", full_trajectory_flag, counter)


    if inside_timer_callback:
        return
    inside_timer_callback = True

    # Move target with joystick.
    trajectory.move_tick()
    msg2 = trajectory.get_target_point_stamped()
    target_pub.publish(msg2)

    # If we want the full path:
    if full_trajectory_flag:
        if counter >= 2.0/options.dt:
            print("SEND BIG PATH/TRAJECTORY")
            msg = trajectory.get_path(options.dt)
            print("GOT BIG PATH/TRAJECTORY MSG from tracetoryclass")
            counter = 0
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


if __name__ == "__main__":
    rospy.init_node ("trajectory")
    ns = rospy.get_namespace ().rstrip("/")

    # If don't specify desired target (x,y,z) and speed, it will be (3,0,2.5) and 3 m/s.
    x = rospy.get_param("~x", 3.0)
    y = rospy.get_param("~y", 0.0)
    z = rospy.get_param("~z", 2.5)
    speed = rospy.get_param("~speed", 3.0)
    control_mode = rospy.get_param("control_mode", "velocity")

    full_trajectory_flag = rospy.get_param("full_trajectory", False)

    print("MODE X Y Z SPEED - trajectory:", control_mode, x, y, z, speed, full_trajectory_flag)

    trajectory = GotoTrajectory(x, y, z, speed)

    full_trajectory_pub = rospy.Publisher("full_trajectory", Path, latch=False, queue_size=10)
    trajectory_pub = rospy.Publisher("trajectory", PointStamped, latch=False, queue_size=10)
    yaw_pub = rospy.Publisher("yaw_trajectory", Float64, latch=False, queue_size=10)
    target_pub = rospy.Publisher("target", PointStamped, latch=False, queue_size=10)

    joy_sub = rospy.Subscriber("/drone/joy", Joy, joy_callback)       #/dji_sdk/local_position
    target_sub = rospy.Subscriber("target/pose", PoseStamped, pose_callback)       #/dji_sdk/local_position
#    pose_sub = rospy.Subscriber("pose", PoseStamped, pose_callback)       #/dji_sdk/local_position
#    pose2_sub = rospy.Subscriber("/mat2/pose", PoseStamped, pose_callback)       #/dji_sdk/local_position

    rospy.Timer(rospy.Duration(options.dt), timer_callback)
    
    print("Spinning pycontroller node")
    
    rospy.spin()
    
