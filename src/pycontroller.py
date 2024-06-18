#!/usr/bin/python3
import rospy
import math
import sys
import os

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import pandas as pd

from geometry_msgs.msg import PoseStamped, PointStamped, QuaternionStamped, Vector3Stamped, Vector3, Point, Quaternion
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64, Header, ColorRGBA, Empty
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Joy, LaserScan, BatteryState
from dji_sdk.srv import SDKControlAuthority

# rosservice call /dji2/dji_sdk/sdk_control_authority "control_enable: 1"
# dji_sdk/SDKControlAuthority


from controller import Controller

from optparse import OptionParser
from threading import Lock, Event

old_x = 0.0
old_y = 0.0
old_z = 0.0
n_samples = 0
x_vel=[]
y_vel=[]
z_vel=[]
avel_index = 0
current_roll = 0.0
current_pitch = 0.0

auto_flag = False
inside_timer_callback = False
controlled_flag = False
control_mode = "velocity"
battery_file = f'{os.environ["HOME"]}/lrs_ws/src/Matrice100_dev/config/matrice_battery_thrust.ods'

parser = OptionParser()
parser.add_option ("", "--vicon", action="store_true", dest="vicon", help="Vicon")
parser.add_option ("", "--hover", action="store_true", dest="hover", help="Hover")

parser.add_option("-x", "", action="store", dest="x", type="float", default=0.0, help='pick a point for x')
parser.add_option("-y", "", action="store", dest="y", type="float", default=0.0, help='pick a point for y')
parser.add_option("-z", "", action="store", dest="z", type="float", default=1.0, help='pick a point for z')
parser.add_option("", "--speed", action="store", dest="speed", type="float", default=0.35, help='set the speed of the drone')

#parser.add_option ("", "--traj", action="store", dest="traj", type="int", help="Trajectory", default=3)
parser.add_option("", "--trajectory_type", action="store", dest="trajectory_type", type="string", help="Trajectory type (e.g., sinusoidal, step_z, step_xyz, spline, long_spline, rectangle, hexagon)", default="go_to_point")
parser.add_option ("", "--thrust", action="store", dest="thrust", type="int", help="Thrust", default=37)    
parser.add_option ("", "--test", action="store_true", dest="test", help="Test")    
parser.add_option("", "--obstacle", action="store", dest="rviz_obstacle", type="string", help="Rviz obstacle type (NO_OBSTACLE, OBSTACLE, MULTI_OBSTACLES)", default="NO_OBSTACLE")
parser.add_option("", "--name", action="store", dest="name", type="str", default='current',
                        help='current or full')
(options, args) = parser.parse_args()

def joy_callback(data):
    global controlled_flag

    if data.axes[7] < -0.5:
        controller.set_yaw_control(False)
    if data.axes[7] > 0.5:
        controller.set_yaw_control(True)


    if data.buttons[0]:
        # get authority
        print("Get authriy")
        proxy = rospy.ServiceProxy('dji_sdk/sdk_control_authority', SDKControlAuthority)
        resp = proxy(1)
        print("AUTH RESP:", resp)
        controller.auto()
        controlled_flag = True

    if data.buttons[1]:
        controlled_flag = False
        # releaseget authority
        print("releaseGet authriy")
        proxy = rospy.ServiceProxy('dji_sdk/sdk_control_authority', SDKControlAuthority)
        resp = proxy(0)
        print("AUTH RESP:", resp)


def pose_callback(data):
    # print("pose_callback:", data)
    if not options.vicon:
        ## rospy.loginfo(f"Before update: {controller.current_state[:3]}")
        controller.notify_position(data.pose.position.x, data.pose.position.y, data.pose.position.z)
        controller.notify_attitude(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)        
        ## rospy.loginfo(f"After update: {controller.current_state[:3]}")

def trajectory_callback(data):
    # print("trajectory_callback:", data.point.x, data.point.y, data.point.z)
    controller.notify_trajectory(data.point.x, data.point.y, data.point.z)
        
def battery_callback(data):
    if controlled_flag:
        return
    current_battery_level = data.percentage
    ##thrust = get_thrust_based_on_battery(int(current_battery_level*100), battery_data)
    if current_battery_level > 80.0:
        thrust = 39.0 + (41.5-39.0)*(current_battery_level-80.0)/20.0
    else:
        thrust = 45.0 - (45.0-39.0)*current_battery_level/80.0
    # rospy.loginfo(f"Received battery level: {current_battery_level} - {thrust}")
    thrust = 38.0
    controller.set_hover_thrust(thrust)
    return


def get_thrust_based_on_battery(battery_level, data):
    val = 0.0
    if battery_level in data['Battery'].values:
        valid_entries = data[data['Battery'] == battery_level]
        val = valid_entries['Thrust'].mean()
    else:
        lower = data[data['Battery'] < battery_level].max()
        upper = data[data['Battery'] > battery_level].min()
        if not lower.empty and not upper.empty:

            val = np.interp(battery_level,
                            [lower['Battery'], upper['Battery']],
                            [lower['Thrust'], upper['Thrust']])   # lienar interpolation
    if val < 38.0:
        rospy.logerr(f"Error: Abnormal value: {val}")
        val = 38.0
    if val > 44.0:
        rospy.logerr(f"Error: Abnormal value: {val}")        
        val = 44.0
    if not options.vicon:
        val = 38.0
    return val

def attitude_callback(data):
    return
    try:
        # print(data)
        global n_samples
        global old_roll, old_pitch, old_yaw, roll_vel, pitch_vel, yaw_vel, avel_index
        if options.vicon:
            global current_roll, current_pitch
            (current_roll, current_pitch, current_yaw) = euler_from_quaternion([data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w])
        else:
            controller.notify_attitude(data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w)
            quat_list = [data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w]
            (roll, pitch, yaw) = euler_from_quaternion (quat_list)
            if not old_roll:
                old_roll = roll
                old_pitch = pitch
                old_yaw = yaw
                
            #rospy.loginfo(f"Before update: {controller.current_state[3:6]}")
            controller.notify_angles(roll, pitch, yaw)
            #rospy.loginfo(f"After update: {controller.current_state[3:6]}")

            n_in_average = 5
            vroll = (roll - old_roll)/dt
            vpitch = (pitch - old_pitch)/dt
            vyaw = (yaw - old_yaw)/dt
            if n_samples < n_in_average:
                #print("SAVERPY:", n_samples)
                roll_vel.append(vroll)
                pitch_vel.append(vpitch)
                yaw_vel.append(vyaw)
                n_samples += 1
            else:
                #print("avel_index:", avel_index)
                roll_vel[avel_index] = vroll
                pitch_vel[avel_index] = vpitch
                yaw_vel[avel_index] = vyaw
                avel_index += 1
                avel_index %= n_in_average
            rsum = 0.0
            psum = 0.0
            ysum = 0.0
            # print("N_SAMPLES:", n_samples)
            for i in range(n_samples):
                rsum += roll_vel[i];
                psum += pitch_vel[i];
                ysum += yaw_vel[i];
                # print("SUMS:", rsum, psum, ysum)
            controller.notify_angle_rates(rsum/n_samples, psum/n_samples, ysum/n_samples);
            old_roll = roll
            old_pitch = pitch
            old_yaw = yaw
    except Exception as e:
        print(f"Exception occurred: {e}")
    
def velocity_callback(data):
    # print("velocity_callback:", data)
    if not options.vicon:
        controller.notify_velocity(data.vector.x, data.vector.y, data.vector.z)

def rc_callback(data):
    # print(data)
    global auto_flag
    old_auto_flag = auto_flag
    auto_flag = data.axes[4] > 5000.0
    if old_auto_flag != auto_flag:
        print("AUTO FLAG CHANGED TO:", auto_flag)
    
def tf_callback(data):
    ## print(data)
    global old_x, old_y, old_z, n_samples, avel_index
    if options.vicon:
        for trans in data.transforms:
            if trans.header.frame_id == "world" and trans.child_frame_id == "mat2":
                vicon_dt = 0.02
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                z = trans.transform.translation.z
                current_x = x
                current_y = y
                current_z = z

                # print("*********************************************************************************")
                controller.notify_position(x, y, z)

                if not old_x:
                    old_x = x
                    old_y = y
                    old_z = z

                vel_n_in_average = 5
                vx = (current_x - old_x)/vicon_dt
                vy = (current_y - old_y)/vicon_dt
                vz = (current_z - old_z)/vicon_dt
                if n_samples < vel_n_in_average:
                    x_vel.append(vx)
                    y_vel.append(vy)
                    z_vel.append(vz)
                    n_samples += 1
                else:
                    #print("UPDATE:", avel_index, vx, vy, vz)
                    x_vel[avel_index] = vx
                    y_vel[avel_index] = vy
                    z_vel[avel_index] = vz
                    avel_index += 1
                    avel_index %= vel_n_in_average
                vx_sum = 0.0
                vy_sum = 0.0
                vz_sum = 0.0
                for i in range(n_samples):
                    vx_sum += x_vel[i];
                    vy_sum += y_vel[i];
                    vz_sum += z_vel[i];
                    # print("VSUM;", i, vx_sum, vy_sum, vz_sum)
                controller.notify_velocity(vx_sum/n_samples, vy_sum/n_samples, vz_sum/n_samples)

                msg = Vector3();
                msg.x = vx;
                msg.y = vy;
                msg.z = vz;

                velocity_pub.publish(msg);

                qx = trans.transform.rotation.x
                qy = trans.transform.rotation.y
                qz = trans.transform.rotation.z
                qw = trans.transform.rotation.w

                global current_roll, current_pitch
                (roll, pitch, current_yaw) = euler_from_quaternion([qx, qy, qz, qw])

                controller.notify_angles(current_roll, current_pitch, current_yaw);     
                old_x = x
                old_y = y
                old_z = z

def publish_value(pub, value):
    msg = Float64()
    msg.data = value
    pub.publish(msg)

def publish_controls(u):
    publish_value(ctrl0_pub, u[0])
    publish_value(ctrl1_pub, u[1])
    publish_value(ctrl2_pub, u[2])
    publish_value(ctrl3_pub, u[3])

def timer_callback(event): 
    # print("timer_callback")

    if not controlled_flag:
        return

    global inside_timer_callback

    if inside_timer_callback:
        return

    inside_timer_callback = True

    u = controller.control(dt)

    # test
    #### u[3] = math.radians(20.0)  # rotate 3 degreesper second

    publish_controls(u)

    # print("U:", u)

    msg = Joy()
    msg.axes = u
    #msg.axes.append(u[0]) # roll limit 0.611 rad (35 degree)
    #msg.axes.append(u[1]) # pitch limit 0.611 rad (35 degree)
    #msg.axes.append(u[2]) # thrust  0-100
    #msg.axes.append(u[3]) # yaw_rate limit 5/6*PI rad/s
    if control_mode == "velocity":
        msg.axes.append(0x40 | 0x01 | 0x08 | 0x20) # Is 0x01 relevant here (Actively break to hold position after stop sending setpoint)
    if control_mode == "angles":
        msg.axes.append(0x02 | 0x01 | 0x08 | 0x20) # Is 0x01 relevant here (Actively break to hold position after stop sending setpoint)
    if control_mode == "rates":
        msg.axes.append(0xC0 | 0x02 | 0x01 | 0x08 | 0x20) # Is 0x01 relevant here (Actively break to hold position after stop sending setpoint)

    ctrl_pub.publish(msg)

    inside_timer_callback = False


if __name__ == "__main__":
    rospy.init_node ("pycontroller")
    ns = rospy.get_namespace ().rstrip("/")

    #try:
    #    battery_data = pd.read_excel(battery_file, engine='odf')
    #    print("BATTERY_DATA:", battery_data)
    #except Exception as e:
    #    rospy.logerr(f"Failed to load battery thrust data: {e}")
    #    sys.exit(1)


    control_mode = rospy.get_param("control_mode", "velocity")

    print("CONTROL_MODE:", control_mode)

    controller = Controller(control_mode)

    battery_sub = rospy.Subscriber("dji_sdk/battery_state", BatteryState, battery_callback)
    dt = 0.1

    # ----------------------------------------------------------------
    marker_array_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, latch=False, queue_size=1000)
    marker_pub = rospy.Publisher("/visualization_marker", Marker, latch=False, queue_size=1000)   # queue_size ....
    ctrl_pub = rospy.Publisher("dji_sdk/flight_control_setpoint_generic", Joy, latch=False, queue_size=10)
    ref_pos_pub = rospy.Publisher("tune/ref_position", PointStamped, latch=False, queue_size=10)
    first_yellow_pub = rospy.Publisher("tune/first_yellow", PointStamped, latch=False, queue_size=10)
    err_x_pub = rospy.Publisher("tune/err_x", Float64, latch=False, queue_size=10)
    err_y_pub = rospy.Publisher("tune/err_y", Float64, latch=False, queue_size=10)
    err_z_pub = rospy.Publisher("tune/err_z", Float64, latch=False, queue_size=10)
    ctrl0_pub = rospy.Publisher("ctrl/u0", Float64, latch=False, queue_size=10)
    ctrl1_pub = rospy.Publisher("ctrl/u1", Float64, latch=False, queue_size=10)
    ctrl2_pub = rospy.Publisher("ctrl/u2", Float64, latch=False, queue_size=10)
    ctrl3_pub = rospy.Publisher("ctrl/u3", Float64, latch=False, queue_size=10)
    velocity_pub = rospy.Publisher("velocity", Vector3, latch=False, queue_size=10)
    #hokuyo_lidar_pub = rospy.Publisher("hokuyo_scan", LaserScan, latch=False, queue_size=10)

    joy_sub = rospy.Subscriber("/drone/joy", Joy, joy_callback)       #/dji_sdk/local_position
    if not options.vicon:
        pose_sub = rospy.Subscriber("pose", PoseStamped, pose_callback)       #/dji_sdk/local_position
        velocity_sub = rospy.Subscriber("dji_sdk/velocity", Vector3Stamped, velocity_callback)

    # Seems like attitude callback is not needed
    ###attitude_sub = rospy.Subscriber("dji_sdk/attitude", QuaternionStamped, attitude_callback)
    trajectory_sub = rospy.Subscriber("trajectory", PointStamped, trajectory_callback)

    rc_sub = rospy.Subscriber("dji_sdk/rc", Joy, rc_callback)

    if options.vicon:
        tf_sub = rospy.Subscriber("/tf", TFMessage, tf_callback)

    rospy.Timer(rospy.Duration(dt), timer_callback)
    
    print("Spinning pycontroller node")
    
    rospy.spin()
    
