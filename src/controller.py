#!/usr/bin/env python3

import rospy

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
import scipy
import scipy.ndimage
import math

class Controller:
    def __init__(self, control_mode):
        self.control_mode = control_mode
        self.current_position = np.array([0.0, 0.0, 2.5])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0
        self.roll = 0.0
        self.pitch = 0.0
        self.thrust = 0.0
        self.yaw_rate = 0.0
        self.target = None
        self.target_speed = 1.0
        self.target_yaw = 0.0
        self.have_target = False
        self.have_current_yaw = False
        self.hover_thrust = 38.0
        self.P_z = 1.5
        self.I_z = 0.0019 
        self.D_z = 6.0
        self.P_yaw = 1.0 
        self.D_yaw = 0.5
        self.P_x = 2.0
        self.D_x = 4.0
        self.P_y = 2.0
        self.D_y = 4.0
        self.old_err_pitch = 0.0
        self.old_err_roll = 0.0
        self.int_err_z = 0.0
        self.int_err_yaw = 0.0
        self.old_err_yaw = 0.0
        self.old_err_z = 0.0
        self.yaw_control_flag = False
        self.mode = "simple_pid"
        self.trajectory_flag = False

    def get_target_pose(self):
        if self.target.any():
            msg = PoseStamped()
            msg.header.frame_id = "world"
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = self.target[0]
            msg.pose.position.y = self.target[1]
            msg.pose.position.z = self.target[2]
            quat = quaternion_from_euler(0.0, 0.0, self.target_yaw)
            print("QUAT:", quat)
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            return msg

    def enable_trajectory(self):
        self.trajectory_flag = True

    def disable_trajectory(self):
        self.trajectory_flag = False

    def reset(self):
        self.int_err_z = 0.0
        self.int_err_yaw = 0.0        

    def auto(self):
        self.target = np.array([self.current_position[0], self.current_position[1], self.current_position[2] ])

    def set_yaw_control(self, flag):
        self.yaw_control_flag = flag

    def set_hover_thrust(self, thrust):
        self.hover_thrust = thrust

    def notify_position(self, x, y, z):
        self.current_position = np.array([x, y, z])

    def notify_trajectory(self, x, y, z):
        if self.trajectory_flag:
            self.target = np.array([x, y, z])
            self.have_target = True
        
    def notify_yaw_trajectory(self, yaw):
        self.target_yaw = yaw

    def notify_velocity(self, x, y, z):
        self.velocity = np.array([x, y, z])

    def notify_angles(self, roll, pitch, yaw):
        self.current_yaw = yaw
        self.have_current_yaw = True
        
    def notify_attitude(self, qx, qy, qz, qw):
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
        # print("YAW:", math.degrees(yaw))
        self.current_yaw = yaw
        self.have_current_yaw = True

    def integral(self, prev_integral_value, error):
        # 0.001 is antiwindup gain
        # # if prev_integral_value >=2: # For modeling saturation
        # # prev_integral_value =2
        # # elif prev_integral_value <= -2:
        # # prev_integral_value =( -2)
        error_anti_windup = error -0.001* prev_integral_value
        integral_new = prev_integral_value + error_anti_windup * self.delt #delt is the period like 0.01 sec if 100hz.
        return integral_new
    
    # Setters
    
    def set_pitch(self, pitch):
        self.pitch = pitch

    def set_roll(self, roll):
        self.roll = roll
    
    def set_thrust(self, thrust):
        self.thrust = thrust

    def set_yawrate(self, yaw_rate):
        self.yaw_rate = yaw_rate

    # Getters

    def get_yaw_control(self):
        return self.yaw_control_flag

    def control_thrust(self, period):
        self.err_z = self.target[2] - self.current_position[2] 
        self.int_err_z += self.err_z
        self.d_err_z = (self.err_z - self.old_err_z) / period
        delta = self.P_z * self.err_z + self.I_z * self.int_err_z + self.D_z * self.d_err_z
        thrust = self.hover_thrust + delta
        limit1 = 80.0
        limit2 = 20.0
        if thrust > limit1:
            thrust = limit1
        if thrust < limit2:
            thrust = limit2
        self.old_err_z = self.err_z
        return thrust
     
    def control_yaw(self, period):
        yaw_rate = 0.0
        self.err_yaw = self.target_yaw - self.current_yaw
        self.int_err_yaw += self.err_yaw
        d_err = (self.err_z - self.old_err_z) / period
        if self.get_yaw_control():
            yaw_rate = self.P_yaw * self.err_yaw + self.D_yaw * d_err
        self.old_err_yaw = self.err_yaw
        return yaw_rate

    def control_horizontal(self, period):
        err_x = self.target[0] - self.current_position[0]
        err_y = self.target[1] - self.current_position[1]
        self.err_pitch = math.cos(-self.current_yaw) * err_x - math.sin(-self.current_yaw) * err_y
        self.err_roll = math.sin(-self.current_yaw) * err_x + math.cos(-self.current_yaw) * err_y

        derr_pitch = (self.err_pitch - self.old_err_pitch) / period
        derr_roll = (self.err_roll - self.old_err_roll) / period

        pitch = self.P_y * self.err_pitch + self.D_y * derr_pitch
        roll = -(self.P_x * self.err_roll + self.D_x * derr_roll)

        limit = 10.0
        if pitch > limit:
            pitch = limit
        if pitch < -limit:
            pitch = -limit
        if roll > limit:
            roll = limit
        if roll < -limit:
            roll = -limit

        self.old_err_pitch = self.err_pitch
        self.old_err_roll = self.err_roll

        return(pitch * math.pi / 180.0, roll * math.pi / 180.0)
        
    def control(self, dt):
        # print("DO CONTROL:", dt, self.control_mode)

        u = [0.0, 0.0, 38.0, 0.0]

        if not self.have_current_yaw:
            print("Do not have current_yaw")
            return u

        error = self.target - self.current_position

        #
        # Thrust
        #
        pthrust = 1.5
        ithrust = 0.0019
        dthrust = 6.0

        #pthrust = 1.5
        ithrust = 0.04
        #dthrust = 0.0

        print("TARGET:", self.target)
        print("CUPOS:", self.current_position)
        print("THRUSTERROR:", error[2])
        print("HOVERTHRUST:", self.hover_thrust)

        self.int_err_z += error[2]
        d_err_z = (error[2] - self.old_err_z)/dt

        print("INTERROR:", self.int_err_z)
        print("D_ERR_Z:", d_err_z)


        delta = error[2]*pthrust + ithrust*self.int_err_z + dthrust*d_err_z

        print("DELTATHRUST:", delta)
        
        u[2] = self.hover_thrust + delta

        if u[2] < 20.0:
            u[2] = 20.0
        if u[2] > 80.0:
            u[2] = 80.0

        self.old_err_z = error[2]

        #
        # Yaw
        #

        pyaw = 1.0
        iyaw = 0.0
        dyaw = 0.0


        yaw_error = self.target_yaw - self.current_yaw
        print("YAW_ERROR:", yaw_error)
        
        self.int_err_yaw += yaw_error
        d_err_yaw = (yaw_error - self.old_err_yaw)/dt

        if self.yaw_control_flag:
            u[3] = yaw_error*pyaw + dyaw*d_err_yaw
        else:
            u[3] = 0.0
            
        if self.control_mode == "velocity":
            # print("ERROR:", error, self.target)
            P = 3.0
            u[0] = P*error[0]            # east
            u[1] = P*error[1]            # north

        if self.control_mode == "angles":
            if self.mode == "gabriel":
                thrust = self.control_thrust(dt)
                yaw = self.control_yaw(dt)
                pitch, roll = self.control_horizontal(dt)
                u[0] = roll 
                u[1] = pitch
                u[2] = thrust 
                u[3] = yaw

                #Saturation
                ##Thrust
                if u[2] < 20.0:
                    u[2] = 20.0
                if u[2] > 80.0:
                    u[2] = 80.0

                ##Roll, pitch
                max = math.radians(20.0)
                if u[0] > max:
                    u[0] = max
                if u[0] < -max:
                    u[0] = -max
                if u[1] > max:
                    u[1] = max
                if u[1] < -max:
                    u[1] = -max

                #Yaw

            if self.mode == "simple_pid":
                # print("ERROR:", error, self.target)

                herror = np.array([error[0], error[1]])
                print("HERROR:", math.degrees(self.current_yaw), herror)

                theta = -self.current_yaw
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c)))
                # print("R:", R)

                rherror = np.dot(R, herror)

                print("ROTATET HERROR:", rherror)

                derr_pitch = (rherror[0] - self.old_err_pitch)/dt
                derr_roll = (rherror[1] - self.old_err_roll)/dt


                self.old_err_pitch = rherror[0]
                self.old_err_roll = rherror[1]
            
                P = 2.0
                D = 4.0
                u[0] = math.radians(-(P*rherror[1] + D*derr_roll))       # roll
                u[1] = math.radians(P*rherror[0] + D*derr_pitch)         # pitch

                max = math.radians(20.0)
                if u[0] > max:
                    u[0] = max
                if u[0] < -max:
                    u[0] = -max
                if u[1] > max:
                    u[1] = max
                if u[1] < -max:
                    u[1] = -max

        if self.control_mode == "rates":
            # print("ERROR:", error, self.target)

            herror = np.array([error[0], error[1]])
            print("HERROR:", math.degrees(self.current_yaw), herror)

            theta = self.current_yaw
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
            # print("R:", R)

            rherror = np.dot(R, herror)

            print("ROTATET HERROR:", rherror)
            
            P = 0.5
            u[0] = -P*rherror[1]             # roll rate
            u[1] = P*rherror[0]            # pitch rate

            print("U:", u)

            max = 5.0/6.0*math.pi
            if u[0] > max:
                u[0] = max
            if u[0] < -max:
                u[0] = -max
            if u[1] > max:
                u[1] = max
            if u[1] < -max:
                u[1] = -max

        return u

        
