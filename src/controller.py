#!/usr/bin/env python3

from tf.transformations import euler_from_quaternion
import numpy as np
import scipy
import scipy.ndimage
import math

class Controller:
    def __init__(self, control_mode):
        self.control_mode = control_mode
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0
        self.roll = 0.0
        self.pitch = 0.0
        self.thrust = 0.0
        self.yaw_rate = 0.0
        self.target0 = None
        self.target = None
        self.target_speed = None
        self.have_target = False
        self.have_target0 = False
        self.have_current_yaw = False


    def notify_position(self, x, y, z):
        self.current_position = np.array([x, y, z])

    def notify_trajectory(self, x, y, z):
        if self.have_target:
            self.target0 = self.target
            self.have_target0 = True
        self.target = np.array([x, y, z])
        self.have_target = True
        if self.have_target0:
            dist = np.linalg.norm(self.target-self.target0)
            self.target_speed = dist/0.1
            # print("TARGET SPEED:", self.target_speed)
        

    def notify_velocity(self, x, y, z):
        self.velocity = np.array([x, y, z])

    def notify_attitude(self, qx, qy, qz, qw):
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
        # print("YAW:", math.degrees(yaw))
        self.current_yaw = yaw
        self.have_current_yaw = True
        
    def control(self, dt):
        # print("DO CONTROL:", dt, self.control_mode)

        u = [0.0, 0.0, 38.0, 0.0]

        if not self.have_target0:
            print("Do not have target0")
            return u

        if not self.have_current_yaw:
            print("Do not have current_yaw")
            return u

        if self.control_mode == "velocity":
            error = self.target - self.current_position
            # print("ERROR:", error, self.target)
            P = 3.0
            u[0] = P*error[0]            # east
            u[1] = P*error[1]            # north

        if self.control_mode == "angles":
            error = self.target - self.current_position
            # print("ERROR:", error, self.target)

            herror = np.array([error[0], error[1]])
            # print("HERROR:", herror)

            theta = self.current_yaw
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
            # print("R:", R)

            rherror = np.dot(R, herror)

            print("ROTATET HERROR:", rherror)
            
            P = 3.0
            u[0] = P*error[1]            # roll
            u[1] = -P*error[0]           # pitch

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
            error = self.target - self.current_position
            # print("ERROR:", error, self.target)

            herror = np.array([error[0], error[1]])
            # print("HERROR:", herror)

            theta = self.current_yaw
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
            # print("R:", R)

            rherror = np.dot(R, herror)

            print("ROTATET HERROR:", rherror)
            
            P = 0.5
            u[0] = P*error[1]             # roll rate
            u[1] = -P*error[0]            # pitch rate

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

        
