#!/usr/bin/env python3

from tf.transformations import euler_from_quaternion
import numpy as np
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
        
    def control(self, dt):
        # print("DO CONTROL:", dt)

        u = [0.0, 0.0, 38.0, 0.0]

        if self.control_mode == "velocity":
            error = self.target - self.current_position
            # print("ERROR:", error, self.target)
            P = 3.0
            u[0] = P*error[0]            # east
            u[1] = P*error[1]            # north

        return u

        
