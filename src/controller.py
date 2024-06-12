#!/usr/bin/env python3

class Controller:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0
        self.roll = 0.0
        self.pitch = 0.0
        self.thrust = 0.0
        self.yaw_rate = 0.0


    def notify_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def notify_velocity(self, x, y, z):
        self.vx = x
        self.vy = y
        self.vz = z

    def notify_attitude(self, qx, qy, qz, qw):
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        
    def control(self, dt):
        print("DO CONTROL:", dt)

        return (self.roll, self.pitch, self.thrust, self.yaw_rate)

        
