import rospy

import math

from geometry_msgs.msg import PointStamped


class GotoTrajectory:
    def __init__(self, x, y, z, speed):
        self.x0 = 0.0
        self.y0 = 0.0
        self.z0 = z
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target1_x = x
        self.target1_y = y
        self.target1_z = z
        self.target0_x = 0.0
        self.target0_y = 0.0
        self.target0_z = z
        self.target_speed = speed
        self.reset()
        self.acc = 1.0
        self.phase = "acc"

    def set_target0(self):
        print("set_target0:", self.target0_x, self.target0_y, self.target0_z)
        self.target_x = self.target0_x
        self.target_y = self.target0_y
        self.target_z = self.target0_z
        self.x0 = self.target1_x
        self.y0 = self.target1_y
        self.z0 = self.target1_z
        self.reset()
        
    def set_target1(self):
        self.target_x = self.target1_x
        self.target_y = self.target1_y
        self.target_z = self.target1_z
        self.x0 = self.target0_x
        self.y0 = self.target0_y
        self.z0 = self.target0_z
        
    def reset(self, x0, y0, z0):
        self.x = self.x0
        self.y = self.y0
        self.z = self.z0
        self.speed = 0
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dz = self.target_z - self.z
        len = math.sqrt(dx*dx+dy*dy)
        self.frac_x = dx/len
        self.frac_y = dy/len
        self.phase = "acc"
        self.acc_len = 0.0

    def get_point_stamped(self):
        msg = PointStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = self.z
        return msg

    def tick(self, dt):
        # print("goto_trajectory tick:", dt)
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dz = self.target_z - self.z
        dist_to_target = math.sqrt(dx*dx+dy*dy)
        print("dist_to_target:", dist_to_target)

        if self.phase == "acc":
            self.speed += self.acc*dt
            if self.speed > self.target_speed:
                self.speed = self.target_speed
                self.phase = "cruise"
                print("ACCLEN:", self.acc_len)

        if self.phase == "brake":
            self.speed -= self.acc*dt
            if self.speed < 0.0:
                self.speed = 0.0
                self.phase = "hover"
                print("HOVER:")

        if self.phase == "cruise":
            if dist_to_target < self.acc_len:
                self.phase = "brake"

        if self.phase == "hover":
            self.x = self.target_x
            self.y = self.target_y

        print("SPEED:", self.speed)
        len = self.speed*dt
        self.x += self.frac_x*len
        self.y += self.frac_y*len
        if self.phase == "acc":
            self.acc_len += len
        
