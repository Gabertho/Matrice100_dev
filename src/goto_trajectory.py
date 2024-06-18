import rospy

import math

from geometry_msgs.msg import PointStamped


class GotoTrajectory:
    def __init__(self, x, y, z, speed):
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_speed = speed
        self.reset()
        self.acc = 1.0
        self.phase = "acc"

    def set_target(self, x, y, z):
        print("set_target:", x, y, z)
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.reset()
        
    def set_target1(self):
        self.target_x = self.target1_x
        self.target_y = self.target1_y
        self.target_z = self.target1_z
        self.x0 = self.target0_x
        self.y0 = self.target0_y
        self.z0 = self.target0_z
        
    def reset(self):
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

    def get_target_point_stamped(self):
        msg = PointStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.point.x = self.target_x
        msg.point.y = self.target_y
        msg.point.z = self.target_z
        return msg

    def set_initial_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

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
        
