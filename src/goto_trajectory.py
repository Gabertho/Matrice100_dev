import rospy

import math

from geometry_msgs.msg import PointStamped


class GotoTrajectory:
    def __init__(self, x, y, z, speed):
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_speed = speed
        self.acc = 1.0
        self.phase = "acc"
        self.have_initial_position = False
        self.speed = 0.0
        self.travel_length = 0.0

    def set_target(self, x, y, z):
        print("set_target:", x, y, z)
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.reset()


    def move_target(self, joy_x, joy_y):
        self.target_x += joy_x
        self.target_y += joy_y
        print("move_target:", self.target_x, self.target_y)
        self.reset()
        
    def reset(self):
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dz = self.target_z - self.z
        len = math.sqrt(dx*dx+dy*dy)
        if len < 0.001:
            return
        self.travel_length = len
        self.frac_x = dx/len
        self.frac_y = dy/len
        self.phase = "acc"
        self.acc_len = 0.0
        self.speed = 0.0

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
        if not self.have_initial_position:
            self.reset()
            self.have_initial_position = True

    def tick(self, dt):
        print("goto_trajectory tick:", dt, self.phase)
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
            else:
                if self.acc_len >= self.travel_length/2.0:
                    self.phase = "brake"
                

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

        print("SPEED:", self.speed, self.frac_x, self.frac_y)
        len = self.speed*dt
        self.x += self.frac_x*len
        self.y += self.frac_y*len
        if self.phase == "acc":
            self.acc_len += len
        
