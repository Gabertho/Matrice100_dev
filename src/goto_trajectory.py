import rospy

import math

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64


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
        self.joy_x = 0.0
        self.joy_y = 0.0
        self.joy_z = 0.0
        self.enabled_flag = False
        self.target_yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


    def enabled(self):
        return self.enabled_flag

    def enable(self):
        self.enabled_flag = True

    def disable(self):
        self.enabled_flag = False

        
    def set_target(self, x, y, z):
        print("set_target:", x, y, z)
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.reset()


    def move_target(self, joy_x, joy_y, joy_z):
        self.joy_x = joy_x
        self.joy_y = joy_y
        self.joy_z = joy_z
        #self.target_x += joy_x
        #self.target_y += joy_y
        # print("move_target:", self.target_x, self.target_y)
        # self.reset()
        
    def reset(self):
        if not self.have_initial_position:
            return
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dz = self.target_z - self.z
        self.target_yaw = math.atan2(dy, dx)
        len = math.sqrt(dx*dx+dy*dy+dz*dz)
        if len < 0.001:
            return
        self.travel_length = len
        self.frac_x = dx/len
        self.frac_y = dy/len
        self.frac_z = dz/len
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

    def get_target_yaw(self):
        msg = Float64()
        msg.data = self.target_yaw
        return msg

    def set_initial_position(self, x, y, z):
        if self.enabled_flag:
            return
        print("set_initial_position:", x, y, z)
        self.x = x
        self.y = y
        self.z = z
        if not self.have_initial_position:
            self.reset()
            self.have_initial_position = True

    def tick(self, dt):
        print("goto_trajectory tick:", dt, self.phase, self.enabled_flag)

        self.target_x += self.joy_x
        self.target_y += self.joy_y
        self.target_z += self.joy_z

        if not self.enabled_flag:
            self.reset()
            return
        
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dz = self.target_z - self.z
        dist_to_target = math.sqrt(dx*dx+dy*dy+dz*dz)
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
            self.z = self.target_z

        len = self.speed*dt
        print("SPEED:", self.speed, self.frac_x, self.frac_y, self.frac_z, len)
        self.x += self.frac_x*len
        self.y += self.frac_y*len
        self.z += self.frac_z*len
        print("POSITION:", self.x, self.y, self.z)
        if self.phase == "acc":
            self.acc_len += len
        
