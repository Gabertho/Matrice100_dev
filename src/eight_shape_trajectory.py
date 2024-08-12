import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float64

class EightShapeTrajectory:
    def __init__(self, x, y, z, speed):
        self.start_x = x
        self.start_y = y
        self.start_z = z
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_speed = speed
        self.acc = 1.0
        self.phase = "acc"
        self.have_initial_position = False
        self.have_initial_position_from_pose = False
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
        self.acc_len = 0.0
        self.radius = 5.0  # Raio do círculo do "oito"
        self.angular_velocity = self.target_speed / self.radius
        self.time_elapsed = 0.0

    def enabled(self):
        return self.enabled_flag

    def enable(self):
        self.enabled_flag = True

    def disable(self):
        self.enabled_flag = False

    def notify_position(self, x, y, z):
        if not self.have_initial_position_from_pose:
            self.have_initial_position_from_pose = True
            self.x = x
            self.y = y
            self.z = z

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

    def reset(self):
        if not self.have_initial_position:
            return
        self.time_elapsed = 0.0
        self.speed = 0.0
        self.phase = "acc"
        self.acc_len = 0.0

        # Resetando para o início da forma de "oito"
        self.x = self.start_x
        self.y = self.start_y
        self.z = self.start_z

    def get_point_stamped(self):
        msg = PointStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = self.z
        return msg

    def get_pose_stamped(self):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = self.z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
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

    def get_path(self, dt):
        x = self.x
        y = self.y
        z = self.z
        phase = "acc"
        speed = 0.0
        acc_len = 0.0

        self.reset()

        pathmsg = Path()

        pathmsg.header.frame_id = "world"
        pathmsg.header.stamp.secs = 0
        pathmsg.header.stamp.nsecs = int(1000000000.0 * dt)

        # While we are not hovering, i.e., we didn't reach the target position:
        while phase != "hover":
            if phase == "acc":
                speed += self.acc * dt
                if speed > self.target_speed:
                    speed = self.target_speed
                    phase = "cruise"
                else:
                    if acc_len >= self.travel_length / 2.0:
                        phase = "brake"

            if phase == "brake":
                speed -= self.acc * dt
                if speed < 0.0:
                    speed = 0.0
                    phase = "hover"

            if phase == "cruise":
                if acc_len >= self.travel_length / 2.0:
                    phase = "brake"

            len_traveled = speed * dt
            acc_len += len_traveled

            # Calculando nova posição na trajetória em forma de "oito"
            self.time_elapsed += dt
            x = self.start_x + self.radius * math.sin(self.angular_velocity * self.time_elapsed) * (speed / self.target_speed)
            y = self.start_y + self.radius * math.sin(2 * self.angular_velocity * self.time_elapsed) / 2 * (speed / self.target_speed)
            z = self.start_z

            msg = PoseStamped()
            msg.header.frame_id = "world"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0

            if phase == "hover":
                pathmsg.poses[-1] = msg
            else:
                pathmsg.poses.append(msg)

        return pathmsg

    def move_tick(self):
        self.target_x += self.joy_x
        self.target_y += self.joy_y
        self.target_z += self.joy_z

        if not self.enabled_flag:
            self.reset()
            return

    def tick(self, dt):
        if not self.enabled_flag:
            return

        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dz = self.target_z - self.z
        dist_to_target = math.sqrt(dx * dx + dy * dy + dz * dz)

        if self.phase == "acc":
            self.speed += self.acc * dt
            if self.speed > self.target_speed:
                self.speed = self.target_speed
                self.phase = "cruise"
            else:
                if self.acc_len >= self.travel_length / 2.0:
                    self.phase = "brake"

        if self.phase == "brake":
            self.speed -= self.acc * dt
            if self.speed < 0.0 or (dist_to_target < 0.05):
                self.speed = 0.0
                self.phase = "hover"

        if self.phase == "cruise":
            if dist_to_target <= self.acc_len:
                self.phase = "brake"

        len_traveled = self.speed * dt

        self.time_elapsed += dt
        self.x = self.start_x + self.radius * math.sin(self.angular_velocity * self.time_elapsed) * (self.speed / self.target_speed)
        self.y = self.start_y + self.radius * math.sin(2 * self.angular_velocity * self.time_elapsed) / 2 * (self.speed / self.target_speed)
        self.z = self.start_z

        self.acc_len += len_traveled
        self.target_yaw = math.atan2(self.y - self.start_y, self.x - self.start_x)
