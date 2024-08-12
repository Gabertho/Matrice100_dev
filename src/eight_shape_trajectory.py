import rospy
import math
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from std_msgs.msg import Float64

class EightShapeTrajectory:
    def __init__(self, x, y, z, speed):
        self.start_x = x
        self.start_y = y
        self.start_z = z
        self.target_speed = speed
        self.acc = 1.0  # Aceleração constante
        self.phase = "acc"
        self.have_initial_position = False
        self.have_initial_position_from_pose = False
        self.speed = 0.0
        self.travel_length = 0.0
        self.acc_len = 0.0
        self.joy_x = 0.0
        self.joy_y = 0.0
        self.joy_z = 0.0
        self.enabled_flag = False
        self.target_yaw = 0.0
        self.x = x
        self.y = y
        self.z = z
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
            self.have_initial_position = True
            self.reset()

    def reset(self):
        self.time_elapsed = 0.0
        self.speed = 0.0
        self.phase = "acc"
        self.acc_len = 0.0

    def get_path_points(self):
        points = []
        time_span = np.arange(0, 2 * math.pi / self.angular_velocity, 0.1)

        for t in time_span:
            x = self.start_x + self.radius * math.sin(self.angular_velocity * t)
            y = self.start_y + self.radius * math.sin(2 * self.angular_velocity * t) / 2
            z = self.start_z  # Mantém altura constante

            point = Point()
            point.x = x
            point.y = y
            point.z = z
            points.append(point)
        return points

    def get_path(self, dt):
        pathmsg = Path()
        pathmsg.header.frame_id = "world"
        pathmsg.header.stamp.secs = 0
        pathmsg.header.stamp.nsecs = int(1000000000.0 * dt)

        points = self.get_path_points()
        for point in points:
            msg = PoseStamped()
            msg.header.frame_id = "world"
            msg.pose.position.x = point.x
            msg.pose.position.y = point.y
            msg.pose.position.z = point.z
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            pathmsg.poses.append(msg)

        return pathmsg

    def move_tick(self):
        # Mantém o método vazio para compatibilidade, mas sem funcionalidade.
        pass

    def tick(self, dt):
        if not self.enabled_flag:
            return

        # Fase de Aceleração
        if self.phase == "acc":
            self.speed += self.acc * dt
            if self.speed >= self.target_speed:
                self.speed = self.target_speed
                self.phase = "cruise"
            self.acc_len += self.speed * dt

        # Fase de Cruzeiro
        if self.phase == "cruise":
            if self.acc_len >= self.travel_length / 2.0:
                self.phase = "brake"

        # Fase de Desaceleração
        if self.phase == "brake":
            self.speed -= self.acc * dt
            if self.speed <= 0.0:
                self.speed = 0.0
                self.phase = "hover"

        # Atualiza a posição com base na fase atual e no tempo decorrido
        self.time_elapsed += dt
        self.x = self.start_x + self.radius * math.sin(self.angular_velocity * self.time_elapsed) * (self.speed / self.target_speed)
        self.y = self.start_y + self.radius * math.sin(2 * self.angular_velocity * self.time_elapsed) / 2 * (self.speed / self.target_speed)
        self.z = self.start_z  # Mantém altura constante

        # Calcula a orientação (yaw) baseada na direção do movimento
        self.target_yaw = math.atan2(self.y - self.start_y, self.x - self.start_x)
