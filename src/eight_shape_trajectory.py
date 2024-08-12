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
        self.radius = 5.0  # Raio do círculo do "oito"
        self.angular_velocity = self.target_speed / self.radius
        self.time_elapsed = 0.0
        self.enabled_flag = False
        self.target_yaw = 0.0
        self.x = x
        self.y = y
        self.z = z

        self.path_points = self.calculate_eight_shape_path()
        self.current_path_index = 0

    def calculate_eight_shape_path(self):
        points = []
        times = np.arange(0.0, 2 * math.pi / self.angular_velocity, 0.1)
        for t in times:
            x = self.start_x + self.radius * math.sin(self.angular_velocity * t)
            y = self.start_y + self.radius * math.sin(2 * self.angular_velocity * t) / 2
            z = self.start_z
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            points.append(p)
        return points

    def enabled(self):
        return self.enabled_flag

    def enable(self):
        self.enabled_flag = True
        self.current_path_index = 0
        self.time_elapsed = 0.0
        print("EightShapeTrajectory: Trajetória ativada")

    def disable(self):
        self.enabled_flag = False
        print("EightShapeTrajectory: Trajetória desativada")

    def notify_position(self, x, y, z):
        if not self.enabled_flag:
            return
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

    def get_path_points(self):
        return self.path_points

    def get_path(self, dt):
        pathmsg = Path()
        pathmsg.header.frame_id = "world"
        pathmsg.header.stamp = rospy.Time.now()

        for point in self.path_points:
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
        # A trajetória é fixa, então move_tick não altera a trajetória
        pass

    def tick(self, dt):
        if not self.enabled_flag:
            return

        if self.current_path_index < len(self.path_points):
            self.x = self.path_points[self.current_path_index].x
            self.y = self.path_points[self.current_path_index].y
            self.z = self.path_points[self.current_path_index].z
            self.target_yaw = math.atan2(self.y - self.start_y, self.x - self.start_x)
            self.current_path_index += 1
        else:
            self.disable()  # Desativa após completar a trajetória
