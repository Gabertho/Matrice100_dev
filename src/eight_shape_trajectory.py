import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseStamped, Point
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
        self.radius = 5.0  # Raio do círculo do "oito"
        self.angular_velocity = self.target_speed / self.radius
        self.time_elapsed = 0.0
        self.enabled_flag = False
        self.target_yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.targets = self.generate_eight_shape()

    def generate_eight_shape(self):
        # Gerar a forma de oito como uma lista de pontos
        targets = []
        num_points = 100
        for i in range(num_points):
            t = i * (2 * math.pi) / num_points
            x = self.start_x + self.radius * math.sin(t)
            y = self.start_y + self.radius * math.sin(2 * t) / 2
            z = self.start_z
            targets.append([x, y, z])
        return targets

    def enabled(self):
        return self.enabled_flag

    def enable(self):
        self.enabled_flag = True
        self.time_elapsed = 0.0
        print("EightShapeTrajectory: Trajetória ativada")

    def disable(self):
        self.enabled_flag = False
        print("EightShapeTrajectory: Trajetória desativada")

    def notify_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        print(f"EightShapeTrajectory: Posição atual notificada - x: {x}, y: {y}, z: {z}")

    def set_target(self, x, y, z):
        print(f"EightShapeTrajectory: Novo alvo definido - x: {x}, y: {y}, z: {z}")
        self.target_x = x
        self.target_y = y
        self.target_z = z

    def set_initial_position(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        print(f"EightShapeTrajectory: Posição inicial configurada - x: {x}, y: {y}, z: {z}")

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

    def get_path_points(self):
        # Retorna os pontos da trajetória em forma de oito
        points = []
        for target in self.targets:
            p = Point()
            p.x = target[0]
            p.y = target[1]
            p.z = target[2]
            points.append(p)
        return points

    def move_tick(self):
        if not self.enabled_flag:
            return
        # Atualiza o tempo e a posição do drone ao longo da trajetória em forma de oito
        self.time_elapsed += 0.02  # Considerando dt = 0.02
        index = int((self.time_elapsed * self.target_speed) % len(self.targets))
        self.target_x, self.target_y, self.target_z = self.targets[index]
        self.x, self.y, self.z = self.target_x, self.target_y, self.target_z
        print(f"EightShapeTrajectory: Movendo para x: {self.x}, y: {self.y}, z: {self.z}")

    def tick(self, dt):
        if not self.enabled_flag:
            return
        self.move_tick()
