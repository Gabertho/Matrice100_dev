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
        self.x = x
        self.y = y
        self.z = z
        self.acc_len = 0.0
        self.radius = 5.0  # Raio do círculo do "oito"
        self.angular_velocity = self.target_speed / self.radius
        self.time_elapsed = 0.0

        print(f"EightShapeTrajectory inicializada: x={x}, y={y}, z={z}, speed={speed}")

    def enabled(self):
        return self.enabled_flag

    def enable(self):
        self.enabled_flag = True
        print("EightShapeTrajectory: Trajetória ativada")

    def disable(self):
        self.enabled_flag = False
        print("EightShapeTrajectory: Trajetória desativada")

    def notify_position(self, x, y, z):
        if not self.have_initial_position_from_pose:
            self.have_initial_position_from_pose = True
            self.x = x
            self.y = y
            self.z = z
            print(f"EightShapeTrajectory: Posição inicial notificada - x: {x}, y: {y}, z: {z}")

    def set_target(self, x, y, z):
        print(f"EightShapeTrajectory: Novo alvo definido - x: {x}, y: {y}, z: {z}")
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.reset()

    def move_target(self, joy_x, joy_y, joy_z):
        self.joy_x = joy_x
        self.joy_y = joy_y
        self.joy_z = joy_z
        print(f"EightShapeTrajectory: Movendo alvo - joy_x: {joy_x}, joy_y: {joy_y}, joy_z: {joy_z}")

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
        print("EightShapeTrajectory: Reset da trajetória realizado")

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
        print(f"EightShapeTrajectory: Posição inicial configurada - x: {x}, y: {y}, z: {z}")
        self.x = x
        self.y = y
        self.z = z
        if not self.have_initial_position:
            self.reset()
            self.have_initial_position = True

    def get_path(self, dt):
        pathmsg = Path()
        pathmsg.header.frame_id = "world"
        pathmsg.header.stamp.secs = 0
        pathmsg.header.stamp.nsecs = int(1000000000.0 * dt)

        points = []
        for i in range(0, 360):
            t = math.radians(i)
            x = self.start_x + self.radius * math.sin(self.angular_velocity * t)
            y = self.start_y + self.radius * math.sin(2 * self.angular_velocity * t) / 2
            z = self.start_z

            point = PoseStamped()
            point.header.frame_id = "world"
            point.pose.position.x = x
            point.pose.position.y = y
            point.pose.position.z = z
            point.pose.orientation.w = 1.0

            pathmsg.poses.append(point)
            points.append((x, y, z))

        print(f"EightShapeTrajectory: Caminho gerado com {len(points)} pontos.")
        return pathmsg

    def move_tick(self):
        self.target_x += self.joy_x
        self.target_y += self.joy_y
        self.target_z += self.joy_z
        print(f"EightShapeTrajectory: move_tick chamado - target_x: {self.target_x}, target_y: {self.target_y}, target_z: {self.target_z}")

        if not self.enabled_flag:
            self.reset()
            return

    def tick(self, dt):
        if not self.enabled_flag:
            return

        # Atualiza a posição ao longo da trajetória em forma de oito
        self.time_elapsed += dt
        self.x = self.start_x + self.radius * math.sin(self.angular_velocity * self.time_elapsed)
        self.y = self.start_y + self.radius * math.sin(2 * self.angular_velocity * self.time_elapsed) / 2
        self.z = self.start_z

        self.target_yaw = math.atan2(self.y - self.start_y, self.x - self.start_x)

        print(f"EightShapeTrajectory: Posição atualizada (tick) - x: {self.x}, y: {self.y}, z: {self.z}, yaw: {self.target_yaw}")
