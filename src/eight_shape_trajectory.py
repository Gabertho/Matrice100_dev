import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from std_msgs.msg import Float64

class EightShapeTrajectory:
    def __init__(self, x=None, y=None, z=None, speed=1.0):
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
        self.targets = []

        if self.start_x is not None and self.start_y is not None and self.start_z is not None:
            self.generate_eight_shape()

    def generate_eight_shape(self):
        # Gerar a forma de oito como uma lista de pontos
        if self.start_x is None or self.start_y is None or self.start_z is None:
            return  # Trajetória ainda não pode ser gerada

        targets = []
        num_points = 100
        for i in range(num_points):
            t = i * (2 * math.pi) / num_points
            x = self.start_x + self.radius * math.sin(t)
            y = self.start_y + self.radius * math.sin(2 * t) / 2
            z = self.start_z
            targets.append([x, y, z])
        self.targets = targets

    def set_initial_position(self, x, y, z):
        if self.start_x is None and self.start_y is None and self.start_z is None:
            # Defina a origem da trajetória como a posição inicial do drone
            self.start_x = x
            self.start_y = y
            self.start_z = z
            self.generate_eight_shape()  # Gera a trajetória apenas uma vez com a posição inicial
            print(f"EightShapeTrajectory: Posição inicial configurada - x: {x}, y: {y}, z: {z}")

        self.target_x = x
        self.target_y = y
        self.target_z = z

    def enabled(self):
        return self.enabled_flag

    def enable(self):
        if not self.enabled_flag and self.start_x is not None:
            self.enabled_flag = True
            self.time_elapsed = 0.0
            print("EightShapeTrajectory: Trajetória ativada")

    def disable(self):
        self.enabled_flag = False
        print("EightShapeTrajectory: Trajetória desativada")

    def notify_position(self, x, y, z):
        # Atualiza a posição atual do drone
        self.x = x
        self.y = y
        self.z = z
        print(f"EightShapeTrajectory: Posição atual notificada - x: {x}, y: {y}, z: {z}")

    def get_point_stamped(self):
        msg = PointStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = self.z
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

    def get_path(self, dt):
        # Retorna o caminho completo como uma mensagem de Path
        pathmsg = Path()
        pathmsg.header.frame_id = "world"
        pathmsg.header.stamp = rospy.Time.now()

        for target in self.targets:
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = target[0]
            pose.pose.position.y = target[1]
            pose.pose.position.z = target[2]
            pose.pose.orientation.w = 1.0  # Sem rotação
            pathmsg.poses.append(pose)

        return pathmsg

    def move_tick(self):
        if not self.enabled_flag:
            return
        # Atualiza o tempo e a posição do drone ao longo da trajetória em forma de oito
        self.time_elapsed += 0.02  # Considerando dt = 0.02
        index = int((self.time_elapsed * self.target_speed) % len(self.targets))
        self.target_x, self.target_y, self.target_z = self.targets[index]
        print(f"EightShapeTrajectory: Movendo para x: {self.target_x}, y: {self.target_y}, z: {self.target_z}")

    def tick(self, dt):
        if not self.enabled_flag:
            return
        self.move_tick()

    def move_target(self, joy_x, joy_y, joy_z):
        # Este método é necessário para compatibilidade, mas não precisa fazer nada para a trajetória em forma de oito.
        print(f"EightShapeTrajectory: move_target chamado com joy_x: {joy_x}, joy_y: {joy_y}, joy_z: {joy_z}")
