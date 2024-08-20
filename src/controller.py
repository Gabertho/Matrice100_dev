#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import euler_from_quaternion
import numpy as np
import scipy
import scipy.ndimage
import math
import scipy.linalg as sp
from scipy.linalg import svd
from scipy import linalg
import torch
import torch.nn as nn
import torch.optim as optim
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from std_srvs.srv import Trigger, TriggerResponse
import time
from mpl_toolkits.mplot3d import Axes3D

# Diretório para salvar as figuras
save_dir = '/home/lrs/lrs_ws/src/Matrice100_dev/plots'

# Certifique-se de que o diretório exista
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Architecture 1
"""class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.HL1 = nn.Linear(6, 20)
        self.HL2 = nn.Linear(20, 10)  # Ajuste o tamanho de saída para 6
        self.OL = nn.Linear(10, 3)
        self.optimizer = optim.Adam(self.parameters(), lr=0.005)
        self.loss_fn = nn.MSELoss()

    def forward(self, x):
        OL_1 = torch.tanh(self.HL1(x))
        OL_2 = torch.tanh(self.HL2(OL_1))
        OL_3 = self.OL(OL_2)
        return OL_2, OL_3"""

#Architecture 2
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.HL1 = nn.Linear(6, 32)
        self.HL2 = nn.Linear(32, 16)  # Ajuste o tamanho de saída para 6
        self.OL = nn.Linear(16, 3)
        self.optimizer = optim.Adam(self.parameters(), lr=0.005)
        self.loss_fn = nn.MSELoss()

    def forward(self, x):
        OL_1 = torch.tanh(self.HL1(x))
        OL_2 = torch.tanh(self.HL2(OL_1))
        OL_3 = self.OL(OL_2)
        return OL_2, OL_3


class Controller:
    def __init__(self, control_mode):
        self.control_mode = control_mode
        self.current_position = np.array([0.0, 0.0, 2.5])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0
        self.roll = 0.0
        self.pitch = 0.0
        self.thrust = 0.0
        self.yaw_rate = 0.0
        self.target = None
        self.target0 = None
        self.targetvel = None
        self.target_speed = 1.0
        self.target_yaw = 0.0
        self.have_target = False
        self.have_current_yaw = False
        self.hover_thrust = 38.0
        self.old_err_pitch = 0.0
        self.old_err_roll = 0.0
        self.int_err_z = 0.0
        self.int_err_yaw = 0.0
        self.old_err_yaw = 0.0
        self.old_err_z = 0.0
        self.yaw_control_flag = False
        self.mode = "DMRAC"
        self.trajectory_flag = False
        self.full_trajectory_x = None
        self.full_trajectory_y = None
        self.full_trajectory_z = None
        self.full_trajectory_time = None
        self.have_full_trajectory = False
        self.full_trajectory_flag = False
        self.current_time = 0.0
        self.sync_flag = False
        self.dt = 0.02
        #flag for start collecting data 
        self.data_collection_started = False
        self.servicos_chamados = False  # Adiciona a variável de controle

        # LQR Parameters
        self.g = 9.81
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0]
        ])

        self.B = np.array([
            [0, 0],
            [self.g, 0],
            [0, 0],
            [0, -self.g]
        ])
        # Weights for LQR
        #Brysons rule
        max_state_values = np.array([0.05, 0.05, 0.05, 0.05])  
        max_control_values = np.array([0.349066, 0.349066]) #20 degrees  
        
        self.Q = np.diag(1.0 / max_state_values**2)
        self.R = np.diag(1.0 / max_control_values**2)
        #self.Q = np.diag([1, 1, 1, 1])
        #self.R = np.diag([0.1, 0.1])

        # Solve Riccati equation
        self.P = scipy.linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)

        # Compute LQR gain
        self.K = np.dot(np.linalg.inv(self.R), np.dot(self.B.T, self.P))

        #MRAC LQR Parameters
        self.A_p = np.array([
            [0, 1, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0]
        ])

        self.B_p = np.array([
            [0, 0],
            [self.g, 0],
            [0, 0],
            [0, -self.g]
        ])

        self.Q_mrac = np.diag([1, 1, 1, 1])
        self.R_mrac = np.diag([0.1, 0.1])
        self.P_mrac = scipy.linalg.solve_continuous_are(self.A_p, self.B_p, self.Q_mrac, self.R_mrac)
        self.K_mrac = np.dot(np.linalg.inv(self.R_mrac), np.dot(self.B_p.T, self.P_mrac))

        # Reference Model
        self.Kx = -self.lqr(self.A_p, self.B_p, np.eye(4), np.eye(2))
        self.Am =  self.A + np.dot(self.B, self.Kx)
        self.Q_lyap = np.eye(4)
        self.P_lyap = sp.solve_continuous_lyapunov(self.Am, self.Q_lyap)
        # Adaptive Parameters
        self.W = np.zeros((5, 2))  # Adjust dimensions based on Phi(x)
        self.Gamma = 0.01 * np.eye(5)  # Learning rate matrix, set to 0.01  # Learning rate matrix


        # Thrust control
        self.m = 3.0 #mass

        self.A_thrust = np.array([
            [0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0]
        ])

        self.B_thrust = np.array([
            [0, 0, 0],
            [0, self.g, 0],
            [0, 0, 0],
            [-self.g, 0, 0],
            [0, 0, 0],
            [0, 0, 1/self.m]
        ])

        ##LQR with Thrust Control 
        #Brysons rule
        self.max_thrust = 80 #4 motors x 20N (maximum thrust per motor according to DJI)
        self.U1_max = self.max_thrust - (self.m*self.g) #
        max_state_values_thrust = np.array([0.06, 0.045, 0.06, 0.045, 0.06, 0.045])  #maximum error for x, dx, y, dy, z, dz 
        max_control_values_thrust = np.array([0.349066, 0.349066, self.U1_max/100]) #20 degrees for roll and pitch and x% for thrust

        self.Q_thrust = np.diag(1.0 / max_state_values_thrust**2)
        self.R_thrust = np.diag(1.0 / max_control_values_thrust**2)

        # Solve Riccati equation
        self.P_thrust = scipy.linalg.solve_continuous_are(self.A_thrust, self.B_thrust, self.Q_thrust, self.R_thrust)

        # Compute LQR gain
        self.K_thrust = np.dot(np.linalg.inv(self.R_thrust), np.dot(self.B_thrust.T, self.P_thrust))


        ##MRAC with Thrust Control
        #MRAC LQR Parameters
        self.Kx_thrust = -self.lqr(self.A_thrust, self.B_thrust, np.eye(6), np.eye(3))
        self.Am_thrust = self.A_thrust + np.dot(self.B_thrust, self.Kx_thrust)
        self.Q_lyap_thrust = np.eye(6)
        self.P_lyap_thrust = sp.solve_continuous_lyapunov(self.Am_thrust, self.Q_lyap_thrust)
        # Adaptive Parameters
        self.Wmrac_thrust = np.zeros((7,3))  # Adjust dimensions based on Phi(x)
        self.Gammamrac_thrust = 0.001 * np.eye(7)  # Learning rate matrix, set to 0.01  # Learning rate matrix

        #self.W_thrust = np.zeros((10,3))  # Adjust dimensions based on Phi(x)
        s#elf.Gamma_thrust = 0.001 * np.eye(10)  # Learning rate matrix, set to 0.01  # Learning rate matrix

        self.W_thrust = np.zeros((16,3))  # Adjust dimensions based on Phi(x)
        self.Gamma_thrust = 0.001 * np.eye(16)  # Learning rate matrix, set to 0.01  # Learning rate matrix

        #self.W_thrust = np.zeros((64,3))  # Adjust dimensions based on Phi(x)
        #self.Gamma_thrust = 0.001 * np.eye(64)  # Learning rate matrix, set to 0.01  # Learning rate matrix

        # DMRAC Parameters
        self.dnn = Net()
        self.replay_buffer = []
        self.buffer_size = 1000
        self.batch_size = 64
        self.zeta_tol = 2 * 0.00001

        #Counter for triggering DNN training
        self.new_samples_collected = 0
        self.training_interval = 100  # Train every 100 new samples

        #Data collecting
        self.desired_trajectory = []
        self.actual_trajectory = []
        self.desired_positions = []
        self.actual_positions = []
        self.time_stamps = []
        self.desired_velocities = []
        self.actual_velocities = []
        self.control_inputs = []

        rospy.Service('plot_trajectories', Trigger, self.plot_service_callback)
        rospy.Service('calculate_errors', Trigger, self.calculate_errors_service_callback)
 

    def set_sync(self, flag):
        self.sync_flag = flag

    # Set full trajectory: get all x, y, z positions and time t from full trajectory and set it.
    def set_full_trajectory(self, path):
        if self.have_full_trajectory:
            return
        x = [data.pose.position.x for data in path.poses]
        y = [data.pose.position.y for data in path.poses]
        z = [data.pose.position.z for data in path.poses]
        vx = []
        vy = []
        vz = []
        dt = path.header.stamp.nsecs/1000000000.0;
        for i in range(len(path.poses)-1):
            dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x
            dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y
            dz = path.poses[i+1].pose.position.z - path.poses[i].pose.position.z
            vx.append(dx/dt)
            vy.append(dy/dt)
            vz.append(dz/dt)
        vx.append(0.0)
        vy.append(0.0)
        vz.append(0.0)
            
        time = []
        t = 0.0
        for data in path.poses:
            time.append(t)
            t += dt
        self.full_trajectory_x = np.array(x)
        self.full_trajectory_y = np.array(y)
        self.full_trajectory_z = np.array(z)
        self.full_trajectory_vx = np.array(vx)
        self.full_trajectory_vy = np.array(vy)
        self.full_trajectory_vz = np.array(vz)
        self.full_trajectory_time = np.array(time)
        print(self.full_trajectory_time)
        print(self.full_trajectory_x)
        self.have_full_trajectory = True

    # Get all x,y,z positions of trajectory.
    def get_full_trajectory_points(self):
        res = []
        for i in range(len(self.full_trajectory_x)):
            p = Point()
            p.x = self.full_trajectory_x[i]
            p.y = self.full_trajectory_y[i]
            p.z = self.full_trajectory_z[i]
            res.append(p)
        return res

    # Get the pose (position + orientation) from target.
    def get_target_pose(self):
        if self.target.any():
            msg = PoseStamped()
            msg.header.frame_id = "world"
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = self.target[0]
            msg.pose.position.y = self.target[1]
            msg.pose.position.z = self.target[2]
            quat = quaternion_from_euler(0.0, 0.0, self.target_yaw)
            #print("QUAT:", quat)
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            return msg
        else:
            return None

#Enable and disable trajectory and full trajectory.
    def enable_trajectory(self):
        self.trajectory_flag = True

    def disable_trajectory(self):
        self.trajectory_flag = False

    def enable_full_trajectory(self):
        self.full_trajectory_flag = True

    def disable_full_trajectory(self):
        self.full_trajectory_flag = False

    # Reset integral error and full trajectory flag.
    def reset(self):
        #self.int_err_z = 0.0
        #self.int_err_yaw = 0.0        
        self.have_full_trajectory = False
        self.current_time = 0.0        
        
    def auto(self):
#        if not self.have_full_trajectory and not self.target:
        self.target = np.array([self.current_position[0], self.current_position[1], self.current_position[2] ])
        self.targetvel = np.array([0.0, 0.0, 0.0])

    # Enable / disable yaw control.
    def set_yaw_control(self, flag):
        self.yaw_control_flag = flag

    # Set required thrust to hover.
    def set_hover_thrust(self, thrust):
        self.hover_thrust = thrust

    # Notify_position: set current position from pose callback (vicon / imu).
    def notify_position(self, x, y, z):
        self.current_position = np.array([x, y, z])

    # Notify trajectory: set actual target position from trajectory callback.
    def notify_trajectory(self, x, y, z):
        if self.trajectory_flag:
            self.target = np.array([x, y, z])
            if self.have_target:
                self.targetvel = (self.target - self.target0)/self.dt
                pass
            self.target0 = np.array([x, y, z])
            self.have_target = True

    #Notify yaw trajectory: set actual yaw position from trajectory callback.
    def notify_yaw_trajectory(self, yaw):
        self.target_yaw = yaw

    #Notify velocity: set actual velocity from velocity callback (vicon / IMU). 
    def notify_velocity(self, x, y, z):
        self.velocity = np.array([x, y, z])

    #Notify angles: Set actual yaw from attitude callback (IMU).
    def notify_angles(self, roll, pitch, yaw):
        self.current_yaw = yaw
        self.have_current_yaw = True
    
    #Notify attittude: Set actual yaw transforming given quaterniuns to euler angles.
    def notify_attitude(self, qx, qy, qz, qw):
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
        # print("YAW:", math.degrees(yaw))
        self.current_yaw = yaw
        self.have_current_yaw = True

    # Setters
    
    def set_pitch(self, pitch):
        self.pitch = pitch

    def set_roll(self, roll):
        self.roll = roll
    
    def set_thrust(self, thrust):
        self.thrust = thrust

    def set_yawrate(self, yaw_rate):
        self.yaw_rate = yaw_rate

    # Função para calcular MSE e RMSE
    def calculate_mse_rmse(self, desired, actual):
        desired = np.array(desired)
        actual = np.array(actual)
        mse = np.mean((desired - actual) ** 2)
        rmse = np.sqrt(mse)
        return mse, rmse

    def plot_service_callback(self, req):
        self.plot_trajectories()
        self.plot_positions()
        self.plot_velocities()
        self.plot_control_inputs()
        self.plot_position_errors()
        self.plot_velocity_errors()
        return TriggerResponse(success=True, message="Dados plotados com sucesso.")
    
    # Implementação do serviço de cálculo de erros
    def calculate_errors_service_callback(self, req):
        mse_position, rmse_position = self.calculate_mse_rmse(self.desired_positions, self.actual_positions)
        mse_velocity, rmse_velocity = self.calculate_mse_rmse(self.desired_velocities, self.actual_velocities)
        
        message = (f"MSE Position: {mse_position:.4f}, RMSE Position: {rmse_position:.4f}\n"
                   f"MSE Velocity: {mse_velocity:.4f}, RMSE Velocity: {rmse_velocity:.4f}")
        
        # Armazenar a mensagem em um arquivo de texto na pasta dos plots
        file_path = os.path.join(save_dir, "error_metrics.txt")
        with open(file_path, "a") as file:
            file.write(message + "\n")
        
        return TriggerResponse(success=True, message=message)


    
  
    # Getters

    def get_yaw_control(self):
        return self.yaw_control_flag
    

    def lqr(self, A, B, Q, R):
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        K = np.dot(np.linalg.inv(R), np.dot(B.T, P))   
        return K
    
    def lqr_control(self, state, K):
        u = -np.dot(K, state)
        return u
    
    def adaptive_control(self, state, e):
        # Phi(x) includes the state and a bias term 1
        phi = np.append(state, 1).reshape(-1, 1)  # `phi` is (5, 1)
        print("phi.shape:", phi.shape)

        # Compute adaptive control law
        v_ad = np.dot(self.W.T, phi).flatten()  # `v_ad` is flattened to (2,)
        print("v_ad.shape:", v_ad.shape)

        # Update adaptive parameters
        e = e.reshape(-1, 1)  # `e` is (4, 1)
        print("e.shape:", e.shape)
        
        # Ensure correct matrix dimensions for adaptation term
        P_Bp = self.P_lyap @ self.B_p  # Resulting in a (4, 2) matrix
        print("P_Bp.shape:", P_Bp.shape)
        
        e_T_P_Bp = e.T @ P_Bp  # (1, 4) @ (4, 2) -> (1, 2)
        print("e_T_P_Bp.shape:", e_T_P_Bp.shape)
        
        phi_e_T_P_Bp = phi @ e_T_P_Bp  # (5, 1) @ (1, 2) -> (5, 2)
        print("phi_e_T_P_Bp.shape:", phi_e_T_P_Bp.shape)
        
        adaptation_term = self.Gamma @ phi_e_T_P_Bp * self.dt  # (5, 5) @ (5, 2) -> (5, 2)
        print("adaptation_term.shape:", adaptation_term.shape)

        # Update self.W to matchthrust dimensions
        self.W += -adaptation_term
        print("self.W.shape:", self.W.shape)

        return v_ad  # Return as a 1D array
    
    def adaptive_control_thrust(self, state, e):
        # Phi(x) includes the state and a bias term 1
        #print("Adaptive control thrust - state:", state)
        phi = np.append(state, 1).reshape(-1, 1)  # `phi` is (7, 1)
        #print("Phi:", phi)

        # Compute adaptive control law
        v_ad = np.dot(self.Wmrac_thrust.T, phi).flatten()  # `v_ad` is flattened to (3,)
        #print("v_ad:", v_ad)

        # Update adaptive parameters
        e = e.reshape(-1, 1)  # `e` is (6, 1)
        #print("e:", e)
        
        # Ensure correct matrix dimensions for adaptation term
        P_Bp = self.P_lyap_thrust @ self.B_thrust  # Resulting in a (6, 3) matrix
        #print("P_Bp:", P_Bp)
        
        e_T_P_Bp = e.T @ P_Bp  # (1, 6) @ (6, 3) -> (1, 3)
        #print("e_T_P_Bp:", e_T_P_Bp)
        
        phi_e_T_P_Bp = phi @ e_T_P_Bp  # (7, 1) @ (1, 3) -> (7, 3)
        #print("phi_e_T_P_Bp:", phi_e_T_P_Bp)
        
        adaptation_term = self.Gammamrac_thrust @ phi_e_T_P_Bp * self.dt  # (7, 7) @ (7, 3) -> (7, 3)
        #print("adaptation_term:", adaptation_term)

        # Update self.W_thrust to match dimensions
        self.Wmrac_thrust += -adaptation_term
        #print("self.W_thrust:", self.W_thrust)

        return v_ad  # Return as a 1D array
    
    def adaptive_control_thrust_dmrac(self, phi, e):
        # `phi` agora tem 10 elementos, ajustando o código para refletir isso
        phi = phi.reshape(-1, 1)  # `phi` agora é (10, 1)
        print(f"Phi for DMRAC: {phi.flatten()}")

        # Calcular a lei de controle adaptativo
        v_ad = np.dot(self.W_thrust.T, phi).flatten()  # `v_ad` agora é (3,)
        print(f"Adaptive control output (v_ad): {v_ad}")
        
        e = e.reshape(-1, 1)  # `e` deve ser (6, 1)

        # Adaptar as dimensões para garantir a compatibilidade
        P_Bp = self.P_lyap_thrust @ self.B_thrust  # Resulta em uma matriz (6, 3)
        e_T_P_Bp = e.T @ P_Bp  # Resulta em (1, 3)
        phi_e_T_P_Bp = phi @ e_T_P_Bp  # Resulta em (10, 3)

        # Termo de adaptação
        adaptation_term = self.Gamma_thrust @ phi_e_T_P_Bp * self.dt  # Resulta em (10, 3)

        # Atualiza `W_thrust` com as novas dimensões
        self.W_thrust += -adaptation_term
        print(f"Updated W_thrust: {self.W_thrust}")

        return v_ad  # Retorna como um array 1D

    def get_dnn_features(self, state):
        state_tensor = torch.FloatTensor(state).unsqueeze(0)  # Convert to tensor and add batch dimension
        #print("Original state:", state)
        #print("State tensor:", state_tensor)
        with torch.no_grad():
            features, _ = self.dnn(state_tensor)
        #print("Features before squeeze:", features)
        features = features.squeeze().numpy()  # Remove batch dimension and convert to numpy array
        #print("Features after squeeze:", features)
        return features  # Ensure this returns 6 elements



    def calculate_independence_metric(self, x, y):
        phi_x = self.get_dnn_features(x)
        y_pred = np.dot(self.W_thrust.T, phi_x)

        independence_metric = np.linalg.norm(y - y_pred)
        print(f"Calculated independence metric: {independence_metric}")
        return independence_metric

    
    def remove_least_representative(self):
        """
        Remove o dado menos representativo usando a decomposição SVD.
        """
        # Cria uma matriz dos dados no buffer
        buffer_matrix = np.array([np.hstack([x, y]) for x, y in self.replay_buffer])

        # Realiza a decomposição SVD
        _, _, vh = svd(buffer_matrix, full_matrices=False)

        # A amostra correspondente ao menor valor singular é a menos representativa
        least_representative_index = np.argmin(vh[-1])

        # Remove essa amostra do buffer
        del self.replay_buffer[least_representative_index]



    def update_replay_buffer(self, x, y):
        independence_metric = self.calculate_independence_metric(x, y)
        print(f"Independence metric: {independence_metric}, Threshold (zeta_tol): {self.zeta_tol}")

        if independence_metric > self.zeta_tol:
            self.replay_buffer.append((x, y))
            self.new_samples_collected += 1
            print(f"Sample added to replay buffer. New buffer size: {len(self.replay_buffer)}")

            if len(self.replay_buffer) > self.buffer_size:
                self.remove_least_representative()

            if self.new_samples_collected >= self.training_interval:
                print("Training DNN...")
                self.train_dnn()
                self.new_samples_collected = 0  # Reseta o contador de amostras
        else:
            print("Sample not added. Independence metric below threshold.")

 


    def train_dnn(self):
        if len(self.replay_buffer) < self.batch_size:
            print("Not enough samples to train DNN.")
            return
        # Converter a lista de tuplas em um array numpy
        replay_buffer_array = np.array(self.replay_buffer, dtype=object)
        # Selecionar índices aleatórios do buffer
        batch_indices = np.random.choice(len(replay_buffer_array), self.batch_size, replace=False)
        # Criar o batch a partir dos índices selecionados
        batch = replay_buffer_array[batch_indices]
        states, v_ads = zip(*batch)
        states = torch.FloatTensor(states)
        v_ads = torch.FloatTensor(v_ads)
        self.dnn.optimizer.zero_grad()
        _, predictions = self.dnn(states)
        loss = self.dnn.loss_fn(predictions, v_ads)
        loss.backward()
        self.dnn.optimizer.step()

        print(f"Training step complete. Loss: {loss.item()}")

    def plot_trajectories(self):
        desired = np.array(self.desired_trajectory)
        actual = np.array(self.actual_trajectory)
        
        plt.figure(figsize=(10, 7), dpi=300)
        ax = plt.axes(projection='3d')
        ax.plot3D(desired[:, 0], desired[:, 1], desired[:, 2], 'r-', label='Reference Trajectory', linewidth=2)
        ax.plot3D(actual[:, 0], actual[:, 1], actual[:, 2], 'b--', label='Actual Trajectory', linewidth=2)
        ax.set_xlabel('X (m)', fontsize=14)
        ax.set_ylabel('Y (m)', fontsize=14)
        ax.set_zlabel('Z (m)', fontsize=14)
        ax.set_title('Reference vs Actual Trajectories', fontsize=16, pad=20)  # Ajusta a posição do título
        ax.legend(fontsize=10, loc='lower left')  # Move a legenda para fora da área de plotagem
        ax.grid(True)
        file_path = os.path.join(save_dir, 'trajectories.png')
        plt.savefig(file_path, bbox_inches='tight')
        print(f"Figura salva em: {file_path}")
        plt.close()



    def plot_positions(self):
        desired = np.array(self.desired_positions)
        actual = np.array(self.actual_positions)
        time = np.array(self.time_stamps)
        
        plt.figure(figsize=(10, 7), dpi=300)
        plt.subplot(3, 1, 1)
        plt.plot(time, desired[:, 0], 'r-', label='Desired X', linewidth=2)
        plt.plot(time, actual[:, 0], 'b--', label='Actual X', linewidth=2)
        plt.ylabel('X Position (m)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)

        plt.subplot(3, 1, 2)
        plt.plot(time, desired[:, 1], 'r-', label='Desired Y', linewidth=2)
        plt.plot(time, actual[:, 1], 'b--', label='Actual Y', linewidth=2)
        plt.ylabel('Y Position (m)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)

        plt.subplot(3, 1, 3)
        plt.plot(time, desired[:, 2], 'r-', label='Desired Z', linewidth=2)
        plt.plot(time, actual[:, 2], 'b--', label='Actual Z', linewidth=2)
        plt.ylabel('Z Position (m)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)
        
        plt.suptitle('Desired vs Actual Positions Over Time', fontsize=16)
        # Salvando a figura
        file_path = os.path.join(save_dir, 'positions.png')
        plt.savefig(file_path)
        print(f"Figura salva em: {file_path}")
        plt.close()

    def plot_position_errors(self):
        desired = np.array(self.desired_positions)
        actual = np.array(self.actual_positions)
        time = np.array(self.time_stamps)

        errors = desired - actual
        
        plt.figure(figsize=(10, 7), dpi=300)
        plt.subplot(3, 1, 1)
        plt.plot(time, errors[:, 0], 'r-', label='X Error', linewidth=2)
        plt.ylabel('X Error (m)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)

        plt.subplot(3, 1, 2)
        plt.plot(time, errors[:, 1], 'g-', label='Y Error', linewidth=2)
        plt.ylabel('Y Error (m)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)

        plt.subplot(3, 1, 3)
        plt.plot(time, errors[:, 2], 'b-', label='Z Error', linewidth=2)
        plt.ylabel('Z Error (m)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)
        
        plt.suptitle('Position Errors Over Time', fontsize=16)
        # Salvando a figura
        file_path = os.path.join(save_dir, 'position_errors.png')
        plt.savefig(file_path)
        print(f"Figura salva em: {file_path}")
        plt.close()

    def plot_velocity_errors(self):
        desired = np.array(self.desired_velocities)
        actual = np.array(self.actual_velocities)
        time = np.array(self.time_stamps)

        errors = desired - actual
        
        plt.figure(figsize=(10, 7), dpi=300)
        plt.subplot(3, 1, 1)
        plt.plot(time, errors[:, 0], 'r-', label='Vx Error', linewidth=2)
        plt.ylabel('X Velocity Error (m/s)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)

        plt.subplot(3, 1, 2)
        plt.plot(time, errors[:, 1], 'g-', label='Vy Error', linewidth=2)
        plt.ylabel('Y Velocity Error (m/s)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)

        plt.subplot(3, 1, 3)
        plt.plot(time, errors[:, 2], 'b-', label='Vz Error', linewidth=2)
        plt.ylabel('Z Velocity Error (m/s)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)
        
        plt.suptitle('Velocity Errors Over Time', fontsize=16)
        # Salvando a figura
        file_path = os.path.join(save_dir, 'velocity_errors.png')
        plt.savefig(file_path)
        print(f"Figura salva em: {file_path}")
        plt.close()




    def plot_velocities(self):
        desired = np.array(self.desired_velocities)
        actual = np.array(self.actual_velocities)
        time = np.array(self.time_stamps)
        
        plt.figure(figsize=(10, 7), dpi=300)
        plt.subplot(3, 1, 1)
        plt.plot(time, desired[:, 0], 'r-', label='Desired Vx', linewidth=2)
        plt.plot(time, actual[:, 0], 'b--', label='Actual Vx', linewidth=2)
        plt.ylabel('X Velocity (m/s)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)

        plt.subplot(3, 1, 2)
        plt.plot(time, desired[:, 1], 'r-', label='Desired Vy', linewidth=2)
        plt.plot(time, actual[:, 1], 'b--', label='Actual Vy', linewidth=2)
        plt.ylabel('Y Velocity (m/s)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)

        plt.subplot(3, 1, 3)
        plt.plot(time, desired[:, 2], 'r-', label='Desired Vz', linewidth=2)
        plt.plot(time, actual[:, 2], 'b--', label='Actual Vz', linewidth=2)
        plt.ylabel('Z Velocity (m/s)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)
        
        plt.suptitle('Desired vs Actual Velocities Over Time', fontsize=16)
        # Salvando a figura
        file_path = os.path.join(save_dir, 'velocities.png')
        plt.savefig(file_path)
        print(f"Figura salva em: {file_path}")
        plt.close()



    def plot_control_inputs(self):
        if len(self.control_inputs) == 0:
            print("Nenhum dado de controle disponível para plotar.")
            return
        
        controls = np.array(self.control_inputs)
        
        if controls.ndim != 2 or controls.shape[1] != 4:
            print("Os dados de controle não têm a estrutura esperada. Esperado: (N, 4). Recebido:", controls.shape)
            return
        
        time = np.array(self.time_stamps)
        
        plt.figure(figsize=(10, 7), dpi=300)

        plt.subplot(3, 1, 1)
        plt.plot(time, controls[:, 0], 'g-', label='Roll', linewidth=2)
        plt.ylabel('Roll (rad)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)

        plt.subplot(3, 1, 2)
        plt.plot(time, controls[:, 1], 'r-', label='Pitch', linewidth=2)
        plt.ylabel('Pitch (rad)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)

        plt.subplot(3, 1, 3)
        plt.plot(time, controls[:, 2], 'b-', label='Thrust', linewidth=2)
        plt.ylabel('Thrust (%)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(fontsize=12)
        plt.grid(True)

        plt.suptitle('Control Inputs Over Time', fontsize=16)

        # Salvando a figura
        file_path = '/home/lrs/lrs_ws/src/Matrice100_dev/plots/control_inputs.png'
        plt.savefig(file_path)
        print(f"Figura salva em: {file_path}")
        plt.close()






    # Control loop: Computes the control signal in different modes.

    def control(self, dt):
        #print("DO CONTROL:", dt, self.control_mode, self.full_trajectory_flag)

        self.dt = dt

        u = [0.0, 0.0, 38.0, 0.0]

        if not self.have_current_yaw:
            print("Do not have current_yaw")
            return (u, 0.0, 0.0, 0.0)

        #if self.full_trajectory_flag and not self.have_full_trajectory:
        #    print("Do not have full trajectory")
        #    return (u, 0.0, 0.0, 0.0)

        # Interpolation to get the target x,y,z in the current time, given full trajectory time (x-axis) and full trajectory position (x,y or z)
        # (y axis).
        if self.full_trajectory_flag and self.have_full_trajectory:
            self.current_time += dt

            target_x = np.interp(self.current_time, self.full_trajectory_time, self.full_trajectory_x)
            target_y = np.interp(self.current_time, self.full_trajectory_time, self.full_trajectory_y)
            target_z = np.interp(self.current_time, self.full_trajectory_time, self.full_trajectory_z)
            target_vx = np.interp(self.current_time, self.full_trajectory_time, self.full_trajectory_vx)
            target_vy = np.interp(self.current_time, self.full_trajectory_time, self.full_trajectory_vy)
            target_vz = np.interp(self.current_time, self.full_trajectory_time, self.full_trajectory_vz)
            #print("FULL TRAJ CONTROL:", dt, self.control_mode, target_x, target_y, target_z)
            self.target = np.array([target_x, target_y, target_z])
            self.targetvel = np.array([target_vx, target_vy, target_vz])
            
        # Error = desired x,y,z position - actual x,y,z position.
        error = self.target - self.current_position
        errorvel = self.targetvel - self.velocity

        # Yaw control = PD.
        pyaw = 1.0
        iyaw = 0.0
        dyaw = 0.0

        yaw_error = self.target_yaw - self.current_yaw
        #print("YAW_ERROR:", yaw_error)
        
        self.int_err_yaw += yaw_error
        d_err_yaw = (yaw_error - self.old_err_yaw)/dt

        if self.yaw_control_flag:
            u[3] = yaw_error*pyaw + dyaw*d_err_yaw
        else:
            u[3] = 0.0
            
        if self.control_mode == "velocity":
            # print("ERROR:", error, self.target)
            P = 3.0
            u[0] = P*error[0]            # east
            u[1] = P*error[1]            # north

        if self.control_mode == "angles":
            if self.mode == "LQR":
                print("======================LQR===============================================")
                print("ERROR:", error, self.target)
    
                herror = np.array([error[0], error[1]]) # 2x1
                herrorvel = np.array([errorvel[0], errorvel[1]])
                print("HERROR:", math.degrees(self.current_yaw), herror)
                theta = -self.current_yaw
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c))) # 2x2
                rherror = np.dot(R, herror)
                rherrorvel = np.dot(R, herrorvel)
    
                print("ROTATED HERROR:", rherror)
                print("ROTATED HERRORVEL:", rherrorvel)
    
                # LQR control
                state = np.array([rherror[0], rherrorvel[0], rherror[1], rherrorvel[1]])
                control_input = self.lqr_control(state, self.K)
    
                u[0] = -math.radians(control_input[1]) # roll
                u[1] = -math.radians(control_input[0]) # pitch

                max = math.radians(20.0)
                if u[0] > max:
                    u[0] = max
                if u[0] < -max:
                    u[0] = -max
                if u[1] > max:
                    u[1] = max
                if u[1] < -max:
                    u[1] = -max

            if self.mode == "LQR_thrust":
                print("======================LQR WITH THRUST CONTROL===============================================")
                ## ROLL AND PITCH 
                #print("ERROR:", error, self.target)
    
                herror = np.array([error[0], error[1]]) # 2x1
                herrorvel = np.array([errorvel[0], errorvel[1]])
                #print("HERROR:", math.degrees(self.current_yaw), herror)
                theta = -self.current_yaw
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c))) # 2x2
                rherror = np.dot(R, herror)
                rherrorvel = np.dot(R, herrorvel)
                #print("ROTATED HERROR:", rherror)
                #print("ROTATED HERRORVEL:", rherrorvel)

                print("ERROR Z", error[2])
                print("ERROR Z VELOCITY", errorvel[2])

    
                # LQR control
                state = np.array([rherror[0], rherrorvel[0], rherror[1], rherrorvel[1], error[2], errorvel[2]]) # errors x, dx, y, dy, z, dz
                print("LQR STATE: ", state)

                control_input = self.lqr_control(state, self.K_thrust)

                print("LQR CONTROL INPUT:", control_input)


                # Roll and pitch
                u[0] = -math.radians(control_input[0]) # roll
                u[1] = -math.radians(control_input[1]) # pitch

                max = math.radians(20.0)
                if u[0] > max:
                    u[0] = max
                if u[0] < -max:
                    u[0] = -max
                if u[1] > max:
                    u[1] = max
                if u[1] < -max:
                    u[1] = -max

                # Thrust
                thrust_force = -control_input[2]
                delta_thrust_percentage = (thrust_force / self.max_thrust) * 100
                print("LQR THRUST FORCE:", thrust_force)
                print("LQR MAX THRUST:", self.max_thrust)
                thrust_percentage = self.hover_thrust + delta_thrust_percentage
                u[2] = thrust_percentage

                print("LQR DELTA THRUST:", delta_thrust_percentage)

                if u[2] < 20.0:
                    u[2] = 20.0
                if u[2] > 80.0:
                    u[2] = 80.0
                
                print("LQR THRUST:", u[2])
                
            if self.mode == "MRAC":
                print("======================MRAC===============================================")
                herror = np.array([error[0], error[1]])  # 1D array with shape (2,)
                herrorvel = np.array([errorvel[0], errorvel[1]])
                print("herror:", herror)
                print("herrorvel:", herrorvel)
                
                theta = -self.current_yaw
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c)))  # 2x2
                rherror = np.dot(R, herror)
                rherrorvel = np.dot(R, herrorvel)
                print("rherror:", rherror)
                print("rherrorvel:", rherrorvel)

                # Define state for MRAC similar to LQR
                state = np.array([rherror[0], rherrorvel[0], rherror[1], rherrorvel[1]])
                print("state:", state)

                # Calculate LQR control action
                control_input = self.lqr_control(state, self.K)  # control_input is (2,)
                print("control_input:", control_input)

                # Compute adaptive control law
                mrac_error = state  # Ensure `mrac_error` is the same dimension as `state`
                v_ad = self.adaptive_control(state, mrac_error)
                print("v_ad:", v_ad)

                # Combine LQR and adaptive control laws
                control_total = control_input + v_ad  # Both should be (2,)
                print("control_total:", control_total)

                # Define roll and pitch based on control action
                u[0] = -math.radians(control_total[1])  # Roll
                u[1] = -math.radians(control_total[0])  # Pitch

                max = math.radians(20.0)
                max = math.radians(20.0)
                if u[0] > max:
                    u[0] = max
                if u[0] < -max:
                    u[0] = -max
                if u[1] > max:
                    u[1] = max
                if u[1] < -max:
                    u[1] = -max


            if self.mode == "MRAC_thrust":
                print("======================MRAC WITH THRUST CONTROL===============================================")
                ## ROLL AND PITCH 
                herror = np.array([error[0], error[1]])  # 1D array with shape (2,)
                herrorvel = np.array([errorvel[0], errorvel[1]])
                #print("herror:", herror)
                #print("herrorvel:", herrorvel)
                
                theta = -self.current_yaw
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c)))  # 2x2
                rherror = np.dot(R, herror)
                rherrorvel = np.dot(R, herrorvel)
                #print("rherror:", rherror)
                #print("rherrorvel:", rherrorvel)

                # Define state for MRAC similar to LQR
                state = np.array([rherror[0], rherrorvel[0], rherror[1], rherrorvel[1], error[2], errorvel[2]])
                #print("state:", state)

                # Calculate LQR control action
                control_input = self.lqr_control(state, self.K_thrust)  # control_input is (3,)
                #print("control_input:", control_input)

                # Compute adaptive control law
                mrac_error = state  # Ensure `mrac_error` is the same dimension as `state`
                v_ad = self.adaptive_control_thrust(state, mrac_error)
                #print("v_ad:", v_ad)

                # Combine LQR and adaptive control laws
                control_total = control_input + v_ad  # Both should be (3,)
                #print("control_total:", control_total)

                # Define roll and pitch based on control action
                u[0] = -math.radians(control_total[0])  # Roll
                u[1] = -math.radians(control_total[1])  # Pitch

                #print("MRAC roll before clip:", u[0])
                #print("MRAC pitch before clip:", u[1])

                max = math.radians(20.0)
                if u[0] > max:
                    u[0] = max
                if u[0] < -max:
                    u[0] = -max
                if u[1] > max:
                    u[1] = max
                if u[1] < -max:
                    u[1] = -max

                

                # Thrust
                thrust_force = -control_total[2]
                delta_thrust_percentage = (thrust_force / self.max_thrust) * 100
                #print("MRAC THRUST FORCE:", thrust_force)
                #print("MRAC MAX THRUST:", self.max_thrust)
                thrust_percentage = self.hover_thrust + delta_thrust_percentage
                u[2] = thrust_percentage

                #print("MRAC DELTA THRUST:", delta_thrust_percentage)

                if u[2] < 20.0:
                    u[2] = 20.0
                if u[2] > 80.0:
                    u[2] = 80.0


                #print("MRAC roll after clip:", u[0])
                #print("MRAC pitch after clip:", u[1])

                #print("MRAC THRUST:", u[2])

            if self.mode == "DMRAC":
                print("======================MRAC DNN WITH THRUST CONTROL===============================================")
                ## ROLL AND PITCH 
                herror = np.array([error[0], error[1]])  # 1D array with shape (2,)
                herrorvel = np.array([errorvel[0], errorvel[1]])
                #print("herror:", herror)
                #print("herrorvel:", herrorvel)
                
                theta = -self.current_yaw
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c)))  # 2x2
                rherror = np.dot(R, herror)
                rherrorvel = np.dot(R, herrorvel)
                #print("rherror:", rherror)
                #print("rherrorvel:", rherrorvel)

                # Define state for MRAC similar to LQR
                state = np.array([rherror[0], rherrorvel[0], rherror[1], rherrorvel[1], error[2], errorvel[2]])
                print("state:", state)

                control_input = self.lqr_control(state, self.K_thrust)
                #print("lqr roll:", math.radians(control_input[0]))
                #print("lqr pitch:", math.radians(control_input[1]))
                #print("lqr thrust:", control_input[2])
                mrac_error = state

                phi = self.get_dnn_features(state)
                #print("phi:", phi)
                v_ad = self.adaptive_control_thrust_dmrac(phi, mrac_error)
                #print("vad roll:", math.radians(v_ad[0]))
                #print("vad pitch:", math.radians(v_ad[1]))
                #print("vad thrust:", v_ad[2])
                self.update_replay_buffer(state, v_ad)
                print(f"Replay buffer length after update: {len(self.replay_buffer)}")

                control_total = control_input + v_ad
                #print("control_total:", control_total)

                u[0] = -math.radians(control_total[0])  # Roll
                u[1] = -math.radians(control_total[1])  # Pitch

                max_angle = math.radians(20.0)
                if u[0] > max_angle:
                    u[0] = max_angle
                if u[0] < -max_angle:
                    u[0] = -max_angle
                if u[1] > max_angle:
                    u[1] = max_angle
                if u[1] < -max_angle:
                    u[1] = -max_angle

                thrust_force = -control_total[2]
                delta_thrust_percentage = (thrust_force / self.max_thrust) * 100
                thrust_percentage = self.hover_thrust + delta_thrust_percentage
                u[2] = thrust_percentage

                if u[2] < 20.0:
                    u[2] = 20.0
                if u[2] > 80.0:
                    u[2] = 80.0

                self.train_dnn()



            if self.mode == "simple_pid":
                print("========================================================================")
                
                #print("ERROR:", error, self.target)

                herror = np.array([error[0], error[1]]) #2x1
                herrorvel = np.array([errorvel[0], errorvel[1]])
                #print("HERROR:", math.degrees(self.current_yaw), herror)
                theta = -self.current_yaw
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c))) #2x2
                # print("R:", R)

                rherror = np.dot(R, herror)
                rherrorvel = np.dot(R, herrorvel)

                #print("ROTATET HERROR:", rherror)
                #print("ROTATET HERRORVEL:", rherrorvel)

                # Thrust control
                pthrust = 1.5
                ithrust = 0.0019
                dthrust = 0.0
                #pthrust = 1.5
                #ithrust = 0.0
                ithrust = 0.01        
                #dthrust = 0.0
                velthrust = 6.0

                #print("TARGET:", self.target)
                #print("CUPOS:", self.current_position)
                #print("THRUSTERROR:", error[2])
                #print("HOVERTHRUST:", self.hover_thrust)

                self.int_err_z += error[2]
                d_err_z = (error[2] - self.old_err_z)/dt

                #print("INTERROR:", self.int_err_z)
                #print("D_ERR_Z:", d_err_z)

                # PID thrust: Kp.e+Ki.int_e+Kd.de/dt
                delta = error[2]*pthrust + ithrust*self.int_err_z + dthrust*d_err_z + velthrust*errorvel[2]

                #print("PID DELTATHRUST:", delta)
                
                # Thrust control signal = thrust required to hover + PID output.
                u[2] = self.hover_thrust + delta

                if u[2] < 20.0:
                    u[2] = 20.0
                if u[2] > 80.0:
                    u[2] = 80.0

                #print("PID THRUST:", u[2])

                self.old_err_z = error[2]

                #PID control
                derr_pitch = (rherror[0] - self.old_err_pitch)/dt
                derr_roll = (rherror[1] - self.old_err_roll)/dt

                self.old_err_pitch = rherror[0]
                self.old_err_roll = rherror[1]
            
                P = 3.0
                D = 0.0
                Pvel = 10.0                
                if self.sync_flag:
                    P = 4.0
                    D = 0.0
                    Pvel = 10.0
                u[0] = math.radians(-(P*rherror[1] + D*derr_roll) - Pvel*rherrorvel[1])       # roll
                u[1] = math.radians(P*rherror[0] + D*derr_pitch + Pvel*rherrorvel[0])         # pitch

                max = math.radians(20.0)
                if u[0] > max:
                    u[0] = max
                if u[0] < -max:
                    u[0] = -max
                if u[1] > max:
                    u[1] = max
                if u[1] < -max:
                    u[1] = -max

        if self.control_mode == "rates":
            # print("ERROR:", error, self.target)

            herror = np.array([error[0], error[1]])
            print("HERROR:", math.degrees(self.current_yaw), herror)

            theta = self.current_yaw
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
            # print("R:", R)

            rherror = np.dot(R, herror)

            print("ROTATET HERROR:", rherror)
            
            P = 0.5
            u[0] = -P*rherror[1]             # roll rate
            u[1] = P*rherror[0]            # pitch rate

            print("U:", u)

            max = 5.0/6.0*math.pi
            if u[0] > max:
                u[0] = max
            if u[0] < -max:
                u[0] = -max
            if u[1] > max:
                u[1] = max
            if u[1] < -max:
                u[1] = -max


        # Coletando dados para plotagem
        self.desired_trajectory.append((self.target[0], self.target[1], self.target[2]))
        self.actual_trajectory.append((self.current_position[0], self.current_position[1], self.current_position[2]))
        self.desired_positions.append(self.target)
        self.actual_positions.append(self.current_position)
        self.time_stamps.append(self.current_time)
        self.desired_velocities.append(self.targetvel)
        self.actual_velocities.append(self.velocity)
        self.control_inputs.append((u[0], u[1], u[2], u[3]))

        tempo_adicional = 5.0  # tempo adicional em segundos
        if not self.servicos_chamados and self.current_time >= self.full_trajectory_time[-1] + tempo_adicional:
            # Chamar os serviços automaticamente ao final do controle
            self.plot_service_callback(None)
            self.calculate_errors_service_callback(None)

            # Marcar que os serviços foram chamados
            self.servicos_chamados = True

                

        return (u, error[0], error[1], error[2])

        
