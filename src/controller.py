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
from scipy import linalg
import torch
import torch.nn as nn
import torch.optim as optim
import random

class ReplayBuffer:
    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer = []
    
    def add(self, state, uncertainty):
        if len(self.buffer) >= self.capacity:
            self.buffer.pop(0)
        self.buffer.append((state, uncertainty))
    
    def sample(self, batch_size):
        return random.sample(self.buffer, batch_size)
    
    def size(self):
        return len(self.buffer)

class DNN(nn.Module):
    def __init__(self, input_size, output_size):
        super(DNN, self).__init__()
        self.fc1 = nn.Linear(input_size, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, output_size)
    
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x


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

        #DMRAC Parameters
        self.dnn = DNN(input_size=4, output_size=2).to("cpu")
        self.replay_buffer = ReplayBuffer(capacity=250)
        self.optimizer = optim.Adam(self.dnn.parameters(), lr=0.0005)
        self.loss_fn = nn.MSELoss()
        self.adaptive_gain = 0.4
        self.last_layer_weight = np.zeros((128, 2))  # Adjust dimensions based on DNN output

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
            print("QUAT:", quat)
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

        # Update self.W to match dimensions
        self.W += -adaptation_term
        print("self.W.shape:", self.W.shape)

        return v_ad  # Return as a 1D array
    
    def dmrac_last_layer_weight_update(self, error, second_last_layer_output_basis):
        # Using Lyapunov matrix P and matrix B defined in the class
        P = self.P_lyap
        B = self.B

        # Ensure proper dimensions
        error = error.reshape(-1, 1)  # Transform error into a column vector
        second_last_layer_output_basis = second_last_layer_output_basis.reshape(1, -1)  # Transform to a row vector

        # Compute adaptation_term
        adaptation_term = (-self.dt) * self.adaptive_gain * (second_last_layer_output_basis.T @ (P @ B @ error))

        # Adjust dimensions of adaptation_term to match last_layer_weight
        # Assuming second_last_layer_output_basis has a shape compatible with last_layer_weight
        if adaptation_term.shape[1] != self.last_layer_weight.shape[1]:
            raise ValueError(f"Shape mismatch: adaptation_term.shape[1] {adaptation_term.shape[1]} "
                            f"does not match last_layer_weight.shape[1] {self.last_layer_weight.shape[1]}")

        # Update last_layer_weight
        self.last_layer_weight += adaptation_term




    def deep_mrac_control(self, second_last_layer_output_basis):
        self.vad = np.dot(self.last_layer_weight.T, second_last_layer_output_basis)
        return self.vad

    def buffer_fill_simple(self, current_iter):
        iter_number = current_iter % (self.replay_buffer.capacity - 1)
        state = np.array([self.current_position[0], self.velocity[0], self.current_position[1], self.velocity[1]])
        self.replay_buffer.add(state, self.vad)

    def dmrac_training(self, current_iter):
        if self.replay_buffer.size() > self.replay_buffer.capacity:
            random_indices = np.random.randint(0, self.replay_buffer.size(), 100)
            batch = [self.replay_buffer.buffer[i] for i in random_indices]
            input_data = np.array([data[0] for data in batch])
            output_data = np.array([data[1] for data in batch])
            
            input_tensor = torch.tensor(input_data, dtype=torch.float32).to("cpu")
            output_tensor = torch.tensor(output_data, dtype=torch.float32).to("cpu")
            
            for epoch in range(10):
                pred = self.dnn(input_tensor)
                loss = self.loss_fn(pred, output_tensor)
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
                print(current_iter, loss.item())



    

    # Control loop: Computes the control signal in different modes.

    def control(self, dt):
        print("DO CONTROL:", dt, self.control_mode, self.full_trajectory_flag)

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
            print("FULL TRAJ CONTROL:", dt, self.control_mode, target_x, target_y, target_z)
            self.target = np.array([target_x, target_y, target_z])
            self.targetvel = np.array([target_vx, target_vy, target_vz])
            
        # Error = desired x,y,z position - actual x,y,z position.
        error = self.target - self.current_position
        errorvel = self.targetvel - self.velocity

        # Thrust control
        pthrust = 1.5
        ithrust = 0.0019
        dthrust = 0.0
        #pthrust = 1.5
        #ithrust = 0.0
        ithrust = 0.01        
        #dthrust = 0.0
        velthrust = 6.0

        print("TARGET:", self.target)
        print("CUPOS:", self.current_position)
        print("THRUSTERROR:", error[2])
        print("HOVERTHRUST:", self.hover_thrust)

        self.int_err_z += error[2]
        d_err_z = (error[2] - self.old_err_z)/dt

        print("INTERROR:", self.int_err_z)
        print("D_ERR_Z:", d_err_z)

        # PID thrust: Kp.e+Ki.int_e+Kd.de/dt
        delta = error[2]*pthrust + ithrust*self.int_err_z + dthrust*d_err_z + velthrust*errorvel[2]

        print("DELTATHRUST:", delta)
        
        # Thrust control signal = thrust required to hover + PID output.
        u[2] = self.hover_thrust + delta

        if u[2] < 20.0:
            u[2] = 20.0
        if u[2] > 80.0:
            u[2] = 80.0

        self.old_err_z = error[2]

        # Yaw control = PD.
        pyaw = 1.0
        iyaw = 0.0
        dyaw = 0.0

        yaw_error = self.target_yaw - self.current_yaw
        print("YAW_ERROR:", yaw_error)
        
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

            if self.mode == "MRAC":
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

            if self.mode == "DMRAC":
                herror = np.array([error[0], error[1]]) 
                herrorvel = np.array([errorvel[0], errorvel[1]])
                theta = -self.current_yaw
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c)))  
                rherror = np.dot(R, herror)
                rherrorvel = np.dot(R, herrorvel)

                state = np.array([rherror[0], rherrorvel[0], rherror[1], rherrorvel[1]])

                second_last_layer_output_basis = self.dnn(torch.tensor(state, dtype=torch.float32).to("cpu"))[0].detach().numpy()
                self.dmrac_last_layer_weight_update(rherror, second_last_layer_output_basis)

                v_ad = self.deep_mrac_control(second_last_layer_output_basis)

                control_total = self.lqr_control(state, self.K) + v_ad

                u[0] = -math.radians(control_total[1]) 
                u[1] = -math.radians(control_total[0]) 

                max_angle = math.radians(20.0)
                u[0] = np.clip(u[0], -max_angle, max_angle)
                u[1] = np.clip(u[1], -max_angle, max_angle)

                self.buffer_fill_simple(current_iter=self.current_time)
                self.dmrac_training(current_iter=self.current_time)


                

            if self.mode == "simple_pid":
                print("========================================================================")
                
                print("ERROR:", error, self.target)

                herror = np.array([error[0], error[1]]) #2x1
                herrorvel = np.array([errorvel[0], errorvel[1]])
                print("HERROR:", math.degrees(self.current_yaw), herror)
                theta = -self.current_yaw
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c))) #2x2
                # print("R:", R)

                rherror = np.dot(R, herror)
                rherrorvel = np.dot(R, herrorvel)

                print("ROTATET HERROR:", rherror)
                print("ROTATET HERRORVEL:", rherrorvel)

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

        return (u, error[0], error[1], error[2])

        
