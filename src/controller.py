#!/usr/bin/env python3

from tf.transformations import euler_from_quaternion
import numpy as np
import scipy
import scipy.ndimage
import math

class Controller:
    def __init__(self, control_mode):
        self.control_mode = control_mode
        self.current_position = np.array([0.0, 0.0, 10.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0
        self.roll = 0.0
        self.pitch = 0.0
        self.thrust = 0.0
        self.yaw_rate = 0.0
        self.target0 = None
        self.target = None
        self.target_speed = None
        self.target_yaw = 0.0
        self.have_target = False
        self.have_target0 = False
        self.have_current_yaw = False
        self.hover_thrust = 38.0
        self.old_err_pitch = 0.0
        self.old_err_roll = 0.0
        self.int_err_z = 0.0
        self.int_err_yaw = 0.0
        self.old_err_yaw = 0.0
        

    def set_hover_thrust(self, thrust):
        self.hover_thrust = thrust

    def notify_position(self, x, y, z):
        self.current_position = np.array([x, y, z])

    def notify_trajectory(self, x, y, z):
        if self.have_target:
            self.target0 = self.target
            self.have_target0 = True
        self.target = np.array([x, y, z])
        self.have_target = True
        if self.have_target0:
            dist = np.linalg.norm(self.target-self.target0)
            self.target_speed = dist/0.1
            # print("TARGET SPEED:", self.target_speed)
        

    def notify_velocity(self, x, y, z):
        self.velocity = np.array([x, y, z])

    def notify_attitude(self, qx, qy, qz, qw):
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
        # print("YAW:", math.degrees(yaw))
        self.current_yaw = yaw
        self.have_current_yaw = True

    def integral(self, prev_integral_value, error):
        # 0.001 is antiwindup gain
        # # if prev_integral_value >=2: # For modeling saturation
        # # prev_integral_value =2
        # # elif prev_integral_value <= -2:
        # # prev_integral_value =( -2)
        error_anti_windup = error -0.001* prev_integral_value
        integral_new = prev_integral_value + error_anti_windup * self.delt #delt is the period like 0.01 sec if 100hz.
        return integral_new
    
    def PID_control(self, Kp, Ki, Kd, error, int_error, d_error):
        PID_control_value = Kp * error + Ki * int_error - Kd *d_error
        return PID_control_value
    
    def adaptive_term(self):
        vad = np.dot(self.output_weight.T, self.basis)
        u_net = - vad
        return u_net
    
    def linear_Cntrl(self, state, ref_signal):
        # K =
        # Kr =
        upd = -K @ state #check it
        ucrm = Kr @ ref_signal 
        cntrl = fb+ff
        return cntrl
    
    def mrac_weight_update(self, ref_model_states):
        current_rpy_state = np.array([[self.roll], [self.D_roll], [self.pitch], [self.D_pitch], [self.yaw], [self.D_yaw]])
        error = ref_model_states - current_rpy_state
        #P = Solution for Lyapunov Equation
        P = np.array([[50.13, 0.0013, 0, 0, 0, 0], [0.0013, 0.1253, 0, 0, 0, 0],
                      [0, 0, 50.13, 0.0013, 0, 0], [0, 0, 0.0013, 0.1253, 0, 0],
                      [0, 0, 0, 0, 50.13, 0.0013], [0, 0, 0, 0, 0.0013, 0.1253]])

        B = np.array([[0, 0, 0], [1, 0, 0], [0, 0, 0],
                      [0, 1, 0], [0, 0, 0], [0, 0, 1]])

        self.output_weight = self.output_weight + (-self.delt) * (self.adaptive_gain) * np.dot(self.basis, np.dot(error.T, np.dot(P, B)))
        
    def control(self, dt):
        # print("DO CONTROL:", dt, self.control_mode)

        u = [0.0, 0.0, 38.0, 0.0]

        if not self.have_target0:
            print("Do not have target0")
            return u

        if not self.have_current_yaw:
            print("Do not have current_yaw")
            return u

        error = self.target - self.current_position

        #
        # Thrust
        #
        pthrust = 1.5
        ithrust = 0.0019
        dthrust = 6.0

        print("TARGET:", self.target)
        print("CUPOS:", self.current_position)
        print("THRUSTERROR:", error[2])

        int_err_z += error[2]
        d_err_z = (error[2] - self.old_err_z)/dt

        delta = error[2]*pthrust + ithrust*int_err_z + dthrust*d_err_z
        u[2] = self.hover_thrust + delta

        if u[2] < 20.0:
            u[2] = 20.0
        if u[2] > 80.0:
            u[2] = 80.0

        self.old_err_z = error[2]

        #
        # Yaw
        #

        pyaw = 1.0
        iyaw = 0.0
        dyaw = 0.5


        yaw_error = self.target_yaw - self.current_yaw
        print("YAW_ERROR:", yaw_error)
        
        self.int_err_yaw += yaw_error
        d_err = (yaw_error - self.old_err_yaw)/dt

        u[3] = yaw_error*pyaw + dyaw*derr

        if self.control_mode == "velocity":
            # print("ERROR:", error, self.target)
            P = 3.0
            u[0] = P*error[0]            # east
            u[1] = P*error[1]            # north

        if self.control_mode == "angles":
            if False:
                #FIRST: Converting x,y,z, yaw from reference trajectory to roll, pitch, yaw angles:
                #Assuming reference trajectory is x,y,z,yaw (adjust reference trajectory later to do this.)
                xref, yref, yaw_ref = self.target[0], self.target[1], self.target[3]

                error_x = xref - self.current_position[0]
                self.integration_val_x = self.integral(self.integration_val_x , error_x) #fiz these selfs etc to be oop

                error_y = yref - self.current_position[1]
                self.integration_val_y = self.integral(self.integration_val_y , error_y) #fiz these selfs etc to be oop

                #Implement the values of P, I, D and get the right velocities (dx dy)
                PID_x = self.PID_control(self.P_x, self.I_x, self.D_x, error_x , self.integration_val_x , self.dx)
                PID_y = self.PID_control(self.P_y, self.I_y , self.D_y, error_y , self.integration_val_y , self.dy)
                pitch_ref = PID_x*math.cos(self.yaw)+PID_y*math.sin(self.yaw) # Approx Model Inversion
                roll_ref = PID_x*math.sin(self.yaw)-PID_y*math.cos(self.yaw) # Approx Model Inversion

                #i am not sure if we should feed the reference model now and get its output or get the last output.

                vad = self.adaptive_term()

                total_control = self.linear_Cntrl(state,ref)

                u = total_control - vad

                #check dimensions etc
                
                #Update reference model and adaptive term weights.
                
                A.reference_model([yaw_ref_OL, pitch_ref_OL, roll_ref_OL ])
                A.mrac_weight_update(A.ref_model_states)
                
                # Now that we have our r(t) = roll_ref, pitch_ref, yaw_ref, we need to feed the reference model and integrate it.


            if True:
                # print("ERROR:", error, self.target)

                herror = np.array([error[0], error[1]])
                print("HERROR:", math.degrees(self.current_yaw), herror)

                theta = -self.current_yaw
                c, s = np.cos(theta), np.sin(theta)
                R = np.array(((c, -s), (s, c)))
                # print("R:", R)

                rherror = np.dot(R, herror)

                print("ROTATET HERROR:", rherror)

                derr_pitch = (rherror[0] - self.old_err_pitch)/dt
                derr_roll = (rherror[1] - self.old_err_roll)/dt


                self.old_err_pitch = rherror[0]
                self.old_err_roll = rherror[1]
            
                P = 2.0
                D = 4.0
                u[0] = math.radians(-(P*rherror[1] + D*derr_roll))       # roll
                u[1] = math.radians(P*rherror[0] + D*derr_pitch)         # pitch

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

        return u

        
