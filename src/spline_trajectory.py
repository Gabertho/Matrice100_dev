import rospy

from scipy.interpolate import CubicSpline
from scipy.interpolate import make_interp_spline

import math
import numpy as np

from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from std_msgs.msg import Float64

# General idea: The drone movement is divided in four phases: acceleration, cruise, brake and hover. 
# Acceleration: We begin in this phase and continue doing it until we reached the desired speed or half of the total travel lenght 
# (because since we are moving with a constant acceleration, we would have the other half to brake and then hover). 
# Cruise: We want to continue on it until the distance to target becomes smaller than the distance traveled in acceleration phase 
# (because this would give the exact distance to brake it and hover, since we are moving on a constant speed).
# Brake: We want to do it until our speed becomes 0, meaning we reached the target position, and then hover.
# Hover: Just remain on the air without moving.
# So the idea is:
# --acc-->--cruise-->--brake-->hover or --acc-->--brake-->hover

class SplineTrajectory:
    def __init__(self, x, y, z, speed, fixed_targets=None):
        # Coordinates of desired point (goal), and desired speed and acceleration to reach it.
        self.targets = [np.array([x, y, z])]
        if fixed_targets:
            self.targets.extend([np.array(t) for t in fixed_targets])
        self.target_speed = speed
        self.acc = 1.0
        self.have_initial_position_from_pose = False
        self.joy_x = 0.0
        self.joy_y = 0.0
        self.joy_z = 0.0
        self.target_index = 0
        self.enabled_flag = False
        self.target_yaw = math.pi / 4
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.have_changed_index = False
        self.spline_x = None
        self.spline_y = None
        self.spline_z = None

    def get_spline_length(self, start, end, spline_x, spline_y, spline_z):
        try:
            itimes = np.linspace(start, end, 100)
            x = spline_x(itimes)
            y = spline_y(itimes)
            z = spline_z(itimes)
            tlen = 0.0
            for i in range(1, len(x)):
                dx = x[i] - x[i-1]
                dy = y[i] - y[i-1]
                dz = z[i] - z[i-1]
                # print("I:", i, dx, dy, dz)
                tlen += math.sqrt(dx*dx+dy*dy+dz*dz)
            return tlen
        except Exception as e:
            print("EXCEPTION get_spline_length:", e)
        return 0.0

    def plot_speed(self):
        times = np.arange(0.0, self.tot_time, 0.1)
        x = self.spline_x(times)
        y = self.spline_y(times)
        z = self.spline_z(times)
        print("X:", x)
        print("Y:", y)
        print("Z:", z)
        res = []
        for i in range(1, len(x)):
            dx = x[i] - x[i-1]
            dy = y[i] - y[i-1]
            dz = z[i] - z[i-1]
            res.append(math.sqrt(dx*dx+dy*dy+dz*dz))
        #print("X:", [t[0] for t in self.targets])
        #print("Y:", [t[1] for t in self.targets])
        #print("Z:", [t[2] for t in self.targets])
        print("SPEED DISTANCE:", res)
    
    def update_spline(self):
        try:
            if not self.have_initial_position_from_pose:
                return
            
            if self.spline_x is not None:  # Já foi gerada uma vez
                return

            time = [0.0]
            for i in range(len(self.targets)):
                time.append(time[i] + 1.0/len(self.targets)*(i+1))
            #print("TIME:", time)
            #print("TARGETS:", self.targets)
            try:
                spline_x = CubicSpline(time, [self.x] + [t[0] for t in self.targets])
                spline_y = CubicSpline(time, [self.y] + [t[1] for t in self.targets])
                spline_z = CubicSpline(time, [self.z] + [t[2] for t in self.targets])
            except:
                print("DUPLICATE TIMES")
                return

            seglengths = []
            for i in range(1, len(time)):
                start = time[i-1]
                end = time[i]
                seglengths.append(self.get_spline_length(start, end, spline_x, spline_y, spline_z))
            spline_tot_length = sum(seglengths)
            # print("SPLINE TOTLEN:", tot_length, spline_tot_length, seglengths)
            
            acctime = 0.0
            time = [0.0]
            for seglen in seglengths:
                acctime += seglen/self.target_speed
                time.append(acctime)
            # print("TIME REAL:", time)
            self.tot_time = time[-1]
            
            try:
                self.spline_x = CubicSpline(time, [self.x] + [t[0] for t in self.targets])
                self.spline_y = CubicSpline(time, [self.y] + [t[1] for t in self.targets])
                self.spline_z = CubicSpline(time, [self.z] + [t[2] for t in self.targets])
            except:
                print("DUPLICATE TIMES")
                return
                
            ### self.plot_speed()
            
        except Exception as e:
            print("EXCEPTION update_spline:", e)
                

                

    def next_target(self):
        if self.have_changed_index:
            return
        if len(self.targets) == (self.target_index + 1):
            t = self.targets[-1]
            self.targets.append(np.array([t[0], t[1], t[2]]))
        self.target_index += 1
        print("TARGET INDEX:", self.target_index)
        self.have_changed_index = True
        self.update_spline()
        
    def previous_target(self):
        if self.have_changed_index:
            return
        self.target_index -= 1
        if self.target_index < 0:
            self.target_index = 0
        print("TARGET INDEX:", self.target_index)
        self.have_changed_index = True
        self.update_spline()
        
    def get_path_points(self):
        res = []
        if not self.have_initial_position_from_pose:
            return res

        times = np.arange(0.0, self.tot_time, 0.1)
        #print("TIMES:", times)
        x_interpolate = self.spline_x(times)
        y_interpolate = self.spline_y(times)
        z_interpolate = self.spline_z(times)
        #print("X_INTERPOLATE:", x_interpolate)
        #print("Y_INTERPOLATE:", y_interpolate)
        #print("Z_INTERPOLATE:", z_interpolate)
        for i in range(len(x_interpolate)):
            p = Point()
            p.x = x_interpolate[i]
            p.y = y_interpolate[i]
            p.z = z_interpolate[i]
            res.append(p)
        return res

    def new_spline(self):
        self.x = self.targets[-1][0]
        self.y = self.targets[-1][1]
        self.z = self.targets[-1][2]
        self.targets = [np.array([self.x, self.y, self.z])]
        self.target_index = 0
    
    def enabled(self):
        return self.enabled_flag

    def enable(self):
        self.new_spline()
        self.enabled_flag = True

    def disable(self):
        self.enabled_flag = False

    def notify_position(self, x, y, z):
        if not self.have_initial_position_from_pose:
            print("notify_position", self.have_initial_position_from_pose)
            self.have_initial_position_from_pose = True
            self.x = x
            self.y = y
            self.z = z
        
    #def set_target(self, x, y, z):
    #    print("set_target:", x, y, z)
    #    self.target_x = x
    #    self.target_y = y
    #    self.target_z = z
    #    self.reset()


    def move_target(self, joy_x, joy_y, joy_z):
        self.joy_x = joy_x
        self.joy_y = joy_y
        self.joy_z = joy_z
        
    #
    # This must be fixed
    #
    def reset(self):
        return

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

    #def get_target_point_stamped(self):
    #    msg = PointStamped()
    #    msg.header.frame_id = "world"
    #    msg.header.stamp = rospy.Time.now()
    #    msg.point.x = self.target_x
    #    msg.point.y = self.target_y
    #    msg.point.z = self.target_z
    #    return msg

    def get_target_yaw(self):
        msg = Float64()
        msg.data = self.target_yaw
        return msg

    def set_initial_position(self, x, y, z):
        # This is target/pose in got.Not neededhere since pressing button sets the start point to last target in spline
        return

    def find_time_from_speed(self, t0, speed, dt, deltat=0.001):
        x0 = self.spline_x(t0)
        y0 = self.spline_y(t0)
        z0 = self.spline_z(t0)
        distance = speed*dt
        # print("find_time_from_speed:", t0, speed, dt, deltat)
        tot_time = self.tot_time
        if deltat < 0.0:
            tot_time = 0.0
        for t in np.arange(t0, tot_time, deltat):
            x = self.spline_x(t)
            y = self.spline_y(t)
            z = self.spline_z(t)
            dx = x - x0
            dy = y - y0
            dz = z - z0
            if dx*dx+dy*dy+dz*dz > distance*distance:
                # print("FOUND:", t)
                return t
        print("find_time_from_speed:", t0, speed, dt, deltat)            
        print("ERROR: FAILED TO FIND TIME")
        return self.tot_time

    def dist_to_time(self, t0, t1, dt):
        x0 = self.spline_x(t0)
        y0 = self.spline_y(t0)
        z0 = self.spline_z(t0)
        t = t0+dt
        distance = 0.0
        while t <= t1:
            x = self.spline_x(t)
            y = self.spline_y(t)
            z = self.spline_z(t)
            dx = x - x0
            dy = y - y0
            dz = z - z0
            distance += math.sqrt(dx*dx+dy*dy+dz*dz)
            x0 = x
            y0 = y
            z0 = z
            t += dt
        return distance

    # Get the full path, i.e, sequence of points in time that lead the initial position to the target position.
    def get_path(self, dt):
        # contruct from spline, add acceleration and brake
        # need to compute lenght of spline...

        pathmsg = Path()

        pathmsg.header.frame_id = "world"
        pathmsg.header.stamp.secs = 0
        pathmsg.header.stamp.nsecs = int(1000000000.0*dt)

        current_time = 0.0
        brake_current_time = self.tot_time
        speed = 0.0
        times = [0.0]
        brake_times = [self.tot_time]

        # print("SPEED - TARGET SPEED", speed, self.target_speed)
        while speed <= self.target_speed:
            speed += self.acc*dt
            # current_time += dt*speed/self.target_speed
            current_time = self.find_time_from_speed(current_time, speed, dt)
            brake_current_time = self.find_time_from_speed(brake_current_time, speed, dt, deltat=-0.001)
            times.append(current_time)
            brake_times.append(brake_current_time)
            # print("SPEED - TARGET SPEED", speed, self.target_speed)

        brake_times.reverse()
        
        speed = self.target_speed
        print("END ACC PHASE:", current_time)

        brake_time = brake_times[0]

        while current_time < brake_time:
            current_time = self.find_time_from_speed(current_time, speed, dt)
            ## current_time+=dt
            if current_time < brake_time:
                times.append(current_time)

        print("END CRUISE PHASE:", current_time)

        times = times + brake_times

            
        #print("END BRAKE PHASE:", current_time, self.tot_time)

        ### times = np.arange(0.0, self.tot_time, dt)        

        x_interpolate = self.spline_x(times)
        y_interpolate = self.spline_y(times)
        z_interpolate = self.spline_z(times)

        for i in range(len(x_interpolate)):
            msg = PoseStamped()
            msg.header.frame_id = "world"
            msg.pose.position.x = x_interpolate[i]
            msg.pose.position.y = y_interpolate[i]
            msg.pose.position.z = z_interpolate[i]
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            pathmsg.poses.append(msg)

        # print("RETURN pathmsg")
        return pathmsg

    def move_tick(self):
        # Moving target position with joystick.
        try:
            self.targets[self.target_index] += np.array([self.joy_x, self.joy_y, self.joy_z])
            self.update_spline()
        except Exception as e:
            print("EXCEPTION move_tick:", self.target_index, e, self.targets)
        
    # Tick: Updates trajectory position in each iteration.
    def tick(self, dt):
        # This mode does not work for spline
        return

            
