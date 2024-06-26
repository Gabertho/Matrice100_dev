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
    def __init__(self, x, y, z, speed):
        # Coordinates of desired point (goal), and desired speed and acceleration to reach it.
        self.targets = [np.array([x, y, z])]
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
        self.target_index = 0
        self.enabled_flag = False
        self.target_yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.have_changed_index = False
        self.spline_x = None
        self.spline_y = None
        self.spline_z = None

    def get_target_path_length(self):
        x0 = self.x
        y0= self.y
        z0 = self.z
        length = 0.0
        lengths = []
        for i in range(0, len(self.targets)):
            t = self.targets[i]
            dx = t[0] - x0
            dy = t[1] - y0
            dz = t[2] - z0
            x0 = t[0]
            y0 = t[1]
            z0 = t[2]
            l = math.sqrt(dx*dx+dy*dy+dx*dz)
            length += l
            lengths.append(l)
        #print("LENGTH:", length)
        #print("LENGTHS:", lengths)
        return (length, lengths)
    
    def update_spline(self):
        if not self.have_initial_position_from_pose or self.have_initial_position:
            return
        (length, lengths) = self.get_target_path_length()
        tot_length = length
        self.tot_time = length/self.target_speed
        #print("TOT TIME:", self.tot_time)

        tlen = 0.0
        time = [0.0]
        for l in lengths:
            tlen += l
            time.append(self.tot_time*tlen/tot_length)
        #print("TIME:", time)
        #print("TARGETS:", self.targets)
        try:
            self.spline_x = CubicSpline(time, [self.x] + [t[0] for t in self.targets])
            self.spline_y = CubicSpline(time, [self.y] + [t[1] for t in self.targets])
            self.spline_z = CubicSpline(time, [self.z] + [t[2] for t in self.targets])
        except:
            print("DUPLICATE TIMES")

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
        if not self.have_initial_position_from_pose or self.have_initial_position:
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
        if not self.have_initial_position:
            return
        # Calcule distance from actual trajectory position to target.
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dz = self.target_z - self.z
        self.target_yaw = math.atan2(dy, dx)
        len = math.sqrt(dx*dx+dy*dy+dz*dz)
        if len < 0.001:
            return
        self.travel_length = len
        #Calculate distance in x,y,z directions
        self.frac_x = dx/len
        self.frac_y = dy/len
        self.frac_z = dz/len
        # Reset acc_len and speed and starts acc phase.
        self.phase = "acc"
        self.acc_len = 0.0
        self.speed = 0.0

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
        if self.enabled_flag:
            return
        print("set_initial_position:", x, y, z)
        self.x = x
        self.y = y
        self.z = z
        self.targets = [np.array([x+5.0, y+2.0, z])]
        if not self.have_initial_position:
            self.reset()
            self.have_initial_position = True

    # Get the full path, i.e, sequence of points in time that lead the initial position to the target position.
    def get_path(self, dt):
        # contruct from spline, add acceleration and brake
        # need to compute lenght of spline...

        pathmsg = Path()

        pathmsg.header.frame_id = "world"
        pathmsg.header.stamp.secs = 0
        pathmsg.header.stamp.nsecs = int(1000000000.0*dt)

        current_time = 0.0
        speed = 0.0
        times = [0.0]
        
        while speed <= self.target_speed:
            speed += self.acc*dt
            current_time += dt*speed/self.target_speed
            times.append(current_time)

        speed = self.target_speed
        print("END ACC PHASE:", current_time, times)

        brake_time = self.tot_time - current_time-dt

        while current_time < brake_time:
            current_time+=dt
            times.append(current_time)

        print("END CRUISE PHASE:", current_time, times)
        
        while current_time <= self.tot_time:
            speed += self.acc*dt
            current_time += dt*speed/self.target_speed
            times.append(current_time)
            
        print("END BRAKE PHASE:", current_time, self.tot_time)
        

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

        print("RETURN pathmsg")
        return pathmsg
        
        x = self.x
        y = self.y
        z = self.z
        phase = "acc"
        speed = 0.0
        acc_len = 0.0

        print("X Y Z", x, y, z)
        print("TARGET X Y Z", self.target_x, self.target_y, self.target_z)

        self.reset()

        dx = self.target_x - x
        dy = self.target_y - y
        dz = self.target_z - z
        dist_to_target = math.sqrt(dx*dx+dy*dy+dz*dz)

        if dist_to_target < 0.05:
            msg = PoseStamped()
            msg.header.frame_id = "world"
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            msg2 = PoseStamped()
            msg2.header.frame_id = "world"
            msg2.pose.position.x = x
            msg2.pose.position.y = y
            msg2.pose.position.z = z
            msg2.pose.orientation.x = 0.0
            msg2.pose.orientation.y = 0.0
            msg2.pose.orientation.z = 0.0
            msg2.pose.orientation.w = 1.0
            pathmsg.poses.append(msg)
            pathmsg.poses.append(msg2)
            print("RETURN hover pathmsg")
            return pathmsg


        # While we are not hovering, i.e, we didn't reach the target position:
        while phase != "hover":
            #Calculate distance from actual trajectory position to target.
            dx = self.target_x - x
            dy = self.target_y - y
            dz = self.target_z - z
            dist_to_target = math.sqrt(dx*dx+dy*dy+dz*dz)
            # If we are accelerating:
            if phase == "acc":
                # Increase the speed (v = v0+a.t)
                speed += self.acc*dt
                # If speed becomes higher than target desired speed, than cruise (moves with constant target speed).
                if speed > self.target_speed:
                    speed = self.target_speed
                    phase = "cruise"
                    print("ACCLEN:", acc_len)
                else:
                    # If we the total distance traveled in acceleration phase becomes equal/higher than half of total distance, than go brake (desacelerate)
                    if acc_len >= self.travel_length/2.0:
                        phase = "brake"
                
            # If we are braking, then decrease speed (v = v0 - a.t)
            if phase == "brake":
                speed -= self.acc*dt
                # If our speed becomes negative, than hover (stops moving -> reached target).
#                if speed < 0.0 or (dist_to_target < 0.05):
                if speed < 0.0:
                    speed = 0.0
                    phase = "hover"
                    print("HOVER:")
    
            if phase == "cruise":
                # If we are cruisng and the distance to target becomes smaller than distance traveled in acceleration, then go brake.
                if dist_to_target <= acc_len:
                    phase = "brake"

            # If we are hovering, than our position is equal target position (because we reached it)
            if phase == "hover":
                pass
                #x = self.target_x
                #y = self.target_y
                #z = self.target_z

            # Calculate new trajectory point (distance = s0+v.t)
            mylen = speed*dt
            ## print("FULL TRAJECTORY SPEED:", speed, self.frac_x, self.frac_y, self.frac_z, mylen, acc_len, dist_to_target)
            # Updates trajectory position.
            x += self.frac_x*mylen
            y += self.frac_y*mylen
            z += self.frac_z*mylen
            ## print("POSITION:", phase, x, y, z)
            if phase == "acc":
                acc_len += mylen

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

        print("RETURN pathmsg")
        return pathmsg

    def move_tick(self):
        # Moving target position with joystick.
        self.targets[self.target_index] += np.array([self.joy_x, self.joy_y, self.joy_z])
        self.update_spline()
        
        # If control is not enabled, than reset in every iteration (to update trajectory initial position in case we start control)
        if not self.enabled_flag:
            self.reset()
            return

    # Tick: Updates trajectory position in each iteration.
    def tick(self, dt):
        if not self.enabled_flag:
            return

        print("goto_trajectory tick:", dt, self.phase, self.enabled_flag)

        
        # Calculating distance from trajectory position to target.
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        dz = self.target_z - self.z
        dist_to_target = math.sqrt(dx*dx+dy*dy+dz*dz)
        print("dist_to_target:", dist_to_target)

        # If we are accelerating, then increase speed (v = v0 + a.t)
        if self.phase == "acc":
            self.speed += self.acc*dt
            # If speed becomes higher than target desired speed, than cruise (moves with constant target speed).
            if self.speed > self.target_speed:
                self.speed = self.target_speed
                self.phase = "cruise"
                print("ACCLEN:", self.acc_len)
            else:
                # If we the total distance traveled in acceleration phase becomes equal/higher than half of total distance, than go brake (desacelerate)
                if self.acc_len >= self.travel_length/2.0:
                    self.phase = "brake"
                
        # If we are braking, then decrease speed (v = v0 - a.t)
        if self.phase == "brake":
            self.speed -= self.acc*dt
            # If our speed becomes negative, than hover (stops moving -> reached target).
            if self.speed < 0.0 or (dist_to_target < 0.05):
                self.speed = 0.0
                self.phase = "hover"
                print("HOVER:")
    
        if self.phase == "cruise":
            # If we are cruisng and the distance to target becomes smaller than distance traveled in acceleration, then go brake.
            if dist_to_target < self.acc_len:
                self.phase = "brake"

        # If we are hovering, than our position is equal target position (because we reached it)
        if self.phase == "hover":
            pass
            #self.x = self.target_x
            #self.y = self.target_y
            #self.z = self.target_z

        # Calculate new trajectory point (distance = s0+v.t)
        len = self.speed*dt
        print("SPEED:", self.speed, self.frac_x, self.frac_y, self.frac_z, len)
        # Updates trajectory position.
        self.x += self.frac_x*len
        self.y += self.frac_y*len
        self.z += self.frac_z*len
        print("POSITION:", self.x, self.y, self.z)
        if self.phase == "acc":
            self.acc_len += len
        

            
