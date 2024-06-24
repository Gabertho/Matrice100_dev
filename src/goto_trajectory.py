import rospy

import math

from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped, PoseStamped
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

class GotoTrajectory:
    def __init__(self, x, y, z, speed):
        # Coordinates of desired point (goal), and desired speed and acceleration to reach it.
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_speed = speed
        self.acc = 1.0
        self.phase = "acc"
        self.have_initial_position = False
        self.speed = 0.0
        self.travel_length = 0.0
        self.joy_x = 0.0
        self.joy_y = 0.0
        self.joy_z = 0.0
        self.enabled_flag = False
        self.target_yaw = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


    def enabled(self):
        return self.enabled_flag

    def enable(self):
        self.enabled_flag = True

    def disable(self):
        self.enabled_flag = False

        
    def set_target(self, x, y, z):
        print("set_target:", x, y, z)
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.reset()


    def move_target(self, joy_x, joy_y, joy_z):
        self.joy_x = joy_x
        self.joy_y = joy_y
        self.joy_z = joy_z
        #self.target_x += joy_x
        #self.target_y += joy_y
        # print("move_target:", self.target_x, self.target_y)
        # self.reset()
        
    def reset(self):
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
        print("set_initial_position:", x, y, z)
        self.x = x
        self.y = y
        self.z = z
        if not self.have_initial_position:
            self.reset()
            self.have_initial_position = True

    # Get the full path, i.e, sequence of points in time that lead the initial position to the target position.
    def get_path(self, dt):
        x = self.x
        y = self.y
        z = self.z
        phase = "acc"
        speed = 0.0
        acc_len = 0.0

        print("X Y Z", x, y, z)
        print("TARGET X Y Z", self.target_x, self.target_y, self.target_z)

        self.reset()

        pathmsg = Path()

        pathmsg.header.frame_id = "world"
        pathmsg.header.stamp.secs = 0
        pathmsg.header.stamp.nsecs = int(1000000000.0*dt)

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
            len = speed*dt
            ## print("FULL TRAJECTORY SPEED:", speed, self.frac_x, self.frac_y, self.frac_z, len, acc_len, dist_to_target)
            # Updates trajectory position.
            x += self.frac_x*len
            y += self.frac_y*len
            z += self.frac_z*len
            ## print("POSITION:", phase, x, y, z)
            if phase == "acc":
                acc_len += len

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
        self.target_x += self.joy_x
        self.target_y += self.joy_y
        self.target_z += self.joy_z
        
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
        
