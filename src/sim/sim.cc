#include "sim.h"

#include "tf/transform_datatypes.h"

Sim::Sim() : x(0.0), y(0.0), z(0.0), dx(0.0), dy(0.0), dz(0.0), yaw(0.0), yaw_rate(0.0), tot_time(0.0) {

}

Sim::Sim(double x0, double y0, double z0, double yaw0) : x(x0), y(y0), z(z0), init_x(x0), init_y(y0), init_z(z0),
                                                         dx(0.0), dy(0.0), dz(0.0), yaw(yaw0), init_yaw(yaw0),
                                                         yaw_rate(0.0), tot_time(0.0),
                                                         max_wind_speed(0.0), wind_speed_x(0.0), wind_speed_y(0.0),
                                                         wind_direction(""), wind_amplitude(""), roll(0.0),
                                                         pitch(0.0), thrust(38.0), update_flag(true),
                                                         acc_forward(0.0), acc_left(0.0), acc_x(0.0), acc_y(0.0) {

}

Sim::~Sim() {

}

void Sim::init() {
//  x = init_x;
//  y = init_y;
//  z = init_z;
//  yaw = init_yaw;
  //  yaw = M_PI/2.0;
  //  yaw = M_PI;
  //yaw = M_PI/4.0;
  //yaw = -M_PI/2.0;
  dx = 0.0;
  dy = 0.0;
  dz = 0.0;
  yaw_rate = 0.0;
}

void Sim::set_update_flag(bool flag) {
  update_flag = flag;
}

void Sim::add_to_yaw_deg(double val) {
  yaw += (val*M_PI/180.0);
}

geometry_msgs::Vector3 Sim::get_acc() {
  geometry_msgs::Vector3 msg;
  msg.x = acc_x;
  msg.y = acc_y;
  msg.z = 0.0;
  return msg;
}

geometry_msgs::Vector3 Sim::get_body_acc() {
  geometry_msgs::Vector3 msg;
  msg.x = acc_forward;
  msg.y = acc_left;
  msg.z = 0.0;
  return msg;
}

geometry_msgs::Vector3 Sim::get_angles() {
  geometry_msgs::Vector3 msg;
  msg.x = roll*180.0/M_PI;
  msg.y = pitch*180.0/M_PI;
  msg.z = 0.0;
  return msg;
}

void Sim::set_velocity_control(double dx_, double dy_, double dz_, double yaw_rate_) {
  dx =dx_;
  dy =dy_;
  dz =dz_;
  dz = 0.0; // handle thrust later
  yaw_rate = yaw_rate_;
  control_mode = "velocity";
}

void Sim::set_angle_control(double roll_, double pitch_, double thrust_, double yaw_rate_) {
  roll = roll_;
  pitch = pitch_;
  thrust = thrust_;
  yaw_rate = yaw_rate_;
  control_mode = "angles";
}

void Sim::set_rate_control(double roll_rate_, double pitch_rate_, double thrust_, double yaw_rate_) {
  roll_rate = roll_rate_;
  pitch_rate = pitch_rate_;
  thrust = thrust_;
  yaw_rate = yaw_rate_;
  control_mode = "rates";
}

geometry_msgs::PoseStamped Sim::get_pose() {
  geometry_msgs::PoseStamped msg;
  msg = geometry_msgs::PoseStamped();
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation  = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  return msg;
}

geometry_msgs::Vector3Stamped Sim::get_velocity() {
  geometry_msgs::Vector3Stamped vel;
  vel.header.stamp = ros::Time::now();
  vel.header.frame_id = "world";
  vel.vector.x = dx + wind_speed_x;
  vel.vector.y = dy + wind_speed_y;
  vel.vector.z = dz;
  return vel;
}


void Sim::tick(double time, bool controlled_flag) {
  ROS_INFO("TICK: %s - %f %f %f - %f %f - %d", control_mode.c_str(), dx, dy, dz, wind_speed_x, wind_speed_y,
           controlled_flag);

  // TODO: rotate properly

  double forward_speed = cos(yaw)*dx + sin(yaw)*dy;
  double left_speed = -sin(yaw)*dx + cos(yaw)*dy;

  ROS_INFO("          roll: %f", roll*180.0/M_PI);
  ROS_INFO("         pitch: %f", pitch*180.0/M_PI);
  ROS_INFO("           yaw: %f", yaw*180.0/M_PI);
  ROS_INFO("            dx: %f", dx);
  ROS_INFO("            dy: %f", dy);
  ROS_INFO("forward_speed: %f", forward_speed);
  ROS_INFO("   left_speed: %f", left_speed);

  // END TODO
  
  double mass = 2.0;
  double F_drag_forward = 2.0*forward_speed;
  double F_drag_left = 2.0*left_speed;
  
  if (controlled_flag) {
    if (control_mode == "velocity") {
      z += time*dz;
    }

    if (control_mode == "rates") {
      roll += roll_rate*time;
      pitch += pitch_rate*time;
    }

    if ((control_mode == "angles") || (control_mode == "rates")) {

      double pitch_C = 10.0*3.0;
      double roll_C = -10.0*3.0;
      
      double F_forward = pitch * pitch_C;
      double F_left = roll * roll_C;    
      
      acc_forward = (F_forward - F_drag_forward)/mass;
      acc_left = (F_left - F_drag_left)/mass;
      
      // TODO: rotate acceleration
      
      acc_x = cos(yaw)*acc_forward - sin(yaw)*acc_left;
      acc_y = sin(yaw)*acc_forward + cos(yaw)*acc_left;

      ROS_INFO("acc_forward: %f", acc_forward);
      ROS_INFO("   acc_left: %f", acc_left);
      ROS_INFO("      acc_x: %f", acc_x);
      ROS_INFO("      acc_y: %f", acc_y);

      if (update_flag) {
        dx += acc_x*time;
        dy += acc_y*time;
      }
    
      // ROS_INFO("ANGLES: ROLL - PITCH - F_forward - acc_forward - dx: %f %f %f - %f %f", roll, pitch, F_forward, acc_forward, dx);

    }
  } else {
    acc_forward = - F_drag_forward/mass;
    acc_left = - F_drag_left/mass;
      
    // TODO: rotate acceleration
      
    acc_x = cos(yaw)*acc_forward - sin(yaw)*acc_left;
    acc_y = sin(yaw)*acc_forward + cos(yaw)*acc_left;

    if (update_flag) {    
      dx += acc_x*time;
      dy += acc_y*time;
    }

  }

  if (update_flag) {
    x += time*dx;
    y += time*dy;
    x += time*wind_speed_x;
    y += time*wind_speed_y;
  }
  yaw += time*yaw_rate;    
  
  tot_time += time;
}

void Sim::compute_wind() {
  double wind_speed = max_wind_speed;
  if (wind_amplitude == "zero") {
    wind_speed = 0.0;
  }
  if (wind_amplitude == "max") {
    wind_speed = max_wind_speed;
  }
  if (wind_direction == "north") {
    wind_speed_x = 0.0;
    wind_speed_y = wind_speed;
  }
  if (wind_direction == "east") {
    wind_speed_x = wind_speed;
    wind_speed_y = 0.0;
  }
  if (wind_direction == "south") {
    wind_speed_x = 0.0;
    wind_speed_y = -wind_speed;
  }
  if (wind_direction == "west") {
    wind_speed_x = -wind_speed;
    wind_speed_y = 0.0;
  }
  ROS_INFO("WIND: %f - %f %f", max_wind_speed, wind_speed_x, wind_speed_y);
}

void Sim::set_max_wind_speed(double val) {
  max_wind_speed = val;
  compute_wind();  
}

void Sim::set_wind_direction(std::string val) {
  wind_direction = val;
  compute_wind();
}

void Sim::set_wind_amplitude(std::string val) {
  wind_amplitude = val;
  compute_wind();
}
