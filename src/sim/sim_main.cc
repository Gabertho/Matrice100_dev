#include "ros/ros.h"
#include "sim.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/BatteryState.h"

ros::Publisher pose_publisher;
ros::Publisher vel_publisher;
ros::Publisher battery_state_publisher;

Sim * sim = 0;
int timer_counter = 0;
double tick_time = 0.001;
bool controlled_flag = false;
bool use_joy = true;
std::string control_mode = "velocity";
int button_index = -1;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg) {
  if (controlled_flag) {
    ROS_INFO("cmd_vel_callback: %f %f %f - %f", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
    sim->set_velocity_control(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
  }
}

void dji_control_callback(const sensor_msgs::Joy::ConstPtr& msg) {
  if (controlled_flag) {
    double vx = msg->axes[0];
    double vy = msg->axes[1];
    double vz = msg->axes[2];
    double yaw_rate = msg->axes[3];
    ROS_INFO("dji_control_callback: %f %f %f - %f", vx, vy, vz, yaw_rate);
    sim->set_velocity_control(vx, vy, vz, yaw_rate);
  }
}

void dji_generic_control_callback(const sensor_msgs::Joy::ConstPtr& msg) {
  if (controlled_flag) {
    if (control_mode == "velocity") {
      double vx = msg->axes[0];
      double vy = msg->axes[1];
      double thrust = msg->axes[2];
      double yaw_rate = msg->axes[3];
      ROS_INFO("dji_generic_control_callback: %f %f - %f - %f", vx, vy, thrust, yaw_rate);
      sim->set_velocity_control(vx, vy, thrust, yaw_rate);
    }
    if (control_mode == "angles") {
      double roll = msg->axes[0];
      double pitch = msg->axes[1];
      double thrust = msg->axes[2];
      double yaw_rate = msg->axes[3];
      ROS_INFO("dji_generic_control_callback RPTY: %f %f - %f - %f", roll, pitch, thrust, yaw_rate);
      sim->set_angle_control(roll, pitch, thrust, yaw_rate);
    }
    if (control_mode == "rates") {
      double roll_rate = msg->axes[0];
      double pitch_rate = msg->axes[1];
      double thrust = msg->axes[2];
      double yaw_rate = msg->axes[3];
      ROS_INFO("dji_generic_control_callback RATES: %f %f - %f - %f", roll_rate, pitch_rate, thrust, yaw_rate);
      sim->set_rate_control(roll_rate, pitch_rate, thrust, yaw_rate);
    }
  }
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg) {
  if (!controlled_flag && msg->buttons[6]) {
    controlled_flag = true;
    button_index = 6;
  }
  if (!controlled_flag && msg->buttons[0]) {
    controlled_flag = true;
    button_index = 0;
  }
  if (controlled_flag && (button_index == 0) && !msg->buttons[0]) {
    button_index = -1;
    controlled_flag = false;
    if (control_mode == "velocity") {
      sim->set_velocity_control(0.0, 0.0, 0.0, 0.0);
    }
    if (control_mode == "angles") {
      sim->set_angle_control(0.0, 0.0, 38.0, 0.0);
    }
    if (control_mode == "rates") {
      sim->set_angle_control(0.0, 0.0, 38.0, 0.0);
    }
  }
  if (controlled_flag && (button_index == 6) && !msg->buttons[6]) {
    button_index = -1;
    controlled_flag = false;
    sim->init();
    if (control_mode == "velocity") {
      sim->set_velocity_control(0.0, 0.0, 0.0, 0.0);
    }
    if (control_mode == "angles") {
      sim->set_angle_control(0.0, 0.0, 38.0, 0.0);
    }
    if (control_mode == "rates") {
      sim->set_angle_control(0.0, 0.0, 38.0, 0.0);
    }
  }
  if (msg->buttons[1]) {
    sim->init();
    if (control_mode == "velocity") {
      sim->set_velocity_control(0.0, 0.0, 0.0, 0.0);
    }
    if (control_mode == "angles") {
      sim->set_angle_control(0.0, 0.0, 38.0, 0.0);
    }
    if (control_mode == "rates") {
      sim->set_angle_control(0.0, 0.0, 38.0, 0.0);
    }
  }
}

void publish_battery_state() {
  sensor_msgs::BatteryState msg;
  msg.percentage = 0.8;
  battery_state_publisher.publish(msg);
}

void timer_callback(const ros::TimerEvent&) {
  sim->tick(tick_time, controlled_flag);

  if (timer_counter % 20 == 0) { // 50 Hz
    geometry_msgs::PoseStamped msg = sim->get_pose();
    pose_publisher.publish(msg);
    
    geometry_msgs::Vector3Stamped velmsg = sim->get_velocity();
    vel_publisher.publish(velmsg);
  }

  if (timer_counter % 100 == 0) { // 10 Hz
    publish_battery_state();
  }
  timer_counter+=1;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "m100sim");

  ros::NodeHandle nh;

  double x0 = 0.0;
  double y0 = 0.0;
  double z0 = 2.0;
  double yaw = 0.0;
  double yaw_deg = 0.0;

  ros::param::get("~use_joy", use_joy);
  ros::param::get("x", x0);
  ros::param::get("y", y0);
  ros::param::get("z", z0);
  ros::param::get("yaw", yaw_deg);
  ros::param::get("control_mode", control_mode);

  yaw = yaw_deg/180.0*M_PI;

  ROS_INFO("Initial position: %f %f %f - %f - %d", x0, y0, z0, yaw_deg, use_joy);

  double max_wind_speed = 0.0;
  std::string wind_direction = "";
  std::string wind_amplitude = "";

  ros::param::get("max_wind_speed", max_wind_speed);
  ros::param::get("wind_direction", wind_direction);
  ros::param::get("wind_amplitude", wind_amplitude);

  ROS_INFO("control_mode: %s", control_mode.c_str());
  ROS_INFO("max_wind_speed: %f", max_wind_speed);
  ROS_INFO("wind_direction: %s", wind_direction.c_str());
  ROS_INFO("wind_amplitude: %s", wind_amplitude.c_str());

  if (!use_joy) {
    max_wind_speed = 0.0;
  }

  sim = new Sim(x0, y0, z0, yaw);

  sim->set_max_wind_speed(max_wind_speed);
  sim->set_wind_direction(wind_direction);
  sim->set_wind_amplitude(wind_amplitude);

  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("pose",1);
  vel_publisher = nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/velocity",1);
  battery_state_publisher = nh.advertise<sensor_msgs::BatteryState>("dji_sdk/battery_state",1);

  //  ros::Subscriber cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, cmd_vel_callback);
  ros::Subscriber dji_control_subscriber = nh.subscribe<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 1, dji_control_callback);

  ros::Subscriber dji_generic_control_subscriber = nh.subscribe<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 1, dji_generic_control_callback);
  
  ros::Subscriber joy_subscriber = nh.subscribe<sensor_msgs::Joy>("/drone/joy", 1, joy_callback);
                
  ros::Timer timer = nh.createTimer(ros::Duration(tick_time), timer_callback);
  
  ros::spin();

  return 0;
}
