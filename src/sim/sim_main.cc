#include "ros/ros.h"
#include "sim.h"


Sim * sim = 0;

int main(int argc, char **argv) {

  ros::init(argc, argv, "m100sim");

  ros::NodeHandle nh;

  double x0 = 0.0;
  double y0 = 0.0;
  double z0 = 2.0;
  double yaw = 0.0;
  double yaw_deg = 0.0;

  ros::param::get("~x", x0);
  ros::param::get("~y", x0);
  ros::param::get("~z", x0);
  ros::param::get("~yaw", yaw_deg);

  yaw = yaw_deg/180.0*M_PI;

  ROS_INFO("Initial position: %f %f %f - %f", x0, y0, z0, yaw_deg);

  sim = new Sim(x0, y0, z0, yaw);

  ros::spin();

  return 0;
}
