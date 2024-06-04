#include "sim.h"

Sim::Sim() : x(0.0), y(0.0), z(0.0), dx(0.0), dy(0.0), dz(0.0), yaw(0.0), yaw_rate(0.0), tot_time(0.0) {

}

Sim::Sim(double x0, double y0, double z0, double yaw0) : x(x0), y(y0), z(z0), init_x(x0), init_y(y0), init_z(z0),
                                                         dx(yaw0), dy(0.0), dz(0.0), yaw(yaw0), init_yaw(yaw0),
                                                         yaw_rate(0.0), tot_time(0.0) {

}

Sim::~Sim() {

}

void Sim::init() {
  x = init_x;
  y = init_y;
  z = init_z;
  yaw = init_yaw;
  dx = 0.0;
  dy = 0.0;
  dz = 0.0;
  yaw_rate = 0.0;
}

void Sim::set_control(double dx_, double dy_, double dz_, double yaw_rate_) {
  dx =dx_;
  dy =dy_;
  dz =dz_;
  yaw_rate =yaw_rate_;
}

geometry_msgs::PoseStamped Sim::get_pose() {
  geometry_msgs::PoseStamped msg;
  msg = geometry_msgs::PoseStamped();
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;
  return msg;
}

void Sim::tick(double time) {
  x += time*dx;
  y += time*dy;
  z += time*dz;
  yaw += time*yaw_rate;
  tot_time += time;
}
