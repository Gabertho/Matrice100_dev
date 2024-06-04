#ifndef SIM_H
#define SIM_H

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

class Sim {
private:
  double x;
  double y;
  double z;
  double init_x;
  double init_y;
  double init_z;
  double dx;
  double dy;
  double dz;
  double yaw;
  double init_yaw;
  double yaw_rate;
  double tot_time;

public:
  Sim();
  Sim(double x0, double y0, double z0, double yaw0);
  ~Sim();

  void init();
  
  void set_control(double dx_, double dy_, double dz_, double yaw_rate_);
  
  void tick(double time);

  geometry_msgs::PoseStamped get_pose();

};


#endif
