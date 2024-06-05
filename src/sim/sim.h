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
  double max_wind_speed;
  double wind_speed_x;
  double wind_speed_y;
  std::string wind_direction;
  std::string wind_amplitude;

public:
  Sim();
  Sim(double x0, double y0, double z0, double yaw0);
  ~Sim();

  void init();
  
  void set_control(double dx_, double dy_, double dz_, double yaw_rate_);
  
  void tick(double time);

  geometry_msgs::PoseStamped get_pose();

  void compute_wind();
  void set_max_wind_speed(double val);
  void set_wind_direction(std::string val);
  void set_wind_amplitude(std::string val);

};


#endif
