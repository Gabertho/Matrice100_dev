#ifndef SIM_H
#define SIM_H

class Sim {
private:
  double x;
  double y;
  double z;
  double dx;
  double dy;
  double dz;
  double yaw;
  double yaw_rate;
  double tot_time;

public:
  Sim();
  Sim(double x0, double y0, double z0, double yaw0);
  ~Sim();

  void tick(double time);

};


#endif
