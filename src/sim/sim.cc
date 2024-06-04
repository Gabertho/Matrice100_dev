#include "sim.h"

Sim::Sim() : x(0.0), y(0.0), z(0.0), dx(0.0), dy(0.0), dz(0.0), yaw(0.0), yaw_rate(0.0), tot_time(0.0) {

}

Sim::Sim(double x0, double y0, double z0, double yaw0) : x(x0), y(y0), z(z0), dx(yaw0),
                                                         dy(0.0), dz(0.0), yaw(0.0), yaw_rate(0.0), tot_time(0.0) {

}

Sim::~Sim() {

}

void Sim::tick(double time) {
  x += time*dx;
  y += time*dy;
  z += time*dz;
  yaw += time*yaw_rate;
  tot_time += time;
}
