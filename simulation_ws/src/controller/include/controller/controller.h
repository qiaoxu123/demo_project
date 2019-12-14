#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace control {
class Control {
public:
  Control();
  ~Control(){};

  void run();

private:
  ros::NodeHandle nh;
  ros::Publisher twist_pub;
  ros::Timer twist_pub_timer;

  int linear_state, angular_state;

  double rate;
  double linear_scale, angular_scale;
  double linear_min, linear_max, linear_step;
  double angular_min, angular_max, angular_step;
  bool send_flag;

  void ControllerCore();
  void twistCallback(const ros::TimerEvent &);
  void twistMessageCheck(int value, int &state, int down, int up);
  void messageScaleCheck(int value, double &scale, double step, double limit);
};
} // namespace control

#endif