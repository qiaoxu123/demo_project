#ifndef SIMULATION_CORE_H_
#define SIMULATION_CORE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <iostream>

namespace simulation
{
class SimulationCore {
public:
    SimulationCore();
    ~SimulationCore() {};

    void run();
private:
    double gaussRand(double mu, double sigma);
    void cmdReceived(const geometry_msgs::Twist::ConstPtr& cmd);
    void pubOdomCallback(const ros::TimerEvent& event);

    void updateOdometry();

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;

    ros::NodeHandle nh;
    ros::Subscriber cmd_sub;
    ros::Publisher odom_pub;
    ros::Timer pub_odom_timer;

    boost::mutex cmd_mutex;
    ros::Time last_time, current_time, last_cb;
    std::string map_frame, odom_frame, base_link_frame, real_map_frame;

    double noise_v_linear, noise_v_theta;

    int rate;
    double max_a_linear, max_a_theta;
    double max_v_linear, max_v_theta;
    double cur_v_linear, cur_v_theta;
    double tar_v_linear, tar_v_theta;
    double real_x, real_y, real_th;
    double odom_x, odom_y, odom_th;

};
} // namespace simulation_core

#endif