#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "controller.h"

namespace control {
Control::Control() : linear_state(0), angular_state(0)
{
    ros::NodeHandle private_node("~");

    private_node.param("linear_min", linear_min, 0.2);
    private_node.param("linear_max", linear_max, 2.0);
    private_node.param("linear_step", linear_step, 2.0);

    private_node.param("angular_min", angular_min, 0.5);
    private_node.param("angular_max", angular_max, 4.0);
    private_node.param("angular_step", angular_step, 0.2);

    private_node.param("rate", rate, 10.0);

    linear_scale = linear_min;
    angular_scale = angular_min;
    send_flag = true;
}

void Control::twistMessageCheck(int value, int& state, int down, int up) {
    if(value == 1) {
        state += down;
    } else if (value == 0) {
        state += up;
    }
}

void Control::messageScaleCheck(int value, double& scale, double step, double limit) {
    if (value == 1) {
        if (step > 0) {
            scale = std::min(scale + step, limit);
        } else {
            scale = std::max(scale + step, limit);
        }
    }
}

void Control::twistCallback(const ros::TimerEvent &) {
    if (true) {
        geometry_msgs::Twist twist;
        twist.linear.x = linear_state * linear_scale;
        twist.angular.z = angular_state * angular_scale;
        twist_pub.publish(twist);
        ROS_DEBUG_STREAM("liear:" << twist.linear.x << " angulear: " << twist.angular.z);
    }
}

void Control::ControllerCore() {
    while (ros::ok()) {
        for (int i = 0;i < 50000; i++)
            twistMessageCheck(1, linear_state, 1, -1);
        for (int i = 0;i < 50000; i++)
            twistMessageCheck(0, linear_state, -1, 1);
    }
}

void Control::run() {
    twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    twist_pub_timer = nh.createTimer(ros::Duration(1.0/rate),&Control::twistCallback, this);
    boost::thread parse_thread(boost::bind(&Control::ControllerCore, this));
    ros::spin();
}

} // namespace control