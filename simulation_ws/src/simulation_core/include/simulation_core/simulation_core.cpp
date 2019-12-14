#include <math.h>

#include "vector"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include "simulation_core/simulation_core.h"

using namespace std;

namespace simulation {

SimulationCore::SimulationCore():cur_v_linear(0.0), cur_v_theta(0.0),
                                 tar_v_linear(0.0), real_x(0.0),
                                 real_y(0.0), odom_x(0.0), odom_y(0.0),
                                 odom_th(0.0) {

    ros::NodeHandle private_node("~");
    private_node.param("map_frame", map_frame, std::string("map"));
    private_node.param("odom_frame", odom_frame, std::string("odom"));
    private_node.param("base_link_frame", base_link_frame, std::string("base_link"));
    private_node.param("real_map_frame", real_map_frame, std::string("real_map"));

    private_node.param("noise_v_linear", noise_v_linear, 0.0);
    private_node.param("noise_v_theta", noise_v_theta, 0.0);

    private_node.param("max_a_linear", max_a_linear, 999.0);
    private_node.param("max_a_theta", max_a_theta, 999.0);

    private_node.param("max_v_linear", max_v_linear, 1.0);
    private_node.param("max_v_theta", max_v_theta, 1.57);

    private_node.param("rate", rate, 30);

    last_time = ros::Time::now();
}

double SimulationCore::gaussRand(double mu, double sigma) {
    static double V1, V2, S;
    static int phase = 0;
    double X;

    if (phase == 0) {
        do {
            double U1 = (double)rand() / RAND_MAX;
            double U2 = (double)rand() / RAND_MAX;

            V1 = 2 * U1 -1;
            V2 = 2 * U2 -1;
            S = V1 * V1 + V2 * V2;
        } while(S >= 1 || S == 0);
        X = V1 * sqrt(-2 * log(S) / S);
    } else 
        X = V2 * sqrt(-2 * log(S) / S);
    
    phase = 1- phase;
    return (X * sigma + mu);
}

void SimulationCore::cmdReceived(const geometry_msgs::Twist::ConstPtr& cmd) {
    cmd_mutex.lock();
    tar_v_linear = cmd->linear.x;
    tar_v_theta = cmd->angular.z;
    cmd_mutex.unlock();
}

void SimulationCore::updateOdometry() {
    double dt = (current_time - last_time).toSec();
    double ds, dth, ns, nth;

    if (tar_v_linear > cur_v_linear) {
        cur_v_linear = std::min(std::min(tar_v_linear, max_v_linear),
                                cur_v_linear + max_a_linear * dt);
    } else if (tar_v_linear < cur_v_linear) {
        cur_v_linear = std::max(std::max(-max_v_linear, tar_v_linear),
                                cur_v_linear - max_a_linear * dt);
    } else {
        cur_v_linear = tar_v_linear;
    }

    ds = cur_v_linear * dt;
    ns = ds == 0 ? 0 : (cur_v_linear + gaussRand(0, noise_v_linear)) * dt;

    if (tar_v_theta > cur_v_theta) {
        cur_v_theta = std::min(std::min(tar_v_theta, max_v_theta),
                               cur_v_theta + max_a_theta * dt);
    } else if (tar_v_theta < cur_v_theta) {
        cur_v_theta = std::max(std::max(-max_v_theta, tar_v_theta),
                               cur_v_theta - max_a_theta * dt);
    } else {
        cur_v_theta = tar_v_theta;
    }

    dth = cur_v_theta * dt;
    nth = dth == 0 ? 0 : (cur_v_theta + gaussRand(0, noise_v_theta)) * dt;

    real_th += dth;
    real_x += ds * cos(real_th);
    real_y += ds * sin(real_th);

    odom_th += nth;
    odom_x += ns * cos(odom_th);
    odom_y += ns * sin(odom_th);
}

void SimulationCore::pubOdomCallback(const ros::TimerEvent &event) {
    current_time = ros::Time::now();
    updateOdometry();

    // tf : odom --> base_link
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame;
    odom_trans.child_frame_id = base_link_frame;
    odom_trans.transform.translation.x = odom_x;
    odom_trans.transform.translation.y = odom_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(odom_th);
    tf_broadcaster.sendTransform(odom_trans);

    geometry_msgs::TransformStamped real_map_trans;
    real_map_trans.header.stamp = current_time;
    real_map_trans.header.frame_id = base_link_frame;
    real_map_trans.child_frame_id = real_map_frame;
    tf::Transform realmap2baselink(tf::createQuaternionFromYaw(real_th),
                                   tf::Vector3(real_x, real_y, 0));
    tf::Transform baselink2realmap = realmap2baselink.inverse();
    real_map_trans.transform.translation.x = baselink2realmap.getOrigin().getX();
    real_map_trans.transform.translation.y = baselink2realmap.getOrigin().getY();
    real_map_trans.transform.translation.z = 0.0;
    real_map_trans.transform.rotation = 
        tf::createQuaternionMsgFromYaw(tf::getYaw(baselink2realmap.getRotation()));
    tf_broadcaster.sendTransform(real_map_trans);

    // publish topic
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = odom_frame;
    odom_msg.child_frame_id = base_link_frame;
    odom_msg.header.stamp = current_time;
    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.position.z = 0;
    odom_msg.pose.pose.orientation.x = odom_trans.transform.rotation.x;
    odom_msg.pose.pose.orientation.y = odom_trans.transform.rotation.y;
    odom_msg.pose.pose.orientation.z = odom_trans.transform.rotation.z;
    odom_msg.pose.pose.orientation.w = odom_trans.transform.rotation.w;
    odom_msg.twist.twist.linear.x = cur_v_linear;
    odom_msg.twist.twist.angular.z = cur_v_theta;
    odom_pub.publish(odom_msg);

    // update time
    last_time = current_time;      

    // cout << "odom_x = " << odom_x << " " << "odom_y = " << odom_y << endl;           
}

void SimulationCore::run() {
    cmd_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &SimulationCore::cmdReceived, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    pub_odom_timer = nh.createTimer(ros::Duration(1.0/rate), 
                                    &SimulationCore::pubOdomCallback, this);
    ros::spin();
}

}
