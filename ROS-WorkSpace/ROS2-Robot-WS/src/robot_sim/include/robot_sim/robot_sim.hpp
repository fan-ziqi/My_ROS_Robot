#ifndef ROBOT_SIM_HPP_
#define ROBOT_SIM_HPP_

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#define LEFT 0
#define RIGHT 1

enum Wheel
{
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
};

class RobotSim : public rclcpp::Node 
{
public:
    RobotSim();
    ~RobotSim();
private:
    // ROS time
    rclcpp::Time last_cmd_vel_time_;
    rclcpp::Time prev_update_time_;

    // ROS timer
    rclcpp::TimerBase::SharedPtr update_timer_;

    // ROS topic publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;
    
    // ROS topic subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;

    nav_msgs::msg::Odometry odom_pub_data_;
    sensor_msgs::msg::JointState joint_states_;
    
    double wheel_speed_cmd_[4];
    double linear_x_speed_;
    double linear_y_speed_;
    double angular_speed_;
    double cmd_vel_timeout_;
    double last_position_[4];
    double last_velocity_[4];
    float odom_pose_[3];
    float odom_vel_[3];
    double wheel_seperation_;
    double wheel_lx_;
    double wheel_ly_;
    double wheel_ax_plus_bx_;
    double wheel_radius_;
    // std::string odom_frame_;
    // std::string base_frame_;
    int control_rate_;

    // Function prototypes
    void init_parameters();
    void init_variables();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
    void update_callback();
    bool update_odometry(const rclcpp::Duration & duration);
    void update_joint_state();
    void update_tf(geometry_msgs::msg::TransformStamped & odom_tf);
};
#endif  // ROBOT_SIM_HPP_