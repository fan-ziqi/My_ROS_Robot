#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>

namespace robot
{
    class robot
    {
        public:
            robot();
            ~robot();
            bool init();                  
            bool deal(double RobotV, double RobotYawRate);
        
        private:
            void calcOdom();               //里程计计算
            void pubOdomAndTf();           //发布Odom和tf
        
        private:
            ros::Time current_time_, last_time_; //时间

            double x_;                     //机器人位姿
            double y_;
            double th_;

            double vx_;                    //机器人x方向速度
            double vy_;                    //机器人y方向速度
            double vth_;                   //机器人角速度
            unsigned char sensFlag_;       //通信预留发送和接收标志位，可进行信号控制使用
            unsigned char receFlag_;
            
            ros::NodeHandle nh;
            ros::Publisher pub_;
            tf::TransformBroadcaster odom_broadcaster_;
    };
    
}

#endif 

