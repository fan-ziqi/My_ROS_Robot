#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <string>
#include <functional>
#include <time.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "builtin_interfaces/msg/time.hpp"

// #include <dynamic_reconfigure/server.h>
// #include <driver/PID_reconfigConfig.h>

#include <string>
#include <vector>
#include <math.h>

#include <thread>
#include <memory>
#include <mutex>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

// 这样就能使用1000ms这种表示方式
using namespace std::chrono_literals;

//占位符
using std::placeholders::_1;

#define G_OF_EARTH     9.8
#define HEAD1 0xAA
#define HEAD2 0x55
#define SENDTYPE_VELOCITY    0x11
#define SENDTYPE_PID         0x12
#define SENDTYPE_PARAMS      0x13
#define SENDTYPE_WHEELSPEED  0x14

#define FOUNDTYPE_PACKAGES    0x06

//#define MAX_STEERING_ANGLE    0.87
//#define M_PI 3.1415926535

enum SerialState
{
    WAITING_FOR_HEAD1,
    WAITING_FOR_HEAD2,
    WAITING_FOR_PAYLOAD_SIZE,
    WAITING_FOR_PAYLOAD_TYPE,
    WAITING_FOR_PAYLOAD,
    WAITING_FOR_CHECKSUM,
    HANDLE_PAYLOAD
};

class Robot : public rclcpp::Node 
{
public:
    Robot();
    ~Robot();
    void robot_loop();
    void serial_send_velocity(double x, double y, double yaw);

private:
    std::mutex mutex_;
    // bool first_init_;

    // 串口相关
    boost::asio::io_service io_service_;
    std::shared_ptr<boost::asio::serial_port> serial_port_;
    // 串口初始化
    bool serial_init();
    void check_sum(uint8_t* data, size_t len, uint8_t& dest);
    std::shared_ptr<std::thread> serial_receive_thread_;
    void serial_receive_callback();
    boost::system::error_code serial_port_ec_;
    void serial_receive_check_and_distribute(uint8_t msg_type, uint8_t* buffer_data);
    void serial_receive_process(const uint8_t* buffer_data);
    void serial_send_pid(int p,int i, int d);
    void serial_send_correction(double linear_correction,double angular_correction);
    SerialState packet_finder_state_;
    geometry_msgs::msg::Twist current_twist_;
    bool recv_flag_;

    // 参数初始化
    void get_parameter_from_file();
    std::string port_name_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string imu_frame_;
    int baud_rate_;
    int control_rate_;
    double linear_correction_factor_;
    double angular_correction_factor_;
    bool publish_odom_transform_;
    int kp_;
    int ki_;
    int kd_;


    builtin_interfaces::msg::Time current_time_;
    builtin_interfaces::msg::Time last_time_;
    builtin_interfaces::msg::Time last_twist_time_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_battery_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_wheelvel_a_get_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_wheelvel_b_get_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_wheelvel_c_get_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_wheelvel_d_get_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_wheelvel_a_set_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_wheelvel_b_set_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_wheelvel_c_set_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_wheelvel_d_set_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    std::mutex cmd_vel_mutex_;

    rclcpp::TimerBase::SharedPtr serial_send_speed_timer_;
    void serial_send_speed_callback();

    // Imu
    std::vector<double> imu_get_vector_; // 接收到的串口数据
    tf2::Quaternion imu_quaternion_tf_; // tf类型的四元数
    geometry_msgs::msg::Quaternion imu_quaternion_msg_; // msg类型的四元数
    sensor_msgs::msg::Imu imu_pub_data_; // 准备pub的data

    // Odom
    std::vector<double> odom_get_vector_; // 接收到的串口数据
    nav_msgs::msg::Odometry odom_pub_data_; // 准备pub的data
    tf2::Quaternion odom_quaternion_tf_; // tf类型的四元数
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; //tf广播器
    geometry_msgs::msg::TransformStamped tf_odom_to_base_data_; // 准备pub的data

    // WheelVel
    std::vector<int>    wheelvel_set_vector_; // 接收到的串口数据
    std::vector<int>    wheelvel_get_vector_; // 接收到的串口数据
    std_msgs::msg::Int32 wheelvel_a_get_pub_data_; // 准备pub的data
    std_msgs::msg::Int32 wheelvel_b_get_pub_data_; // 准备pub的data
    std_msgs::msg::Int32 wheelvel_c_get_pub_data_; // 准备pub的data
    std_msgs::msg::Int32 wheelvel_d_get_pub_data_; // 准备pub的data
    std_msgs::msg::Int32 wheelvel_a_set_pub_data_; // 准备pub的data
    std_msgs::msg::Int32 wheelvel_b_set_pub_data_; // 准备pub的data
    std_msgs::msg::Int32 wheelvel_c_set_pub_data_; // 准备pub的data
    std_msgs::msg::Int32 wheelvel_d_set_pub_data_; // 准备pub的data

    // Battery
    std_msgs::msg::Float32  battery_pub_data_; // 准备pub的data

};


