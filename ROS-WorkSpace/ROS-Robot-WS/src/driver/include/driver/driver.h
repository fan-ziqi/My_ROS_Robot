#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <driver/PID_reconfigConfig.h>


#include <string>
#include <vector>
#include <math.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#define G     9.8
#define head1 0xAA
#define head2 0x55
#define sendType_velocity    0x11
#define sendType_pid         0x12
#define sendType_params      0x13
#define sendType_wheelspeed  0x14

#define foundType_Packages    0x06

//#define MAX_STEERING_ANGLE    0.87
//#define M_PI 3.1415926535

enum packetFinderState
{
    waitingForHead1,
    waitingForHead2,
    waitingForPayloadSize,
    waitingForPayloadType,
    waitingForPayload,
    waitingForCheckSum,
    handlePayload
};



struct pid_param
{
    int kp;
    int ki;
    int kd;
};

struct imu_data
{
    float angle_x;
    float angle_y;
    float angle_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float q0;
    float q1;
    float q2;
    float q3;
};

typedef boost::shared_ptr<boost::asio::serial_port> serialp_ptr;

class Driver
{
    public:
        Driver();
        ~Driver();
        void loop();
    
    private:
        bool initRobot();

        void recv_msg();

        void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
        void send_speed_callback(const ros::TimerEvent&);
        void dynamic_reconfig_callback(driver::PID_reconfigConfig &config, uint32_t level);

        void handle_base_data(const uint8_t* buffer_data);
        void SetPID(int p,int i, int d);
        void SetParams(double linear_correction,double angular_correction);
        void SetVelocity(double x, double y, double yaw);

        void check_sum(uint8_t* data, size_t len, uint8_t& dest);
        void distribute_data(uint8_t msg_type, uint8_t* buffer_data);
        void upload_pid_param();

        packetFinderState state_;

        std_msgs::Float32  battery_pub_data_;

        boost::mutex cmd_vel_mutex_;
        boost::system::error_code ec_;
        boost::asio::io_service io_service_;
        boost::mutex mutex_;
        serialp_ptr sp_;

        bool recv_flag_;
        bool publish_odom_transform_;
        bool start_flag_;
        bool first_init_;
        uint8_t msg_seq_;

        geometry_msgs::Twist current_twist_;
        nav_msgs::Odometry odom_;
        geometry_msgs::TransformStamped transformStamped_;
        tf2_ros::TransformBroadcaster br_;

        ros::Time now_;
        ros::Time last_time_;
        ros::Time last_twist_time_;

        ros::Publisher odom_pub_;
        ros::Publisher battery_pub_;
        ros::Publisher imu_pub_;

        ros::Publisher avel_pub_;
        ros::Publisher bvel_pub_;
        ros::Publisher cvel_pub_;
        ros::Publisher dvel_pub_;

        ros::Publisher aset_pub_;
        ros::Publisher bset_pub_;
        ros::Publisher cset_pub_;
        ros::Publisher dset_pub_;

        std_msgs::Int32 avel_pub_data_;
        std_msgs::Int32 bvel_pub_data_;
        std_msgs::Int32 cvel_pub_data_;
        std_msgs::Int32 dvel_pub_data_;
       
        std_msgs::Int32 aset_pub_data_;
        std_msgs::Int32 bset_pub_data_;
        std_msgs::Int32 cset_pub_data_;
        std_msgs::Int32 dset_pub_data_;
        sensor_msgs::Imu imu_pub_data_;

        ros::Subscriber cmd_sub_;

        std::string port_name_;
        
        int baud_rate_;

        std::string odom_frame_;
        std::string imu_frame_;
        std::string base_frame_;
        std::string code_version_;
        int control_rate_;

        std::vector<double> imu_list_;
        std::vector<double> odom_list_;
        std::vector<int>    wheelspeedSet_list_;
        std::vector<int>    wheelspeedGet_list_;


        double wheel_track_;
        double wheel_diameter_;

	    double linear_correction_factor_;
	    double angular_correction_factor_;
        
        int kp_;
        int ki_;
        int kd_;

};


