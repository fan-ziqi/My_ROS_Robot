#include "../include/robot_sim_2/robot_sim_2.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

RobotSim::RobotSim() : Node("RobotSimNode")
{
    init_parameters();

    init_variables();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // 初始化publishers
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);

    joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

    pub_tf_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", qos);

    // 初始化subscribers
    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", qos, std::bind(&RobotSim::cmd_vel_callback, this, _1));

    // 初始化timer
    update_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0/control_rate_), std::bind(&RobotSim::update_callback, this));

    RCLCPP_INFO(this->get_logger(), "Robot sim node has been initialised");
}

RobotSim::~RobotSim()
{
    RCLCPP_INFO(this->get_logger(), "Robot sim node has been terminated");
}

// 从配置文件中获取机器人参数
void RobotSim::init_parameters()
{
    this->declare_parameter<std::string>("joint_states_frame", "base_footprint");
    this->get_parameter("joint_states_frame", joint_states_.header.frame_id);

    this->declare_parameter<std::string>("odom_frame", "odom");
    this->get_parameter("odom_frame", odom_pub_data_.header.frame_id);
    // this->get_parameter<std::string>("odom_frame", odom_frame_);

    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->get_parameter("base_frame", odom_pub_data_.child_frame_id);
    // this->get_parameter("base_frame", base_frame_);

    this->declare_parameter<double>("wheels.separation", 0.0);
    this->get_parameter("wheels.separation", wheel_seperation_);

    this->declare_parameter<double>("wheels.radius", 0.0);
    this->get_parameter("wheels.radius", wheel_radius_);

    this->declare_parameter<int>("control_rate", 50);
    this->get_parameter("control_rate", control_rate_);
}

// 初始化变量
void RobotSim::init_variables()
{
    // Initialise variables
    wheel_speed_cmd_[LEFT] = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
    linear_x_speed_ = 0.0;
    linear_y_speed_ = 0.0;
    angular_speed_ = 0.0;
    cmd_vel_timeout_ = 0.1;
    last_position_[LEFT] = 0.0;
    last_position_[RIGHT] = 0.0;
    last_velocity_[LEFT] = 0.0;
    last_velocity_[RIGHT] = 0.0;
    odom_pose_[0] = 0.0;
    odom_pose_[1] = 0.0;
    odom_pose_[2] = 0.0;
    odom_vel_[0] = 0.0;
    odom_vel_[1] = 0.0;
    odom_vel_[2] = 0.0;

    joint_states_.name.push_back("wheel_left_joint");
    joint_states_.name.push_back("wheel_right_joint");
    joint_states_.position.resize(2, 0.0);
    joint_states_.velocity.resize(2, 0.0);
    joint_states_.effort.resize(2, 0.0);

    prev_update_time_ = this->now();
    last_cmd_vel_time_ = this->now();
}

// sub_cmd_ Subscriber的回调函数
void RobotSim::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
{
    last_cmd_vel_time_ = this->now();

    linear_x_speed_ = cmd_vel_msg->linear.x;
    linear_y_speed_ = cmd_vel_msg->linear.y;
    angular_speed_ = cmd_vel_msg->angular.z;

    // current_twist_ = *cmd_vel_msg.get();

    // 计算四个轮子速度, 仿真先用差速
    wheel_speed_cmd_[LEFT]  = linear_x_speed_ - (angular_speed_ * wheel_seperation_ / 2);
    wheel_speed_cmd_[RIGHT] = linear_x_speed_ + (angular_speed_ * wheel_seperation_ / 2);

    
}

// update_timer_ Timer的回调函数
void RobotSim::update_callback()
{
    rclcpp::Time time_now = this->now(); //获取当前时间
    rclcpp::Duration duration(time_now - prev_update_time_); //计算时间段
    prev_update_time_ = time_now; //更新上一时间

    // 如果未收到cmd_vel, 则设置速度为0
    if((time_now - last_cmd_vel_time_).nanoseconds() / 1e9 >= cmd_vel_timeout_)
    {
        wheel_speed_cmd_[LEFT] = 0.0;
        wheel_speed_cmd_[RIGHT] = 0.0;
    }

    // odom
    update_odometry(duration);
    odom_pub_data_.header.stamp = time_now;
    pub_odom_->publish(odom_pub_data_);

    // joint_states
    update_joint_state();
    joint_states_.header.stamp = time_now;
    joint_states_pub_->publish(joint_states_);


    // tf odom_frame_->base_frame_
    geometry_msgs::msg::TransformStamped odom_tf;
    update_tf(odom_tf);
    tf2_msgs::msg::TFMessage odom_tf_msg;
    odom_tf_msg.transforms.push_back(odom_tf);
    pub_tf_->publish(odom_tf_msg);
    
}

// odom解算/更新函数
bool RobotSim::update_odometry(const rclcpp::Duration & duration)
{
    double theta_of_wheel_l, theta_of_wheel_r; // 轮子转过的角度 [rad]
    double delta_s_of_car, delta_theta_of_car;
    double v[2], w[2];
    double step_time = duration.nanoseconds() / 1e9;  // [sec]

    theta_of_wheel_l = theta_of_wheel_r = 0.0;
    delta_s_of_car = delta_theta_of_car = 0.0;

    // v = 平移速度 [m/s]
    // w = 旋转速度 [rad/s]
    // w = v / r
    v[LEFT] = wheel_speed_cmd_[LEFT]; //轮子平移速度
    w[LEFT] = v[LEFT] / wheel_radius_; //轮子旋转速度
    v[RIGHT] = wheel_speed_cmd_[RIGHT];
    w[RIGHT] = v[RIGHT] / wheel_radius_;

    // 更新当前角速度
    last_velocity_[LEFT] = w[LEFT];
    last_velocity_[RIGHT] = w[RIGHT];

    // 求当前时间段内旋转过的角度
    theta_of_wheel_l = w[LEFT] * step_time;
    theta_of_wheel_r = w[RIGHT] * step_time;

    // 判断非法数字
    if (isnan(theta_of_wheel_l))
    {
        theta_of_wheel_l = 0.0;
    }
    if (isnan(theta_of_wheel_r))
    {
        theta_of_wheel_r = 0.0;
    }

    // 更新旋转过的角度值
    last_position_[LEFT] += theta_of_wheel_l;
    last_position_[RIGHT] += theta_of_wheel_r;

    // 更新小车中心移动的位移
    // 双轮小车, s=r*(θ_l+θ_r)/2
    delta_s_of_car = wheel_radius_ * (theta_of_wheel_r + theta_of_wheel_l) / 2.0;
    // 更新小车中心旋转的角度
    // 双轮小车, θ=r*(θ_l-θ_r)/轮距
    delta_theta_of_car = wheel_radius_ * (theta_of_wheel_r - theta_of_wheel_l) / wheel_seperation_;

    // 计算里程计位移, x, y, θ
    // x' = x + ds * cos(θ + dθ/2)  (θ+dθ/2):两次航向角的夹角
    odom_pose_[0] += delta_s_of_car * cos(odom_pose_[2] + (delta_theta_of_car / 2.0));
    odom_pose_[1] += delta_s_of_car * sin(odom_pose_[2] + (delta_theta_of_car / 2.0));
    odom_pose_[2] += delta_theta_of_car;

    // 计算里程瞬时速度, vx, vy, vθ
    odom_vel_[0] = delta_s_of_car / step_time;     // v = ds/dt
    odom_vel_[1] = 0.0;
    odom_vel_[2] = delta_theta_of_car / step_time;  // w = dtheta/dt

    //更新里程计平移变换
    odom_pub_data_.pose.pose.position.x = odom_pose_[0];
    odom_pub_data_.pose.pose.position.y = odom_pose_[1];
    odom_pub_data_.pose.pose.position.z = 0;

    // 将旋转变换转化为四元数表示
    tf2::Quaternion q;
    q.setRPY(0, 0, odom_pose_[2]);

    // // debug
    // std::cout << "- Rotation: in Quaternion [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]" << std::endl
    //           << "            in RPY (radian) [" <<  0 << ", " << 0 << ", " << odom_pose_[2] << "]" << std::endl
    //           << "            in RPY (degree) [" <<  0*180.0/3.14 << ", " << 0*180.0/3.14 << ", " << odom_pose_[2]*180.0/3.14 << "]" 
    //           << std::endl;

    // 更新里程计旋转变换
    odom_pub_data_.pose.pose.orientation.x = q.x();
    odom_pub_data_.pose.pose.orientation.y = q.y();
    odom_pub_data_.pose.pose.orientation.z = q.z();
    odom_pub_data_.pose.pose.orientation.w = q.w();

    // 更新里程计瞬时速度
    odom_pub_data_.twist.twist.linear.x = odom_vel_[0];
    odom_pub_data_.twist.twist.angular.z = odom_vel_[2];
    // odom_pub_data_.twist.covariance = 
    //     {
    //         1e-9, 0   , 0   , 0  , 0  , 0  ,
    //         0   , 1e-3, 1e-9, 0  , 0  , 0  ,
    //         0   , 0   , 1e6 , 0  , 0  , 0  ,
    //         0   , 0   , 0   , 1e6, 0  , 0  ,
    //         0   , 0   , 0   , 0  , 1e6, 0  ,
    //         0   , 0   , 0   , 0  , 0  , 1e-9
    //     };
    // odom_pub_data_.pose.covariance = 
    //     {
    //         1e-9, 0   , 0   , 0  , 0  , 0  ,
    //         0   , 1e-3, 1e-9, 0  , 0  , 0  ,
    //         0   , 0   , 1e6 , 0  , 0  , 0  ,
    //         0   , 0   , 0   , 1e6, 0  , 0  ,
    //         0   , 0   , 0   , 0  , 1e6, 0  ,
    //         0   , 0   , 0   , 0  , 0  , 1e-9
    //     };

    return true;
}

void RobotSim::update_joint_state()
{
  joint_states_.position[LEFT] = last_position_[LEFT];
  joint_states_.position[RIGHT] = last_position_[RIGHT];
  joint_states_.velocity[LEFT] = last_velocity_[LEFT];
  joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
}

// tf解算/更新函数
void RobotSim::update_tf(geometry_msgs::msg::TransformStamped & odom_tf)
{
  odom_tf.header = odom_pub_data_.header;
  odom_tf.child_frame_id = odom_pub_data_.child_frame_id;
  odom_tf.transform.translation.x = odom_pub_data_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_pub_data_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_pub_data_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_pub_data_.pose.pose.orientation;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotSim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
