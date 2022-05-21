#include "../include/robot/robot.hpp"

Robot::Robot() : 
    Node("RobotNode")
{
    if(rcutils_logging_set_logger_level(this->get_name(), RCUTILS_LOG_SEVERITY_DEBUG))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to set logger level");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Logger level set to DEBUG");
    }

    RCLCPP_INFO(this->get_logger(), "创建机器人节点RobotNode");
    robot_loop();
}

Robot::~Robot()
{
    RCLCPP_INFO(this->get_logger(), "销毁机器人节点RobotNode");
}

// TBD 动态调参

// 主循环函数
void Robot::robot_loop()
{
    using namespace std::chrono_literals;

    // TBD? qos

    get_parameter_from_file();

    odom_get_vector_.resize(6,0.0);
    imu_get_vector_.resize(9,0.0);
    wheelvel_set_vector_.resize(4,0);
    wheelvel_get_vector_.resize(4,0);

    //初始化串口
    my_serial = std::make_shared<Serial>(port_name_);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this, rclcpp::SystemDefaultsQoS());

    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pub_battery_ = this->create_publisher<std_msgs::msg::Float32>("voltage", 10);
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    pub_wheelvel_a_get_ = this->create_publisher<std_msgs::msg::Int32>("robot/avel", 10);
    pub_wheelvel_b_get_ = this->create_publisher<std_msgs::msg::Int32>("robot/bvel", 10);
    pub_wheelvel_c_get_ = this->create_publisher<std_msgs::msg::Int32>("robot/cvel", 10);
    pub_wheelvel_d_get_ = this->create_publisher<std_msgs::msg::Int32>("robot/dvel", 10);
    pub_wheelvel_a_set_ = this->create_publisher<std_msgs::msg::Int32>("robot/aset", 10);
    pub_wheelvel_b_set_ = this->create_publisher<std_msgs::msg::Int32>("robot/bset", 10);
    pub_wheelvel_c_set_ = this->create_publisher<std_msgs::msg::Int32>("robot/cset", 10);
    pub_wheelvel_d_set_ = this->create_publisher<std_msgs::msg::Int32>("robot/dset", 10);

    sub_cmd_  = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 
        10, 
        std::bind(&Robot::cmd_vel_callback, this, std::placeholders::_1));
    
    // 串口发送参数
    serial_send_correction(linear_correction_factor_,angular_correction_factor_);
    serial_send_pid(kp_,ki_,kd_);

    // 串口发送速度定时器
    serial_send_speed_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0/control_rate_), 
        std::bind(&Robot::serial_send_speed_callback, this));

    // 串口接收线程
    serial_receive_thread_ = std::make_shared<std::thread>(&Robot::serial_receive_callback, this);

    RCLCPP_INFO(this->get_logger(), "初始化成功，机器人开始运行");
}

// 从配置文件中获取机器人参数
void Robot::get_parameter_from_file()
{
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    this->get_parameter("port_name", port_name_);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->get_parameter("odom_frame", odom_frame_);
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->get_parameter("base_frame", base_frame_);
    this->declare_parameter<std::string>("imu_frame", "base_imu_link");
    this->get_parameter("imu_frame", imu_frame_);

    this->declare_parameter<int>("baud_rate", 115200);
    this->get_parameter("baud_rate", baud_rate_);
    this->declare_parameter<int>("control_rate", 50);
    this->get_parameter("control_rate", control_rate_);

    this->declare_parameter<double>("linear_correction_factor", 1.0);
    this->get_parameter("linear_correction_factor", linear_correction_factor_);
    this->declare_parameter<double>("angular_correction_factor", 1.0);
    this->get_parameter("angular_correction_factor", angular_correction_factor_);

    this->declare_parameter<bool>("publish_odom_transform", true);
    this->get_parameter("publish_odom_transform", publish_odom_transform_);

    this->declare_parameter<int>("Kp", 300);
    this->get_parameter("Kp", kp_);
    this->declare_parameter<int>("Ki", 0);
    this->get_parameter("Ki", ki_);
    this->declare_parameter<int>("Kd", 52000);
    this->get_parameter("Kd", kd_);

    RCLCPP_INFO(this->get_logger(), "成功从配置文件中获取参数");
}

// 串口发送/cmd_vel速度话题
void Robot::serial_send_speed_callback()
{
    double linear_x_speed, linear_y_speed, angular_speed;

    if((this->now().seconds()-last_twist_time_.seconds()) < 0.1)
    {
        linear_x_speed = current_twist_.linear.x;
        linear_y_speed = current_twist_.linear.y;
        angular_speed = current_twist_.angular.z;
    }
    else
    {
        linear_x_speed  = 0;
        linear_y_speed  = 0;
        angular_speed = 0;
    }

    // 如果超过1秒没有接收到里程计消息，则认为连接错误，给出warning
    if((this->now().seconds() - last_serial_time_.seconds()) >= 1.0)
    {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_WARN_THROTTLE(this->get_logger(), 
                             steady_clock, 
                             1000, 
                             "与机器人通信错误！");
    }
    serial_send_velocity(linear_x_speed,linear_y_speed,angular_speed);
}

// cmd_vel Subscriber的回调函数
void Robot::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_twist_time_ = this->now();
    current_twist_ = *msg.get();
    // RCLCPP_DEBUG(this->get_logger(), "收到cmd_vel信息");
}

// 串口数据接收线程的回调函数
void Robot::serial_receive_callback()
{
    while(1)
	{
        short msg_data[255] = {0};
        unsigned int msg_lenth = 0;
        unsigned int msg_code = 0;
        
	 	my_serial->SerialRead(msg_data, &msg_lenth, &msg_code);
		
		// std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // RCLCPP_DEBUG(this->get_logger(), "串口接收到机器人端发送的数据");

        //数据处理
        if(msg_code == RESEIVETYPE_PACKAGES)
        {
            serial_receive_classify(msg_data, msg_lenth, msg_code);
        }
	}
}

// 数据校验与分发
void Robot::serial_receive_classify(short * msg_data, unsigned int msg_lenth, unsigned int msg_code)
{
    if(msg_code == RESEIVETYPE_PACKAGES)
    {
        serial_receive_process(msg_data);
    }
    // else...
}

// 串口数据包解析
void Robot::serial_receive_process(const short * buffer_data)
{
    

    // 记录当前时间
    current_time_ = this->get_clock()->now();
    last_serial_time_ = this->now();
    
    // imu部分
    // 解析串口接受的imu数据
    // gyro 陀螺仪角速度
    imu_get_vector_[0] = ((double)(buffer_data[0]) / 32768 * 2000 / 180 * 3.1415);
    imu_get_vector_[1] = ((double)(buffer_data[1]) / 32768 * 2000 / 180 * 3.1415);
    imu_get_vector_[2] = ((double)(buffer_data[2]) / 32768 * 2000 / 180 * 3.1415);
    // Acc 加速度
    imu_get_vector_[3] = ((double)(buffer_data[3]) / 32768 * 2 * 9.8);
    imu_get_vector_[4] = ((double)(buffer_data[4]) / 32768 * 2 * 9.8);
    imu_get_vector_[5] = ((double)(buffer_data[5]) / 32768 * 2 * 9.8);
    // Angle 姿态角
    imu_get_vector_[6] = ((double)(buffer_data[6]) / 100);
    imu_get_vector_[7] = ((double)(buffer_data[7]) / 100);
    imu_get_vector_[8] = ((double)(buffer_data[8]) / 100);
    // 把角度转换成四元数(tf类型)
    imu_quaternion_tf_.setRPY(0.0, 0.0, (imu_get_vector_[8] / 180 * 3.1415926));
    tf2::convert(imu_quaternion_tf_, imu_quaternion_msg_); //把tf类型的四元数转换成msg类型
    // 准备odom的pub数据
    imu_pub_data_.header.stamp = current_time_;
    imu_pub_data_.header.frame_id = imu_frame_;
    imu_pub_data_.orientation = imu_quaternion_msg_;
    imu_pub_data_.angular_velocity.x = imu_get_vector_[0];
    imu_pub_data_.angular_velocity.y = imu_get_vector_[1];
    imu_pub_data_.angular_velocity.z = imu_get_vector_[2];
    imu_pub_data_.linear_acceleration.x = imu_get_vector_[3];
    imu_pub_data_.linear_acceleration.y = imu_get_vector_[4];
    imu_pub_data_.linear_acceleration.z = imu_get_vector_[5];
    imu_pub_data_.orientation_covariance = 
        {
            1e6, 0  , 0   ,
            0  , 1e6, 0   ,
            0  , 0  , 1e-6
        };
    imu_pub_data_.angular_velocity_covariance = 
        {
            1e6, 0  , 0  ,
            0  , 1e6, 0  ,
            0  , 0  , 1e-6
        };
    imu_pub_data_.linear_acceleration_covariance = 
        {
            -1, 0, 0,
            0 , 0, 0,
            0 , 0, 0
        };
    // pub imu
    pub_imu_->publish(imu_pub_data_);

    // odom部分
    // 里程计坐标 x(m) y(m) yaw(rad)  odom_frame
    odom_get_vector_[0] = ((double)(buffer_data[9]) / 1000);
    odom_get_vector_[1] = ((double)(buffer_data[10]) / 1000);
    odom_get_vector_[2] = ((double)(buffer_data[11]) / 1000);
    // 里程计坐标变化量  d_x(m) d_y(m) d_yaw(rad)  base_frame
    odom_get_vector_[3] = ((double)(buffer_data[12]) / 1000);
    odom_get_vector_[4] = ((double)(buffer_data[13]) / 1000);
    odom_get_vector_[5] = ((double)(buffer_data[14]) / 1000);
    // 把角度转换成四元数(tf类型)
    odom_quaternion_tf_.setRPY(0,0,odom_get_vector_[2]);
    // 准备odom的pub数据
    odom_pub_data_.header.stamp    = current_time_;
    odom_pub_data_.header.frame_id = odom_frame_;
    odom_pub_data_.child_frame_id  = base_frame_;
    odom_pub_data_.pose.pose.position.x = odom_get_vector_[0];
    odom_pub_data_.pose.pose.position.y = odom_get_vector_[1];
    odom_pub_data_.pose.pose.position.z = 0;
    odom_pub_data_.pose.pose.orientation.x = odom_quaternion_tf_.getX();
    odom_pub_data_.pose.pose.orientation.y = odom_quaternion_tf_.getY();
    odom_pub_data_.pose.pose.orientation.z = odom_quaternion_tf_.getZ();
    odom_pub_data_.pose.pose.orientation.w = odom_quaternion_tf_.getW();
    odom_pub_data_.twist.twist.linear.x = odom_get_vector_[3]/((current_time_.sec-last_time_.sec));
    odom_pub_data_.twist.twist.linear.y = odom_get_vector_[4]/((current_time_.sec-last_time_.sec));
    odom_pub_data_.twist.twist.angular.z = odom_get_vector_[5]/((current_time_.sec-last_time_.sec));
    odom_pub_data_.twist.covariance = 
        {
            1e-9, 0   , 0   , 0  , 0  , 0  ,
            0   , 1e-3, 1e-9, 0  , 0  , 0  ,
            0   , 0   , 1e6 , 0  , 0  , 0  ,
            0   , 0   , 0   , 1e6, 0  , 0  ,
            0   , 0   , 0   , 0  , 1e6, 0  ,
            0   , 0   , 0   , 0  , 0  , 1e-9
        };
    odom_pub_data_.pose.covariance = 
        {
            1e-9, 0   , 0   , 0  , 0  , 0  ,
            0   , 1e-3, 1e-9, 0  , 0  , 0  ,
            0   , 0   , 1e6 , 0  , 0  , 0  ,
            0   , 0   , 0   , 1e6, 0  , 0  ,
            0   , 0   , 0   , 0  , 1e6, 0  ,
            0   , 0   , 0   , 0  , 0  , 1e-9
        };
    // pub odom
    pub_odom_->publish(odom_pub_data_);

    // 发布tf变换, odom_frame_->base_frame_
    tf_odom_to_base_data_.header.stamp    = current_time_;
    tf_odom_to_base_data_.header.frame_id = odom_frame_;
    tf_odom_to_base_data_.child_frame_id  = base_frame_;
    tf_odom_to_base_data_.transform.translation.x = odom_get_vector_[0];
    tf_odom_to_base_data_.transform.translation.y = odom_get_vector_[1];
    tf_odom_to_base_data_.transform.translation.z = 0.0;
    tf_odom_to_base_data_.transform.rotation.x = odom_quaternion_tf_.getX();
    tf_odom_to_base_data_.transform.rotation.y = odom_quaternion_tf_.getY();
    tf_odom_to_base_data_.transform.rotation.z = odom_quaternion_tf_.getZ();
    tf_odom_to_base_data_.transform.rotation.w = odom_quaternion_tf_.getW();
    if(publish_odom_transform_)
        tf_broadcaster_->sendTransform(tf_odom_to_base_data_);

    // wheelvel部分
    // 解析串口接受的wheelvel数据
    wheelvel_get_vector_[0]=buffer_data[15];
    wheelvel_get_vector_[1]=buffer_data[16];
    wheelvel_get_vector_[2]=buffer_data[17];
    wheelvel_get_vector_[3]=buffer_data[18];
    wheelvel_set_vector_[0]=buffer_data[19];
    wheelvel_set_vector_[1]=buffer_data[20];
    wheelvel_set_vector_[2]=buffer_data[21];
    wheelvel_set_vector_[3]=buffer_data[22];
    // 准备的pub数据
    wheelvel_a_get_pub_data_.data = wheelvel_get_vector_[0];
    wheelvel_b_get_pub_data_.data = wheelvel_get_vector_[1];
    wheelvel_c_get_pub_data_.data = wheelvel_get_vector_[2];
    wheelvel_d_get_pub_data_.data = wheelvel_get_vector_[3];
    wheelvel_a_set_pub_data_.data = wheelvel_set_vector_[0];
    wheelvel_b_set_pub_data_.data = wheelvel_set_vector_[1];
    wheelvel_c_set_pub_data_.data = wheelvel_set_vector_[2];
    wheelvel_d_set_pub_data_.data = wheelvel_set_vector_[3];
    // pub wheelvel
    pub_wheelvel_a_get_->publish(wheelvel_a_get_pub_data_);
    pub_wheelvel_b_get_->publish(wheelvel_b_get_pub_data_);
    pub_wheelvel_c_get_->publish(wheelvel_c_get_pub_data_);
    pub_wheelvel_d_get_->publish(wheelvel_d_get_pub_data_);
    pub_wheelvel_a_set_->publish(wheelvel_a_set_pub_data_);
    pub_wheelvel_b_set_->publish(wheelvel_b_set_pub_data_);
    pub_wheelvel_c_set_->publish(wheelvel_c_set_pub_data_);
    pub_wheelvel_d_set_->publish(wheelvel_d_set_pub_data_);

    // battery部分
    // 解析串口接受的电池电压
    battery_pub_data_.data = (double)(buffer_data[23])/100;
    // pub battery
    pub_battery_->publish(battery_pub_data_);

    // 更新时间
    last_time_ = current_time_;

    // RCLCPP_DEBUG(this->get_logger(), "当前时间: %d.%d\r\n", 
    //     current_time_.sec, current_time_.nanosec);
    // RCLCPP_DEBUG(this->get_logger(), "角速度: x:%f\ty:%f\tz:%f\r\n",
    //     imu_get_vector_[0], imu_get_vector_[1], imu_get_vector_[2]);
    // RCLCPP_DEBUG(this->get_logger(), "加速度: x:%f\ty:%f\tz:%f\r\n",
    //     imu_get_vector_[3], imu_get_vector_[4], imu_get_vector_[5]);
    // RCLCPP_DEBUG(this->get_logger(), "角度: pitch:%f\troll:%f\tyaw:%f\r\n",
    //     imu_get_vector_[6], imu_get_vector_[7], imu_get_vector_[8]);
    // RCLCPP_DEBUG(this->get_logger(), "里程计坐标: x:%f(m)\ty:%f(m)\tyaw:%f(rad)\r\n",
    //     odom_get_vector_[0], odom_get_vector_[1], odom_get_vector_[2]);
    // RCLCPP_DEBUG(this->get_logger(), "里程计坐标变化量: base_frame dx%f(m)\tdy%f(m)\tdyaw%f(rad)\r\n",
    //     odom_get_vector_[3], odom_get_vector_[4], odom_get_vector_[5]);
    // RCLCPP_DEBUG(this->get_logger(), "获得轮速: a:%d\tb:%d\tc:%d\td:%d\r\n",
    //     wheelvel_get_vector_[0], wheelvel_get_vector_[1], wheelvel_get_vector_[2], wheelvel_get_vector_[3]);
    // RCLCPP_DEBUG(this->get_logger(), "设置轮速: a:%d\tb:%d\tc:%d\td:%d\r\n",
    //     wheelvel_set_vector_[0], wheelvel_set_vector_[1], wheelvel_set_vector_[2], wheelvel_set_vector_[3]);
    // RCLCPP_DEBUG(this->get_logger(), "电池电压: %f\r\n", battery_pub_data_.data);
}

// 串口发送PID参数
void Robot::serial_send_pid(int p,int i, int d)
{
    static short pid_data[3];
    pid_data[0] = p;
    pid_data[1] = i;
    pid_data[2] = d;
    my_serial->SerialWrite(pid_data, 3, SENDTYPE_PID);
    RCLCPP_INFO(this->get_logger(), 
                "向机器人发送PID参数：p=%d, i=%d, d=%d", 
                pid_data[0], pid_data[1], pid_data[2]);
}

// 串口发送底盘矫正参数
void Robot::serial_send_correction(double linear_correction,double angular_correction)
{
    static short param_data[2];
    param_data[0] = (short)(linear_correction * 1000);
    param_data[1] = (short)(angular_correction * 1000);
    my_serial->SerialWrite(param_data, 2, SENDTYPE_PARAMS);
    RCLCPP_INFO(this->get_logger(), 
                "向机器人发送底盘矫正参数：linear_correction=%d, angular_correction=%d",
                param_data[0], param_data[1]);
}

// 串口发送底盘速度
void Robot::serial_send_velocity(double x, double y, double yaw)
{
    static short vel_data[3];
    vel_data[0] = (short)(x * 1000);
    vel_data[1] = (short)(y * 1000);
    vel_data[2] = (short)(yaw * 1000);
    my_serial->SerialWrite(vel_data, 3, SENDTYPE_VELOCITY);
    // RCLCPP_DEBUG(this->get_logger(), 
    //              "串口发送底盘速度: %d, %d, %d", 
    //              vel_data[0], vel_data[1], vel_data[2]);
    
}

int main(int argc, char ** argv)
{
    // 强制刷新标准输出缓冲区
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    // 创建一个执行器，负责执行一组节点的回调。所有的回调都将在这个线程（主线程）中被调用。
    rclcpp::executors::SingleThreadedExecutor executor;

    auto node = std::make_shared<Robot>();

    // 向执行器添加节点，这些节点在spin函数中工作。
    executor.add_node(node);

    // spin将阻塞直到工作进入，当它可用时执行工作，并保持阻塞。它只会被 Ctrl-C 打断
    executor.spin();

    // 底盘速度设为0
    node->serial_send_velocity(0, 0, 0);

    rclcpp::shutdown();
    return 0;
}
