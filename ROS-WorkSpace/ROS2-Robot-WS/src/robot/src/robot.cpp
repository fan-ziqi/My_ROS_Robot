#include "../include/robot/robot.hpp"

Robot::Robot() : 
    Node("RobotNode"), 
    // first_init_(false), 
    packet_finder_state_(WAITING_FOR_HEAD1)
{
    RCLCPP_INFO(this->get_logger(), "Creating RobotNode");
    robot_loop();
}

Robot::~Robot()
{
    std::unique_lock<std::mutex> look(mutex_);
    recv_flag_ = false;
    if(serial_port_)
    {
        serial_port_->cancel();
        serial_port_->close();
        serial_port_.reset();
        RCLCPP_INFO(this->get_logger(), "Closing Serial");
    }
    io_service_.stop();
    io_service_.reset();
    RCLCPP_INFO(this->get_logger(), "Destroying RobotNode");
}

// TBD 动态调参

// 主循环函数
void Robot::robot_loop()
{
    using std::placeholders::_1;
    using namespace std::chrono_literals;

    // TBD? qos

    get_parameter_from_file();

    odom_get_vector_.resize(6,0.0);
    imu_get_vector_.resize(9,0.0);
    wheelvel_set_vector_.resize(4,0);
    wheelvel_get_vector_.resize(4,0);

    if(!serial_init())
    {
        RCLCPP_INFO(this->get_logger(), "serial init faild");
    }

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

    sub_cmd_  = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Robot::cmd_vel_callback, this, _1));
    
    // 串口发送参数
    serial_send_correction(linear_correction_factor_,angular_correction_factor_);
    serial_send_pid(kp_,ki_,kd_);

    // 串口发送速度定时器
    serial_send_speed_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0/control_rate_), std::bind(&Robot::serial_send_speed_callback, this));

    // 串口接收线程
    serial_receive_thread_ = std::make_shared<std::thread>(&Robot::serial_receive_callback, this);
    serial_receive_thread_->detach();

    RCLCPP_INFO(this->get_logger(), "Robot Running!");
}

// 从配置文件中获取机器人参数
void Robot::get_parameter_from_file()
{
    this->declare_parameter<std::string>("port_name", "/dev/ttys005"); // For Ubuntu
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
}

// 串口初始化
bool Robot::serial_init()
{
    if(serial_port_)
    {
        RCLCPP_INFO(this->get_logger(), "The SerialPort is already opened!");
        return false;
    }

    serial_port_ = std::shared_ptr<boost::asio::serial_port>(new boost::asio::serial_port(io_service_, port_name_));


    // serial_port_->open(port_name_,serial_port_ec_);
    // if(serial_port_ec_)
    // {
    //     RCLCPP_INFO_STREAM(this->get_logger(), "error : serial_port_->open() failed...port_name=" << port_name_ << ", e=" << serial_port_ec_.message().c_str());
    //     // return false;
    // }

    //串口初始化
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_)); //波特率
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8)); //字符大小
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one)); //停止位 one /onepointfive /two
    serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none)); //奇偶校验 none / odd / even
    serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none)); //流控制 none /software /hardware
    
    return true;
}

// 串口发送收到的/cmd_vel速度话题
void Robot::serial_send_speed_callback()
{
    double linear_x_speed, linear_y_speed, angular_speed;

    if((this->get_clock()->now().seconds() - last_twist_time_.sec) <= 0.5)
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
    if((this->get_clock()->now().seconds() - current_time_.sec) >= 1.0)
    {
        rclcpp::Clock steady_clock{RCL_STEADY_TIME};
        RCLCPP_WARN_THROTTLE(this->get_logger(), steady_clock, 1000, "Didn't received odom data,Please check your connection!");
    }
    serial_send_velocity(linear_x_speed,linear_y_speed,angular_speed);
}

// cmd_vel Subscriber的回调函数
void Robot::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    try
    {
        cmd_vel_mutex_.lock();
        last_twist_time_ = this->get_clock()->now();
        current_twist_ = *msg.get();
        cmd_vel_mutex_.unlock();
    }
    catch(...)
    {
        cmd_vel_mutex_.unlock();
    }
}

// 串口通信校验位计算
void Robot::check_sum(uint8_t* data, size_t len, uint8_t& dest)
{
    dest = 0x00;
    for(size_t i=0;i<len;i++)
    {
        dest += *(data + i);
    }
}

// 串口数据接收线程的回调函数
void Robot::serial_receive_callback()
{
    uint8_t payload_size, check_num, buffer_data[255], payload_type;
    packet_finder_state_ = WAITING_FOR_HEAD1;
    recv_flag_ = true;
    while(recv_flag_)
    {
        switch (packet_finder_state_)
        {
            case WAITING_FOR_HEAD1:
                check_num = 0x00;
                boost::asio::read(*serial_port_.get(), boost::asio::buffer(&buffer_data[0], 1), serial_port_ec_);
                packet_finder_state_ = buffer_data[0] == HEAD1 ? WAITING_FOR_HEAD2 : WAITING_FOR_HEAD1;
                if(packet_finder_state_ == WAITING_FOR_HEAD1)
                {
                    RCLCPP_DEBUG_STREAM(this->get_logger(), "recv HEAD1 error : ->" << (int)buffer_data[0]);
                }
                break;
            case WAITING_FOR_HEAD2:
                boost::asio::read(*serial_port_.get(),boost::asio::buffer(&buffer_data[1],1),serial_port_ec_);
                packet_finder_state_ = buffer_data[1] == HEAD2 ? WAITING_FOR_PAYLOAD_SIZE : WAITING_FOR_HEAD1;
                if(packet_finder_state_ == WAITING_FOR_HEAD1)
                {
                    RCLCPP_DEBUG_STREAM(this->get_logger(), "recv HEAD2 error : ->" << (int)buffer_data[1]);
                }
                break;
            case WAITING_FOR_PAYLOAD_SIZE:
                boost::asio::read(*serial_port_.get(),boost::asio::buffer(&buffer_data[2],1),serial_port_ec_);
                payload_size = buffer_data[2] - 4;
                packet_finder_state_ = WAITING_FOR_PAYLOAD;
                break;
            case WAITING_FOR_PAYLOAD:
                boost::asio::read(*serial_port_.get(),boost::asio::buffer(&buffer_data[3],payload_size),serial_port_ec_);
                payload_type = buffer_data[3];
                packet_finder_state_ = WAITING_FOR_CHECKSUM;
                break;
            case WAITING_FOR_CHECKSUM:
                boost::asio::read(*serial_port_.get(),boost::asio::buffer(&buffer_data[3+payload_size],1),serial_port_ec_);
                check_sum(buffer_data, 3+payload_size, check_num);
                packet_finder_state_ = buffer_data[3+payload_size] == check_num ? HANDLE_PAYLOAD : WAITING_FOR_HEAD1;
                if(packet_finder_state_ == WAITING_FOR_HEAD1)
                {
                    RCLCPP_DEBUG_STREAM(this->get_logger(), "check sum error! recv is  : ->" << (int)buffer_data[3 + payload_size] << "  calc is " << check_num);
                }
                break;
            case HANDLE_PAYLOAD:
                serial_receive_check_and_distribute(payload_type, buffer_data);
                packet_finder_state_ = WAITING_FOR_HEAD1;
                break;
            default:
                packet_finder_state_ = WAITING_FOR_HEAD1;
                break;
        }
    }
}

// 数据校验与分发
void Robot::serial_receive_check_and_distribute(uint8_t msg_type, uint8_t* buffer_data)
{
     if(msg_type == FOUNDTYPE_PACKAGES)
     {
        serial_receive_process(buffer_data);
     }
}

// 串口数据包解析
void Robot::serial_receive_process(const uint8_t * buffer_data)
{
    // 记录当前时间
    current_time_ = this->get_clock()->now();
    
    // imu部分
    // 解析串口接受的imu数据
    // gyro
    imu_get_vector_[0] = ((double)((int16_t)(buffer_data[4] * 256 + buffer_data[5])) / 32768 * 2000 / 180 * 3.1415);
    imu_get_vector_[1] = ((double)((int16_t)(buffer_data[6] * 256 + buffer_data[7])) / 32768 * 2000 / 180 * 3.1415);
    imu_get_vector_[2] = ((double)((int16_t)(buffer_data[8] * 256 + buffer_data[9])) / 32768 * 2000 / 180 * 3.1415);
    // Acc
    imu_get_vector_[3] = ((double)((int16_t)(buffer_data[10] * 256 + buffer_data[11])) / 32768 * 2 * 9.8);
    imu_get_vector_[4] = ((double)((int16_t)(buffer_data[12] * 256 + buffer_data[13])) / 32768 * 2 * 9.8);
    imu_get_vector_[5] = ((double)((int16_t)(buffer_data[14] * 256 + buffer_data[15])) / 32768 * 2 * 9.8);
    // Angle
    imu_get_vector_[6] = ((double)((int16_t)(buffer_data[16] * 256 + buffer_data[17])) / 100);
    imu_get_vector_[7] = ((double)((int16_t)(buffer_data[18] * 256 + buffer_data[19])) / 100);
    imu_get_vector_[8] = ((double)((int16_t)(buffer_data[20] * 256 + buffer_data[21])) / 100);
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
    // 解析串口接受的odom数据
    odom_get_vector_[0] = ((double)((int16_t)(buffer_data[22] * 256 + buffer_data[23])) / 1000);
    odom_get_vector_[1] = ((double)((int16_t)(buffer_data[24] * 256 + buffer_data[25])) / 1000);
    odom_get_vector_[2] = ((double)((int16_t)(buffer_data[26] * 256 + buffer_data[27])) / 1000);
    // dx dy dyaw base_frame
    odom_get_vector_[3] = ((double)((int16_t)(buffer_data[28] * 256 + buffer_data[29])) / 1000);
    odom_get_vector_[4] = ((double)((int16_t)(buffer_data[30] * 256 + buffer_data[31])) / 1000);
    odom_get_vector_[5] = ((double)((int16_t)(buffer_data[32] * 256 + buffer_data[33])) / 1000);
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
    wheelvel_get_vector_[0]=((int16_t)(buffer_data[34]*256+buffer_data[35]));
    wheelvel_get_vector_[1]=((int16_t)(buffer_data[36]*256+buffer_data[37]));
    wheelvel_get_vector_[2]=((int16_t)(buffer_data[38]*256+buffer_data[39]));
    wheelvel_get_vector_[3]=((int16_t)(buffer_data[40]*256+buffer_data[41]));
    wheelvel_set_vector_[0]=((int16_t)(buffer_data[42]*256+buffer_data[43]));
    wheelvel_set_vector_[1]=((int16_t)(buffer_data[44]*256+buffer_data[45]));
    wheelvel_set_vector_[2]=((int16_t)(buffer_data[46]*256+buffer_data[47]));
    wheelvel_set_vector_[3]=((int16_t)(buffer_data[48]*256+buffer_data[49]));
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
    // 解析串口接受的battery数据
    battery_pub_data_.data = (double)(((buffer_data[50]<<8)+buffer_data[51]))/100;
    // pub battery
    pub_battery_->publish(battery_pub_data_);

    // 更新时间
    last_time_ = current_time_;
}

// 串口发送PID参数
void Robot::serial_send_pid(int p,int i, int d)
{
    static uint8_t pid_data[11];
    pid_data[0] = HEAD1;
    pid_data[1] = HEAD2;
    pid_data[2] = 0x0b;
    pid_data[3] = SENDTYPE_PID;
    pid_data[4] = (p >> 8) & 0xff;
    pid_data[5] = p & 0xff;
    pid_data[6] = (i >> 8) & 0xff;
    pid_data[7] = i & 0xff;
    pid_data[8] = (d >> 8) & 0xff;
    pid_data[9] = d & 0xff;
    check_sum(pid_data,10,pid_data[10]);
    RCLCPP_INFO(this->get_logger(), "Send Time: %f", this->get_clock()->now().seconds());
    boost::asio::write(*serial_port_.get(), boost::asio::buffer(pid_data, 11), serial_port_ec_);
    RCLCPP_INFO(this->get_logger(), "Send End Time: %f", this->get_clock()->now().seconds());
}

// 串口发送底盘矫正参数
void Robot::serial_send_correction(double linear_correction,double angular_correction)
{
    RCLCPP_INFO(this->get_logger(), "linear_correction:%f angular_correction:%f", linear_correction, angular_correction);
    static uint8_t param_data[20];
    param_data[0]  = HEAD1;
    param_data[1]  = HEAD2;
    param_data[2]  = 0x09;
    param_data[3]  = SENDTYPE_PARAMS;
    param_data[4] = (int16_t)((int16_t)(linear_correction * 1000) >> 8) & 0xff;
    param_data[5] = (int16_t)(linear_correction * 1000) & 0xff;
    param_data[6] = (int16_t)((int16_t)(angular_correction * 1000) >> 8) & 0xff;
    param_data[7] = (int16_t)(angular_correction * 1000) & 0xff;
    check_sum(param_data, 8, param_data[8]);
    boost::asio::write(*serial_port_.get(), boost::asio::buffer(param_data, 9), serial_port_ec_);
}

// 串口发送底盘速度
void Robot::serial_send_velocity(double x, double y, double yaw)
{
    static uint8_t vel_data[11];
    vel_data[0] = HEAD1;
    vel_data[1] = HEAD2;
    vel_data[2] = 0x0b;
    vel_data[3] = SENDTYPE_VELOCITY;
    vel_data[4] = ((int16_t)(x * 1000) >> 8) & 0xff;
    vel_data[5] = ((int16_t)(x * 1000)) & 0xff;
    vel_data[6] = ((int16_t)(y * 1000) >> 8) & 0xff;
    vel_data[7] = ((int16_t)(y * 1000)) & 0xff;
    vel_data[8] = ((int16_t)(yaw * 1000) >> 8) & 0xff;
    vel_data[9] = ((int16_t)(yaw * 1000)) & 0xff;
    check_sum(vel_data, 10, vel_data[10]);
    boost::asio::write(*serial_port_.get(), boost::asio::buffer(vel_data, 11), serial_port_ec_); //这句没理解,先用着, 以后再说吧
    // if(x==1&&y==0&&yaw==0) RCLCPP_INFO(this->get_logger(), "DEBUG");
}

void help_print()
{
  printf("使用帮助: \n");
  printf("robot [-p custom_port_name] [-h]\n");
  printf("参数:\n");
  printf("-h 显示帮助\n");
  printf("-p 自定义串口号\n");
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Robot>();

    //多线程
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // node->robot_loop();

    executor.spin();
    // 底盘速度设为0
    node->serial_send_velocity(0, 0, 0);
    rclcpp::shutdown();
    return 0;
}
