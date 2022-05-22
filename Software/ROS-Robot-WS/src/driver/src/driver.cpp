#include "../include/driver/driver.h"

Driver::Driver():msg_seq_(0),start_flag_(true),state_(waitingForHead1),first_init_(false){}
Driver::~Driver()
{
    boost::mutex::scoped_lock look(mutex_);
    recv_flag_ = false;
    if(sp_)
    {
        sp_->cancel();
        sp_->close();
        sp_.reset();
    }
    io_service_.stop();
    io_service_.reset();
}
/*动态参数配置服务回调函数*/
void Driver::dynamic_reconfig_callback(driver::PID_reconfigConfig &config,uint32_t level)
{
   if(first_init_)
   {
	    ROS_INFO("Set PID P:[%d], I:[%d], D:[%d]",config.Kp,config.Ki,config.Kd);
        SetPID(config.Kp,config.Ki,config.Kd);
        ros::Duration(0.02).sleep();
   }
    else
    {
	    first_init_=true;
	    ROS_INFO("Set PID P:[%d], I:[%d], D:[%d]",kp_,ki_,kd_);
    }   
}
/*主循环函数*/
void Driver::loop()
{
    uint8_t stop_buf[13];
    ros::NodeHandle nh_;
    ros::NodeHandle nh_p_("~");

    /*从配置文件中获取机器人参数*/
    nh_p_.param<std::string>("port_name",port_name_,std::string("/dev/ttyUSB0"));
    nh_p_.param<std::string>("odom_frame",odom_frame_,std::string("odom"));
    nh_p_.param<std::string>("base_frame",base_frame_,std::string("base_footprint"));
    nh_p_.param<std::string>("imu_frame",imu_frame_,std::string("base_imu_link"));

    nh_p_.param<int>("baud_rate",baud_rate_,115200);
    nh_p_.param<int>("control_rate",control_rate_,50);
    
    nh_p_.param<double>("linear_correction_factor",linear_correction_factor_,1.0);
    nh_p_.param<double>("angular_correction_factor",angular_correction_factor_,1.0);

    nh_p_.param<bool>("publish_odom_transform",publish_odom_transform_,true);
    nh_p_.param<int>("Kp",kp_,300);
    nh_p_.param<int>("Ki",ki_,0);
    nh_p_.param<int>("Kd",kd_,200);
    

    odom_list_.resize(6,0.0);
    imu_list_.resize(9,0.0);
    wheelspeedSet_list_.resize(4,0);
    wheelspeedGet_list_.resize(4,0);
    /*初始化机器人硬件串口以及publisher发布器和定时器与动态配置参数服务器等的回调函数，然后进入ros::spin()循环*/
    if(initRobot())
    {
        odom_pub_    = nh_.advertise<nav_msgs::Odometry>("odom",10);
        battery_pub_ = nh_.advertise<std_msgs::Float32>("voltage",1);
        imu_pub_     = nh_.advertise<sensor_msgs::Imu>("imu",10);
        avel_pub_    = nh_.advertise<std_msgs::Int32>("robot/avel",10);
        bvel_pub_    = nh_.advertise<std_msgs::Int32>("robot/bvel",10);
        cvel_pub_    = nh_.advertise<std_msgs::Int32>("robot/cvel",10);
        dvel_pub_    = nh_.advertise<std_msgs::Int32>("robot/dvel",10);

        aset_pub_    = nh_.advertise<std_msgs::Int32>("robot/aset",10);
        bset_pub_    = nh_.advertise<std_msgs::Int32>("robot/bset",10);
        cset_pub_    = nh_.advertise<std_msgs::Int32>("robot/cset",10);
        dset_pub_    = nh_.advertise<std_msgs::Int32>("robot/dset",10);

        cmd_sub_     = nh_.subscribe<geometry_msgs::Twist>("cmd_vel",10,&Driver::cmd_vel_callback,this);
        
        SetParams(linear_correction_factor_,angular_correction_factor_);
        ros::Duration(0.02).sleep();
        SetPID(kp_,ki_,kd_);

        ros::Timer send_speed_timer = nh_.createTimer(ros::Duration(1.0/control_rate_),&Driver::send_speed_callback,this);

        boost::thread recv_thread(boost::bind(&Driver::recv_msg,this));

	    dynamic_reconfigure::Server<driver::PID_reconfigConfig> reconfig_server;
	    dynamic_reconfigure::Server<driver::PID_reconfigConfig>::CallbackType f;

	    f = boost::bind(&Driver::dynamic_reconfig_callback,this,_1,_2);
	    reconfig_server.setCallback(f);
        ROS_INFO("Robot Running!");
        ros::spin();
    	SetVelocity(0,0,0);
        return ;
    }
}
/*初始化硬件端口*/
bool Driver::initRobot()
{
    if(sp_)
    {
        ROS_ERROR("The SerialPort is already opened!");
        return false;
    }
     sp_ = serialp_ptr(new boost::asio::serial_port(io_service_));
     sp_->open(port_name_,ec_);
     if(ec_)
     {
        ROS_ERROR_STREAM("error : port_->open() failed...port_name=" << port_name_ << ", e=" << ec_.message().c_str());
        return false;
     }
    sp_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    sp_->set_option(boost::asio::serial_port_base::character_size(8));
    sp_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    sp_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    sp_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    
    return true;
}
/*将收到的/cmd_vel上的速度话题消息通过串口发送给机器人*/
void Driver::send_speed_callback(const ros::TimerEvent&)
{
    double linear_x_speed, linear_y_speed,angular_speed;
    if((ros::Time::now() - last_twist_time_).toSec() <= 1.0)
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

    if((ros::Time::now() - now_).toSec() >=1)
    {
        ROS_WARN_THROTTLE(1,"Didn't received odom data,Please check your connection!");
    }
    SetVelocity(linear_x_speed,linear_y_speed,angular_speed);
}
/*cmd_vel Subscriber的回调函数*/
void Driver::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    try
    {
        cmd_vel_mutex_.lock();
        last_twist_time_ = ros::Time::now();
        current_twist_ = *msg.get();
        cmd_vel_mutex_.unlock();
    }
    catch(...)
    {
        cmd_vel_mutex_.unlock();
    }
}
/*串口通信校验位计算辅助函数*/
void Driver::check_sum(uint8_t* data, size_t len, uint8_t& dest)
{
    dest = 0x00;
    for(int i=0;i<len;i++)
    {
        dest += *(data + i);
    }
}
/*与OpenCRP串口通信接收数据线程*/
void Driver::recv_msg()
{
    uint8_t payload_size, check_num, buffer_data[255],payload_type;
    state_ = waitingForHead1;
    recv_flag_ = true;
    while(recv_flag_)
    {
        switch (state_)
        {
            case waitingForHead1:
                check_num = 0x00;
                boost::asio::read(*sp_.get(), boost::asio::buffer(&buffer_data[0], 1), ec_);
                state_ = buffer_data[0] == head1 ? waitingForHead2 : waitingForHead1;
                if(state_ == waitingForHead1)
                {
                    ROS_DEBUG_STREAM("recv head1 error : ->"<<(int)buffer_data[0]);
                }
                break;
            case waitingForHead2:
                boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[1],1),ec_);
                state_ = buffer_data[1] == head2 ? waitingForPayloadSize : waitingForHead1;
                if(state_ == waitingForHead1)
                {
                    ROS_DEBUG_STREAM("recv head2 error : ->"<<(int)buffer_data[1]);
                }
                break;
            case waitingForPayloadSize:
                boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[2],1),ec_);
                payload_size = buffer_data[2] - 4;
                state_ = waitingForPayload;
                break;
            case waitingForPayload:
                boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[3],payload_size),ec_);
                payload_type = buffer_data[3];
                state_ = waitingForCheckSum;
                break;
            case waitingForCheckSum:
                boost::asio::read(*sp_.get(),boost::asio::buffer(&buffer_data[3+payload_size],1),ec_);
                check_sum(buffer_data,3+payload_size,check_num);
                state_ = buffer_data[3+payload_size] == check_num ? handlePayload : waitingForHead1;
                if(state_ == waitingForHead1)
                {
                    ROS_DEBUG_STREAM("check sum error! recv is  : ->"<<(int)buffer_data[3+payload_size]<<"  calc is "<<check_num);
                }
                break;
            case handlePayload:
                distribute_data(payload_type, buffer_data);
                state_ = waitingForHead1;
                break;
            default:
                state_ = waitingForHead1;
                break;
        }
    }
}
/*数据校验分发*/
void Driver::distribute_data(uint8_t msg_type, uint8_t* buffer_data)
{
     if(msg_type == foundType_Packages)
        handle_base_data(buffer_data);
}
/*PID参数发送函数*/
void Driver::SetPID(int p,int i, int d)
{
    static uint8_t pid_data[11];
    pid_data[0] = head1;
    pid_data[1] = head2;
    pid_data[2] = 0x0b;
    pid_data[3] = sendType_pid;
    pid_data[4] = (p>>8)&0xff;
    pid_data[5] = p&0xff;
    pid_data[6] = (i>>8)&0xff;
    pid_data[7] = i&0xff;
    pid_data[8] = (d>>8)&0xff;
    pid_data[9] = d&0xff;
    check_sum(pid_data,10,pid_data[10]);
    ROS_INFO("Send Time: %f",ros::Time::now().toSec());
    boost::asio::write(*sp_.get(),boost::asio::buffer(pid_data,11),ec_);
    ROS_INFO("Send End Time: %f",ros::Time::now().toSec());
}
/*底盘参数发送函数*/
void Driver::SetParams(double linear_correction,double angular_correction) {
    printf("%f %f \r\n",linear_correction,angular_correction);
    static uint8_t param_data[20];
    param_data[0]  = head1;
    param_data[1]  = head2;
    param_data[2]  = 0x09;
    param_data[3]  = sendType_params;
    param_data[4]  = (int16_t)((int16_t)(linear_correction*1000)>>8)   &0xff;
    param_data[5]  = (int16_t)(linear_correction*1000)        &0xff;
    param_data[6]  = (int16_t)((int16_t)(angular_correction*1000)>>8)      &0xff;
    param_data[7]  = (int16_t)(angular_correction*1000)                    &0xff;
    check_sum(param_data,8,param_data[8]);
    boost::asio::write(*sp_.get(),boost::asio::buffer(param_data,9),ec_);
}
/*底盘速度发送函数*/
void Driver::SetVelocity(double x, double y, double yaw)
{
    static uint8_t vel_data[11];
    vel_data[0] = head1;
    vel_data[1] = head2;
    vel_data[2] = 0x0b;
    vel_data[3] = sendType_velocity;
    vel_data[4] = ((int16_t)(x*1000)>>8) & 0xff;
    vel_data[5] = ((int16_t)(x*1000)) & 0xff;
    vel_data[6] = ((int16_t)(y*1000)>>8) & 0xff;
    vel_data[7] = ((int16_t)(y*1000)) & 0xff;
    vel_data[8] = ((int16_t)(yaw*1000)>>8) & 0xff;
    vel_data[9] = ((int16_t)(yaw*1000)) & 0xff;
    check_sum(vel_data,10,vel_data[10]);
    boost::asio::write(*sp_.get(),boost::asio::buffer(vel_data,11),ec_);
}
/*收到串口数据包解析函数*/
void Driver::handle_base_data(const uint8_t* buffer_data)
{
        now_ = ros::Time::now();
        //gyro
        imu_list_[0]=((double)((int16_t)(buffer_data[4]*256+buffer_data[5]))/32768*2000/180*3.1415);
        imu_list_[1]=((double)((int16_t)(buffer_data[6]*256+buffer_data[7]))/32768*2000/180*3.1415);
        imu_list_[2]=((double)((int16_t)(buffer_data[8]*256+buffer_data[9]))/32768*2000/180*3.1415);
        //Acc 
        imu_list_[3]=((double)((int16_t)(buffer_data[10]*256+buffer_data[11]))/32768*2*9.8);
        imu_list_[4]=((double)((int16_t)(buffer_data[12]*256+buffer_data[13]))/32768*2*9.8);
        imu_list_[5]=((double)((int16_t)(buffer_data[14]*256+buffer_data[15]))/32768*2*9.8);
        //Angle 
        imu_list_[6]=((double)((int16_t)(buffer_data[16]*256+buffer_data[17]))/100);
        imu_list_[7]=((double)((int16_t)(buffer_data[18]*256+buffer_data[19]))/100);
        imu_list_[8]=((double)((int16_t)(buffer_data[20]*256+buffer_data[21]))/100);
        imu_pub_data_.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,(imu_list_[8]/180*3.1415926));
        imu_pub_data_.header.stamp = ros::Time::now();
        imu_pub_data_.header.frame_id = imu_frame_;
        imu_pub_data_.angular_velocity.x = imu_list_[0];
        imu_pub_data_.angular_velocity.y = imu_list_[1];
        imu_pub_data_.angular_velocity.z = imu_list_[2];
        imu_pub_data_.linear_acceleration.x = imu_list_[3];
        imu_pub_data_.linear_acceleration.y = imu_list_[4];
        imu_pub_data_.linear_acceleration.z = imu_list_[5];
        imu_pub_data_.orientation_covariance = {1e6, 0, 0,
                                            0, 1e6, 0,
                                            0, 0, 0.05};
        imu_pub_data_.angular_velocity_covariance = {1e6, 0, 0,
                                                 0, 1e6, 0,
                                                 0, 0, 1e6};
        imu_pub_data_.linear_acceleration_covariance = {1e-2, 0, 0,
                                                     0, 0, 0,
                                                     0, 0, 0};
       	imu_pub_.publish(imu_pub_data_);


        odom_list_[0]=((double)((int16_t)(buffer_data[22]*256+buffer_data[23]))/1000);
        odom_list_[1]=((double)((int16_t)(buffer_data[24]*256+buffer_data[25]))/1000);
        odom_list_[2]=((double)((int16_t)(buffer_data[26]*256+buffer_data[27]))/1000);
        //dx dy dyaw base_frame
        odom_list_[3]=((double)((int16_t)(buffer_data[28]*256+buffer_data[29]))/1000);
        odom_list_[4]=((double)((int16_t)(buffer_data[30]*256+buffer_data[31]))/1000);
        odom_list_[5]=((double)((int16_t)(buffer_data[32]*256+buffer_data[33]))/1000);

        transformStamped_.header.stamp    = now_;
        transformStamped_.header.frame_id = odom_frame_;
        transformStamped_.child_frame_id  = base_frame_;
        transformStamped_.transform.translation.x = odom_list_[0];
        transformStamped_.transform.translation.y = odom_list_[1];
        transformStamped_.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0,0,odom_list_[2]);
        transformStamped_.transform.rotation.x = q.x();
        transformStamped_.transform.rotation.y = q.y();
        transformStamped_.transform.rotation.z = q.z();
        transformStamped_.transform.rotation.w = q.w();
        if(publish_odom_transform_)
            br_.sendTransform(transformStamped_);

        odom_.header.frame_id = odom_frame_;
        odom_.child_frame_id  = base_frame_;
        odom_.header.stamp    = now_;
        odom_.pose.pose.position.x = odom_list_[0];
        odom_.pose.pose.position.y = odom_list_[1];
        odom_.pose.pose.position.z = 0;
        odom_.pose.pose.orientation.x = q.getX();
        odom_.pose.pose.orientation.y = q.getY();
        odom_.pose.pose.orientation.z = q.getZ();
        odom_.pose.pose.orientation.w = q.getW();
        odom_.twist.twist.linear.x = odom_list_[3]/((now_-last_time_).toSec());
        odom_.twist.twist.linear.y = odom_list_[4]/((now_-last_time_).toSec());
        odom_.twist.twist.angular.z = odom_list_[5]/((now_-last_time_).toSec());
        odom_.twist.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                              0, 1e-3, 1e-9, 0, 0, 0, 
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0, 
                              0, 0, 0, 0, 1e6, 0, 
                              0, 0, 0, 0, 0, 0.1 };
        odom_.pose.covariance = { 1e-9, 0, 0, 0, 0, 0, 
                              0, 1e-3, 1e-9, 0, 0, 0, 
                              0, 0, 1e6, 0, 0, 0,
                              0, 0, 0, 1e6, 0, 0, 
                              0, 0, 0, 0, 1e6, 0, 
                              0, 0, 0, 0, 0, 1e3 };
        odom_pub_.publish(odom_);
        last_time_ = now_;

        wheelspeedGet_list_[0]=((int16_t)(buffer_data[34]*256+buffer_data[35]));
        wheelspeedGet_list_[1]=((int16_t)(buffer_data[36]*256+buffer_data[37]));
        wheelspeedGet_list_[2]=((int16_t)(buffer_data[38]*256+buffer_data[39]));
        wheelspeedGet_list_[3]=((int16_t)(buffer_data[40]*256+buffer_data[41]));
        
        wheelspeedSet_list_[0]=((int16_t)(buffer_data[42]*256+buffer_data[43]));
        wheelspeedSet_list_[1]=((int16_t)(buffer_data[44]*256+buffer_data[45]));
        wheelspeedSet_list_[2]=((int16_t)(buffer_data[46]*256+buffer_data[47]));
        wheelspeedSet_list_[3]=((int16_t)(buffer_data[48]*256+buffer_data[49]));
       
        avel_pub_data_.data = wheelspeedGet_list_[0];
        bvel_pub_data_.data = wheelspeedGet_list_[1];
        cvel_pub_data_.data = wheelspeedGet_list_[2];
        dvel_pub_data_.data = wheelspeedGet_list_[3];

        aset_pub_data_.data = wheelspeedSet_list_[0];
        bset_pub_data_.data = wheelspeedSet_list_[1];
        cset_pub_data_.data = wheelspeedSet_list_[2];
        dset_pub_data_.data = wheelspeedSet_list_[3];

        avel_pub_.publish(avel_pub_data_);
        bvel_pub_.publish(bvel_pub_data_);
        cvel_pub_.publish(cvel_pub_data_);
        dvel_pub_.publish(dvel_pub_data_);

        aset_pub_.publish(aset_pub_data_);
        bset_pub_.publish(bset_pub_data_);
        cset_pub_.publish(cset_pub_data_);
        dset_pub_.publish(dset_pub_data_);

        battery_pub_data_.data = (double)(((buffer_data[50]<<8)+buffer_data[51]))/100;
        battery_pub_.publish(battery_pub_data_);
}
int main(int argc,char** argv)
{
    ros::init(argc,argv,"Driver_node");
    Driver driver;
    driver.loop();
    return 0;
}
