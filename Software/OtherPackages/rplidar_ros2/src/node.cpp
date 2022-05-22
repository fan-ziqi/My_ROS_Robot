/*
 *  RPLIDAR ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *  Copyright (c) 2020 - 2023 Wheel Hub Intelligent Co., Ltd.
 *  refactoring author: xinjue.zou.whi@gmail.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include "rplidar.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace rp::standalone::rplidar;
using namespace std::chrono_literals;

class rplidarROS2 : public rclcpp::Node
{
public:
	rplidarROS2()
		: Node("rplidar_ros2")
		, driver_(nullptr)
		, angle_compensate_multiple_(1) // it stand of angle compensate at per 1 degree
		, frequency_(5.5)
		, channel_type_("serial")
		, tcp_ip_("192.168.0.7")
		, tcp_port_(20108)
		// , serial_port_("/dev/ttyUSB0")
		, serial_port_("/dev/rplidar")
		, serial_baudrate_(115200/*256000*/) // ros run for A1 A2, change to 256000 if A3;
		, frame_id_("laser")
		, inverted_(false)
		, angle_compensate_(false)
		, scan_mode_(std::string())
		, max_distance_(8.0)
		, screened_begin_(91)
		, screened_end_(179)
	{
		RCLCPP_INFO(get_logger(), "RPLIDAR running on ROS package rplidar_ros. SDK Version:"RPLIDAR_SDK_VERSION"");
		
		declare_parameters();
		get_parameters();
		
		rclcpp::QoS qos(rclcpp::KeepLast(50));
		scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);
		start_motor_server_ = create_service<std_srvs::srv::Empty>("start_motor", std::bind(&rplidarROS2::start_motor, this, std::placeholders::_1, std::placeholders::_2));
		stop_motor_server_ = create_service<std_srvs::srv::Empty>("stop_motor", std::bind(&rplidarROS2::stop_motor, this, std::placeholders::_1, std::placeholders::_2));

		timer_ = create_wall_timer(std::chrono::milliseconds(int(ceil(1000.0 / frequency_))), std::bind(&rplidarROS2::spin, this));
		
		connect_driver();
		check_scan_mode();
	};
	
	virtual ~rplidarROS2()
	{
		if (driver_)
		{
			driver_->stop();
			driver_->stopMotor();
			RPlidarDriver::DisposeDriver(driver_.get());
		}
	};
	
protected:
	void declare_parameters()
	{
		declare_parameter<double>("frequency", frequency_);
		declare_parameter<std::string>("channel_type", channel_type_);
		declare_parameter<std::string>("tcp_ip", tcp_ip_);
		declare_parameter<int>("tcp_port", tcp_port_);
		declare_parameter<std::string>("serial_port", serial_port_);
		declare_parameter<int>("serial_baudrate", serial_baudrate_);
		declare_parameter<std::string>("frame_id", frame_id_);
		declare_parameter<bool>("inverted", inverted_);
		declare_parameter<bool>("angle_compensate", angle_compensate_);
		declare_parameter<std::string>("scan_mode", scan_mode_);
		declare_parameter<double>("max_distance", max_distance_);
		declare_parameter<int>("screened_begin", screened_begin_);
		declare_parameter<int>("screened_end", screened_end_);
	}
	
	void get_parameters()
	{
		get_parameter<double>("frequency", frequency_);
		get_parameter<std::string>("channel_type", channel_type_);
		get_parameter<std::string>("tcp_ip", tcp_ip_);
		get_parameter<int>("tcp_port", tcp_port_);
		get_parameter<std::string>("serial_port", serial_port_);
		get_parameter<int>("serial_baudrate", serial_baudrate_);
		get_parameter<std::string>("frame_id", frame_id_);
		get_parameter<bool>("inverted", inverted_);
		get_parameter<bool>("angle_compensate", angle_compensate_);
		get_parameter<std::string>("scan_mode", scan_mode_);
		get_parameter<double>("max_distance", max_distance_);
		get_parameter<int>("screened_begin", screened_begin_);
		get_parameter<int>("screened_end", screened_end_);
	}

	void connect_driver()
	{
		// create the driver instance
		if (channel_type_ == "tcp")
		{
			driver_.reset(RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP));
		}
		else
		{
			driver_.reset(RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT));
		}

    
		if (!driver_)
		{
			RCLCPP_ERROR(get_logger(), "Create Driver fail, exit");
			throw std::runtime_error("runtime_error");
		}

		if (channel_type_ == "tcp")
		{
			// make connection...
			if (IS_FAIL(driver_->connect(tcp_ip_.c_str(), (_u32)tcp_port_)))
			{
				RCLCPP_ERROR(get_logger(), "Error, cannot bind to the specified serial port %s.", serial_port_.c_str());
				RPlidarDriver::DisposeDriver(driver_.get());
				driver_.reset(nullptr);
				throw std::runtime_error("runtime_error");
			}
		}
		else
		{
			// make connection...
			if (IS_FAIL(driver_->connect(serial_port_.c_str(), (_u32)serial_baudrate_)))
			{
				RCLCPP_ERROR(get_logger(), "Error, cannot bind to the specified serial port %s.", serial_port_.c_str());
				RPlidarDriver::DisposeDriver(driver_.get());
				driver_.reset(nullptr);
				throw std::runtime_error("runtime_error");
			}
		}
    
		// get rplidar device info
		if (!get_device_info())
		{
			throw std::runtime_error("runtime_error");
		}

		// check health...
		if (!check_health())
		{
			throw std::runtime_error("runtime_error");
		}

		driver_->startMotor();
	};
	
	void check_scan_mode()
	{
		u_result op_result;
	    RplidarScanMode current_scan_mode;
		if (scan_mode_.empty())
		{
			op_result = driver_->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
		}
		else
		{
			std::vector<RplidarScanMode> allSupportedScanModes;
			op_result = driver_->getAllSupportedScanModes(allSupportedScanModes);

			if (IS_OK(op_result))
			{
				_u16 selectedScanMode = _u16(-1);
				for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++)
				{
					if (iter->scan_mode == scan_mode_)
					{
						selectedScanMode = iter->id;
						break;
					}
				}

				if (selectedScanMode == _u16(-1))
				{
					RCLCPP_ERROR(get_logger(), "scan mode `%s' is not supported by lidar, supported modes:", scan_mode_.c_str());
					for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++)
					{
						RCLCPP_ERROR(get_logger(), "\t%s: max_distance: %.1f m, Point number: %.1fK", iter->scan_mode, iter->max_distance, (1000/iter->us_per_sample));
					}
					op_result = RESULT_OPERATION_FAIL;
				}
				else
				{
					op_result = driver_->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
				}
			}
		}

		if (IS_OK(op_result))
		{
			//default frequent is 10 hz (by motor pwm value), current_scan_mode.us_per_sample is the number of scan point per us
			angle_compensate_multiple_ = (int)(1000  *1000 / current_scan_mode.us_per_sample / 10.0 / 360.0);
			if (angle_compensate_multiple_ < 1) 
			{
				angle_compensate_multiple_ = 1;
			}
			max_distance_ = current_scan_mode.max_distance;
			RCLCPP_INFO(get_logger(), "current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d", current_scan_mode.scan_mode, current_scan_mode.max_distance, (1000 / current_scan_mode.us_per_sample), angle_compensate_multiple_);
		}
		else
		{
			RCLCPP_ERROR(get_logger(), "Can not start scan: %08x!", op_result);
			throw std::runtime_error("runtime_error");
		}	
	};
	
	bool get_device_info()
	{
		u_result result;
		rplidar_response_device_info_t dev_info;

		result = driver_->getDeviceInfo(dev_info);
		if (IS_FAIL(result))
		{
			if (result == RESULT_OPERATION_TIMEOUT)
			{
				RCLCPP_ERROR(get_logger(), "Operation time out. RESULT_OPERATION_TIMEOUT!");
			}
			else
			{
				RCLCPP_ERROR(get_logger(), "Unexpected error, code: %x", result);
			}
			
			return false;
		}

		// print out the device serial number, firmware and hardware version number..
		printf("RPLIDAR S/N: ");
		for (int pos = 0; pos < 16 ;++pos)
		{
			printf("%02X", dev_info.serialnum[pos]);
		}

		RCLCPP_INFO(get_logger(), "Firmware Ver: %d.%02d",
		dev_info.firmware_version >> 8, dev_info.firmware_version & 0xFF);
		RCLCPP_INFO(get_logger(), "Hardware Rev: %d", (int)dev_info.hardware_version);
		
		return true;
	};
	
	bool check_health()
	{
		u_result op_result;
		rplidar_response_device_health_t healthinfo;
		op_result = driver_->getHealth(healthinfo);
		if (IS_OK(op_result))
		{ 
			RCLCPP_INFO(get_logger(), "RPLidar health status : %d", healthinfo.status);
			if (healthinfo.status == RPLIDAR_STATUS_ERROR)
			{
				RCLCPP_ERROR(get_logger(), "Error, rplidar internal error detected. Please reboot the device to retry.");
				return false;
			}
			else
			{
				return true;
			}
		}
		else
		{
			RCLCPP_ERROR(get_logger(), "Error, cannot retrieve rplidar health code: %x", op_result);
			return false;
		}
	};
	
	void stop_motor(std_srvs::srv::Empty::Request::SharedPtr Request, std_srvs::srv::Empty::Response::SharedPtr Response)
	{
		RCLCPP_INFO(get_logger(), "Stop motor");
		driver_->stopMotor();
	};
	
    void start_motor(std_srvs::srv::Empty::Request::SharedPtr Request, std_srvs::srv::Empty::Response::SharedPtr Response)
	{
		if(driver_->isConnected())
		{
			RCLCPP_INFO(get_logger(), "Start motor");
			u_result ans = driver_->startMotor();
			ans = driver_->startScan(0,1);
		}
		else
		{
			RCLCPP_INFO(get_logger(), "lost connection");
		}
	};
	
	void spin()
	{
		rplidar_response_measurement_node_hq_t nodes[360 * 8];
        size_t count = _countof(nodes);

        rclcpp::Time start_scan_time = this->now();
        u_result op_result = driver_->grabScanDataHq(nodes, count); // sycnronized function
        rclcpp::Time end_scan_time = this->now();
        double scan_duration = (end_scan_time - start_scan_time).seconds();

        if (op_result == RESULT_OK)
		{
            op_result = driver_->ascendScanData(nodes, count);
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);
            if (op_result == RESULT_OK)
			{
                if (angle_compensate_)
				{
                    const int angle_compensate_nodes_count = 360 * angle_compensate_multiple_;
                    int angle_compensate_offset = 0;
                    rplidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
                    memset(angle_compensate_nodes, 0, angle_compensate_nodes_count * sizeof(rplidar_response_measurement_node_hq_t));

                    for(int i = 0 ; i < count; i++ )
					{
                        if (nodes[i].dist_mm_q2 != 0)
						{
                            float angle = get_angle(nodes[i]);
                            int angle_value = (int)(angle * angle_compensate_multiple_);
                            if ((angle_value - angle_compensate_offset) < 0)
							{
								angle_compensate_offset = angle_value;
							}
                            for (int j = 0; j < angle_compensate_multiple_; j++)
							{

                                int angle_compensate_nodes_index = angle_value - angle_compensate_offset + j;
                                if(angle_compensate_nodes_index >= angle_compensate_nodes_count)
								{
									angle_compensate_nodes_index = angle_compensate_nodes_count - 1;
								}
                                angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                            }
                        }
                    }
  
					publish_scan(angle_compensate_nodes, angle_compensate_nodes_count, start_scan_time, scan_duration, angle_min, angle_max);
                }
				else
				{
                    int start_node = 0, end_node = 0;
                    int i = 0;
                    // find the first valid node and last valid node
                    while (nodes[i++].dist_mm_q2 == 0) {};
					start_node = i - 1;
                    i = count -1;
                    while (nodes[i--].dist_mm_q2 == 0) {};
					end_node = i+1;

                    angle_min = DEG2RAD(get_angle(nodes[start_node]));
                    angle_max = DEG2RAD(get_angle(nodes[end_node]));

					publish_scan(&nodes[start_node], end_node - start_node + 1, start_scan_time, scan_duration, angle_min, angle_max);
				}
            }
			else if (op_result == RESULT_OPERATION_FAIL)
			{
                // All the data is invalid, just publish them
				publish_scan(nodes, count, start_scan_time, scan_duration, angle_min, angle_max);
            }
			else
			{
				RCLCPP_WARN(get_logger(), "cannot grab scan data");
			}
        }
	};
	
	void publish_scan(rplidar_response_measurement_node_hq_t* nodes, size_t node_count, rclcpp::Time& start, double scan_time, float angle_min, float angle_max)
	{
		static int scan_count = 0;
		sensor_msgs::msg::LaserScan scan_msg;

		scan_msg.header.stamp = start;
		scan_msg.header.frame_id = frame_id_;
		scan_count++;

		bool reversed = (angle_max > angle_min);
		if (reversed)
		{
			scan_msg.angle_min =  M_PI - angle_max;
			scan_msg.angle_max =  M_PI - angle_min;
		}
		else
		{
			scan_msg.angle_min =  M_PI - angle_min;
			scan_msg.angle_max =  M_PI - angle_max;
		}
		scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count - 1);

		scan_msg.scan_time = scan_time;
		scan_msg.time_increment = scan_time / (double)(node_count - 1);
		scan_msg.range_min = 0.15;
		scan_msg.range_max = max_distance_;

		scan_msg.intensities.resize(node_count);
		scan_msg.ranges.resize(node_count);
		bool reverse_data = (!inverted_ && reversed) || (inverted_ && !reversed);
		if (!reverse_data)
		{
			for (size_t i = 0; i < node_count; i++)
			{
				if (i >= screened_begin_ && i <= screened_end_)
				{
					scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
				}
				else
				{
					float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
					if (read_value == 0.0)
                    {
						scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
					}
					else
                    {
						scan_msg.ranges[i] = read_value;
					}
				}
				scan_msg.intensities[i] = (float) (nodes[i].quality >> 2);
			}
		}
		else
		{
			for (size_t i = 0; i < node_count; i++)
			{
				if (i >= screened_begin_ && i <= screened_end_)
				{
					scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
				}
				else
				{
					float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
					if (read_value == 0.0)
					{
						scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
					}
					else
                    {
						scan_msg.ranges[node_count-1-i] = read_value;
					}
				}
				scan_msg.intensities[node_count-1-i] = (float) (nodes[i].quality >> 2);
			}
		}

		scan_publisher_->publish(scan_msg);
	};
	
protected:
	static float get_angle(const rplidar_response_measurement_node_hq_t& node)
	{
		return node.angle_z_q14 * 90.f / 16384.f;
	};
	
protected:
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_motor_server_;
	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_motor_server_;
	std::unique_ptr<RPlidarDriver> driver_;
	rclcpp::TimerBase::SharedPtr timer_;
	int angle_compensate_multiple_;
	
protected:
	double frequency_;
	std::string channel_type_;
	std::string tcp_ip_;
	int tcp_port_;
	std::string serial_port_;
	int serial_baudrate_;
	std::string frame_id_;
	bool inverted_;
	bool angle_compensate_;
	std::string scan_mode_;
	double max_distance_;
	int screened_begin_;
	int screened_end_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
   
	try
	{
		auto node = std::make_shared<rplidarROS2>();
		rclcpp::spin(node);
		rclcpp::shutdown();
	}
	catch (const std::runtime_error& error)
	{
		printf((std::string(error.what()) + "\n").c_str());
		return -1;
	}
	
    return 0;
}

