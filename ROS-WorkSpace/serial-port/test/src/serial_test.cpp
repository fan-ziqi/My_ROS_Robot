#include "serial_linux.hpp"
#include <glog/logging.h>
using namespace std::chrono_literals;

std::string port = "/dev/ttyUSB1";

Serial my_serial(port);

short ReceiveTest[4] = {0};
unsigned int ReceiveCodeTest = 0x11;//why????????????????????????????????????????

short SendTest[4] = {123, 456, 789, 1120};
unsigned int SendCodeTest = 0x66;

class SerialNode
{
public:
	// 构造函数,有一个参数为节点名称
	SerialNode()
	{
		// 打印一句自我介绍
		printf("Create serial_test_node Success");

		if(my_serial.Serial_EC)
		{
			printf("Serial Init Failed, Error_Code = %d", my_serial.Serial_EC);
		}
		else
		{
			printf("Serial Init Success");
		}

//        // 串口发送速度定时器
//        write_timer_ = this->create_wall_timer(std::chrono::duration<double>(1000ms), std::bind(&SerialNode::write_timer_callback, this));

		// read_timer_ = this->create_wall_timer(std::chrono::duration<double>(100ms), std::bind(&SerialNode::read_timer_callback, this));


	}

private:

//    void write_timer_callback()
//    {
//        //test string
//		printf("Write test string");
//        my_serial.write_to_serial("Hello World\r\n");
//		printf("Read test string");
//        my_serial.read_from_serial();
//        std::cout << std::endl;
//
//        // //test format
//        // RCLCPP_INFO(this->get_logger(), "Write test format");
//        // my_serial.SerialWrite(SendTest, 4, SendCodeTest);
//        // RCLCPP_INFO(this->get_logger(), "Read test format");
//        // my_serial.SerialRead(ReceiveTest, &ReceiveCodeTest);
//        // RCLCPP_INFO(this->get_logger(), "ReceiveTest[0] = %d, ReceiveTest[1] = %d, ReceiveTest[2] = %d, ReceiveTest[3] = %d, ReceiveCodeTest = %x", ReceiveTest[0], ReceiveTest[1], ReceiveTest[2], ReceiveTest[3], ReceiveCodeTest);
//        // std::cout << std::endl;
//    }

//     void read_timer_callback()
//     {
//         // RCLCPP_INFO(this->get_logger(), "Read test string");
//         // my_serial.read_from_serial();
//
//         my_serial.io_service_.run();
//         RCLCPP_INFO(this->get_logger(), "Read test format");
//         my_serial.SerialRead(ReceiveTest, &ReceiveCodeTest);
//         RCLCPP_INFO(this->get_logger(), "ReceiveTest[0] = %d, ReceiveTest[1] = %d, ReceiveTest[2] = %d, ReceiveTest[3] = %d, ReceiveCodeTest = %x", ReceiveTest[0], ReceiveTest[1], ReceiveTest[2], ReceiveTest[3], ReceiveCodeTest);

	// }

};

void read_callback()
{
	while(1)
	{
		printf("read_thread\r\n");
		my_serial.SerialRead(ReceiveTest, &ReceiveCodeTest);
		printf("ReceiveTest[0] = %d, ReceiveTest[1] = %d, ReceiveTest[2] = %d, ReceiveTest[3] = %d, ReceiveCodeTest = %x\r\n", ReceiveTest[0], ReceiveTest[1], ReceiveTest[2], ReceiveTest[3], ReceiveCodeTest);
		std::cout << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}

void write_callback()
{
	while(1)
	{
		printf("write_thread ");
		my_serial.SerialWrite(SendTest, 4, SendCodeTest);
		std::cout << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}



int main(int agrc,char **argv)
{
	google::InitGoogleLogging("test");
	FLAGS_logtostderr = 1;  //输出到控制台

	LOG(INFO) 	<< "<INFO> Hello World!";
	LOG(WARNING) << "<WARNING> Hello World!";
//	LOG(ERROR) 	<< "<ERROR>Hello World!";
//	LOG(FATAL) 	<< "<FATAL>Hello World!";

	std::thread th1(write_callback);
//	std::thread th2(read_callback);

	th1.join();
//	th2.join();

	google::ShutdownGoogleLogging();
	return 0;
}
 



