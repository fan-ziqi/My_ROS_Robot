#include "rclcpp/rclcpp.hpp"
#include "serial_linux.hpp"
using namespace std::chrono_literals;

std::string port = "/dev/ttyUSB0";

Serial my_serial(port);

short ReceiveTest[4] = {0};
unsigned int ReceiveCodeTest = 0x11;//why????????????????????????????????????????

short SendTest[4] = {123, 456, 789, 1120};
unsigned int SendCodeTest = 0x66;

class SerialNode : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    SerialNode() : Node("serial_test_node")
    {
        // 打印一句自我介绍
        RCLCPP_INFO(this->get_logger(), "Create serial_test_node Success");

        if(my_serial.Serial_EC)
        {
            RCLCPP_INFO(this->get_logger(), "Serial Init Failed, Error_Code = %d", my_serial.Serial_EC);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Serial Init Success");
        }
        
        // 串口发送速度定时器
        write_timer_ = this->create_wall_timer(std::chrono::duration<double>(1000ms), std::bind(&SerialNode::write_timer_callback, this));

        // read_timer_ = this->create_wall_timer(std::chrono::duration<double>(100ms), std::bind(&SerialNode::read_timer_callback, this));
    
        
    }

private:
    rclcpp::TimerBase::SharedPtr write_timer_;
    // rclcpp::TimerBase::SharedPtr read_timer_;

    void write_timer_callback()
    {
        //test string
        RCLCPP_INFO(this->get_logger(), "Write test string");
        my_serial.write_to_serial("Hello World\r\n");
        RCLCPP_INFO(this->get_logger(), "Read test string");
        my_serial.read_from_serial();
        std::cout << std::endl;

        // //test format
        // RCLCPP_INFO(this->get_logger(), "Write test format");
        // my_serial.SerialWrite(SendTest, 4, SendCodeTest);
        // RCLCPP_INFO(this->get_logger(), "Read test format");
        // my_serial.SerialRead(ReceiveTest, &ReceiveCodeTest);
        // RCLCPP_INFO(this->get_logger(), "ReceiveTest[0] = %d, ReceiveTest[1] = %d, ReceiveTest[2] = %d, ReceiveTest[3] = %d, ReceiveCodeTest = %x", ReceiveTest[0], ReceiveTest[1], ReceiveTest[2], ReceiveTest[3], ReceiveCodeTest);
        // std::cout << std::endl;
    }

    // void read_timer_callback()
    // {
    //     // RCLCPP_INFO(this->get_logger(), "Read test string");
    //     // my_serial.read_from_serial();
        
    //     my_serial.io_service_.run();
    //     RCLCPP_INFO(this->get_logger(), "Read test format");
    //     my_serial.SerialRead(ReceiveTest, &ReceiveCodeTest);
    //     RCLCPP_INFO(this->get_logger(), "ReceiveTest[0] = %d, ReceiveTest[1] = %d, ReceiveTest[2] = %d, ReceiveTest[3] = %d, ReceiveCodeTest = %x", ReceiveTest[0], ReceiveTest[1], ReceiveTest[2], ReceiveTest[3], ReceiveCodeTest);
        
    // }
   
};



int main(int agrc,char **argv)
{
    rclcpp::init(agrc, argv);

    auto node = std::make_shared<SerialNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
 



