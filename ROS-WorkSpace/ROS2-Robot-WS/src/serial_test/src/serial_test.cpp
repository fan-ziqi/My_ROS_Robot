#include "rclcpp/rclcpp.hpp"
#include "serial_linux.hpp"
using namespace std::chrono_literals;

Serial serial;

std::string port = "/dev/ttys003";

double ReceiveTest[2] = {0};
unsigned char ReceiveCodeTest;

double SendTest[4] = {123, 456, 789, 1120};
unsigned char SendCodeTest = 0x66;

class SerialNode : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    SerialNode() : Node("serial_node")
    {
        // 打印一句自我介绍
        RCLCPP_INFO(this->get_logger(), "Create SerialNode");

        serial.SerialInit(port);
        RCLCPP_INFO(this->get_logger(), "INIT");
        
        // 串口发送速度定时器
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1000ms), std::bind(&SerialNode::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        serial.SerialWrite(SendTest, 4, SendCodeTest);
        RCLCPP_INFO(this->get_logger(), "SEND");
        // serial.SerialRead(ReceiveTest, &ReceiveCodeTest);
        // RCLCPP_INFO(node->get_logger(), "ReceiveTest[0] = %f, ReceiveTest[1] = %f, ReceiveCodeTest = %d\n", ReceiveTest[0], ReceiveTest[1], ReceiveCodeTest);

    }
   
};



int main(int agrc,char **argv)
{
    rclcpp::init(agrc, argv);

    auto node = std::make_shared<SerialNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
 



