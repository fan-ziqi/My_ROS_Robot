#ifndef SERIAL_LINUX_H
#define SERIAL_LINUX_H

#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <iostream>
#include <iomanip>

// 消息头/消息尾定义
const unsigned char HEAD[2] = {0x55, 0xaa};
const unsigned char END[2] = {0x0d, 0x0a};

class Serial
{
public:
    void SerialInit(std::string &port_name_);
    void SerialWrite(double * WriteData, unsigned char MessageLenth, unsigned char MessageCode);
    bool SerialRead(double * ReceiveData, unsigned char * ReceiveCode);
    unsigned char CalCrc(unsigned char * VectorData, unsigned short len);

private:
    boost::system::error_code error_code_;
    boost::asio::io_service io_service_;
    std::shared_ptr<boost::asio::serial_port> serial_port_;
};

#endif
