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
    char Serial_EC;
    Serial(const std::string &port_name_);
    ~Serial();
    bool SerialInit();
    bool SerialWrite(short * WriteData, unsigned char MessageLenth, unsigned int MessageCode);
    bool SerialRead(short * ReceiveData, unsigned int * ReceiveCode);
    unsigned char CalCrc(unsigned char * VectorData, unsigned short len);


    void write_to_serial(std::string data);
    void read_from_serial();

private:
    boost::system::error_code error_code_;
    boost::asio::io_service io_service_;

    //use normal pointer step1
    // boost::asio::serial_port *serial_port_;

    //use shared_ptr step1
    std::shared_ptr<boost::asio::serial_port> serial_port_;
    
};
#endif

