#include "serial_linux.hpp"

unsigned char SendLength = 3;
unsigned char * test_data = (unsigned char *)malloc(SendLength * sizeof(unsigned char));

// 串口发送/接收联合体, data_transmit大小由data数据类型的字节数决定
union SerialMessageUnion
{
	short data;
	unsigned char data_transmit[2];
}SendUnion, ReceiveUnion;

// 消息格式: 
// 消息头1 消息头2 消息长度 消息码 [消息内容] 校验码 消息尾1 消息尾2

// 串口初始化
void Serial::SerialInit(std::string & port_name_)
{
    auto serial_port_ = std::make_shared<boost::asio::serial_port>(io_service_, port_name_);

    serial_port_->set_option(boost::asio::serial_port::baud_rate(115200));
    serial_port_->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial_port_->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial_port_->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial_port_->set_option(boost::asio::serial_port::character_size(8));    
    
    SendUnion.data = 0;
    ReceiveUnion.data = 0;

    // serial_port_->open(port_name_,error_code_);
    // if(error_code_)
    // {
    //     std::cout << "error : serial_port_->open() failed...port_name=" << port_name_ << ", e=" << error_code_.message().c_str() << std::endl;
    // }


    test_data[0] = 0x01;
    test_data[1] = 0x02;
    test_data[2] = 0x03;

    // boost::asio::write(*serial_port_.get(), boost::asio::buffer(test_data, SendLength), error_code_);

    // double send_init[10] = {0,1,2,3,4,5,6,7,8,9};
    // SerialWrite(send_init, 2, 0);
}

// 串口发送函数
void Serial::SerialWrite(double * WriteData, unsigned char MessageLenth, unsigned char MessageCode)
{
    // 计算发送消息长度
    unsigned char SendLength = (unsigned int)(2 * MessageLenth) + 7;

    // 发送消息缓冲区(变长数组)
    unsigned char * SendBuf = (unsigned char *)malloc(SendLength * sizeof(unsigned char));

    // 消息头 SendBuf[0] SendBuf[1]
    for(int i = 0; i < 2; i++)
    {
        SendBuf[i] = HEAD[i];
    }

    // 消息长度 SendBuf[2]
    SendBuf[2] =  SendLength;

    // 消息码 SendBuf[3]
    SendBuf[3] = MessageCode;

    // 消息内容 SendBuf[4] ~ SendBuf[4 + 2*MessageLenth - 1]
    for(unsigned int i = 0; i < (unsigned int)MessageLenth; i++)
    {
        SendUnion.data = WriteData[i]; //更新消息Union内容
        // DEBUG BEGIN
        std::cout << (double)SendUnion.data << "->" 
                  << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)SendUnion.data_transmit[0] << " " 
                  << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)SendUnion.data_transmit[1] << std::endl;
        // DEBUG END
        for(int j = 0; j < 2; j++)
        {
            //送入缓冲区
            SendBuf[i * 2 + 4 + j] = SendUnion.data_transmit[j];
        }
    }

    // 校验码计算 SendBuf[2*MessageLenth + 4]
    SendBuf[2*MessageLenth + 4] = CalCrc(SendBuf, 2*MessageLenth + 4); //校验MessageLenth消息+2消息头+1消息长度+1消息码=MessageLenth + 4

    // 消息尾 SendBuf[MessageLenth + 5] SendBuf[MessageLenth + 6]
    SendBuf[2*MessageLenth + 5] = END[0];
    SendBuf[2*MessageLenth + 6] = END[1];

    SendUnion.data = 0; //消息Union清零

    // DEBUG BEGIN
    for(int i = 0; i < SendLength; i++)
    {
        std::cout << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)SendBuf[i] << " ";
    }
    std::cout << std::endl;
    std::cout << std::dec << (unsigned int)SendLength << std::endl;
    // DEBUG END

    //BUG FIND!!!
    boost::asio::write(*serial_port_.get(), boost::asio::buffer(test_data, 3), error_code_);

    // 串口发送消息
    boost::asio::write(*serial_port_.get(), boost::asio::buffer(SendBuf, (unsigned int)SendLength), error_code_);

    
    
}

// 串口接收函数, 成功返回true
bool Serial::SerialRead(double * ReceiveData, unsigned char * ReceiveCode)
{
    // 接收数据缓冲区
    unsigned char ReceiveBuf[255] = {0};

    // 接收消息长度
    unsigned char ReceiveLength;

    // 校验码
    unsigned char CrcNum;

    try
    {
        boost::asio::streambuf response;
        boost::asio::read_until(*serial_port_.get(), response, "\r\n", error_code_);
        std::copy(
            std::istream_iterator<unsigned char>(std::istream(&response) >> std::noskipws), 
            std::istream_iterator<unsigned char>(), 
            ReceiveBuf);
    }
    catch(boost::system::system_error &error_code_)
    {
        // RCLCPP_INFO(get_logger(), "RECEIVE ERROR: read_until error");
    }

    // 检查消息头
    for(int i = 0; i < 2; i++)
    {
        if (ReceiveBuf[i] != HEAD[i])
        {
            // RCLCPP_INFO(this->get_logger(), "RECEIVE ERROR: HEAD%d is : %d , but it should be %d .", i, (int)ReceiveBuf[i], (int)HEAD[i]);
            return false;
        }
    }

    // 消息长度
    ReceiveLength = ReceiveBuf[2];

    // 消息码传递
    *ReceiveCode = ReceiveBuf[3];

    // 消息内容
    for(int i = 0; i < ReceiveLength; i++)
    {
        // 缓冲区数据送入Union
        for(int j = 0; j < 2; j++)
        {
            ReceiveUnion.data_transmit[j] = ReceiveBuf[i * 2 + 4 + j];
        }
        ReceiveData[i] = ReceiveUnion.data; //读取数据
    }

    // 校验码
    CrcNum = CalCrc(ReceiveBuf, ReceiveLength + 4); //求得校验码
    if (CrcNum != ReceiveBuf[ReceiveLength + 4]) //与收到的校验码对比
    {
        // RCLCPP_INFO(this->get_logger(), "RECEIVE ERROR: CrcNum is : %d , but it should be %d ." , (int)ReceiveBuf[ReceiveLength + 4], (int)CrcNum);
        return false;
    }

    //消息Union清零
    ReceiveUnion.data_transmit[0] = 0;
    ReceiveUnion.data_transmit[1] = 0;

    return true;
}

// 8位循环冗余校验计算函数
unsigned char Serial::CalCrc(unsigned char * VectorData, unsigned short len)
{
    unsigned char crc;
    crc = 0;
    while(len--)
    {
        crc ^= (*VectorData++);
        for (int i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}
