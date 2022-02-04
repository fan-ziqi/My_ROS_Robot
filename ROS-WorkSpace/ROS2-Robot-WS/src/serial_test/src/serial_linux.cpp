#include "serial_linux.hpp"

unsigned char SendLength = 3;
unsigned char * test_data = (unsigned char *)malloc(SendLength * sizeof(unsigned char));

// 串口发送/接收联合体, data_transmit大小由data数据类型的字节数决定
union SerialMessageUnion
{
	short data;
	unsigned char data_transmit[2];
}SendUnion, ReceiveUnion;

Serial::Serial(const std::string &port_name_):Serial_EC(0)
{
    //use normal pointer step2
    // serial_port_ = new boost::asio::serial_port(io_service_, port_name_);
    //use shared_ptr step2
    serial_port_ = std::make_shared<boost::asio::serial_port>(io_service_, port_name_);
    
    //or need to be opened later
    //use normal pointer step2
    // serial_port_ = new boost::asio::serial_port(io_service_);
    //use shared_ptr step2
    // serial_port_ = std::make_shared<boost::asio::serial_port>(io_service_);
    
    if(serial_port_)
    {
        std::cout << "New Serial_port Success" << std::endl;
        
        //if didn't fill port_name_, then now need to open it
        // serial_port_->open(port_name_, error_code_);
        // if(error_code_)
        // {
        //     Serial_EC = 2;
        //     std::cout << "[Error] serial_port_->open() failed. port_name=" << port_name_ << ", reason: " << error_code_.message().c_str() << std::endl;
        // }

        SerialInit();
    }
    else
    {
        Serial_EC = 1;
        std::cout << "New Serial_port Failed" << std::endl;
    }

    Serial_EC = 0;

    
}

Serial::~Serial()
{
    if(serial_port_)
    {
        std::cout << "Delete Serial_port" << std::endl;

        //use normal pointer need to delete
        // delete serial_port_;

        //use shared_ptr don't need to delete
    }
}

void Serial::write_to_serial(std::string data)
{
    boost::asio::write(*serial_port_, boost::asio::buffer(data), error_code_);//return lenth

    // DEBUG
    std::cout << "Write: " << data;
}

void Serial::read_from_serial()
{
    boost::asio::streambuf response;
    boost::asio::read_until(*serial_port_, response, "\r\n", error_code_);
    unsigned char ReceiveBuf[255] = {0};
    std::copy(std::istream_iterator<unsigned char>(std::istream(&response) >> std::noskipws), std::istream_iterator<unsigned char>(), ReceiveBuf);

    // DEBUG
    std::cout << "Read: " << ReceiveBuf;
}



// 消息格式: 
// 消息头1 消息头2 消息长度 消息码 [消息内容] 校验码 消息尾1 消息尾2

// 串口初始化
bool Serial::SerialInit()
{
    // set serial option
    serial_port_->set_option(boost::asio::serial_port::baud_rate(115200));
    serial_port_->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    serial_port_->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial_port_->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial_port_->set_option(boost::asio::serial_port::character_size(8));    

    // // DEBUG
    // for(int i=0;i<10;i++)
    // {
    //     write_to_serial("Hello World\r\n");
    //     // boost::asio::write(*serial_port_, boost::asio::buffer("Hello world", 12));
    // }

    SendUnion.data = 0;
    ReceiveUnion.data = 0;

    return true;
}

// 串口发送函数
bool Serial::SerialWrite(short * WriteData, unsigned char MessageLenth, unsigned int MessageCode)
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
    SendBuf[3] = (unsigned char)MessageCode;

    // 消息内容 SendBuf[4] ~ SendBuf[4 + 2*MessageLenth - 1]
    for(unsigned int i = 0; i < (unsigned int)MessageLenth; i++)
    {
        SendUnion.data = WriteData[i]; //更新消息Union内容
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

    // DEBUG BEGIN
    std::cout << "Write:(DEC -> HEX) " << std::endl;
    for(unsigned int i = 0; i < (unsigned int)MessageLenth; i++)
    {
        SendUnion.data = WriteData[i]; //更新消息Union内容
        std::cout << std::dec << SendUnion.data << " -> " 
                  << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)SendUnion.data_transmit[0] << " " 
                  << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)SendUnion.data_transmit[1] << "\t";
    }
    std::cout << std::endl;
    std::cout << "Write:(ALL HEX) " << std::endl;
    for(int i = 0; i < SendLength; i++)
    {
        std::cout << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)SendBuf[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "Write: SendLength: " << std::dec << (unsigned int)SendLength << std::endl;
    // DEBUG END

    // 串口发送消息
    boost::asio::write(*serial_port_, boost::asio::buffer(SendBuf, (unsigned int)SendLength), error_code_);

    //消息Union清零
    SendUnion.data = 0; 

    return true;
}

// 串口接收函数, 成功返回true
bool Serial::SerialRead(short * ReceiveData, unsigned int * ReceiveCode)
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
        boost::asio::read_until(*serial_port_, response, "\r\n", error_code_);
        std::copy(std::istream_iterator<unsigned char>(std::istream(&response) >> std::noskipws), std::istream_iterator<unsigned char>(), ReceiveBuf);
        // DEBUG
        std::cout << "Read:(ALL HEX) " << std::endl;
        for(int i = 0; i < ReceiveBuf[2]; i++)
        {
            std::cout << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)ReceiveBuf[i] << " ";
        }
        std::cout << std::endl;
    }
    catch(boost::system::system_error &error_code_)
    {
        std::cout << "RECEIVE ERROR: read_until error" << std::endl;
        return false;
    }

    // 检查消息头
    for(int i = 0; i < 2; i++)
    {
        if (ReceiveBuf[i] != HEAD[i])
        {
            std::cout << "RECEIVE ERROR: HEAD" << i <<" is: " << (int)ReceiveBuf[i] << ", but it should be: " << (int)HEAD[i] << std::endl;
            return false;
        }
    }

    // 消息长度
    ReceiveLength = ReceiveBuf[2];
    // debug ReceiveLength
    // std::cout << "ReceiveLength: " << std::dec << (unsigned int)ReceiveLength << std::endl;

    // 消息码传递
    *ReceiveCode = (unsigned int)ReceiveBuf[3];
    // debug ReceiveCode
    // std::cout << "ReceiveCode: " << std::setw(2) << std::hex << *ReceiveCode << std::endl;

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
    CrcNum = CalCrc(ReceiveBuf, ReceiveLength - 3); //求得校验码
    if (CrcNum != (unsigned int)ReceiveBuf[ReceiveLength - 3]) //与收到的校验码对比
    {
        std::cout << "RECEIVE ERROR: CrcNum is: " << (unsigned int)ReceiveBuf[ReceiveLength - 3] << " , but it should be: " << (unsigned int)CrcNum << std::endl;
        return false;
    }
    // debug CrcNum
    // std::cout << "CrcNum: " << std::setw(2) << std::hex << (unsigned int)ReceiveBuf[ReceiveLength - 3] << std::endl;

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
