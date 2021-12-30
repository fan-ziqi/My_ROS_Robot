#include "robot_bringup/mbot_linux_serial.h"

using namespace std;
using namespace boost::asio;
//串口相关对象
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyUSB1");
boost::system::error_code err;
/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};

//发送左右轮速控制速度共用体,传感器的X，Z，Angle
union sendData
{
	short d;
	unsigned char data[2];
}leftVelSet,rightVelSet;

//接收数据（左轮速、右轮速、角度）共用体（-32767 - +32768）
union receiveData
{
	short d;
	unsigned char data[2];
}leftVelNow,rightVelNow,angleNow;

const double ROBOT_LENGTH = 157.00;  //mm
const double ROBOT_RADIUS = 78.50;  //两轮之间的半径长度mm

/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}

/********************************************************
函数功能：将机器人的线速度和角速度分解成左右轮子速度，打包发送给下位机
入口参数：机器人线速度、角速度
出口参数：
********************************************************/
void writeSpeed(double RobotV, double YawRate,unsigned char ctrlFlag)
{
    unsigned char buf[11] = {0};//
    int i, length = 0;
    double r = RobotV / YawRate;//mm

    // 计算左右轮期望速度
    if(RobotV == 0)      //旋转
    {
        leftVelSet.d  = (short)(-YawRate * ROBOT_RADIUS);//mm/s
        rightVelSet.d = (short)(YawRate * ROBOT_RADIUS);//mm/s
    } 
    else if(YawRate == 0)//直线
    {
        leftVelSet.d  = (short)RobotV;//mm/s
        rightVelSet.d = (short)RobotV;
    }
    else                //速度不一致
    {
        leftVelSet.d  = (short)(YawRate * (r - ROBOT_RADIUS));//mm/s
        rightVelSet.d = (short)(YawRate * (r + ROBOT_RADIUS));
    }

    // 设置消息头
    for(i = 0; i < 2; i++)
        buf[i] = header[i];             //buf[0]  buf[1]
    
    // 设置机器人左右轮速度
    length = 5;
    buf[2] = length;                    //buf[2]
    for(i = 0; i < 2; i++)
    {
        buf[i + 3] = leftVelSet.data[i];  //buf[3] buf[4]
        buf[i + 5] = rightVelSet.data[i]; //buf[5] buf[6]
    }
    // 预留控制指令
    buf[3 + length - 1] = ctrlFlag;       //buf[7]

    // 设置校验值、消息尾
    buf[3 + length] = getCrc8(buf, 3 + length);//buf[8]
    buf[3 + length + 1] = ender[0];     //buf[9]
    buf[3 + length + 2] = ender[1];     //buf[10]

    // 通过串口下发数据
    boost::asio::write(sp, boost::asio::buffer(buf));
}
/********************************************************
函数功能：从下位机读取数据，解析出线速度、角速度、角度
入口参数：机器人线速度、角速度、角度，引用参数
出口参数：bool
********************************************************/
bool readSpeed(double &vx,double &vth,double &th,unsigned char &ctrlFlag)
{
    char i, length = 0;
    unsigned char checkSum;
    unsigned char buf[150]={0};
    //=========================================================
    //此段代码可以读数据的结尾，进而来进行读取数据的头部
    try
    {
        boost::asio::streambuf response;
        boost::asio::read_until(sp, response, "\r\n",err);   
        copy(istream_iterator<unsigned char>(istream(&response)>>noskipws),
        istream_iterator<unsigned char>(),
        buf); 
    }  
    catch(boost::system::system_error &err)
    {
        ROS_INFO("read_until error");
    } 
    //=========================================================        

    // 检查信息头
    if (buf[0]!= header[0] || buf[1] != header[1])   //buf[0] buf[1]
    {
        ROS_ERROR("Received message header error!");
        return false;
    }
    // 数据长度
    length = buf[2];                                 //buf[2]

    // 检查信息校验值
    checkSum = getCrc8(buf, 3 + length);             //buf[10] 计算得出
    if (checkSum != buf[3 + length])                 //buf[10] 串口接收
    {
        ROS_ERROR("Received data check sum error!");
        return false;
    }    

    // 读取速度值
    for(i = 0; i < 2; i++)
    {
        leftVelNow.data[i]  = buf[i + 3]; //buf[3] buf[4]
        rightVelNow.data[i] = buf[i + 5]; //buf[5] buf[6]
        angleNow.data[i]    = buf[i + 7]; //buf[7] buf[8]
    }

    // 读取控制标志位
    ctrlFlag = buf[9];

    // 打印数据信息
/*    
    ROS_INFO("Left_v:%d\n",leftVelNow.d);
    ROS_INFO("Right_v:%d\n",rightVelNow.d);
    ROS_INFO("Angle:%d\n",angleNow.d);
    ROS_INFO("crtlFlag:%d\n",ctrlFlag);
*/  
    //===========================速度计算和Angle获取===========================================================
    // x方向速度，以及角速度
    vx  = (rightVelNow.d + leftVelNow.d) / 2.0 / 1000.0;        //m/s
    vth = (rightVelNow.d - leftVelNow.d) / ROBOT_LENGTH ;       //rad/s
    th  = angleNow.d*0.01745;//实时角度信息(rad)

    return true;
}
/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
