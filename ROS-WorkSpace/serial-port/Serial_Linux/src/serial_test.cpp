#include "serial_linux.hpp"
using namespace std::chrono_literals;

std::string port = "/dev/ttyUSB0";

Serial my_serial(port);

short ReceiveTest[4] = {0};
unsigned int ReceiveCodeTest = 0x01; //这个地方赋初值0x00就不行，为什么？

short SendTest[4] = {123, 456, 789, 1120};
unsigned int SendCodeTest = 0x66;

void read_callback()
{
	while(1)
	{
		printf("read_thread\r\n");
	 	my_serial.SerialRead(ReceiveTest, &ReceiveCodeTest);
		printf("ReceiveTest[0] = %d, ReceiveTest[1] = %d, ReceiveTest[2] = %d, ReceiveTest[3] = %d, ReceiveCodeTest = %x\r\n", ReceiveTest[0], ReceiveTest[1], ReceiveTest[2], ReceiveTest[3], ReceiveCodeTest);
		ReceiveTest[0] = 0; ReceiveTest[1] = 0; ReceiveTest[2] = 0; ReceiveTest[3] = 0;
		ReceiveCodeTest = 0x01;
		std::cout << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
	//只读不写
//	std::thread th1(write_callback);
	std::thread th2(read_callback);

//	th1.join();
	th2.join();
    return 0;
}
 



