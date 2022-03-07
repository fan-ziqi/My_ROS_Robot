
#include <iostream>
#include "SimpleSerial.h"

using namespace std;
using namespace boost;

int main(int argc, char* argv[])
{
	int count = 1;
	char* to_send;
	while(1)
	{
		try
		{
			SimpleSerial serial("/dev/pts/3",115200);

			sprintf(to_send, "send from test, count = %d\r\n", count);

			serial.writeString(to_send);

//			cout<<"Received : "<<serial.readLine()<<" : end"<<endl;

			this_thread::sleep_for(chrono::milliseconds(2000));

		}
		catch(boost::system::system_error& e)
		{
			cout<<"Error: "<<e.what()<<endl;
			return 1;
		}

		count++;
	}

}


