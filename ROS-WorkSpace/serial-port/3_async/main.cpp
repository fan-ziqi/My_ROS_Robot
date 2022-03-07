
#include <iostream>
#include <thread>

#include "BufferedAsyncSerial.h"

using namespace std;

int main(int argc, char* argv[])
{
	BufferedAsyncSerial serial("/dev/ttyUSB0",115200);
	while(1)
	{
		try {



			//Return immediately. String is written *after* the function returns,
			//in a separate thread.
//			serial.writeString("Hello world\r\n");

			//Simulate doing something else while the serial device replies.
			//When the serial device replies, the second thread stores the received
			//data in a buffer.
			this_thread::sleep_for(chrono::milliseconds(1000));

			//Always returns immediately. If the terminator \r\n has not yet
			//arrived, returns an empty string.
			cout<<serial.readStringUntil("\r\n")<<endl;

//			serial.close();

		}
		catch(boost::system::system_error& e)
		{
			cout<<"Error: "<<e.what()<<endl;
			return 1;
		}
	}

}
