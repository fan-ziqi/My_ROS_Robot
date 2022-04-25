#include "bsp_key.h"

#define LOG_TAG    "KEY"
#include "bsp_log.h"

//20ms
void key_task(void const * argument)
{
	while(1)
	{
		Key_Scan();
		
		osDelay(20);
	}
}
