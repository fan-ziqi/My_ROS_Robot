#include "main.h"
#include "bsp_define.h"
#include "bsp_cmd.h"

#define LOG_TAG    "CMD"
#include "bsp_log.h"

void cmd_task(void const * argument)
{
	while(1)
	{
		cmd();
		osDelay(10);
	}
}
