#ifndef BSP_CMD_H
#define BSP_CMD_H
#include "bsp_define.h"

void reboot(void);
void help(void);
void cmd(void);
void debug_on(void);
void debug_off(void);
void show_imu(void);
void show_battery(void);
void show_battery_once(void);
void show_send_to_pi(void);
#endif
