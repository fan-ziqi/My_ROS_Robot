#ifndef BSP_VIN_H
#define BSP_VIN_H
#include "bsp_define.h"
#include "main.h"

enum
{
	IS_FULL = 1,
	NEED_CHARGE,
	NEED_POWEROFF
};

void VIN_Init(void);

void init_vrefint_reciprocal(void);
float get_temprate(void);
float get_battery_voltage(void);

uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch);
float adcx_get_chx_value_average(ADC_HandleTypeDef *ADCx, uint32_t ch, uint8_t times);


#endif
