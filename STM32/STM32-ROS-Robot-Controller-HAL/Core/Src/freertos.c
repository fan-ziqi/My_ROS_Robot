/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId KEYHandle;
uint32_t KEYBuffer[ 128 ];
osStaticThreadDef_t KEYControlBlock;
osThreadId VINHandle;
uint32_t VINBuffer[ 128 ];
osStaticThreadDef_t VINControlBlock;
osThreadId UART_PIHandle;
uint32_t UART_PIBuffer[ 128 ];
osStaticThreadDef_t UART_PIControlBlock;
osThreadId UART_DEBUGHandle;
uint32_t UART_DEBUGBuffer[ 128 ];
osStaticThreadDef_t UART_DEBUGControlBlock;
osThreadId ROBOT_MOVEHandle;
uint32_t ROBOT_MOVEBuffer[ 128 ];
osStaticThreadDef_t ROBOT_MOVEControlBlock;
osThreadId IMUHandle;
osThreadId LEDHandle;
osThreadId OTHERHandle;
osThreadId CMDHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void key_task(void const * argument);
extern void vin_task(void const * argument);
extern void uart_pi_task(void const * argument);
extern void uart_debug_task(void const * argument);
extern void robot_move_task(void const * argument);
extern void imu_task(void const * argument);
extern void led_task(void const * argument);
extern void other_task(void const * argument);
extern void cmd_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of KEY */
  osThreadStaticDef(KEY, key_task, osPriorityBelowNormal, 0, 128, KEYBuffer, &KEYControlBlock);
  KEYHandle = osThreadCreate(osThread(KEY), NULL);

  /* definition and creation of VIN */
  osThreadStaticDef(VIN, vin_task, osPriorityBelowNormal, 0, 128, VINBuffer, &VINControlBlock);
  VINHandle = osThreadCreate(osThread(VIN), NULL);

  /* definition and creation of UART_PI */
  osThreadStaticDef(UART_PI, uart_pi_task, osPriorityHigh, 0, 128, UART_PIBuffer, &UART_PIControlBlock);
  UART_PIHandle = osThreadCreate(osThread(UART_PI), NULL);

  /* definition and creation of UART_DEBUG */
  osThreadStaticDef(UART_DEBUG, uart_debug_task, osPriorityNormal, 0, 128, UART_DEBUGBuffer, &UART_DEBUGControlBlock);
  UART_DEBUGHandle = osThreadCreate(osThread(UART_DEBUG), NULL);

  /* definition and creation of ROBOT_MOVE */
  osThreadStaticDef(ROBOT_MOVE, robot_move_task, osPriorityHigh, 0, 128, ROBOT_MOVEBuffer, &ROBOT_MOVEControlBlock);
  ROBOT_MOVEHandle = osThreadCreate(osThread(ROBOT_MOVE), NULL);

  /* definition and creation of IMU */
  osThreadDef(IMU, imu_task, osPriorityAboveNormal, 0, 128);
  IMUHandle = osThreadCreate(osThread(IMU), NULL);

  /* definition and creation of LED */
  osThreadDef(LED, led_task, osPriorityBelowNormal, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* definition and creation of OTHER */
  osThreadDef(OTHER, other_task, osPriorityNormal, 0, 128);
  OTHERHandle = osThreadCreate(osThread(OTHER), NULL);

  /* definition and creation of CMD */
  osThreadDef(CMD, cmd_task, osPriorityNormal, 0, 128);
  CMDHandle = osThreadCreate(osThread(CMD), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

