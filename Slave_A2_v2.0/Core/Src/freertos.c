/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
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
#include "motor.h"
#include "rotate.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t state_flag = 0;
float a, b;
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
osThreadId State_SwitchHandle;
osThreadId Tarangle_SwitchHandle;
osThreadId DataHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void task_state_switch(void const * argument);
void task_tarangle_switch(void const * argument);
void task_process_transmit(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
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
  /* definition and creation of State_Switch */
  osThreadDef(State_Switch, task_state_switch, osPriorityRealtime, 0, 128);
  State_SwitchHandle = osThreadCreate(osThread(State_Switch), NULL);

  /* definition and creation of Tarangle_Switch */
  osThreadDef(Tarangle_Switch, task_tarangle_switch, osPriorityHigh, 0, 128);
  Tarangle_SwitchHandle = osThreadCreate(osThread(Tarangle_Switch), NULL);

  /* definition and creation of Data */
  osThreadDef(Data, task_process_transmit, osPriorityHigh, 0, 128);
  DataHandle = osThreadCreate(osThread(Data), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_task_state_switch */
/**
 * @brief  Function implementing the State_Switch thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_task_state_switch */
void task_state_switch(void const * argument)
{
  /* USER CODE BEGIN task_state_switch */
    /* Infinite loop */
    for (;;) {
        state_flag = rx_state;
        cnt = 0;
		osDelay(4500);
    }
  /* USER CODE END task_state_switch */
}

/* USER CODE BEGIN Header_task_tarangle_switch */
/**
 * @brief Function implementing the Tarangle_Switch thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_task_tarangle_switch */
void task_tarangle_switch(void const * argument)
{
  /* USER CODE BEGIN task_tarangle_switch */
    /* 每隔50ms目标角度切换一次 */
    /* Infinite loop */
    for (;;) {
		taskENTER_CRITICAL();
		
        switch (state_flag) {
            case 1:
            case 3:
                tarangle_switch(&traj1[0]);
                break;
            case 2:
                tarangle_switch(&traj2[0]);
                break;
        }
		
		taskEXIT_CRITICAL();
        osDelay(50);
    }
  /* USER CODE END task_tarangle_switch */
}

/* USER CODE BEGIN Header_task_process_transmit */
/**
* @brief Function implementing the Data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_process_transmit */
void task_process_transmit(void const * argument)
{
  /* USER CODE BEGIN task_process_transmit */
  /* Infinite loop */
  for(;;)
  {
	 taskENTER_CRITICAL();

	 for (int i = 0; i < 9; i++) {
          data_process(&angle_data[i], &motor_2006[i]);
     }
	 output_transmit();
	 a = motor_2006[1].rx_data.continuous_angle;
	 b = motor_2006[1].tarangle;
	  
	 taskEXIT_CRITICAL();
	 osDelay(1);
  }
  /* USER CODE END task_process_transmit */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
