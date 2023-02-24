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
#include "stm32h7xx.h"
#include "stdbool.h"
#include "sdmmc.h"
#include "usbd_composite_if.h"
#include "tim.h"
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
/* Definitions for Empty_Task */
osThreadId_t Empty_TaskHandle;
const osThreadAttr_t Empty_Task_attributes = {
  .name = "Empty_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FatFs_Task */
osThreadId_t FatFs_TaskHandle;
const osThreadAttr_t FatFs_Task_attributes = {
  .name = "FatFs_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LED_Task */
osThreadId_t LED_TaskHandle;
const osThreadAttr_t LED_Task_attributes = {
  .name = "LED_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for KEY_Task */
osThreadId_t KEY_TaskHandle;
const osThreadAttr_t KEY_Task_attributes = {
  .name = "KEY_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for SDCrad_Task */
osThreadId_t SDCrad_TaskHandle;
const osThreadAttr_t SDCrad_Task_attributes = {
  .name = "SDCrad_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for USB_CDC_Reciver_Parameter */
osMessageQueueId_t USB_CDC_Reciver_ParameterHandle;
const osMessageQueueAttr_t USB_CDC_Reciver_Parameter_attributes = {
  .name = "USB_CDC_Reciver_Parameter"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask01(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the queue(s) */
  /* creation of USB_CDC_Reciver_Parameter */
  USB_CDC_Reciver_ParameterHandle = osMessageQueueNew (1, sizeof(uint16_t), &USB_CDC_Reciver_Parameter_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Empty_Task */
  Empty_TaskHandle = osThreadNew(StartTask01, NULL, &Empty_Task_attributes);

  /* creation of FatFs_Task */
  FatFs_TaskHandle = osThreadNew(StartTask02, NULL, &FatFs_Task_attributes);

  /* creation of LED_Task */
  LED_TaskHandle = osThreadNew(StartTask03, NULL, &LED_Task_attributes);

  /* creation of KEY_Task */
  KEY_TaskHandle = osThreadNew(StartTask04, NULL, &KEY_Task_attributes);

  /* creation of SDCrad_Task */
  SDCrad_TaskHandle = osThreadNew(StartTask05, NULL, &SDCrad_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Empty_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartTask01 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask01 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the FatFs_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the LED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    osDelay(500);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the KEY_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  bool Buttom = false;
  /* Infinite loop */
  for(;;)
  {
    if(!HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin))
    {
      osDelay(100);
      if(!HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin))
      {
        if(false == Buttom)
        {
          vTaskSuspend(LED_TaskHandle);
          Buttom = true;
        }
        else
        {
          vTaskResume(LED_TaskHandle);
          Buttom = false;
        }
      }
    }
    if(!HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin))
    {
      osDelay(100);
      if(!HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin))
      {
        
      }
    }
    osDelay(100);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the SDCrad_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  char c;
  float i;
//  HAL_SD_CardInfoTypeDef info;
//  if(HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER)
//    HAL_SD_GetCardInfo(&hsd1, &info);
  /* Infinite loop */
  for(;;)
  {
    usb_scanf("$GPGSV,3,1,10,04,55,226,28,07,21,318,42,08,80,217,18,09,43,285,43,0*6D%f", &i);
    usb_printf("$GPGSV,3,1,10,04,55,226,28,07,21,318,42,08,80,217,18,09,43,285,43,0*6D%f", i);
    // if((Recive_Finish == Recive_State) && (New_Package == Tag))
    // {
    //   CDC_Transmit_FS(UserRxBufferFS, Length);
    //   Recive_State = Recive_UnFinish;
    //   Length = 0U;
    // }
    osDelay(1);
  }
  /* USER CODE END StartTask05 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

