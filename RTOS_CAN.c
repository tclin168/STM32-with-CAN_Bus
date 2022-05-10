//FREERTOS-Tasks and Queues-Task Name, Entry Function

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//since we are using string functions, we will need to include the string and stdio libraries
#include <string.h>
#include <stdio.h> //output the timer value to the serial terminal

#include "fonts.h"
#include "ssd1306.h"
#include "bitmap.h"

/* USER CODE END Includes */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint16_t TxData[8];
uint16_t RxData[8];

uint32_t TxMailbox;

int datacheck = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
		TxData[0] = 200;   // ms Delay
		TxData[1] = 10;    // loop rep

		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if (RxHeader.DLC == 2)
	{
		datacheck = 1;
	}
}


void send_deftask (void)
{
	uint8_t data[] = "Hello from def_task\r\n";
	HAL_UART_Transmit(&huart2, data, sizeof(data), HAL_MAX_DELAY);
}

void send_task2 (void)
{
	uint8_t data[] = "Hello from task2\r\n";
	HAL_UART_Transmit(&huart2, data, sizeof(data), HAL_MAX_DELAY);
}

void send_task3 (void)
{
	uint8_t data[] = "Hello from task3\r\n";
	HAL_UART_Transmit(&huart2, data, sizeof(data), HAL_MAX_DELAY);
}

/* USER CODE END 0 */


/* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Task2 */
  osThreadDef(Task2, StartTask02, osPriorityIdle, 0, 128);
  Task2Handle = osThreadCreate(osThread(Task2), NULL);

  /* definition and creation of Task3 */
  osThreadDef(Task3, StartTask03, osPriorityIdle, 0, 128);
  Task3Handle = osThreadCreate(osThread(Task3), NULL);




/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	send_deftask();
	osDelay(1000);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
		if (datacheck)
		{
			char snum[10];
			itoa(RxData[0], snum, 10);
			SSD1306_GotoXY (0, 30);
			SSD1306_Puts ("           ", &Font_16x26, 1);
			SSD1306_UpdateScreen();

			SSD1306_GotoXY (30, 30);  // 4 DIGIS
			SSD1306_Puts (snum, &Font_16x26, 1);
			SSD1306_UpdateScreen();
			HAL_Delay (500); //12.5 millisecond
		}

	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	send_task2();
	osDelay(1000);

  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Task3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	send_task3();
	osDelay(1000);
  }
  /* USER CODE END StartTask03 */
}