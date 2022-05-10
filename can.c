

//Read Me
/* in file (Core-Src-main.c) ----------------------------------------------------------*/
//Peripheral Pinout and Configuration

ADC1_INO

USART2--Mode:Asynchronous-Configuration-Parameter Settings-Baud Rate:115200-Word Length:8 Bits

CAN1_TX-PA12
CAN1_RX-PA11
CAN1-NVIC Settings:RXO interrupt

GPIO_EXTI13-PC13
GPIO_Output-PA5

/* in file (Core-Src-main.c) ----------------------------------------------------------*/


//------------------------------------------------------------------------------------------
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//since we are using string functions, we will need to include the string and stdio libraries
#include <string.h>
#include <stdio.h> //output the timer value to the serial terminal

/* USER CODE END Includes */


//------------------------------------------------------------------------------------------
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint16_t TxData[8];
uint16_t RxData[8];

uint16_t readValue; //Task4
char msg[10]; //10 character buffer that will fill out transmit over UART
uint8_t buf[12];

char uart_buf[50];
int uart_buf_len;
uint16_t timer_val; //timer value used to hold our timer value and elapsed time calculations


uint32_t TxMailbox;

int datacheck = 0;

/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
		TxData[0] = 100;   // ms Delay
		TxData[1] = 40;    // loop rep

		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	}
}*/

/*void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if (RxHeader.DLC == 2)
	{
		datacheck = 1;
	}
}*/

/* USER CODE END 0 */

//------------------------------------------------------------------------------------------
/* USER CODE BEGIN 2 */

  // Start timer
  HAL_TIM_Base_Start(&htim10);

  //Get current time
  timer_val = __HAL_TIM_GET_COUNTER(&htim10);



  HAL_CAN_Start(&hcan1);

  // Activate the notification
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  TxHeader.DLC = 2;  // data length
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x446;  // ID

  /* USER CODE END 2 */

//------------------------------------------------------------------------------------------
/* USER CODE BEGIN WHILE */
  while (1)
  {
	//Start ADC conversion and read Potentiometer value
	HAL_ADC_Start(&hadc1); //Passing in the address of our ADC handle
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); //causes the processor to hang while it waits for an ADC conversion to complete
	readValue = HAL_ADC_GetValue(&hadc1); //once the ADC conversion is done, we get the value from the ADC channel register and store the raw value


	//Measure the Potentiometer value

	//if enough time has passed (0.0125 second), toggle LED and get new timestamp

		if (__HAL_TIM_GET_COUNTER(&htim10) - timer_val >= 125) //for the easy way to see, I set 1 second(10000) in here
		  {
			//Blink the LED on the first board
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			timer_val = __HAL_TIM_GET_COUNTER(&htim10);

			//Print the elapsed time
			uart_buf_len = sprintf(uart_buf, "Elapsed time: %u us\r\n", timer_val);
			HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, HAL_MAX_DELAY);

			//Convert to string and print the Potentiometer-value
			sprintf(msg,"Potentiometer: %hu. \r\n", readValue);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); //spit the message out over the uart2 port, which is connected to the ST link on our nuclio board

			//Transfer the Potentiometer data by CAN-Bus
			TxData[0] = readValue;   // ms Delay
			TxData[1] = 40;    // loop rep (I just test it, for transfer 2 data
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);


		  }

	/*HAL_ADC_Start(&hadc1); //start ADC conversion and passing in the address of our ADC handle
	HAL_ADC_PollForConversion(&hadc1,1000); //causes the processor to hang while it waits for an ADC conversion to complete
	readValue = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	HAL_Delay(1000);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //Test: Set GPIO pin high, timer
	// Get ADC value
	HAL_ADC_Start(&hadc1); //start ADC conversion and passing in the address of our ADC handle
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); //causes the processor to hang while it waits for an ADC conversion to complete
	readValue = HAL_ADC_GetValue(&hadc1); //once the ADC conversion is done, we get the value from the ADC channel register and store the raw value

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); //Test: Set GPIO pin low, timer

	//Convert to string and print
	sprintf(msg,"Potentiometer: %hu. \r\n", readValue);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); //spit the message out over the uart2 port, which is connected to the ST link on our nuclio board

	strcpy((char*)buf, "Hello!\r\n");
	HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);

	// Pretend we have sth. else to do for a while
	HAL_Delay(125);

	timer_val = __HAL_TIM_GET_COUNTER(&htim10);

	//Wait for 50 ms
	HAL_Delay(1000);
	*/
	/* -------------------------------------------------------- */

	//Say something --- the serial terminal to let us know that the program is working
	//uart_buf_len = sprintf(uart_buf, "Timer test begin\r\n");
	//HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	//Get time elapsed
	//timer_val = __HAL_TIM_GET_COUNTER(&htim10) - timer_val;

	//Show elapsed time
	//uart_buf_len = sprintf(uart_buf, "Elapsed time: %u us\r\n", timer_val);
	//HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

	//Wait again so we do not flood the Serial terminal;
	//HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/*if (datacheck)
	  {
		  // blink the LED
		  for (int i=0; i<RxData[1]; i++)
		  {
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			  HAL_Delay(RxData[0]);
		  }

		  datacheck = 0;

			TxData[0] = 100;   // ms Delay
			TxData[1] = 40;    // loop rep

			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
	  } */

  }
  /* USER CODE END 3 */
}


//------------------------------------------------------------------------------------------
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x103<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x103<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */