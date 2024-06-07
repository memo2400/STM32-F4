/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t serial [16] = "\n Hola Mundo!! \r";
uint8_t UsartRxData [4] ;
uint8_t InterrupcionUsart [21] = "\nInterrupcion Masiva\r";
uint8_t contartoggle = 0;
char BufferEnvio [60];


//Cosas CAN
CAN_RxHeaderTypeDef CanRxHeader;
CAN_TxHeaderTypeDef canTxHeader;

uint8_t canReadData [8];
uint8_t canRxData [8];
uint8_t canTxData [8];
uint32_t canListTxData [8] =
		{
				0x00001,
				0x00002,
				0xe5001
		};

uint32_t canTxMailbox;
uint8_t canenviado [18] = "\nSe envio Msg CAN\r";
uint8_t CanError [27] = "\nNo se pudo enviar mensaje\r";


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxHeader, canRxData);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		// Mando el mensaje can por RS332
		/*sprintf(BufferEnvio, " PGN Recibido: Datos: %lX %X %X %X %X %X %X %X %X\r",
					  CanRxHeader.ExtId,
					  canRxData[0],
					  canRxData[1],
					  canRxData[2],
					  canRxData[3],
					  canRxData[4],
					  canRxData[5],
					  canRxData[6],
					  canRxData[7]); */
	}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan2);  // segun yo 	 agregue
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // esta IT llama al callback
  HAL_UART_Receive_IT(&huart1, UsartRxData, sizeof(UsartRxData));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*
  canRxData [0] = 0x10;
  canRxData [1] = 0xb8;
  canRxData [2] = 0xa2;
  canRxData [3] = 0x01;

  canRxData [4] = 0xff;
  canRxData [5] = 0xff;
  canRxData [6] = 0xff;
  canRxData [7] = 0xff;
  */

  while (1)
  {
	  // trama de datos

	  canTxData [0] = 0x26;
	  canTxData [1] = 0x5C;
	  canTxData [2] = 0x03;
	  canTxData [3] = 0x00;
	  canTxData [4] = 0xff;
	  canTxData [5] = 0xff;
	  canTxData [6] = 0xff;
	  canTxData [7] = 0xff;

	  // Leds encendido apagado

	  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

	  //USART
	  //HAL_UART_Transmit(&huart1, (char*)contartoggle, sizeof(contartoggle), 300);
	  HAL_UART_Transmit(&huart1, serial, sizeof(serial), 300);

	  // convierto variable entero en texto // (char que recibe, "texto", Variable que convertimos en %d)

	  //sprintf(BufferEnvio, "\n Contar: %d\r", contartoggle);
	  //HAL_UART_Transmit (&huart1, (uint8_t*)BufferEnvio, strlen(BufferEnvio), HAL_MAX_DELAY); // sizeof(BufferLectura) = es si funcion no tambien

	  // envio de un byte

	  //sprintf(BufferEnvio, "\n El Byte 0 es: %X %X\r", canRxData[0],canRxData[1]); //%d convierte hex a Dec
	  //HAL_UART_Transmit (&huart1, (uint8_t*)BufferEnvio, strlen(BufferEnvio), HAL_MAX_DELAY); // sizeof(BufferLectura) = es si funcion no tambien

	  //Enviar CAN Si funciono.

	  HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTxData, &canTxMailbox);

	  // Revisar si el conteo mensaje se envio

	  uint8_t ContarIntentoCan = 0;
	  while (HAL_CAN_IsTxMessagePending(&hcan2, canTxMailbox)) // si ya no hay mensajes pendientes se ejecuta el while
	  	  {
	  	  	  HAL_UART_Transmit (&huart1, canenviado, sizeof(canenviado), 300);
	  		  HAL_Delay(300);

	  		  if (ContarIntentoCan == 3)
	  		  {
	  			  HAL_UART_Transmit (&huart1, CanError, sizeof(CanError), 300);
	  			  break;
	  		  }
	  		ContarIntentoCan++;
	  	  }


	  // Leer CAN, no ha funcionado

	  //HAL_CAN_GetRxMessage(&hcan1, canReadMailbox, &CANRxHeader, canReadData); // yo
	  //HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CANRxHeader, canReadData); // Video British lo leido se guarda en RxData

	  // Envio Lo que lei en can


	  /*  //mostrar solo data.
	  sprintf(BufferEnvio, "\n\n MSM CAN es: %X %X %X %X\r", canRxData[0],canRxData[1],canRxData[2],canRxData[3]); //%d convierte hex a Dec
	  HAL_UART_Transmit (&huart1, (uint8_t*)BufferEnvio, strlen(BufferEnvio), HAL_MAX_DELAY); // sizeof(BufferLectura) = es si funcion no tambien
	  */


	  sprintf(BufferEnvio, " El PGN Enviado: %lX ", canTxHeader.ExtId); //%d convierte hex a Dec
	  HAL_UART_Transmit (&huart1, (uint8_t*)BufferEnvio, strlen(BufferEnvio), HAL_MAX_DELAY);

	  sprintf(BufferEnvio, " PGN Recibido: Datos: %lX %X %X %X %X %X %X %X %X\r",
			  CanRxHeader.ExtId,
			  canRxData[0],
			  canRxData[1],
			  canRxData[2],
			  canRxData[3],
			  canRxData[4],
			  canRxData[5],
			  canRxData[6],
			  canRxData[7]);

	  	  	  canRxData[0] = 0;
	  			  canRxData[1] = 0;
	  			  canRxData[2] = 0;
	  			  canRxData[3] = 0;
	  			  canRxData[4] = 0;
	  			  canRxData[5] = 0;
	  			  canRxData[6] = 0;
	  			  canRxData[7] = 0;

	  HAL_UART_Transmit (&huart1, (uint8_t*)BufferEnvio, strlen(BufferEnvio), HAL_MAX_DELAY);


	  //Boton
	  if (HAL_GPIO_ReadPin(Boton_S1_GPIO_Port, Boton_S1_Pin))
	  {
		  //HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  }

	  HAL_Delay(500);
	  contartoggle ++;

	  // ######## Envio de varios can



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

    // ####### Trama de datos FULL HD
    canTxHeader.DLC = 0x08;
    //canTxHeader.ExtId = 0x18fee000; //temp aceite
    canTxHeader.ExtId = 0x18fec100; // Odometro HR.
    canTxHeader.IDE = CAN_ID_EXT;
    canTxHeader.RTR = CAN_RTR_DATA;
    //canTxHeader.StdId = 0x00; // lo comento
    canTxHeader.TransmitGlobalTime = DISABLE;

    // ####### Configutacion del filtro.
    CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 10;  //  ver si es de can2
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0; //ver si este es para CAN2
    canfilterconfig.FilterIdHigh =0;
    canfilterconfig.FilterIdLow =0x0000;
    canfilterconfig.FilterMaskIdHigh = 0;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.SlaveStartFilterBank =0; // ver si es de can2

    HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig); //asigno el filtro a la funcion.


  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Boton_S1_Pin Boton_S2_Pin Boton_S3_Pin */
  GPIO_InitStruct.Pin = Boton_S1_Pin|Boton_S2_Pin|Boton_S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
	HAL_UART_Transmit (&huart1, InterrupcionUsart, sizeof(InterrupcionUsart), HAL_MAX_DELAY);
	//HAL_UART_Receive_IT(&huart1, InterrupcionUsart, sizeof(InterrupcionUsart));
	HAL_UART_Receive_IT(&huart1, UsartRxData, sizeof(UsartRxData));
	//UsartRxData
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
