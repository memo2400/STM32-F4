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

TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

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
//char serial_test [60] = {"h","o","l","a"} //,0xff, 0xfa, 0x98, 0xff};


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
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
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
  MX_TIM13_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan2);  // segun yo 	 agregue
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // esta IT llama al callback
  HAL_UART_Receive_IT(&huart1, UsartRxData, sizeof(UsartRxData));

  //timmer
  HAL_TIM_Base_Start_IT(&htim13);
  HAL_TIM_Base_Start_IT(&htim14); // iniciamos el timer



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

	HAL_GPIO_WritePin(LED_A4_GPIO_Port, LED_A4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_A5_GPIO_Port, LED_A5_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_A6_GPIO_Port, LED_A6_Pin, GPIO_PIN_SET);

	uint8_t contador_display = 0;
	uint8_t display[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

	void DisplayPort(uint8_t patron){

		    HAL_GPIO_WritePin(GPIOD, A_Pin, (((patron & 0x01) >> 0) & 0x01));
		    HAL_GPIO_WritePin(GPIOD, B_Pin, (((patron & 0x02) >> 1) & 0x01));
		    HAL_GPIO_WritePin(GPIOD, C_Pin, (((patron & 0x04) >> 2) & 0x01));
		    HAL_GPIO_WritePin(GPIOD, D_Pin, (((patron & 0x08) >> 3) & 0x01));
		    HAL_GPIO_WritePin(GPIOD, E_Pin, (((patron & 0x10) >> 4) & 0x01));
		    HAL_GPIO_WritePin(GPIOD, F_Pin, (((patron & 0x20) >> 5) & 0x01));
		    HAL_GPIO_WritePin(GPIOD, G_Pin, (((patron & 0x40) >> 6) & 0x01));
	}

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
	  //HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

	  // Diplay - Funcional
	  // DisplayPort(display[contador_display]);
	  contador_display ++;
	  if (contador_display > 9){
		  contador_display = 0;
	  }

	  //Boton S2
	  if (HAL_GPIO_ReadPin(Boton_S2_GPIO_Port, Boton_S2_Pin))
	  {
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		  //HAL_GPIO_TogglePin(LED_A5_GPIO_Port, LED_A5_Pin);
		  //HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, RESET);
		  //HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, SET);
	  }


	  //USART
	  //HAL_UART_Transmit(&huart1, (char*)contartoggle, sizeof(contartoggle), 300);
	  //HAL_UART_Transmit(&huart1, serial, sizeof(serial), 300);
	  HAL_UART_Transmit(&huart1, (uint8_t*)0x98, 16, HAL_MAX_DELAY);

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
	  		  //HAL_Delay(300);

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
	  //HAL_UART_Transmit (&huart1, (uint8_t*)BufferEnvio, strlen(BufferEnvio), HAL_MAX_DELAY);

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

	  //HAL_UART_Transmit (&huart1, (uint8_t*)BufferEnvio, strlen(BufferEnvio), HAL_MAX_DELAY);



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
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 8000-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 10000;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 8000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 5000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_A4_Pin|LED_A5_Pin|LED_A6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, A_Pin|B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, C_Pin|D_Pin|E_Pin|F_Pin
                          |G_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_A4_Pin LED_A5_Pin LED_A6_Pin */
  GPIO_InitStruct.Pin = LED_A4_Pin|LED_A5_Pin|LED_A6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Boton_S1_IT_Pin */
  GPIO_InitStruct.Pin = Boton_S1_IT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Boton_S1_IT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Boton_S2_Pin Boton_S3_Pin */
  GPIO_InitStruct.Pin = Boton_S2_Pin|Boton_S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : A_Pin B_Pin */
  GPIO_InitStruct.Pin = A_Pin|B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : C_Pin D_Pin E_Pin F_Pin
                           G_Pin */
  GPIO_InitStruct.Pin = C_Pin|D_Pin|E_Pin|F_Pin
                          |G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

}

// Tiempo de debounce en milisegundos
#define DEBOUNCE_TIME 50

volatile uint8_t FlagBotonPresionado = 0;
uint32_t lastInterruptTime = 0;
int contador_IT = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t currentTime = HAL_GetTick();

	//GPIOA->ODR=0B000000;

	// Verifica si tiempo transcurrido es mayor que tiempo de debounce
	if ((currentTime - lastInterruptTime) > DEBOUNCE_TIME) {


		if (GPIO_Pin == Boton_S1_IT_Pin)  // Asegurar que es el pin correcto
		{
			FlagBotonPresionado = 1;
			//HAL_GPIO_TogglePin(LED_A5_GPIO_Port, LED_A5_Pin);  // Ejemplo: Cambiar el estado de un LED
			if (contador_IT == 1)
			{
				HAL_GPIO_WritePin(LED_A4_GPIO_Port, LED_A4_Pin, GPIO_PIN_RESET);
			}
			if (contador_IT == 2)
			{
				HAL_GPIO_WritePin(LED_A5_GPIO_Port, LED_A5_Pin, GPIO_PIN_RESET);
			}
			if (contador_IT == 3)
			{
				HAL_GPIO_WritePin(LED_A5_GPIO_Port, LED_A5_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_A6_GPIO_Port, LED_A6_Pin, GPIO_PIN_RESET);
			}
			if (contador_IT == 4)
			{
				HAL_GPIO_WritePin(LED_A4_GPIO_Port, LED_A4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_A5_GPIO_Port, LED_A5_Pin, GPIO_PIN_RESET);
			}
			if (contador_IT == 5)
			{
				HAL_GPIO_WritePin(LED_A6_GPIO_Port, LED_A6_Pin, GPIO_PIN_SET);
			}
			if (contador_IT == 6)
			{
				HAL_GPIO_WritePin(LED_A5_GPIO_Port, LED_A5_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_A6_GPIO_Port, LED_A6_Pin, GPIO_PIN_RESET);
			}
			if (contador_IT >= 7)
			{
				HAL_GPIO_WritePin(LED_A4_GPIO_Port, LED_A4_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_A5_GPIO_Port, LED_A5_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LED_A6_GPIO_Port, LED_A6_Pin, GPIO_PIN_SET);
				contador_IT = 0;
			}
			contador_IT++;
		}
	}
	lastInterruptTime = currentTime;
}


/*
 * Interrupcion del timmer
 */
float banderatim = 0;
uint8_t contador_display_tim = 9;
uint8_t display[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM14) // aqui le indicamos que usamos el timmer 1 para no revolver timers
	{
		//HAL_GPIO_TogglePin(LedRojo_GPIO_Port, LedRojo_Pin);
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

		/*HAL_GPIO_TogglePin(A_GPIO_Port, A_Pin);
		HAL_GPIO_TogglePin(B_GPIO_Port, B_Pin);*/

		DisplayPort(display[contador_display_tim]);

		if (contador_display_tim == 0){
			  contador_display_tim = 10;
		}
		contador_display_tim --;

		if (banderatim == 0){
			//GPIOD->ODR=0B0000000100000001;
			/*HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, SET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, SET);

			banderatim = 1;*/
		}
		if (banderatim == 1){
			//GPIOD->ODR=0B1111111011111110;
/*
			HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, SET);
			HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, SET);
			banderatim = 0;*/
		}


		/*
			HAL_GPIO_WritePin(DISPLAY_PORT, SEGMENT_A, (patron & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(DISPLAY_PORT, SEGMENT_B, (patron & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(DISPLAY_PORT, SEGMENT_C, (patron & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(DISPLAY_PORT, SEGMENT_D, (patron & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(DISPLAY_PORT, SEGMENT_E, (patron & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(DISPLAY_PORT, SEGMENT_F, (patron & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(DISPLAY_PORT, SEGMENT_G, (patron & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		    */
	}
}

void DisplayPort(uint8_t patron){

	    HAL_GPIO_WritePin(GPIOD, A_Pin, (((patron & 0x01) >> 0) & 0x01));
	    HAL_GPIO_WritePin(GPIOD, B_Pin, (((patron & 0x02) >> 1) & 0x01));
	    HAL_GPIO_WritePin(GPIOD, C_Pin, (((patron & 0x04) >> 2) & 0x01));
	    HAL_GPIO_WritePin(GPIOD, D_Pin, (((patron & 0x08) >> 3) & 0x01));
	    HAL_GPIO_WritePin(GPIOD, E_Pin, (((patron & 0x10) >> 4) & 0x01));
	    HAL_GPIO_WritePin(GPIOD, F_Pin, (((patron & 0x20) >> 5) & 0x01));
	    HAL_GPIO_WritePin(GPIOD, G_Pin, (((patron & 0x40) >> 6) & 0x01));
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
