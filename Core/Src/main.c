/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{CONTINUOUS_MODE=0,ONDEMAND_MODE} mode_t;
typedef enum{READY=0,NOT_READY} status_t;
typedef enum{MSG_RX_BYTE1=0,MSG_RX_BYTE2,MSG_RX_BYTE3,MSG_RX_BYTE4,MSG_RX_BYTE_COMPLETE} bytereceived_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define	ADCWITHTIMER
//#define	LEDWITHPWM
#define LED_ON				GPIO_PIN_RESET						//Define el estado de encendido del led
#define LED_OFF				GPIO_PIN_SET						//Define el estado de apagado del led
#define LED_ON_CONTINUOUS	(uint16_t)100						//Tiempo en ms de encendido del led en modo continuo
#define LED_OFF_CONTINUOUS	(uint16_t)300						//Tiempo en ms de apagado del led en modo continuo
#define LED_ON_ONDEMAND		(uint16_t)500						//Tiempo en ms de encendido del led en modo demanda
#define LED_OFF_ONDEMAND	(uint16_t)400						//Tiempo en ms de apagado del led en modo demanda
#define ADC_BUFFER_SIZE 	ADC_CHANNELS * ADC_SAMPLES			//Tamaño del buffer del ADC
#define ADC_CHANNELS		2									//Cantidad de canales del ADC
#define ADC_SAMPLES			4									//Cantidad de muestreos por canal, hasta 16
#define ADC_TIME_TO_SAMPLE	50									//20Hz, tiempo entre muestras
#define	MSG_WELCOME			"Bienvenidos\r\n"					//Mensaje inicial emitido por la UART
#define MSG_TX_LENGTH		3 + 2 * ADC_CHANNELS				//Largo del mensaje a enviar
#define MSG_RX_LENGTH		4									//Largo del mensaje a recibir
#define MSG_HEADER_BYTE 	0x24								//$, encabezado del mensaje
#define	MSG_TAIL_BYTE1		0x0D								//\r, cola 1 del mensaje
#define	MSG_TAIL_BYTE2		0x0A								//\n, cola 2 del mensaje
#define CMD_CONTINUOUS		0x43								//C, comando para modo continuo
#define CMD_ONDEMAND		0x44								//D, comando para modo demanda
#define CMD_REQUEST			0x50								//P, comando para pedir un dato en modo demanda
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t 				led_time_on, led_time_off;				//Definen el tiempo de encendido y apagado en ms
uint32_t 				led_time;								//Lleva el tiempo tomado del Systickpara el led, va a ser comparado con led_time_on y led_time_off
status_t				led_on_ready;							//Tiene el estado del led, READY como encendido y NOT_READY como apagado
ADC_HandleTypeDef 		hadc1;
DMA_HandleTypeDef 		hdma_adc1;
volatile uint16_t   	adc_values_buffer[ADC_BUFFER_SIZE];		//Se almacenan los valores convertidos por el ADC
volatile uint16_t   	adc_values_complete[ADC_BUFFER_SIZE];	//Copia de seguridad de los valores convertidos del ADC
volatile uint16_t 		adc_time;								//Lleva el tiempo tomado del Systick para el ADC
volatile status_t		adc_read_ready,adc_data_ready;			//Banderas que informan cuando el ADC convirtió los datos y cuando operó con ellos
uint16_t 				adc_channels[ADC_CHANNELS];				//Información del ADC promediada para cada canal
uint8_t 				data_rx;								//Almacena el dato leído por la UART, 1 caracter
uint8_t 				data_tx[MSG_TX_LENGTH];					//Vector que almacena el mensaje de salida por la UART
status_t				data_requested;							//Bandera que informa si hubo pedido de información, habilitada en modo Continuo o por el ingreso de una P en modo demanda
volatile status_t 		uart_tx_ready, uart_rx_ready; 			//Banderas que informan el correcto fin de la Tx y la lectura correcta de la Rx
uint8_t					uart_command;							//Variable donde se almacena el comando recibido por la UART
volatile bytereceived_t	uart_rx_index;							//El estado muestra la posición que se espera por la UART
mode_t 					mode;									//Contiene el modo de funcionamiento
status_t				mode_change;							//Bandera que habilita el cambio de modo
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Initialize(void);											//Inicializa los dispositivos
void ChangeMode(void);											//Cambia el modo
void LedInit(void);												//Inicializa el led
void LedVerifyAndToggle(void);									//Verifica si es su turno y modifica el estado del led
void AdcInit(void);												//Inicializa el ADC
#ifndef ADCWITHTIMER
void AdcRead(void);												//Solicita la conversión de datos en el ADC
#else
void AdcTimerInit(void);										//Inicializa el timer
#endif
void AdcVerifyAndOperate(void);									//Si los datos del ADC ya fueron recibidos los promedia
void Uart_Init(void);											//Inicializa la Uart
void UartTransmit(void);										//Transmite el mensaje por la UART
void UartReceive(void);											//Recibe de a un caracter por la UART
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
#ifdef ADCWITHTIMER
  MX_TIM2_Init();
#endif
  Initialize();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  LedVerifyAndToggle();			//Verifica si es momento de encender o apagar el led
#ifndef ADCWITHTIMER
	  AdcRead();					//Dispara la conversión del ADC si corresponde
#endif
	  AdcVerifyAndOperate();		//Opera con los datos del ADC si corresponde
	  UartReceive();				//Dispara la recepción por UART
	  UartTransmit();				//Transmite por la UART si corresponde
	  ChangeMode();					//Cambia el modo de funcionamiento si corresponde
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void ChangeMode(void)
{
	if(mode_change == READY)
	{
		mode_change = NOT_READY;				//Limpia la bandera
		switch (mode)
		{
		case CONTINUOUS_MODE:
			led_time_on = LED_ON_CONTINUOUS;
			led_time_off = LED_OFF_CONTINUOUS;
			data_requested = READY;				//En modo Continuo siempre solicita el envio de dato
			break;
		case ONDEMAND_MODE:
			led_time_on = LED_ON_ONDEMAND;
			led_time_off = LED_OFF_ONDEMAND;
			data_requested = NOT_READY;			//Se mantiene no listo y espera el comando de pedido
			break;
		default:
			mode = CONTINUOUS_MODE;
			mode_change = READY;
			break;
		}
	}
}
void LedInit(void)
{
	led_time = HAL_GetTick();
	HAL_GPIO_WritePin(LEDGP_GPIO_Port, LEDGP_Pin, LED_ON);
	led_on_ready = READY;
}
void AdcInit(void)
{
	uint8_t i = 0;

	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)	//Calibración
	{
		Error_Handler();
	}
//	HAL_Delay(1);
	adc_time = HAL_GetTick();							//Toma el tiempo de inicio
	adc_read_ready = NOT_READY;
	adc_data_ready = NOT_READY;
	for (i = 0; i < ADC_BUFFER_SIZE; i++)				//Inicializa el buffer
	{
		adc_values_buffer[i] = 0;
	}
	for (i = 0; i < ADC_CHANNELS; i++)					//Inicializa el vector donde estarán los datos finales
		{
			adc_channels[i] = 0;
		}
	if (HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_values_buffer, ADC_BUFFER_SIZE) != HAL_OK) //Primera lectura del ADC
	{
		Error_Handler();
	}
}
void Uart_Init(void)
{
	uart_tx_ready = READY;								//Listo para transmitir
	uart_rx_ready = NOT_READY;							//No hay caracter ingresando en la UART
	uart_rx_index = MSG_RX_BYTE1;
	data_tx[0] = MSG_HEADER_BYTE;
	data_tx[MSG_TX_LENGTH-2] = MSG_TAIL_BYTE1;
	data_tx[MSG_TX_LENGTH-1] = MSG_TAIL_BYTE2;

	if (HAL_UART_Transmit(&huart1, (uint8_t*) MSG_WELCOME, strlen(MSG_WELCOME), 10) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UART_Receive_IT(&huart1, &data_rx, 1) != HAL_OK)
	{
		Error_Handler();
	}

}
#ifdef ADCWITHTIMER
void AdcTimerInit(void)
{
	HAL_TIM_Base_Start_IT(&htim2);				//Enciende el Timer
}
#endif
void Initialize(void)
{
	mode = CONTINUOUS_MODE;						//Inicia en modo Continuo
	mode_change = READY;
	ChangeMode();
	AdcInit();
	LedInit();
	Uart_Init();
#ifdef ADCWITHTIMER
	AdcTimerInit();
#endif
}
void LedVerifyAndToggle(void)
{
	switch(led_on_ready)
	{
	case READY:
		if(HAL_GetTick()-led_time >= led_time_on)		//Verifica si le corresponde apagar el led
		{
			led_time = HAL_GetTick();
			HAL_GPIO_WritePin(LEDGP_GPIO_Port, LEDGP_Pin, LED_OFF);
			led_on_ready = NOT_READY;
		}
		break;
	case NOT_READY:
		if(HAL_GetTick()-led_time >= led_time_off)		//Verifica si le corresponde encender el led
		{
			led_time = HAL_GetTick();
			HAL_GPIO_WritePin(LEDGP_GPIO_Port, LEDGP_Pin, LED_ON);
			led_on_ready = READY;
		}
		break;
	}
}
#ifndef ADCWITHTIMER
void AdcRead(void)
{
	if (HAL_GetTick() - adc_time >= ADC_TIME_TO_SAMPLE)	//Verifica si tiene que disparar la conversión del ADC
	{
		adc_time = HAL_GetTick();

		if (HAL_ADC_Start(&hadc1) != HAL_OK)
		{
			Error_Handler();
		}
	}
}
#endif
void AdcVerifyAndOperate(void)							//Si el ADC cargó el vector completo calcula el promedio para cada canal
{
	volatile uint8_t i,j;
	uint16_t adc_channels_aux[ADC_CHANNELS];

	if (adc_read_ready == READY)
	{
		adc_read_ready = NOT_READY;
		for (i = 0; i < ADC_CHANNELS; i++)
		{
			adc_channels_aux[i] = 0;
		}
		for (i = 0; i < ADC_SAMPLES; i++)
		{
			for (j = 0; j < ADC_CHANNELS; j++)
			{
				adc_channels_aux[j] += adc_values_complete[i * ADC_CHANNELS + j];
			}
		}
		for (i = 0; i < ADC_CHANNELS; i++)
		{
			adc_channels[i] = (uint16_t) (adc_channels_aux[i] / ADC_SAMPLES);
		}
		adc_data_ready = READY;
	}
}

void UartTransmit(void)
{
	if (uart_tx_ready == READY && adc_data_ready == READY && data_requested == READY)	//Si no está trasmitiendo la UART, la data del ADC está lista y hay pedido de dato
	{
		uart_tx_ready = NOT_READY;
		adc_data_ready = NOT_READY;
		if(mode == ONDEMAND_MODE)
		{
			data_requested = NOT_READY;
		}

		data_tx[1] = (uint8_t)(adc_channels[0]>>8);										//Se cargan los datos promediados del ADC en el vector de salida
		data_tx[2] = (uint8_t)adc_channels[0];
		data_tx[3] = (uint8_t)(adc_channels[1]>>8);
		data_tx[4] = (uint8_t)adc_channels[1];
		HAL_UART_Transmit_IT(&huart1, &(data_tx[0]), MSG_TX_LENGTH);					//Se envian
	}
}
void UartReceive(void)
{
	if (uart_rx_ready == READY)
	{
		uart_rx_ready = NOT_READY;
		if(data_rx == MSG_HEADER_BYTE)			//Si se recibe el caracter "$" se reinicia el conteo
		{
			uart_rx_index = MSG_RX_BYTE1;
			uart_command = 0;
		}
		switch(uart_rx_index)					//Se verifican los caracteres recibidos y se almacena el comando
		{
			case MSG_RX_BYTE1:
				if(data_rx == MSG_HEADER_BYTE)
				{
					uart_rx_index = MSG_RX_BYTE2;
				}
				break;
			case MSG_RX_BYTE2:
				if(data_rx == CMD_CONTINUOUS || data_rx == CMD_ONDEMAND || data_rx == CMD_REQUEST)
				{
					uart_command = data_rx;
					uart_rx_index = MSG_RX_BYTE3;
				}
				else
				{
					uart_rx_index = MSG_RX_BYTE1;
					uart_command = 0;
				}
				break;
			case MSG_RX_BYTE3:
				if(data_rx == MSG_TAIL_BYTE1)
				{
					uart_rx_index = MSG_RX_BYTE4;
				}
				else
				{
					uart_rx_index = MSG_RX_BYTE1;
					uart_command = 0;
				}
				break;
			case MSG_RX_BYTE4:
				if(data_rx == MSG_TAIL_BYTE2)
				{
					uart_rx_index = MSG_RX_BYTE_COMPLETE;
				}
				else
				{
					uart_rx_index = MSG_RX_BYTE1;
					uart_command = 0;
				}
				break;
			default:
				uart_rx_index = MSG_RX_BYTE1;
				uart_command = 0;
				break;
		}
		if (uart_rx_index == MSG_RX_BYTE_COMPLETE)			//En caso de que el mensaje sea compatible se procede a utilizar la informacion recibida
		{
			switch(uart_command)
			{
			case CMD_CONTINUOUS:
				mode = CONTINUOUS_MODE;
				mode_change = READY;
				break;
			case CMD_ONDEMAND:
				mode = ONDEMAND_MODE;
				mode_change = READY;
				break;
			case CMD_REQUEST:
				data_requested = READY;
				break;
			default:
				break;
			}
			uart_rx_index = MSG_RX_BYTE1;
			uart_command = 0;
		}
		if (HAL_OK != HAL_UART_Receive_IT(&huart1, &data_rx, 1))		//Se solicita un nuevo caracter
		{
			Error_Handler();
		}
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
	volatile uint8_t i;

	for (i = 0; i < ADC_BUFFER_SIZE; i++)								//Almacenamos los datos convertidos
	{
		adc_values_complete[i] = adc_values_buffer[i];
	}
	adc_read_ready = READY;												//Informamos que los datos están listos para utilizar
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_tx_ready = READY;												//Informamos que finalizó la transmisión
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_rx_ready = READY;												//Informamos la recepción de un caracter
}
#ifdef ADCWITHTIMER
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	adc_time = HAL_GetTick();
	if (HAL_ADC_Start(&hadc1) != HAL_OK)								//Se dispara la conversión en el ADC
	{
		Error_Handler();
	}
}
#endif
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
