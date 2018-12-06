
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "usb_device.h"
#include "usb_host.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "usbh_def.h"
#include "stm32f4xx.h"

#include "usbh_core.h"
#include "usbh_hid.h"
#include "usbh_hub.h"

#include "usbh_hid_digital_io.h"
#include "usbh_hid_analog_io.h"
#include "log.h"

#include "ring_buffer.h"
#include "cmd_interpreter.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */
USBH_HandleTypeDef hUSBHost[5];

static void USBH_UserProcess (USBH_HandleTypeDef *pHost, uint8_t vId);
static void hub_process();
/* USER CODE BEGIN 0 */
uint8_t enabled = 0;
uint8_t trigger = 0;
extern uint8_t UserRxBuffer[100];
ringBuffer_type VCP_Buffer;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	//MX_USART3_UART_Init();
	//MX_USB_HOST_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */
	LOG_INIT(USART3, 115200);

	//LOG("\033[2J\033[H");
	//LOG(" ");
	LOG("APP RUNNING...");
	LOG("MCU-ID %08X", DBGMCU->IDCODE);

	memset(&hUSBHost[0], 0, sizeof(USBH_HandleTypeDef));

	hUSBHost[0].valid   = 1;
	hUSBHost[0].address = USBH_DEVICE_ADDRESS;
	hUSBHost[0].Pipes   = USBH_malloc(sizeof(uint32_t) * USBH_MAX_PIPES_NBR);

	USBH_Init(&hUSBHost[0], USBH_UserProcess, ID_USB_HOST_FS);
	USBH_RegisterClass(&hUSBHost[0], USBH_HID_CLASS);
	USBH_RegisterClass(&hUSBHost[0], USBH_HUB_CLASS);

	USBH_Start(&hUSBHost[0]);

	Ring_Buffer_Reset(&VCP_Buffer);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */
    //MX_USB_HOST_Process();
    hub_process();
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}



/* USER CODE BEGIN 4 */
void hub_process()
{
	static uint8_t current_loop = -1;
	static USBH_HandleTypeDef *_phost = 0;

	if(_phost != NULL && _phost->valid == 1)
	{
		USBH_Process(_phost);

		if(_phost->busy)
			return;
	}

	while(1)
	{
		uint8_t UserRxBuffer[100];
		uint8_t UserRxLength;
		current_loop++;
		volatile USBH_StatusTypeDef status;

		UserRxLength = Ring_Buffer_Search_Frame(&VCP_Buffer, UserRxBuffer);
		if (UserRxLength != 0)
		{
			if (UserRxBuffer[1] == 'D')
			{
				Command_Interpreter_Digital(UserRxBuffer);
			}
		}
		if(cmd_change_flag)
		{
			do
			{
				static uint8_t cnt = 0;
				change_buffer[0] = 6;
				//uint8_t test[7] = {6,0b11111111,0,0,0,0,0};
				status = USBH_HID_SetReport(&hUSBHost[0],cnt,0,change_buffer,7);
			}
			while(status != USBH_OK);
			CDC_Transmit_HS("CHANGE_OK\r\n", 12);
			HAL_Delay(1);
			cmd_change_flag = 0;
		}

		if(cmd_trigger_flag){
			do
			{
				static uint8_t cnt = 0;
				uint8_t test[2] = {1, 0xfe};
				status = USBH_HID_SetReport(&hUSBHost[0],cnt,0,test,2);
			}
			while(status != USBH_OK);
			CDC_Transmit_HS( "TRIGGER_OK\r\n", 12);
			HAL_Delay(1);
			cmd_trigger_flag = 0;
		}

		if(cmd_trigger_event_flag){
			do
			{
				static uint8_t cnt = 0;
				trig_event_buffer[0] = 5;
				status = USBH_HID_SetReport(&hUSBHost[0],cnt,0,trig_event_buffer,6);
			}
			while(status != USBH_OK);
			CDC_Transmit_HS( "TRIGGER_EVENT_OK\r\n", 19);
			HAL_Delay(1);
			cmd_trigger_event_flag = 0;
		}

		if(_phost != NULL && _phost->valid)
		{
			HID_DIGITAL_IO_Info_TypeDef *dio;
			dio = USBH_HID_Get_Digital_IO_Info(_phost);
			if(dio != NULL)
			{
				uint8_t dir1 = (dio->ports[0].direction) ? 'O' : 'I';
				uint8_t dir2 = (dio->ports[1].direction) ? 'O' : 'I';
				uint8_t dir3 = (dio->ports[2].direction) ? 'O' : 'I';
				uint8_t dir4 = (dio->ports[3].direction) ? 'O' : 'I';
				uint8_t dir5 = (dio->ports[4].direction) ? 'O' : 'I';
				uint8_t dir6 = (dio->ports[5].direction) ? 'O' : 'I';
				LOG("%c0 %d %d %d %d \r\n%c1 %d %d %d %d \r\n%c2 %d %d %d %d \r\n%c3 %d %d %d %d \r\n%c4 %d %d %d %d \r\n%c5 %d %d %d %d ",
						dir1,
						dio->ports[0].pins[0],
						dio->ports[0].pins[1],
						dio->ports[0].pins[2],
						dio->ports[0].pins[3],
						dir2,
						dio->ports[1].pins[0],
						dio->ports[1].pins[1],
						dio->ports[1].pins[2],
						dio->ports[1].pins[3],
						dir3,
						dio->ports[2].pins[0],
						dio->ports[2].pins[1],
						dio->ports[2].pins[2],
						dio->ports[2].pins[3],
						dir4,
						dio->ports[3].pins[0],
						dio->ports[3].pins[1],
						dio->ports[3].pins[2],
						dio->ports[3].pins[3],
						dir5,
						dio->ports[4].pins[0],
						dio->ports[4].pins[1],
						dio->ports[4].pins[2],
						dio->ports[4].pins[3],
						dir6,
						dio->ports[5].pins[0],
						dio->ports[5].pins[1],
						dio->ports[5].pins[2],
						dio->ports[5].pins[3]);
				HAL_Delay(1);
			}
			HID_ANALOG_IO_Info_TypeDef *aio;
			aio = USBH_HID_Get_Analog_IO_Info(_phost);
			if(aio != NULL)
			{
				LOG("ANALOG ADC0: %d ADC1: %d", aio->in[0], aio->in[1]);
				HAL_Delay(1);
			}
		}
		if(current_loop > MAX_HUB_PORTS)
			current_loop = 0;

		if(hUSBHost[current_loop].valid)
		{

			_phost = &hUSBHost[current_loop];
			USBH_LL_SetupEP0(_phost);

			if(_phost->valid == 3)
			{
LOG("PROCESSING ATTACH %d \r\n", _phost->address);
				_phost->valid = 1;
				_phost->busy  = 1;
			}

			break;
		}
	}



}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void USBH_UserProcess (USBH_HandleTypeDef *pHost, uint8_t vId)
{
	switch (vId)
	{
		case HOST_USER_SELECT_CONFIGURATION:
			break;

		case HOST_USER_CLASS_SELECTED:
			break;

		case HOST_USER_CLASS_ACTIVE:
			break;

		case HOST_USER_CONNECTION:
			break;

		case HOST_USER_DISCONNECTION:
			break;

		case HOST_USER_UNRECOVERED_ERROR:
			break;

		default:
			break;
	}
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
	if(GPIO_Pin == GPIO_PIN_13)
	{
		enabled = 1;
	}
	else
	{
		trigger = 1;
	}

  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
