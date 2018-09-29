
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
#include "stm32l4xx_hal.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
typedef enum {
	TM_USB_HIDDEVICE_Button_Released = 0x00, /*!< Button is not pressed */
	TM_USB_HIDDEVICE_Button_Pressed = 0x01   /*!< Button is pressed */
} TM_USB_HIDDEVICE_Button_t;
typedef struct {
	TM_USB_HIDDEVICE_Button_t Button1;  /*!< Game pad button 1 status. This parameter can be a value of @ref TM_USB_HIDDEVICE_Button_t enumeration */
	TM_USB_HIDDEVICE_Button_t Button2;  /*!< Game pad button 2 status. This parameter can be a value of @ref TM_USB_HIDDEVICE_Button_t enumeration */
	TM_USB_HIDDEVICE_Button_t Button3;  /*!< Game pad button 3 status. This parameter can be a value of @ref TM_USB_HIDDEVICE_Button_t enumeration */
	TM_USB_HIDDEVICE_Button_t Button4;  /*!< Game pad button 4 status. This parameter can be a value of @ref TM_USB_HIDDEVICE_Button_t enumeration */
	TM_USB_HIDDEVICE_Button_t Button5;  /*!< Game pad button 5 status. This parameter can be a value of @ref TM_USB_HIDDEVICE_Button_t enumeration */
	TM_USB_HIDDEVICE_Button_t Button6;  /*!< Game pad button 6 status. This parameter can be a value of @ref TM_USB_HIDDEVICE_Button_t enumeration */
	TM_USB_HIDDEVICE_Button_t Button7;  /*!< Game pad button 7 status. This parameter can be a value of @ref TM_USB_HIDDEVICE_Button_t enumeration */
	TM_USB_HIDDEVICE_Button_t Button8;  /*!< Game pad button 8 status. This parameter can be a value of @ref TM_USB_HIDDEVICE_Button_t enumeration */
	TM_USB_HIDDEVICE_Button_t Button9;  /*!< Game pad button 9 status. This parameter can be a value of @ref TM_USB_HIDDEVICE_Button_t enumeration */
	TM_USB_HIDDEVICE_Button_t Button10; /*!< Game pad button 10 status. This parameter can be a value of @ref TM_USB_HIDDEVICE_Button_t enumeration */
	TM_USB_HIDDEVICE_Button_t Button11; /*!< Game pad button 11 status. This parameter can be a value of @ref TM_USB_HIDDEVICE_Button_t enumeration */
	TM_USB_HIDDEVICE_Button_t Button12; /*!< Game pad button 12 status. This parameter can be a value of @ref TM_USB_HIDDEVICE_Button_t enumeration */

	int8_t XAxis;                   /*!< Left joystick X axis */
	int8_t YAxis;                   /*!< Left joystick Y axis */
	int8_t RotateZAxis;             /*!< Right joystick X axis */
	int8_t ZAxis;                   /*!< Right joystick Y axis */
	
	uint8_t Hat0;              /*!< "angle" is 0,45,90,135,180,225,270,315,-1 */ 
	uint8_t Hat1; 
	uint8_t Hat2; 
	uint8_t Hat3; 
	
} TM_USB_HIDDEVICE_Gamepad_t;

int TM_USB_HIDDEVICE_GamepadSend(TM_USB_HIDDEVICE_Gamepad_t* Gamepad_Data) 
{
	uint8_t buff[8];
			
	/* Buttons pressed, byte 1 */
	buff[0] = 0;
	buff[0] |= Gamepad_Data->Button1 	<< 0;	/* Bit 0 */
	buff[0] |= Gamepad_Data->Button2 	<< 1;	/* Bit 1 */
	buff[0] |= Gamepad_Data->Button3 	<< 2;	/* Bit 2 */
	buff[0] |= Gamepad_Data->Button4 	<< 3;	/* Bit 3 */
	buff[0] |= Gamepad_Data->Button5 	<< 4;	/* Bit 4 */
	buff[0] |= Gamepad_Data->Button6 	<< 5;	/* Bit 5 */
	buff[0] |= Gamepad_Data->Button7 	<< 6;	/* Bit 6 */
	buff[0] |= Gamepad_Data->Button8 	<< 7;	/* Bit 7 */
	
	/* Buttons pressed, byte 2 */
	buff[1] = 0;
	buff[1] |= Gamepad_Data->Button9 	  << 0;	/* Bit 0 */
	buff[1] |= Gamepad_Data->Button10 	<< 1;	/* Bit 1 */
	buff[1] |= Gamepad_Data->Button11 	<< 2;	/* Bit 2 */
	buff[1] |= Gamepad_Data->Button12 	<< 3;	/* Bit 3 */

	
	/* Left joystick*/
	buff[2] = Gamepad_Data->XAxis;
	buff[3] = Gamepad_Data->YAxis;
	
	/* Right joystick*/
	buff[4] = Gamepad_Data->ZAxis;
	buff[5] = Gamepad_Data->RotateZAxis;
	
	buff[6] = (Gamepad_Data->Hat1)<<4 + Gamepad_Data->Hat0;
	buff[7] = (Gamepad_Data->Hat3)<<4 + Gamepad_Data->Hat2;
	/* Send to USB gamepad data */
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, buff, 8);
	
	/* Return connected */
	return 0;
}

TM_USB_HIDDEVICE_Gamepad_t Gamepad_Data;

/*
		     |  bit7		|   bit6	 |	  bit5	|	   bit4	 |	  bit3	|	   bit2	 |   bit1	  |   bit0   |
------------------------------------------------------------------------------------------------
| byte0  |  Button8 | Button7  | Button6  |  Button5 | Button4  |  Button3 |  Button2 |  Button1 |
------------------------------------------------------------------------------------------------
| byte1  |                   Reserved                | Button12 |  Butto11 | Butto10  |  Butto9  | 
------------------------------------------------------------------------------------------------
| byte2  |          XAxis                                                                        |
------------------------------------------------------------------------------------------------
| byte3  |         YAxis                                                                         |
------------------------------------------------------------------------------------------------
| byte4  |         ZAxis                                                                         |
------------------------------------------------------------------------------------------------
| byte5  |        RotateZAxis                                                                    |
------------------------------------------------------------------------------------------------
| byte6  |          |          |          |          |          |          |          |          |
------------------------------------------------------------------------------------------------
| byte7  |          |          |          |          |          |          |          |          |
------------------------------------------------------------------------------------------------
*/

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
  MX_LPUART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	
	printf("\n\r UART Printf Example: retarget the C library printf function to the UART\n\r");
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    //USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,  GamePad_Report, sizeof(GamePad_Report));

//		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_SET);
//		HAL_Delay(50);
//		HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
//		if(HAL_GPIO_ReadPin(UserButton_GPIO_Port, UserButton_Pin)==1)
//		{
//			HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
//			Gamepad_Data.Button1=TM_USB_HIDDEVICE_Button_Pressed;

//			Gamepad_Data.Button3=TM_USB_HIDDEVICE_Button_Pressed;


//			Gamepad_Data.Button6=TM_USB_HIDDEVICE_Button_Pressed;

//			Gamepad_Data.Button8=TM_USB_HIDDEVICE_Button_Pressed;
//			Gamepad_Data.Button10=TM_USB_HIDDEVICE_Button_Pressed;
//			Gamepad_Data.Button12=TM_USB_HIDDEVICE_Button_Pressed;

			Gamepad_Data.ZAxis += 10;
			Gamepad_Data.XAxis+=10;
			Gamepad_Data.Hat0+=2;
			Gamepad_Data.Hat1+=2;
//			Gamepad_Data.Hat2+=3;
//			Gamepad_Data.Hat3+=4;
//		}
//		else
//		{
//			Gamepad_Data.Button1=TM_USB_HIDDEVICE_Button_Released;
//			Gamepad_Data.Button2=TM_USB_HIDDEVICE_Button_Released;
//			Gamepad_Data.Button3=TM_USB_HIDDEVICE_Button_Released;
//			Gamepad_Data.Button4=TM_USB_HIDDEVICE_Button_Released;
//			Gamepad_Data.Button5=TM_USB_HIDDEVICE_Button_Released;
//			Gamepad_Data.Button6=TM_USB_HIDDEVICE_Button_Released;
//			Gamepad_Data.Button7=TM_USB_HIDDEVICE_Button_Released;
//			Gamepad_Data.Button8=TM_USB_HIDDEVICE_Button_Released;
//			Gamepad_Data.Button10=TM_USB_HIDDEVICE_Button_Released;
//			Gamepad_Data.Button12=TM_USB_HIDDEVICE_Button_Released;

//		}
	
		TM_USB_HIDDEVICE_GamepadSend(&Gamepad_Data); 
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_USB;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration 
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

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
