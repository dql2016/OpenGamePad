
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
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "multi_button.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
struct Button key1,key2,key3,key4,key5,key6,key7,key8,key9,key10,key11,key12,key13,key14,key15,key16;

uint16_t keyflag=0;//total 12 keys

#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80
#define BIT8 0x100
#define BIT9 0x200
#define BIT10 0x400
#define BIT11 0x800
#define BIT12 0x1000
#define BIT13 0x2000
#define BIT14 0x4000
#define BIT15 0x8000

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

typedef enum {
	TM_USB_HIDDEVICE_HATSWITCH_NULL_STATE = 128, /*!< null state */
	TM_USB_HIDDEVICE_HATSWITCH_UP = 0,   
	TM_USB_HIDDEVICE_HATSWITCH_RIGHT_UP = 1,  
	TM_USB_HIDDEVICE_HATSWITCH_RIGHT = 2,   
	TM_USB_HIDDEVICE_HATSWITCH_RIGHT_DOWN = 3,  
	TM_USB_HIDDEVICE_HATSWITCH_DOWN = 4,   
	TM_USB_HIDDEVICE_HATSWITCH_LEFT_DOWN = 5,  
	TM_USB_HIDDEVICE_HATSWITCH_LEFT = 6,  
	TM_USB_HIDDEVICE_HATSWITCH_LEFT_UP = 7,   
} TM_USB_HIDDEVICE_HATSWITCH_t;
typedef struct TM_USB_HIDDEVICE_Gamepad{
	
	uint8_t XAxis;                   /*!< Left joystick X axis */
	uint8_t YAxis;                   /*!< Left joystick Y axis */
	uint8_t RotateZAxis;             /*!< Right joystick RZ axis */
	uint8_t ZAxis;                   /*!< Right joystick Z axis */
	
	TM_USB_HIDDEVICE_HATSWITCH_t HatSwitch;              /*!< "angle" is 0,45,90,135,180,225,270,315(0~7,any value >7 will make the hatswitch in the middle) This parameter can be a value of @ref TM_USB_HIDDEVICE_HATSWITCH_t enumeration*/ 

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
 	
	TM_USB_HIDDEVICE_Button_t Button13;
	TM_USB_HIDDEVICE_Button_t Button14;
	TM_USB_HIDDEVICE_Button_t Button15;
	TM_USB_HIDDEVICE_Button_t Button16;
	
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
	buff[1] |= Gamepad_Data->Button9  	<< 0;	/* Bit 0 */
	buff[1] |= Gamepad_Data->Button10 	<< 1;	/* Bit 1 */
	buff[1] |= Gamepad_Data->Button11 	<< 2;	/* Bit 2 */
	buff[1] |= Gamepad_Data->Button12 	<< 3;	/* Bit 3 */

	
	/* Left joystick*/
	buff[2] = Gamepad_Data->XAxis;
	buff[3] = Gamepad_Data->YAxis;
	
	/* Right joystick*/
	buff[4] = Gamepad_Data->ZAxis;
	buff[5] = Gamepad_Data->RotateZAxis;
	
	buff[6] = Gamepad_Data->HatSwitch;
	
	buff[7] = 0;
	
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
| byte6  |       Hat switch                                                                      |
------------------------------------------------------------------------------------------------
| byte7  |                                           Reserved                                    |
------------------------------------------------------------------------------------------------
*/

#define FILTER_WINDOW_SIZE			10

uint16_t FilterWindow (uint32_t * p_buf, uint16_t new_val)
{
	uint16_t ret;
	
	p_buf[FILTER_WINDOW_SIZE-1] = new_val;
	
	for (uint8_t i=2; i<=FILTER_WINDOW_SIZE; i++)
	{
		p_buf[FILTER_WINDOW_SIZE-1] += p_buf[FILTER_WINDOW_SIZE - i];
	}
	
	p_buf[FILTER_WINDOW_SIZE-1] /= FILTER_WINDOW_SIZE;
	
	ret = p_buf[FILTER_WINDOW_SIZE-1];

	for (int i=0; i<FILTER_WINDOW_SIZE; i++)
	{
		p_buf[i-1] = p_buf[i];
	}
	
		return ret;
}
int flag=0;
uint16_t ADC_data[4];
uint8_t data_ready = 0;
uint32_t filter_buf[4][FILTER_WINDOW_SIZE];

uint8_t read_key1_GPIO() 
{
    return HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin);
}
uint8_t read_key2_GPIO() 
{
    return HAL_GPIO_ReadPin(key2_GPIO_Port, key2_Pin);
}
uint8_t read_key3_GPIO() 
{
    return HAL_GPIO_ReadPin(key3_GPIO_Port, key3_Pin);
}
uint8_t read_key4_GPIO() 
{
    return HAL_GPIO_ReadPin(key4_GPIO_Port, key4_Pin);
}
uint8_t read_key5_GPIO() 
{
    return HAL_GPIO_ReadPin(key5_GPIO_Port, key5_Pin);
}
uint8_t read_key6_GPIO() 
{
    return HAL_GPIO_ReadPin(key6_GPIO_Port, key6_Pin);
}
uint8_t read_key7_GPIO() 
{
    return HAL_GPIO_ReadPin(key7_GPIO_Port, key7_Pin);
}
uint8_t read_key8_GPIO() 
{
    return HAL_GPIO_ReadPin(key8_GPIO_Port, key8_Pin);
}
uint8_t read_key9_GPIO() 
{
    return HAL_GPIO_ReadPin(key9_GPIO_Port, key9_Pin);
}
uint8_t read_key10_GPIO() 
{
    return HAL_GPIO_ReadPin(key10_GPIO_Port, key10_Pin);
}
uint8_t read_key11_GPIO() 
{
    return HAL_GPIO_ReadPin(key11_GPIO_Port, key11_Pin);
}
uint8_t read_key12_GPIO() 
{
    return HAL_GPIO_ReadPin(key12_GPIO_Port, key12_Pin);
}
uint8_t read_key13_GPIO() 
{
    return HAL_GPIO_ReadPin(key13_GPIO_Port, key13_Pin);
}
uint8_t read_key14_GPIO() 
{
    return HAL_GPIO_ReadPin(key14_GPIO_Port, key14_Pin);
}
uint8_t read_key15_GPIO() 
{
    return HAL_GPIO_ReadPin(key15_GPIO_Port, key15_Pin);
}
uint8_t read_key16_GPIO() 
{
    return HAL_GPIO_ReadPin(key16_GPIO_Port, key16_Pin);
}

void key1_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT0;
	Gamepad_Data.Button1=TM_USB_HIDDEVICE_Button_Pressed;
}
void key1_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT0;
	Gamepad_Data.Button1=TM_USB_HIDDEVICE_Button_Released;
}
void key2_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT1;
	Gamepad_Data.Button2=TM_USB_HIDDEVICE_Button_Pressed;
}
void key2_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT1;
	Gamepad_Data.Button2=TM_USB_HIDDEVICE_Button_Released;
}
void key3_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT2;
	Gamepad_Data.Button3=TM_USB_HIDDEVICE_Button_Pressed;
}
void key3_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT2;
	Gamepad_Data.Button3=TM_USB_HIDDEVICE_Button_Released;
}
void key4_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT3;
	Gamepad_Data.Button4=TM_USB_HIDDEVICE_Button_Pressed;
}
void key4_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT3;
	Gamepad_Data.Button4=TM_USB_HIDDEVICE_Button_Released;
}
void key5_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT4;
	Gamepad_Data.Button5=TM_USB_HIDDEVICE_Button_Pressed;
}
void key5_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT4;
	Gamepad_Data.Button5=TM_USB_HIDDEVICE_Button_Released;
}
void key6_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT5;
	Gamepad_Data.Button6=TM_USB_HIDDEVICE_Button_Pressed;
}
void key6_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT5;
	Gamepad_Data.Button6=TM_USB_HIDDEVICE_Button_Released;
}
void key7_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT6;
	Gamepad_Data.Button7=TM_USB_HIDDEVICE_Button_Pressed;
}
void key7_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT6;
	Gamepad_Data.Button7=TM_USB_HIDDEVICE_Button_Released;
}
void key8_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT7;
	Gamepad_Data.Button8=TM_USB_HIDDEVICE_Button_Pressed;
}
void key8_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT7;
	Gamepad_Data.Button8=TM_USB_HIDDEVICE_Button_Released;
}
void key9_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT8;
	Gamepad_Data.Button9=TM_USB_HIDDEVICE_Button_Pressed;
}
void key9_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT8;
	Gamepad_Data.Button9=TM_USB_HIDDEVICE_Button_Released;
}
void key10_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT9;
	Gamepad_Data.Button10=TM_USB_HIDDEVICE_Button_Pressed;
}
void key10_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT9;
	Gamepad_Data.Button10=TM_USB_HIDDEVICE_Button_Released;
}
void key11_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT10;
	Gamepad_Data.Button11=TM_USB_HIDDEVICE_Button_Pressed;
}
void key11_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT10;
	Gamepad_Data.Button11=TM_USB_HIDDEVICE_Button_Released;
}
void key12_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT11;
	Gamepad_Data.Button12=TM_USB_HIDDEVICE_Button_Pressed;
}
void key12_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT11;
	Gamepad_Data.Button12=TM_USB_HIDDEVICE_Button_Released;
}


//hat
void key13_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT12;
	Gamepad_Data.HatSwitch=TM_USB_HIDDEVICE_HATSWITCH_LEFT;
}
void key13_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT12;
	Gamepad_Data.HatSwitch=TM_USB_HIDDEVICE_HATSWITCH_NULL_STATE;
}

void key14_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT13;
	Gamepad_Data.HatSwitch=TM_USB_HIDDEVICE_HATSWITCH_RIGHT;
}
void key14_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT13;
	Gamepad_Data.HatSwitch=TM_USB_HIDDEVICE_HATSWITCH_NULL_STATE;
}

void key15_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT14;
	Gamepad_Data.HatSwitch=TM_USB_HIDDEVICE_HATSWITCH_UP;
}
void key15_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT14;
	Gamepad_Data.HatSwitch=TM_USB_HIDDEVICE_HATSWITCH_NULL_STATE;
}

void key16_PRESS_DOWN_Handler(void* btn)
{
  keyflag|=BIT15;
	Gamepad_Data.HatSwitch=TM_USB_HIDDEVICE_HATSWITCH_DOWN;
}
void key16_PRESS_UP_Handler(void* btn)
{
  keyflag&=~BIT15;
	Gamepad_Data.HatSwitch=TM_USB_HIDDEVICE_HATSWITCH_NULL_STATE;
}
void UserKeys_Init(void)
{
 
  button_init(&key1, read_key1_GPIO, 0);//key is Low voltage valid
  button_attach(&key1, PRESS_DOWN,  key1_PRESS_DOWN_Handler);
	button_attach(&key1, PRESS_UP,    key1_PRESS_UP_Handler);
  button_start(&key1);
  
  button_init(&key2, read_key2_GPIO, 0);//key is Low voltage valid
  button_attach(&key2, PRESS_DOWN,  key2_PRESS_DOWN_Handler); 
	button_attach(&key2, PRESS_UP,    key2_PRESS_UP_Handler);
  button_start(&key2);
  
  button_init(&key3, read_key3_GPIO, 0);//key is Low voltage valid
  button_attach(&key3, PRESS_DOWN,  key3_PRESS_DOWN_Handler); 
	button_attach(&key3, PRESS_UP,    key3_PRESS_UP_Handler);
  button_start(&key3);
  
  button_init(&key4, read_key4_GPIO, 0);//key is Low voltage valid
  button_attach(&key4, PRESS_DOWN,  key4_PRESS_DOWN_Handler);
	button_attach(&key4, PRESS_UP,    key4_PRESS_UP_Handler);
  button_start(&key4);
  
  button_init(&key5, read_key5_GPIO, 0);//key is Low voltage valid
  button_attach(&key5, PRESS_DOWN,  key5_PRESS_DOWN_Handler);
	button_attach(&key5, PRESS_UP,    key5_PRESS_UP_Handler);
  button_start(&key5);
  
  button_init(&key6, read_key6_GPIO, 0);//key is Low voltage valid
  button_attach(&key6, PRESS_DOWN,  key6_PRESS_DOWN_Handler);
	button_attach(&key6, PRESS_UP,    key6_PRESS_UP_Handler);
  button_start(&key6);
  
  button_init(&key7, read_key7_GPIO, 0);//key is Low voltage valid
  button_attach(&key7, PRESS_DOWN,  key7_PRESS_DOWN_Handler);
	button_attach(&key7, PRESS_UP,    key7_PRESS_UP_Handler);
  button_start(&key7);
  
  button_init(&key8, read_key8_GPIO, 0);//key is Low voltage valid
  button_attach(&key8, PRESS_DOWN,  key8_PRESS_DOWN_Handler);
	button_attach(&key8, PRESS_UP,    key8_PRESS_UP_Handler);
  button_start(&key8);
	
	button_init(&key9, read_key9_GPIO, 0);//key is Low voltage valid
  button_attach(&key9, PRESS_DOWN,  key9_PRESS_DOWN_Handler);
	button_attach(&key9, PRESS_UP,    key9_PRESS_UP_Handler);
  button_start(&key9);

	button_init(&key10, read_key10_GPIO, 0);//key is Low voltage valid
  button_attach(&key10, PRESS_DOWN,  key10_PRESS_DOWN_Handler);
	button_attach(&key10, PRESS_UP,    key10_PRESS_UP_Handler);
  button_start(&key10);
	
	button_init(&key11, read_key11_GPIO, 0);//key is Low voltage valid
  button_attach(&key11, PRESS_DOWN,  key11_PRESS_DOWN_Handler);
	button_attach(&key11, PRESS_UP,    key11_PRESS_UP_Handler);
  button_start(&key11);
	
	button_init(&key12, read_key12_GPIO, 0);//key is Low voltage valid
  button_attach(&key12, PRESS_DOWN,  key12_PRESS_DOWN_Handler);
	button_attach(&key12, PRESS_UP,    key12_PRESS_UP_Handler);
  button_start(&key12);
	
	button_init(&key13, read_key13_GPIO, 0);//key is Low voltage valid
  button_attach(&key13, PRESS_DOWN,  key13_PRESS_DOWN_Handler);
	button_attach(&key13, PRESS_UP,    key13_PRESS_UP_Handler);
  button_start(&key13);
	
	button_init(&key14, read_key14_GPIO, 0);//key is Low voltage valid
  button_attach(&key14, PRESS_DOWN, key14_PRESS_DOWN_Handler);
	button_attach(&key14, PRESS_UP,    key14_PRESS_UP_Handler);
  button_start(&key14);
	
	button_init(&key15, read_key15_GPIO, 0);//key is Low voltage valid
  button_attach(&key15, PRESS_DOWN, key15_PRESS_DOWN_Handler);
	button_attach(&key15, PRESS_UP,    key15_PRESS_UP_Handler);
  button_start(&key15);
	
	button_init(&key16, read_key16_GPIO, 0);//key is Low voltage valid
  button_attach(&key16, PRESS_DOWN, key16_PRESS_DOWN_Handler);
	button_attach(&key16, PRESS_UP,    key16_PRESS_UP_Handler);
  button_start(&key16);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance ==TIM2)//tim2 5ms
  {
    button_ticks();//recommand 5ms for key scan
  }
}

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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
  if (HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_data[0],4*sizeof(uint16_t)) != HAL_OK)
	{	
		Error_Handler();
	}
	
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	
	UserKeys_Init();
	
	Gamepad_Data.HatSwitch=TM_USB_HIDDEVICE_HATSWITCH_NULL_STATE;
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		      				
//				for(int i=0;i<=255;i++)
//				{
//				//	Gamepad_Data.HatSwitch = i;
//					
//					Gamepad_Data.ZAxis += 1;
//										
//					Gamepad_Data.RotateZAxis += 1;
//					
//					//Gamepad_Data.XAxis+=1;
//					
//					Gamepad_Data.YAxis+=1;
//					
//					if(flag)
//					{
//						Gamepad_Data.Button1=TM_USB_HIDDEVICE_Button_Pressed;
//						Gamepad_Data.Button3=TM_USB_HIDDEVICE_Button_Pressed;
//						Gamepad_Data.Button6=TM_USB_HIDDEVICE_Button_Pressed;
//						Gamepad_Data.Button8=TM_USB_HIDDEVICE_Button_Pressed;
//						Gamepad_Data.Button10=TM_USB_HIDDEVICE_Button_Pressed;
//						Gamepad_Data.Button12=TM_USB_HIDDEVICE_Button_Pressed;
//					}
//					else
//					{
//						Gamepad_Data.Button1=TM_USB_HIDDEVICE_Button_Released;
//						Gamepad_Data.Button3=TM_USB_HIDDEVICE_Button_Released;
//						Gamepad_Data.Button6=TM_USB_HIDDEVICE_Button_Released;
//						Gamepad_Data.Button8=TM_USB_HIDDEVICE_Button_Released;
//						Gamepad_Data.Button10=TM_USB_HIDDEVICE_Button_Released;
//						Gamepad_Data.Button12=TM_USB_HIDDEVICE_Button_Released;
//					}
//					TM_USB_HIDDEVICE_GamepadSend(&Gamepad_Data);
//					LL_mDelay(10);
//					flag=!flag;
//					HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  	
//				}
		if (data_ready != 0)
		{

			Gamepad_Data.XAxis = 255*FilterWindow(filter_buf[0], ADC_data[0])/4095.0;
			Gamepad_Data.YAxis= 255*FilterWindow(filter_buf[1], ADC_data[1])/4095.0;
			Gamepad_Data.ZAxis= 255*FilterWindow(filter_buf[2], ADC_data[2])/4095.0;
			Gamepad_Data.RotateZAxis= 255*FilterWindow(filter_buf[3], ADC_data[3])/4095.0;

			TM_USB_HIDDEVICE_GamepadSend(&Gamepad_Data);

			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  
			
			data_ready = 0;		
		}
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

   if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
    Error_Handler();  
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);

  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);

  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(72000000);

  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);

  LL_SetSystemCoreClock(72000000);

  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);

  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL_DIV_1_5);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
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
