/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define Axis_1_Pin GPIO_PIN_0
#define Axis_1_GPIO_Port GPIOA
#define Axis_2_Pin GPIO_PIN_1
#define Axis_2_GPIO_Port GPIOA
#define Axis_3_Pin GPIO_PIN_2
#define Axis_3_GPIO_Port GPIOA
#define Axis_4_Pin GPIO_PIN_3
#define Axis_4_GPIO_Port GPIOA
#define Hat_DOWN_Pin GPIO_PIN_1
#define Hat_DOWN_GPIO_Port GPIOB
#define Hat_UP_Pin GPIO_PIN_11
#define Hat_UP_GPIO_Port GPIOB
#define Button1_Pin GPIO_PIN_12
#define Button1_GPIO_Port GPIOB
#define Button2_Pin GPIO_PIN_13
#define Button2_GPIO_Port GPIOB
#define Button3_Pin GPIO_PIN_14
#define Button3_GPIO_Port GPIOB
#define Button4_Pin GPIO_PIN_15
#define Button4_GPIO_Port GPIOB
#define Button5_Pin GPIO_PIN_8
#define Button5_GPIO_Port GPIOA
#define Button6_Pin GPIO_PIN_9
#define Button6_GPIO_Port GPIOA
#define Button7_Pin GPIO_PIN_10
#define Button7_GPIO_Port GPIOA
#define Button8_Pin GPIO_PIN_3
#define Button8_GPIO_Port GPIOB
#define Button9_Pin GPIO_PIN_4
#define Button9_GPIO_Port GPIOB
#define Button10_Pin GPIO_PIN_5
#define Button10_GPIO_Port GPIOB
#define Button11_Pin GPIO_PIN_6
#define Button11_GPIO_Port GPIOB
#define Button12_Pin GPIO_PIN_7
#define Button12_GPIO_Port GPIOB
#define Hat_RIGHT_Pin GPIO_PIN_8
#define Hat_RIGHT_GPIO_Port GPIOB
#define Hat_LEFT_Pin GPIO_PIN_9
#define Hat_LEFT_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

#define key1_GPIO_Port    Button1_GPIO_Port
#define key1_Pin          Button1_Pin

#define key2_GPIO_Port    Button2_GPIO_Port
#define key2_Pin          Button2_Pin

#define key3_GPIO_Port    Button3_GPIO_Port
#define key3_Pin          Button3_Pin

#define key4_GPIO_Port    Button4_GPIO_Port
#define key4_Pin          Button4_Pin

#define key5_GPIO_Port    Button5_GPIO_Port
#define key5_Pin          Button5_Pin

#define key6_GPIO_Port    Button6_GPIO_Port
#define key6_Pin          Button6_Pin

#define key7_GPIO_Port    Button7_GPIO_Port
#define key7_Pin          Button7_Pin

#define key8_GPIO_Port    Button8_GPIO_Port
#define key8_Pin          Button8_Pin

#define key9_GPIO_Port    Button9_GPIO_Port
#define key9_Pin          Button9_Pin

#define key10_GPIO_Port    Button10_GPIO_Port
#define key10_Pin          Button10_Pin

#define key11_GPIO_Port    Button11_GPIO_Port
#define key11_Pin          Button11_Pin

#define key12_GPIO_Port    Button12_GPIO_Port
#define key12_Pin          Button12_Pin




#define key13_GPIO_Port    Hat_LEFT_GPIO_Port
#define key13_Pin          Hat_LEFT_Pin

#define key14_GPIO_Port    Hat_RIGHT_GPIO_Port
#define key14_Pin          Hat_RIGHT_Pin

#define key15_GPIO_Port    Hat_UP_GPIO_Port
#define key15_Pin          Hat_UP_Pin

#define key16_GPIO_Port    Hat_DOWN_GPIO_Port
#define key16_Pin          Hat_DOWN_Pin


/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
