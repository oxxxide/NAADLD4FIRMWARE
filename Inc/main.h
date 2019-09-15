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
  * Copyright (c) 2019 STMicroelectronics International N.V. 
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

/* USER CODE BEGIN Includes */
#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define TRG1_Pin GPIO_PIN_2
#define TRG1_GPIO_Port GPIOE
#define TRG1_EXTI_IRQn EXTI2_IRQn
#define TRG2_Pin GPIO_PIN_3
#define TRG2_GPIO_Port GPIOE
#define TRG2_EXTI_IRQn EXTI3_IRQn
#define TRG3_Pin GPIO_PIN_4
#define TRG3_GPIO_Port GPIOE
#define TRG3_EXTI_IRQn EXTI4_IRQn
#define TRG4_Pin GPIO_PIN_5
#define TRG4_GPIO_Port GPIOE
#define TRG4_EXTI_IRQn EXTI9_5_IRQn
#define GPIO_EXTSW1_Pin GPIO_PIN_6
#define GPIO_EXTSW1_GPIO_Port GPIOE
#define GPIO_EXTSW2_Pin GPIO_PIN_13
#define GPIO_EXTSW2_GPIO_Port GPIOC
#define RoEncA1_Pin GPIO_PIN_2
#define RoEncA1_GPIO_Port GPIOC
#define RoEncA2_Pin GPIO_PIN_3
#define RoEncA2_GPIO_Port GPIOC
#define RoEncE1_Pin GPIO_PIN_2
#define RoEncE1_GPIO_Port GPIOA
#define RoEncE2_Pin GPIO_PIN_3
#define RoEncE2_GPIO_Port GPIOA
#define RoEncB1_Pin GPIO_PIN_4
#define RoEncB1_GPIO_Port GPIOC
#define RoEncB2_Pin GPIO_PIN_5
#define RoEncB2_GPIO_Port GPIOC
#define GPIO_INPUT_ADCSW1_Pin GPIO_PIN_2
#define GPIO_INPUT_ADCSW1_GPIO_Port GPIOB
#define RoEncC1_Pin GPIO_PIN_7
#define RoEncC1_GPIO_Port GPIOE
#define RoEncC2_Pin GPIO_PIN_8
#define RoEncC2_GPIO_Port GPIOE
#define RoEncD1_Pin GPIO_PIN_9
#define RoEncD1_GPIO_Port GPIOE
#define RoEncD2_Pin GPIO_PIN_10
#define RoEncD2_GPIO_Port GPIOE
#define RoEncS1_Pin GPIO_PIN_11
#define RoEncS1_GPIO_Port GPIOE
#define RoEncS2_Pin GPIO_PIN_12
#define RoEncS2_GPIO_Port GPIOE
#define GPIO_INPUT_ADCSW2_Pin GPIO_PIN_13
#define GPIO_INPUT_ADCSW2_GPIO_Port GPIOE
#define GPIO_INPUT_ADCSW3_Pin GPIO_PIN_14
#define GPIO_INPUT_ADCSW3_GPIO_Port GPIOE
#define GPIO_INPUT_ADCSW4_Pin GPIO_PIN_15
#define GPIO_INPUT_ADCSW4_GPIO_Port GPIOE
#define GPIO_IN_PAD1_Pin GPIO_PIN_12
#define GPIO_IN_PAD1_GPIO_Port GPIOB
#define GPIO_IN_PAD2_Pin GPIO_PIN_13
#define GPIO_IN_PAD2_GPIO_Port GPIOB
#define GPIO_IN_PAD3_Pin GPIO_PIN_14
#define GPIO_IN_PAD3_GPIO_Port GPIOB
#define GPIO_IN_PAD4_Pin GPIO_PIN_15
#define GPIO_IN_PAD4_GPIO_Port GPIOB
#define BTN9_Pin GPIO_PIN_10
#define BTN9_GPIO_Port GPIOD
#define BTN4_Pin GPIO_PIN_11
#define BTN4_GPIO_Port GPIOD
#define BTN5_Pin GPIO_PIN_12
#define BTN5_GPIO_Port GPIOD
#define BTN6_Pin GPIO_PIN_13
#define BTN6_GPIO_Port GPIOD
#define BTN7_Pin GPIO_PIN_14
#define BTN7_GPIO_Port GPIOD
#define BTN8_Pin GPIO_PIN_15
#define BTN8_GPIO_Port GPIOD
#define BTN1_Pin GPIO_PIN_6
#define BTN1_GPIO_Port GPIOC
#define BTN2_Pin GPIO_PIN_7
#define BTN2_GPIO_Port GPIOC
#define BLED1_Pin GPIO_PIN_0
#define BLED1_GPIO_Port GPIOD
#define BLED2_Pin GPIO_PIN_1
#define BLED2_GPIO_Port GPIOD
#define BLED3_Pin GPIO_PIN_2
#define BLED3_GPIO_Port GPIOD
#define BLED4_Pin GPIO_PIN_3
#define BLED4_GPIO_Port GPIOD
#define BLED5_Pin GPIO_PIN_4
#define BLED5_GPIO_Port GPIOD
#define BLED6_Pin GPIO_PIN_5
#define BLED6_GPIO_Port GPIOD
#define BLED7_Pin GPIO_PIN_6
#define BLED7_GPIO_Port GPIOD
#define BTN3_Pin GPIO_PIN_5
#define BTN3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define ARM_MATH_CM4
#define AUDIO_BLOCK_SIZE 64
#define ADC_SAMPLE_LENGTH 4
#define USE_EXTERNAL_GATE 1

//System Info
#define FIRMWARE_VERSION "FW Version 1.2  "
#define FIRMWARE_VERSION_CODE 5

#define I2C_LCD_ADDRESS 0x27 /*PCF8574T*/
//#define I2C_LCD_ADDRESS 0x3F /*PCF8574A*/

#define LIMIT(v,max,min)( (v)>(max) ? (max) : ( (v)<(min) ? (min):(v) ) )

//extern volatile LCD_STATE LcdMenuState;

extern uint16_t adcResultA;
extern uint16_t adcResultB;
extern uint16_t adcResultC;
extern uint16_t adcResultD;


void updateEncodersState(void);
void restoreFactorySet(void);
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
