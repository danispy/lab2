/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define phy_tx_clock_Pin GPIO_PIN_13
#define phy_tx_clock_GPIO_Port GPIOC
#define phy_rx_clock_Pin GPIO_PIN_14
#define phy_rx_clock_GPIO_Port GPIOC
#define phy_tx_data_Pin GPIO_PIN_15
#define phy_tx_data_GPIO_Port GPIOC
#define dll_tx_0_Pin GPIO_PIN_0
#define dll_tx_0_GPIO_Port GPIOA
#define dll_tx_1_Pin GPIO_PIN_1
#define dll_tx_1_GPIO_Port GPIOA
#define dll_tx_2_Pin GPIO_PIN_2
#define dll_tx_2_GPIO_Port GPIOA
#define dll_tx_3_Pin GPIO_PIN_3
#define dll_tx_3_GPIO_Port GPIOA
#define dll_tx_4_Pin GPIO_PIN_4
#define dll_tx_4_GPIO_Port GPIOA
#define dll_tx_5_Pin GPIO_PIN_5
#define dll_tx_5_GPIO_Port GPIOA
#define dll_tx_6_Pin GPIO_PIN_6
#define dll_tx_6_GPIO_Port GPIOA
#define dll_tx_7_Pin GPIO_PIN_7
#define dll_tx_7_GPIO_Port GPIOA
#define phy_rx_data_Pin GPIO_PIN_0
#define phy_rx_data_GPIO_Port GPIOB
#define dll_rx_2_Pin GPIO_PIN_10
#define dll_rx_2_GPIO_Port GPIOB
#define dll_rx_3_Pin GPIO_PIN_11
#define dll_rx_3_GPIO_Port GPIOB
#define dll_rx_4_Pin GPIO_PIN_12
#define dll_rx_4_GPIO_Port GPIOB
#define dll_rx_5_Pin GPIO_PIN_13
#define dll_rx_5_GPIO_Port GPIOB
#define dll_rx_6_Pin GPIO_PIN_14
#define dll_rx_6_GPIO_Port GPIOB
#define dll_rx_7_Pin GPIO_PIN_15
#define dll_rx_7_GPIO_Port GPIOB
#define dll_to_phy_tx_bus_valid_Pin GPIO_PIN_3
#define dll_to_phy_tx_bus_valid_GPIO_Port GPIOB
#define interface_clock_Pin GPIO_PIN_4
#define interface_clock_GPIO_Port GPIOB
#define phy_alive_Pin GPIO_PIN_5
#define phy_alive_GPIO_Port GPIOB
#define phy_tx_busy_Pin GPIO_PIN_6
#define phy_tx_busy_GPIO_Port GPIOB
#define phy_to_dll_rx_bus_valid_Pin GPIO_PIN_7
#define phy_to_dll_rx_bus_valid_GPIO_Port GPIOB
#define dll_rx_0_Pin GPIO_PIN_8
#define dll_rx_0_GPIO_Port GPIOB
#define dll_rx_1_Pin GPIO_PIN_9
#define dll_rx_1_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

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
