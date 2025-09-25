/**
* Copyright (c) Mipot S.p.A. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       mip_b.c
* @date
* @version
*
*/

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "stm32_lpm.h"
#include "utilities_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "trinamic_spi.h"
#include "trinamic_gpio.h"
#include "trinamic_pulse_gen.h"
#include "trinamic_uart.h"
#include "trinamic_tim.h"
#include "trinamic_iic.h"
#include "trinamic_conf.h"
#include "trinamic_dac.h"
#include "trinamic_diagnosis.h"
#include "app_cli.h"
#include "ipcc.h"
#include "app_mipd.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RADIO_ON

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void SystemClock_Config(void);
static void SystemClock_GetInfo(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void vApplicationIdleHook(void)
{
	/* User adds here LOW POWER features */
	__WFI();
}

/*-----------------------------------------------------------*/

int main(void)
{
	/* Low Level clock configuration */
	(void)HAL_Init();
	(void)SystemClock_Config();
#ifdef RADIO_ON
	/* Set default state from potential system reset: Clear Stop2 flag of CPU1 */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOP2);
	/* Boot CPU2 */
	(void)HAL_PWREx_ReleaseCore(PWR_CORE_CPU2);
	/* Wait until cpu2 is ready to receive commands */
	while (*cpu2InitDone != CPU2_INITIALISED)
	{
		__asm("NOP");
	}
#endif
	/* Get Clock Info */
	(void)LPUART_Init();
	(void)SystemClock_GetInfo();
	/* Peripheral and GPIO Init */
	(void)TIM16_Init();
	(void)TIM2_Init();
	(void)I2C1_Init();
	(void)DAC_Init();
	(void)TrinamicGPIO_Init();
	(void)SPI2_Init();
	(void)TrinamicInit();
	/* Starting Tasks */
	(void)vTaskCli();
#ifdef RADIO_ON
	(void)MipdTask();
#endif
	/* Enable FreeRTOS */
	(void)vTaskStartScheduler();
	for(;;);
}

/*-----------------------------------------------------------*/

static void SystemClock_GetInfo(void)
{
    uint32_t hclk1 = HAL_RCC_GetHCLKFreq();
    uint32_t hclk3 = HAL_RCC_GetHCLK3Freq();
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
	uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    char buffer[100];
    (void)LPUART_TxPolling("\r\n\r\n********************\r\n");
    (void)LPUART_TxPolling("    MIP CM4 Init    \r\n");
    (void)LPUART_TxPolling("********************\r\n");
    sprintf(buffer, "Compiled on: %s %s\r\n", __DATE__, __TIME__);
    (void)LPUART_TxPolling(buffer);
    sprintf(buffer, "SYSCLK Frequency: %lu Hz\r\n", sysclk);
    (void)LPUART_TxPolling(buffer);
    sprintf(buffer, "HCLK1 Frequency: %lu Hz\r\n", hclk1);
    (void)LPUART_TxPolling(buffer);
    sprintf(buffer, "HCLK3 Frequency: %lu Hz\r\n", hclk3);
    (void)LPUART_TxPolling(buffer);
    sprintf(buffer, "PCLK1 Frequency: %lu Hz\r\n", pclk1);
    (void)LPUART_TxPolling(buffer);
    sprintf(buffer, "PCLK2 Frequency: %lu Hz\r\n", pclk2);
    (void)LPUART_TxPolling(buffer);
    (void)LPUART_TxPolling("********************\r\n");
}

/*-----------------------------------------------------------*/

static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/* Configure the main internal regulator output voltage */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* MSI is enabled after System reset, activate PLL with MSI as source */
	RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI |
									  RCC_OSCILLATORTYPE_HSE |RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_PWR;
	RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11; /*System Clock set at 48 MHz*/
	/*Activate PLL, HSE clock source, System Clock 48 MHz*/
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 12;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/**Initializes the CPU, AHB and APB busses clocks*/
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
								 RCC_CLOCKTYPE_HCLK2 | RCC_CLOCKTYPE_PCLK1 |
								 RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_HCLK3);
	RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}
