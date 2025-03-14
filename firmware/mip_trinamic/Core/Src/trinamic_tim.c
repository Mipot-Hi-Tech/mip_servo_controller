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
#include "trinamic_tim.h"
#include "trinamic_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
TIM_HandleTypeDef htim16;
volatile uint32_t tim16_cnt = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
void TIM16_Init(void)
{
	/* Enable clock @ TIM16 */
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	__NOP();
	__NOP();

	/* General purpose timer generating irq @ 1ms */
	htim16.Instance               = TIM16;
	htim16.Init.Prescaler         = 0;
	htim16.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim16.Init.Period            = 47999;
	htim16.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	(void)HAL_TIM_Base_Init(&htim16);
	(void)HAL_NVIC_EnableIRQ(TIM16_IRQn);
	(void)HAL_TIM_Base_Start_IT(&htim16);
}

/*-----------------------------------------------------------*/

uint32_t GetTick(void)
{
	return tim16_cnt;
}

/*-----------------------------------------------------------*/

void Delay_ms(uint32_t ms)
{
	uint32_t ticks =  GetTick();
	while( (GetTick() - ticks) < ms);
}

/*-----------------------------------------------------------*/

void TIM16_IRQHandler(void)
{
	__HAL_TIM_CLEAR_IT(&htim16 ,TIM_IT_UPDATE);
	 tim16_cnt++;
}
