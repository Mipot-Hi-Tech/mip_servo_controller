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
#include "trinamic_pulse_gen.h"
#include "trinamic_gpio.h"
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PULSE_GEN_GPIO_SPEED GPIO_SPEED_FREQ_HIGH
#define PULSE_GEN_PULL_TYPE  GPIO_PULLDOWN
//#define TIM2_GPIO_DEBUG
/* Stepper configuration */
#define TIM2_PERIOD_STEPPER 47999
/* Servo configuration */
#define TIM2_PERIOD_SERVO 799 /* 20 ms */
#define TIM2_SERVO_MIN_PULSE_CNT  40  /* 1 ms */
#define TIM2_SERVO_IDLE_PULSE_CNT 60  /* 1.5 ms */
#define TIM2_SERVO_MAX_PULSE_CNT  80  /* 2 ms */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void TIM2_Init_Stepper(void);
static void TIM2_Init_Servo(void);
static void TIM2_GPIO_Init(void);
static inline void HandleMot0StepPulseGeneration(void);
static inline void HandleMot1StepPulseGeneration(void);
static inline void HandleMot2StepPulseGeneration(void);
static inline void ServoTestMove(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile enum timer_mode_t timer_mode = TIMER_MODE_STEPPER;
volatile enum servo_pos_t servo_mode  = SERVO_MODE_DISABLED;
TIM_HandleTypeDef htim2;
volatile uint32_t tim2_cnt          = 0;
volatile uint32_t mot0_steps        = 0;
volatile uint32_t mot1_steps        = 0;
volatile uint32_t mot2_steps        = 0;
volatile uint32_t mot0_steps_shadow = 0;
volatile uint32_t mot1_steps_shadow = 0;
volatile uint32_t mot2_steps_shadow = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
void TIM2_Init(void)
{
	if (TIMER_MODE_SERVO == timer_mode)
	{
		(void)TIM2_Init_Servo();
	}
	else if (TIMER_MODE_STEPPER == timer_mode)
	{
		(void)TIM2_Init_Stepper();
	}
	else
	{
		;
	}
}

/*-----------------------------------------------------------*/

static void TIM2_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* Configure GPIO pin : DRV0_STEP_Pin -> TIM2 CHANNEL 3 */
    GPIO_InitStruct.Pin       = DRV0_STEP_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = PULSE_GEN_PULL_TYPE;
    GPIO_InitStruct.Speed     = PULSE_GEN_GPIO_SPEED;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    (void)HAL_GPIO_Init(DRV0_STEP_Port, &GPIO_InitStruct);

    /* Configure GPIO pin : DRV1_STEP_Pin -> TIM2 CHANNEL 4 */
    GPIO_InitStruct.Pin       = DRV1_STEP_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = PULSE_GEN_PULL_TYPE;
    GPIO_InitStruct.Speed     = PULSE_GEN_GPIO_SPEED;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    (void)HAL_GPIO_Init(DRV1_STEP_Port, &GPIO_InitStruct);

    /* Configure GPIO pin : DRV2_STEP_Pin -> TIM2 CHANNEL 1 */
    GPIO_InitStruct.Pin       = DRV2_STEP_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = PULSE_GEN_PULL_TYPE;
    GPIO_InitStruct.Speed     = PULSE_GEN_GPIO_SPEED;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    (void)HAL_GPIO_Init(DRV2_STEP_Port, &GPIO_InitStruct);
}

/*-----------------------------------------------------------*/

static void TIM2_Init_Servo(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_SlaveConfigTypeDef sSlaveConfig       = {0};
	TIM_MasterConfigTypeDef sMasterConfig     = {0};
	TIM_OC_InitTypeDef sConfigOC              = {0};

	/* Enable clock @ TIM2 */
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	__NOP();
	__NOP();

	/* TIM2 CONFIGURATION */
	htim2.Instance               = TIM2;
	htim2.Init.Prescaler         = 1199;
	htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim2.Init.Period            = TIM2_PERIOD_SERVO;
	htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV4;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	(void)HAL_TIM_Base_Init(&htim2);

	/* TIM2 clocked @ 48 MHz */
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	(void)HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
	(void)HAL_TIM_PWM_Init(&htim2);

	sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	(void)HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
	(void)HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

	sConfigOC.OCMode     = TIM_OCMODE_PWM1;
	sConfigOC.Pulse      = TIM2_SERVO_IDLE_PULSE_CNT;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	(void)HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
	(void)HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
	(void)HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

	(void)TIM2_GPIO_Init();
	(void)HAL_NVIC_EnableIRQ(TIM2_IRQn);
	(void)HAL_TIM_Base_Start_IT(&htim2);
	(void)HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_1);
	(void)HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_3);
	(void)HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_4);
	__HAL_TIM_ENABLE(&htim2);
	TIM2->CCER |= TIM_CCER_CC3E;
}

/*-----------------------------------------------------------*/

static void TIM2_Init_Stepper(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_SlaveConfigTypeDef sSlaveConfig       = {0};
	TIM_MasterConfigTypeDef sMasterConfig     = {0};
	TIM_OC_InitTypeDef sConfigOC              = {0};

	/* Enable clock @ TIM2 */
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	__NOP();
	__NOP();

	/* TIM2 CONFIGURATION */
	htim2.Instance               = TIM2;
	htim2.Init.Prescaler         = 0;
	htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim2.Init.Period            = TIM2_PERIOD_STEPPER;
	htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	(void)HAL_TIM_Base_Init(&htim2);

	/* TIM2 clocked @ 48 MHz */
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	(void)HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
	(void)HAL_TIM_PWM_Init(&htim2);

	sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	(void)HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
	(void)HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

	sConfigOC.OCMode     = TIM_OCMODE_PWM1;
	sConfigOC.Pulse      = TIM2_PERIOD_STEPPER-1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	(void)HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
	(void)HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
	(void)HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

	(void)TIM2_GPIO_Init();
	(void)HAL_NVIC_EnableIRQ(TIM2_IRQn);
	(void)HAL_TIM_Base_Start_IT(&htim2);
	(void)HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_1); /* Routed to DRV2_STEP GPIO */
	(void)HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_3); /* Routed to DRV0_STEP GPIO */
	(void)HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_4); /* Routed to DRV1_STEP GPIO */
	__HAL_TIM_ENABLE(&htim2);
}

/*-----------------------------------------------------------*/

void TIM2_IRQHandler(void)
{
#ifdef TIM2_GPIO_DEBUG
	(void)HAL_GPIO_TogglePin(GPIO0_NWAKE_Port, GPIO0_NWAKE_Pin);
#endif
	if (TIMER_MODE_STEPPER == timer_mode)
	{
		(void)HandleMot0StepPulseGeneration();
		(void)HandleMot1StepPulseGeneration();
		(void)HandleMot2StepPulseGeneration();
	}
	else if (TIMER_MODE_SERVO == timer_mode)
	{
		if (tim2_cnt % 100 == 0)
		{
			(void)ServoTestMove();
		}
	}
	else
	{
		;
	}
	tim2_cnt++;
	__HAL_TIM_CLEAR_IT(&htim2 ,TIM_IT_UPDATE);
}

/*-----------------------------------------------------------*/

static inline void HandleMot0StepPulseGeneration(void)
{
	if(mot0_steps != 0)
	{
		mot0_steps--;
		if(mot0_steps == 0)
		{
			/* Disable output on TIM_CHANNEL_3 */
			TIM2->CCER &= ~TIM_CCER_CC3E;
		}
	}
	if( (mot0_steps != mot0_steps_shadow) && (mot0_steps_shadow != 0) )
	{
		mot0_steps        = mot0_steps_shadow;
		mot0_steps_shadow = 0;
		/* Enable output on TIM_CHANNEL_3 */
		TIM2->CCER |= TIM_CCER_CC3E;
	}
}

/*-----------------------------------------------------------*/

static inline void HandleMot1StepPulseGeneration(void)
{
	if(mot1_steps != 0)
	{
		mot1_steps--;
		if(mot1_steps == 0)
		{
			/* Disable output on TIM_CHANNEL_4 */
			TIM2->CCER &= ~TIM_CCER_CC4E;
		}
	}
	if( (mot1_steps != mot1_steps_shadow) && (mot1_steps_shadow != 0) )
	{
		mot1_steps        = mot1_steps_shadow;
		mot1_steps_shadow = 0;
		/* Enable output on TIM_CHANNEL_4 */
		TIM2->CCER |= TIM_CCER_CC4E;
	}
}

/*-----------------------------------------------------------*/

static inline void HandleMot2StepPulseGeneration(void)
{
	if(mot2_steps != 0)
	{
		mot2_steps--;
		if(mot2_steps == 0)
		{
			/* Disable output on TIM_CHANNEL_1 */
			TIM2->CCER &= ~TIM_CCER_CC1E;
		}
	}
	if( (mot2_steps != mot2_steps_shadow) && (mot2_steps_shadow != 0) )
	{
		mot2_steps        = mot2_steps_shadow;
		mot2_steps_shadow = 0;
		/* Enable output on TIM_CHANNEL_1 */
		TIM2->CCER |= TIM_CCER_CC1E;
	}
}

/*-----------------------------------------------------------*/

static inline void ServoTestMove(void)
{
	switch(servo_mode)
	{
		case SERVO_MODE_DISABLED:
		servo_mode = SERVO_MODE_1000US_PULSE;
		break;

		case SERVO_MODE_1000US_PULSE:
		TIM2->CCR1 = TIM2_SERVO_MIN_PULSE_CNT;
		TIM2->CCR2 = TIM2_SERVO_MIN_PULSE_CNT;
		TIM2->CCR3 = TIM2_SERVO_MIN_PULSE_CNT;
		TIM2->CCR4 = TIM2_SERVO_MIN_PULSE_CNT;
		servo_mode = SERVO_MODE_1500US_PULSE;
		break;

		case SERVO_MODE_1500US_PULSE:
		TIM2->CCR1 = TIM2_SERVO_IDLE_PULSE_CNT;
		TIM2->CCR2 = TIM2_SERVO_IDLE_PULSE_CNT;
		TIM2->CCR3 = TIM2_SERVO_IDLE_PULSE_CNT;
		TIM2->CCR4 = TIM2_SERVO_IDLE_PULSE_CNT;
		servo_mode = SERVO_MODE_2000US_PULSE;
		break;

		case SERVO_MODE_2000US_PULSE:
		TIM2->CCR1 = TIM2_SERVO_MAX_PULSE_CNT;
		TIM2->CCR2 = TIM2_SERVO_MAX_PULSE_CNT;
		TIM2->CCR3 = TIM2_SERVO_MAX_PULSE_CNT;
		TIM2->CCR4 = TIM2_SERVO_MAX_PULSE_CNT;
		servo_mode = SERVO_MODE_1000US_PULSE;
		break;

		default:
		servo_mode = SERVO_MODE_1000US_PULSE;
		break;
	}
}
