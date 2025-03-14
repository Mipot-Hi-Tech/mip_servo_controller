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
#include "trinamic_dac.h"
#include "trinamic_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void DAC_GPIO_Init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
DAC_HandleTypeDef hdac;
uint8_t dac_output_voltage = DAC_OUTPUT_VOLTAGE_DEF;

/*******************************************************************************
 * Code
 ******************************************************************************/
void DAC_Init(void)
{
	/* Enable clock @ DAC */
	RCC->APB1ENR1 |= RCC_APB1ENR1_DACEN;
	__NOP();
	__NOP();

	DAC_ChannelConfTypeDef sConfig = {0};
	hdac.Instance = DAC;
	(void)HAL_DAC_Init(&hdac);
	sConfig.DAC_SampleAndHold           = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger                 = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer            = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming            = DAC_TRIMMING_FACTORY;
	(void)HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);
	DAC_GPIO_Init();

	/* Setup the AIN_IREF pins of drivers to 2.5 Vdc */
	DACUpdate();

	/* DAC channel1 enabled */
	DAC->CR |= DAC_CR_EN1;
}

/*-----------------------------------------------------------*/

void DACUpdate(void)
{
	DAC->DHR8R1 = dac_output_voltage;
}

/*-----------------------------------------------------------*/

static void DAC_GPIO_Init(void)
{
	/* Configure GPIO pin : AIN_IREF_MIP_Pin */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin  = AIN_IREF_MIP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(AIN_IREF_MIP_Port, &GPIO_InitStruct);
}
