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
#include "trinamic_spi.h"
#include "trinamic_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void SPI2_GPIO_Init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
SPI_HandleTypeDef hspi2;

/*******************************************************************************
 * Code
 ******************************************************************************/
void SPI2_Init(void)
{
	/* Enable clock @ SPI2 */
	RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
	__NOP();
	__NOP();

	/* SPI2 CONFIGURATION */
	hspi2.Instance               = SPI2;
	hspi2.Init.Mode              = SPI_MODE_MASTER;
	hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
	hspi2.Init.NSS               = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial     = 7;
	hspi2.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
	(void)SPI2_GPIO_Init();
	(void)HAL_SPI_Init(&hspi2);

	/* Start up */
	SPI2->CR1 |= SPI_CR1_SPE;
}

/*-----------------------------------------------------------*/

static void SPI2_GPIO_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* Configure GPIO pin : SPI2_MOSI_Pin */
	GPIO_InitStruct.Pin        = SPI2_MOSI_Pin;
	GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_DOWN;
	GPIO_InitStruct.Alternate  = LL_GPIO_AF_5;
	(void)LL_GPIO_Init(SPI2_MOSI_Port, &GPIO_InitStruct);

	/* Configure GPIO pin : SPI2_MISO_Pin */
	GPIO_InitStruct.Pin        = SPI2_MISO_Pin;
	GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_DOWN;
	GPIO_InitStruct.Alternate  = LL_GPIO_AF_5;
	(void)LL_GPIO_Init(SPI2_MISO_Port, &GPIO_InitStruct);

	/* Configure GPIO pin : SPI2_CLK_Pin */
	GPIO_InitStruct.Pin        = SPI2_CLK_Pin;
	GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_DOWN;
	GPIO_InitStruct.Alternate  = LL_GPIO_AF_5;
	(void)LL_GPIO_Init(SPI2_CLK_Port, &GPIO_InitStruct);
}
