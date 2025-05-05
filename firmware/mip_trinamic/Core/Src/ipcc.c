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
#include "ipcc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
__IO uint8_t CM0_rx;
__IO uint8_t *ipccCommBuff = (uint8_t *)IPCC_COMM_ADDR;
__IO uint8_t *ipccServBuff = (uint8_t *)IPCC_SERV_ADDR;
/* The address 0x2000FEFC is reserved to indicate the state of the CPU2: Initialised or not */
__IO uint8_t *cpu2InitDone = (uint8_t *)CPU2_INIT_ADDR;
IPCC_HandleTypeDef hipcc;
uint8_t  *CommRxBusy;
uint8_t  *CommTxBusy;
uint8_t ndata_indicate_event;

/*******************************************************************************
 * Code
 ******************************************************************************/
uint8_t IPCC_Init(void)
{
	uint8_t stat = 0;
	memset ((uint8_t *) ipccCommBuff, 0, IPCC_COMM_SIZE);
	memset ((uint8_t *) ipccServBuff, 0, IPCC_SERV_SIZE);
	*cpu2InitDone = CPU2_NOT_INITIALISED;
	CM0_rx = 0;
	hipcc.Instance = IPCC;
	if (HAL_IPCC_Init(&hipcc) != HAL_OK)
	{
		stat++;
	}
	if (HAL_IPCC_ActivateNotification(&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_TX, CM0_COMM_tx_callback) != HAL_OK)
	{
		stat++;
	}
	/* The callback is triggered when the remote cpu send a message. (IPCC_CHANNEL_DIR_RX) */
	if (HAL_IPCC_ActivateNotification(&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_RX, CM0_COMM_rx_callback) != HAL_OK)
	{
		stat++;
	}
	/* The callback is triggered when the remote cpu send a message. (IPCC_CHANNEL_DIR_RX) */
	if (HAL_IPCC_ActivateNotification(&hipcc, CH_ID_SERV, IPCC_CHANNEL_DIR_TX, CM0_SERV_tx_callback) != HAL_OK)
	{
		stat++;
	}
	/* The callback is triggered when the remote cpu send a message. (IPCC_CHANNEL_DIR_RX) */
	if (HAL_IPCC_ActivateNotification(&hipcc, CH_ID_SERV, IPCC_CHANNEL_DIR_RX, CM0_SERV_rx_callback) != HAL_OK)
	{
		stat++;
	}
	return stat;
}

/*-----------------------------------------------------------*/

void HAL_IPCC_MspInit(IPCC_HandleTypeDef* ipccHandle)
{
	if(ipccHandle->Instance==IPCC)
	{
		__HAL_RCC_IPCC_CLK_ENABLE();
		HAL_NVIC_SetPriority(IPCC_C1_RX_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(IPCC_C1_RX_IRQn);
		HAL_NVIC_SetPriority(IPCC_C1_TX_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(IPCC_C1_TX_IRQn);
	}
}

/*-----------------------------------------------------------*/

void HAL_IPCC_MspDeInit(IPCC_HandleTypeDef* ipccHandle)
{
	if(ipccHandle->Instance==IPCC)
	{
		__HAL_RCC_IPCC_CLK_DISABLE();
		HAL_NVIC_DisableIRQ(IPCC_C1_RX_IRQn);
		HAL_NVIC_DisableIRQ(IPCC_C1_TX_IRQn);
	}
}

/*-----------------------------------------------------------*/

void CM0_COMM_tx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir)
{
	/* CORTEX M0(CPU2) respond to CORTEX M4(CPU1) */
	CM0_rx = CM0_RX_COMM;
}

/*-----------------------------------------------------------*/

void CM0_COMM_rx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir)
{
	/* CORTEX M0(CPU2) send spontaneus message to CORTEX M4(CPU1) */
	CM0_rx = CM0_TX_COMM;
	ndata_indicate_event++;
}

/*-----------------------------------------------------------*/

void CM0_SERV_tx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir)
{
	/* CORTEX M0(CPU2) respond to CM0_TX_END event from CORTEX M4(CPU1) */
}

/*-----------------------------------------------------------*/

void CM0_SERV_rx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir)
{
	CM0_rx = CM0_RX_SERV;
	HAL_IPCC_NotifyCPU(hipcc, CH_ID_SERV, IPCC_CHANNEL_DIR_RX);
}
