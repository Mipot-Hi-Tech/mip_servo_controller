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

#ifndef __IPCC_H__
#define __IPCC_H__

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "main.h"
#include <string.h>
#include "core_cm4.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define IPCC_COMM_ADDR			0x200082FC;
#define IPCC_SERV_ADDR 			0x200082EC;
/* The address 0x200002E8 is reserved to indicate the state of the CPU2: Initialised or not */
#define CPU2_INIT_ADDR 			0x200082E8;

#define IPCC_COMM_SIZE 			260
#define IPCC_SERV_SIZE 			16
#define CPU2_INITIALISED 		0xAA
#define CPU2_NOT_INITIALISED 	0xBB

#define CH_ID_COMM				0
#define CH_ID_SERV				1

#define CM0_TX_COMM				1
#define CM0_RX_COMM				2
#define CM0_RX_SERV				3

#define CM0_TX_END				0xA5

#define CM0_REQ_LPM				0x01
#define CM0_REQ_WUP				0x02
#define CM0_REQ_UPD_BRATE		0x05

/*******************************************************************************
 * API
 ******************************************************************************/
uint8_t IPCC_Init(void);
void CM0_COMM_tx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir);
void CM0_COMM_rx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir);
void CM0_SERV_tx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir);
void CM0_SERV_rx_callback(struct __IPCC_HandleTypeDef *hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir);

extern __IO uint8_t CM0_rx;
extern __IO uint8_t *ipccCommBuff;
extern __IO uint8_t *ipccServBuff;
extern __IO uint8_t *cpu2InitDone;
extern IPCC_HandleTypeDef hipcc;
extern uint8_t *CommRxBusy;
extern uint8_t *CommTxBusy;

#ifdef __cplusplus
}
#endif

#endif /* __IPCC_H__ */
