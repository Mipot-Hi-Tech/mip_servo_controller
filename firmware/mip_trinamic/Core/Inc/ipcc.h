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
