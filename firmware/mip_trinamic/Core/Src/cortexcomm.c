/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "cortexcomm.h"
#include "trinamic_tim.h"
#include "ipcc.h"
#include "mip_d.h"
#include "mip_d_def.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MIP_IPCC_TX_TIMEOUT 100
#define MIP_IPCC_RX_TIMEOUT 100

typedef enum{
	IPCC_TXRX_HANDLER_IDLE                  = 0x00U,
	IPCC_TXRX_HANDLER_TX_DATA_TRANSMISSION  = 0x01U,
	IPCC_TXRX_HANDLER_RX_DATA_WAIT          = 0x02U,
	IPCC_TXRX_HANDLER_DATA_COPY             = 0x03U,
	IPCC_TXRX_HANDLER_COMPLETED             = 0x04U,
	IPCC_TXRX_HANDLER_ERROR                 = 0x05U
}cortexcomm_ipcc_txrx_t;

typedef enum{
	IPCC_RX_HANDLER_IDLE                   = 0x00U,
	IPCC_RX_HANDLER_RX_DATA_WAIT           = 0x01U,
	IPCC_RX_HANDLER_DATA_COPY              = 0x02U,
	IPCC_RX_HANDLER_COMPLETED              = 0x03U,
	IPCC_RX_HANDLER_ERROR                  = 0x04U
}cortexcomm_ipcc_rx_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
cortexcomm_ipcc_txrx_t ipcc_txrx_handler = IPCC_TXRX_HANDLER_IDLE;
cortexcomm_ipcc_rx_t   ipcc_rx_handler   = IPCC_RX_HANDLER_IDLE;
extern IPCC_HandleTypeDef hipcc;

/*******************************************************************************
 * Code
 ******************************************************************************/
enum mip_error_t MipTransmitAndReceiveData(uint8_t *tx_buff, uint16_t tx_dim, uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms)
{
	enum mip_error_t stat = unknown_error;
	uint32_t time;
	ipcc_txrx_handler = IPCC_TXRX_HANDLER_IDLE;
	while( (ipcc_txrx_handler != IPCC_TXRX_HANDLER_COMPLETED) && (ipcc_txrx_handler != IPCC_TXRX_HANDLER_ERROR) )
	{
		switch(ipcc_txrx_handler)
		{
			case IPCC_TXRX_HANDLER_IDLE:
			{
				if(tx_dim > 0)
				{
					ipcc_txrx_handler = IPCC_TXRX_HANDLER_TX_DATA_TRANSMISSION;
				}
				else
				{
					ipcc_txrx_handler = IPCC_TXRX_HANDLER_ERROR;
				}
				break;
			}

			case IPCC_TXRX_HANDLER_TX_DATA_TRANSMISSION:
			{
				/* Copy message to IPCC common buffer */
				memcpy((uint8_t *)ipccCommBuff, tx_buff, tx_dim);
				/* Notify remote cpu of the on-going transaction */
				CM0_rx = 0;
				(void)HAL_IPCC_NotifyCPU(&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_TX);
				time = GetTick();
				ipcc_txrx_handler = IPCC_TXRX_HANDLER_RX_DATA_WAIT;
				break;
			}

			case IPCC_TXRX_HANDLER_RX_DATA_WAIT:
			{
				if( GetTick() - time >= timeout_ms)
				{
					stat = rx_timeout;
					ipcc_txrx_handler = IPCC_TXRX_HANDLER_ERROR;
				}
				if(CM0_RX_COMM == CM0_rx)
				{
					CM0_rx = 0;
					ipcc_txrx_handler = IPCC_TXRX_HANDLER_DATA_COPY;
				}
				break;
			}

			case IPCC_TXRX_HANDLER_DATA_COPY:
			{
				memcpy(rx_buff, (uint8_t *)ipccCommBuff, ipccCommBuff[2] + 4);
				*rx_dim = ipccCommBuff[2] + 4;
				memset((uint8_t *) ipccCommBuff, 0, IPCC_COMM_SIZE);
				ipccServBuff[0] = CM0_TX_END;
				(void)HAL_IPCC_NotifyCPU(&hipcc, CH_ID_SERV, IPCC_CHANNEL_DIR_TX);
				stat = no_error;
				ipcc_txrx_handler = IPCC_TXRX_HANDLER_COMPLETED;
				break;
			}

			case IPCC_TXRX_HANDLER_COMPLETED:
			case IPCC_TXRX_HANDLER_ERROR:
			{
				break;
			}

			default:
			{
				ipcc_txrx_handler = IPCC_TXRX_HANDLER_IDLE;
				break;
			}
		}
	}
	return stat;
}

enum mip_error_t MipReceiveData(uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms)
{
	enum mip_error_t stat = unknown_error;
	uint32_t time;
	ipcc_rx_handler = IPCC_RX_HANDLER_IDLE;
	while( (ipcc_rx_handler != IPCC_RX_HANDLER_COMPLETED) && (ipcc_rx_handler != IPCC_RX_HANDLER_ERROR) )
	{
		switch(ipcc_rx_handler)
		{
			case IPCC_RX_HANDLER_IDLE:
			{
				time = GetTick();
				ipcc_rx_handler = IPCC_RX_HANDLER_RX_DATA_WAIT;
				break;
			}

			case IPCC_RX_HANDLER_RX_DATA_WAIT:
			{
				if( GetTick() - time >= timeout_ms)
				{
					stat =  rx_timeout;
					ipcc_rx_handler = IPCC_RX_HANDLER_ERROR;
				}
				if(CM0_TX_COMM == CM0_rx)
				{
					ipcc_rx_handler = IPCC_RX_HANDLER_DATA_COPY;
				}
				break;
			}

			case IPCC_RX_HANDLER_DATA_COPY:
			{
				memcpy(rx_buff, (uint8_t *)ipccCommBuff, ipccCommBuff[2] + 4);
				*rx_dim = ipccCommBuff[2] + 4;
				(void)HAL_IPCC_NotifyCPU(&hipcc, CH_ID_COMM, IPCC_CHANNEL_DIR_RX);
				CM0_rx = 0;
				memset((uint8_t *) ipccCommBuff, 0, IPCC_COMM_SIZE);
				ipccServBuff[0] = CM0_TX_END;
				HAL_IPCC_NotifyCPU(&hipcc, CH_ID_SERV, IPCC_CHANNEL_DIR_TX);
				stat = no_error;
				ipcc_rx_handler = IPCC_RX_HANDLER_COMPLETED;
				break;
			}

			case IPCC_RX_HANDLER_COMPLETED:
			case IPCC_RX_HANDLER_ERROR:
			{
				break;
			}

			default:
			{
				ipcc_rx_handler = IPCC_RX_HANDLER_IDLE;
				break;
			}
		}
	}
	return stat;
}
