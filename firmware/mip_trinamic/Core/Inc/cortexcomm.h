#ifndef CORE_INC_CORTEXCOMM_H_
#define CORE_INC_CORTEXCOMM_H_

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "mip_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
enum mip_error_t MipTransmitAndReceiveData(uint8_t *tx_buff, uint16_t tx_dim, uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms);
enum mip_error_t MipReceiveData(uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms);

#endif
