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
* @file
* @date
* @version
*
*/

#ifndef INC_MIP_C_DEF_H_
#define INC_MIP_C_DEF_H_

#include <stdint.h>
#include <stdbool.h>
#include "mip_common.h"

/******************************************************************************/
/*! @name        C Series Macro Definitions                                   */
/******************************************************************************/

#define MIPC_ENABLE_PAIRING_CMD         (0x40U) /* Enable/Disable pairing command (Only for MASTER) */
#define MIPC_DEVICE_PAIRING_IND         (0x41U) /* MASTER pairing indicate (Only for MASTER) */
#define MIPC_GET_NETWORK_TABLE_SIZE_CMD (0x42U) /* Get MASTER Network Table Size (Only for MASTER) */
#define MIPC_GET_NETWORK_TABLE_ROW_CMD  (0x43U) /* Get MASTER Network Table Row (Only for MASTER) */
#define MIPC_DEL_END_DEVICE_CMD         (0x44U) /* Delete END NODE from MASTER Network Table (Only for MASTER) */
#define MIPC_DEL_ALL_EN_DEVICE_CMD      (0x45U) /* Delete whole Network Table (Only for MASTER) */
#define MIPC_PAIRING_REQ_CMD            (0x48U) /* Pairing request (Only for END NODE) */
#define MIPC_PAIRING_CONFIRM_IND        (0x49U) /* Indication of Activation (Only for END NODE) */
#define MIPC_GET_ACTIVATION_STATUS_CMD  (0x4AU) /* Get Activation Status Command (Only for END NODE) */
#define MIPC_TX_MSG_CMD                 (0x50U) /* Transmission of radio message */
#define MIPC_TX_MSG_CONFIRMED_IND       (0x51U) /* This command indicates the end of a confirmed transmission session. */
#define MIPC_TX_MSG_IND                 (0x52U) /* Indication of radio message transmission */
#define MIPC_RX_MSG_IND                 (0x53U) /* Indicate radio message reception */
#define MIPC_LINK_CHECK_REQ_CMD         (0x56U) /* Request for a link Check */
#define MIPC_LINK_CHECK_ANS_IND         (0x57U) /* Link Check Indication */
#define MIPC_SET_APP_KEY_CMD            (0x58U) /* Write EEPROM parameter Application encryption key */
#define MIPC_TX_SESSION_ABORT_IND       (0x59U) /* Indicate the abort of current communication session (Only for END NODE) */

/*!
 * @brief Mipot module 3200150xC data address
 */
#define MIPC_DEVICE_TYPE_ADDR           (0x00U)
#define MIPC_UNCONFIRMED_TX_NUM_ADDR    (0x01U)
#define MIPC_CONFIRMED_TX_NUM_ADDR      (0x02U)
#define MIPC_E_NODE_REQ_PAYLOAD_ADDR    (0x03U)
#define MIPC_E_NODE_PAIRING_MASTER_ADDR (0x04U)
#define MIPC_E_NODE_MASTER_TBL_IDX_ADDR (0x08U)
#define MIPC_POWER_ADDR                 (0x10U)
#define MIPC_FREQ_ADDR                  (0x11U)
#define MIPC_RSSI_ADDR                  (0x12U)
#define MIPC_DATA_INDICATE_TIMEOUT_ADDR (0x80U)
#define MIPC_UART_BAUDRATE_ADDR         (0x81U)
#define MIPC_APP_EN_AES_ADDR            (0x82U)

/*!
 * @brief mipc Stack Parameters default values
 */
#define MIPC_UnconfirmedTxNumber_MIN_VAL (0x01U)
#define MIPC_UnconfirmedTxNumber_MAX_VAL (0x0FU)
#define MIPC_UnconfirmedTxNumber_DEF     (0x03U)

#define MIPC_ConfirmedTxNumber_MIN_VAL   (0x01U)
#define MIPC_ConfirmedTxNumber_MAX_VAL   (0x0FU)
#define MIPC_ConfirmedTxNumber_DEF       (0x03U)

#define MIPC_POWER_MIN_VAL               (0x02U)
#define MIPC_POWER_MAX_VAL               (0x0EU)
#define MIPC_POWER_DEF                   (0x0EU)

#define MIPC_CHANNEL_FREQUENCY_MIN_VAL   (0x00U)
#define MIPC_CHANNEL_FREQUENCY_MAX_VAL   (0x02U)
#define MIPC_CHANNEL_FREQUENCY_DEF       (0x02U)

#define MIPC_RSSI_Th_MIN_VAL             (0x50U)
#define MIPC_RSSI_Th_MAX_VAL             (0x6EU)
#define MIPC_RSSI_Th_DEF                 (0x5AU)

#define MIPC_UartBaudrate_MIN_VAL        (0x00U)
#define MIPC_UartBaudrate_MAX_VAL        (0x04U)
#define MIPC_UartBaudrate_DEF_VAL        (0x04U)

#define MIPC_AppEnAes_MIN_VAL            (0x00U)
#define MIPC_AppEnAes_MAX_VAL            (0x01U)
#define MIPC_AppEnAes_DEF_VAL            (0x00U)


/*!
 * @brief The maximum number of the end node devices in user application
 */
#define MIPC_ENODE_NUM_MAX 255

/*!
 * @brief Mipot module 3200150xC device type
 */
enum device_type_t {
	master   = 0x00U,
	end_node = 0x01U
};

/*!
 * @brief Mipot module 3200150xC avaliable power levels
 */
enum c_power_t {
	power_level_2dBm  = 0x02U,
	power_level_3dBm  = 0x03U,
	power_level_4dBm  = 0x04U,
	power_level_5dBm  = 0x05U,
	power_level_6dBm  = 0x06U,
	power_level_7dBm  = 0x07U,
	power_level_8dBm  = 0x08U,
	power_level_9dBm  = 0x09U,
	power_level_10dBm = 0x0AU,
	power_level_11dBm = 0x0BU,
	power_level_12dBm = 0x0CU,
	power_level_13dBm = 0x0DU,
	power_level_14dBm = 0x0EU
};

/*!
 * @brief Mipot module 3200150xC avaliable frequency channels
 */
enum c_frequency_channel_t {
	frequency_channel_868_1MHz = 0x00U,
	frequency_channel_868_3MHz = 0x01U,
	frequency_channel_868_5MHz = 0x02U
};

/*!
 * @brief Mipot module 3200150xC Application AES Key Enable/Disable
 */
enum C_AppEnAes_t {
	AppEnAes_Disabled = 0x00U,
	AppEnAes_Enabled  = 0x01U
};

/*!
 * @brief Mipot c message type
 */
typedef enum {
	UnconfirmedDataTransmission = 0x00U,
	ConfirmedDataTransmission   = 0x01U
}c_messageTrasmission_t;

/*!
 * @brief Mipot link check result
 */
enum LinkCheckResult_t{
	LinkCheckNotPerformed = 0x00U,
	LinkCheckOK           = 0x01U,
	LinkCheckKO           = 0x02U
};

/*!
 * @brief Mipot module 3200150xC stack parameters
 */
struct c_stack_param_t {
	/* MASTER – END NODE Selection */
	enum device_type_t device_type;

	/* Define the Number of broadcast consecutive message transmissions */
    uint8_t UnconfirmedTxNumber;

    /* Define the Number of transmission retries if ACK is not received */
    uint8_t ConfirmedTxNumber;

    /* Define the Pairing Request Payload */
    uint8_t EndNodePairingReqPayload;

    /* Define the paired MASTER address */
    uint8_t EndNodePairingMstAddress[4];

    /* Define the END NODE index assigned by MASTER after the pairing phase. */
    uint8_t EndNodeMstTblIdx;
};

/*!
 * @brief Mipot module 3200150xC radio parameters
 */
struct c_radio_phy_t {
	/* Power expressed in dBm */
    enum c_power_t power;

    /* Channel Frequency selection */
    enum c_frequency_channel_t frequency_channel;

    /* Define the RSSI threshold for channel free assessment. It is an absolute value */
    uint8_t RSSI_Th;
};

/*!
 * @brief Mipot module 3200150xC module parameters
 */
struct c_module_param_t {
	/* Timeout in ms */
    uint8_t DATA_INDICATE_TIMEOUT;

    /* Uart baudrate selection */
    enum UartBaudrate_t UartBaudrate;

    /* Application AES Key Enable/Disable */
    enum C_AppEnAes_t AppEnAes;
};

/*!
 * @brief end node specific
 */
struct c_rx_data_t {
    /* 16-bit Rssi Value expressed in dBm */
    uint8_t RssiLSB;
    uint8_t RssiMSB;
    /* 8-bit Signal-to-Noise Ratio */
    uint8_t SNR;
    /* Source device Serial Number (4 bytes) */
    uint8_t SRC_ID[4];
    /* Received message info */
    uint8_t last_rx_msg[240];
	uint8_t last_rx_msg_len;
	uint16_t rx_msg_num;
};

struct c_LinkCheck_t {
	enum LinkCheckResult_t result;
	/*
	Defines the power used for the link check procedure expressed in dBm.
	Allowed range is from 2 to 14dBm.
	Suggested value is 11dBm.
	*/
	uint8_t power;

	/* Defines the number of messages to be transmitted. Allowed Range is from 4 to 20. Suggested Value is 5. */
	uint8_t MessageNumber;

	/*
	Defines pass/fail criteria threshold as the number of received messages.
	If the number of received messages is >= to MessageTh test succeeded. The suggested value is 4.
	Minimum value is 1 and maximum Value is Message Number.
	*/
	uint8_t MessageTh;

	/* The number of message received from master */
	uint8_t MessageReceived;
};

/*!
 * @brief end node specific
 */
struct c_end_node_data_t {
	bool is_paired;
	/* MASTER Serial Number */
	uint8_t master_serial_number[4];
	uint32_t SessionTxTime;
	/* MASTER table index assigned after pairing phase. */
	uint8_t idx;
};

/*!
 * @brief end node info in the master Network Table
 */
struct c_node_data_t {
/* ID of the paired end node */
uint32_t enode_serial_number;
/* Byte received at pairing phase */
uint8_t custom_pairing_payload_byte;
};

/*!
 * @brief master device data specific
 */
struct c_master_data_t {
	/* the size of Network Table */
	uint8_t ROUTING_SIZE;
	/* information about connected en node */
	struct c_node_data_t enode[MIPC_ENODE_NUM_MAX];
};

/*!
 * @brief Mipot module 3200150xC tx unconfirmed info
 */
struct c_unconfirmed_msg_ind_t {
	uint32_t SessionTxTime;
};

/*!
 * @brief Mipot module 3200150xC tx confirmed info
 */
struct c_confirmed_msg_ind_t {
	uint32_t SessionTxTime;
	bool AckReceived;
	uint8_t NbRetries;
};

struct mip_c
{
    uint32_t fw_version;
    uint32_t serialno;
    struct c_stack_param_t stack_param;
    struct c_radio_phy_t radio_phy_param;
    struct c_module_param_t module_param;
    struct c_unconfirmed_msg_ind_t unconfirmed_msg_ind;
    struct c_confirmed_msg_ind_t confirmed_msg_ind;
    struct c_rx_data_t rx_data;
    /* end node specific */
    struct c_end_node_data_t end_node_data;
	struct c_LinkCheck_t link_check;
    /* master specific */
    struct c_master_data_t master_data;
    /* porting function pointers */
    void (*hardware_init_fn) (enum UartBaudrate_t baudrate);
    enum mip_error_t (*send_and_receive_fn) (uint8_t *tx_buff, uint16_t tx_dim, uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms);
    enum mip_error_t (*receive_fn) (uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms);
    void (*delay_ms_fn) (uint32_t ms);
    void (*hardware_reset_fn) (void);
};

#endif
