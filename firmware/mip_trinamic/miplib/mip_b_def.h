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

#ifndef MIP_B_DEF_H_
#define MIP_B_DEF_H_

#include <stdint.h>
#include <stdbool.h>
#include "mip_common.h"

/******************************************************************************/
/*! @name        B Series Macro Definitions                                   */
/******************************************************************************/

#define MIPB_GET_DEV_EUI_CMD           (0x36U) /* Get DevEUI provided by Mipot. */
#define MIPB_JOIN_CMD                  (0x40U) /* This command performs the network activation. */
#define MIPB_JOIN_IND                  (0x41U) /* This command indicates the result of OTAA join procedure. */
#define MIPB_GET_ACTIVATION_STATUS_CMD (0x42U) /* This command gets the module activation status. */
#define MIPB_SET_APP_KEY_CMD           (0x43U) /* This command performs the EEPROM data write. */
#define MIPB_SET_APP_SESSION_KEY_CMD   (0x44U) /* This command performs the EEPROM data write. */
#define MIPB_SET_NWK_SESSION_KEY_CMD   (0x45U) /* This command performs the EEPROM data write. */
#define MIPB_TX_MSG_CMD                (0x46U) /* This command performs the transmission of a radio frame. */
#define MIPB_TX_MSG_CONFIRMED_IND      (0x47U) /* This command indicates that a confirmed radio frame transmission has been performed. */
#define MIPB_TX_MSG_UNCONFIRMED_IND    (0x48U) /* This command indicates that an unconfirmed radio frame transmission has been performed. */
#define MIPB_RX_MSG_IND                (0x49U) /* This command indicates that a radio frame has been received. */
#define MIPB_GET_SESSION_STATUS_CMD    (0x4AU) /* This command gets the module current status. */
#define MIPB_SET_NEXT_DR_CMD           (0x4BU) /* This command will set next transmission DR. */
#define MIPB_SET_BATTERY_LEVEL_CMD     (0x50U) /* This command will set the battery level required for DevStatusReq frame used in LoRaWAN class A protocol. */
#define MIPB_GET_BATTERY_LVL_CMD       (0x51U) /* This command will get the battery level value. */
#define MIPB_SET_UPLINK_CNT_CMD        (0x52U) /* This command will set the uplink counter in RAM memory. */
#define MIPB_GET_UPLINK_CNT_CMD        (0x53U) /* This command will get the uplink counter from RAM memory. */
#define MIPB_SET_DOWNLINK_CNT_CMD      (0x54U) /* This command will set the downlink counter in RAM memory. */
#define MIPB_GET_DOWNLINK_CNT_CMD      (0x55U) /* This command will get the downlink counter from RAM memory. */
#define MIPB_SET_CH_PARAMETERS_CMD     (0x57U) /* This command will set channel parameters and enable/disable optional channels. */
#define MIPB_GET_CH_PARAMETERS_CMD     (0x58U) /* This command will get channel parameters. */
#define MIPB_LINK_CHECK_REQUEST_CMD    (0x60U) /* This command will start a link check with available gateways. */
#define MIPB_LINK_CHECK_REQUEST_IND    (0x61U) /* This command indicates the link margin and the available gateways count. */
#define MIPB_DEVICE_TIME_REQUEST_CMD   (0x62U) /* This command requests the network time and date. */
#define MIPB_DEVICE_TIME_REQUEST_IND   (0x63U) /* This command indicates the link margin and the available gateways count. */
#define MIPB_RESET_ABP_CMD             (0x64U) /* This command will renegotiate the ABP join. */
#define MIPB_RESET_ABP_IND             (0x65U) /* This command indicates the server LoRaWAN version. */
#define MIPB_REKEY_OTAA_CMD            (0x66U) /* This command will renegotiate the OTAA join. */
#define MIPB_REKEY_OTAA_IND            (0x67U) /* This command indicates the server LoRaWAN version. */

/*!
 * @brief mipot module 3200150xB data address
 */
#define MIPB_CUSTOMER_DEV_EUI_ADDR  (0x00U)
#define MIPB_APPEUI_ADDR            (0x08U)
#define MIPB_DEVADDR_ADDR           (0x10U)
#define MIPB_CLASS_ADDR             (0x20U)
#define MIPB_DR_SF_ADDR             (0x21U)
#define MIPB_TX_POWER_LEVEL_ADDR    (0x22U)
#define MIPB_ADR_ADDR               (0x23U)
#define MIPB_DC_CONTROL_ADDR        (0x24U)
#define MIPB_UNCONF_TX_NUM_ADDR     (0x25U)
#define MIPB_EN_CUSTOM_EUI_ADDR     (0x26U)
#define MIPB_RX2_DATA_RATE_ADDR     (0x27U)
#define MIPB_RX2_FREQUENCY_ADDR     (0x28U)
#define MIPB_LINKCHECK_TIMEOUT_ADDR (0x2CU) /* RESERVED */
#define MIPB_PUB_NET_EN_ADDR        (0x2EU)

/*!
 * @brief mipb Stack Parameters default values
 */

#define MIPB_LORAWAN_CLASS_MIN_VAL       (0x00U)
#define MIPB_LORAWAN_CLASS_MAX_VAL       (0x02U)
#define MIPB_LORAWAN_CLASS_DEF           (0x00U) /* Class A */

#define MIPB_DR_SF_MIN_VAL               (0x00U)
#define MIPB_DR_SF_MAX_VAL               (0x05U)
#define MIPB_DR_SF_DEF                   (0x00U) /* SF12/125 kHz */

#define MIPB_POWER_MIN_VAL               (0x00U)
#define MIPB_POWER_MAX_VAL               (0x07U)
#define MIPB_POWER_DEF                   (0x00U) /* 14 dBm */

#define MIPB_UNCONF_TX_NUM_MIN_VAL       (0x00U)
#define MIPB_UNCONF_TX_NUM_MAX_VAL       (0x0FU)
#define MIPB_UNCONF_TX_NUM_DEF           (0x00U)

#define MIPB_RX2_DATA_RATE_MIN_VAL       (0x00U)
#define MIPB_RX2_DATA_RATE_MAX_VAL       (0x07U)
#define MIPB_RX2_DATA_RATE_DEF           (0x00U) /* SF12/125 kHz */

#define MIPB_RX2_FREQUENCY_MIN_VAL       (0x33BCA100U) /* 863000000 Hz */
#define MIPB_RX2_FREQUENCY_MAX_VAL       (0x33DB2580U) /* 870000000 Hz */
#define MIPB_RX2_FREQUENCY_DEF           (0x33D3E608U) /* 869525000 Hz */

/*!
 * @brief Mipot module 3200150xB avaliable LoRaWAN Classes
 */
enum b_class_t {
	LoRaWAN_Class_A = 0x00U,
	LoRaWAN_Class_B = 0x01U,
	LoRaWAN_Class_C = 0x02U,
};

/*!
 * @brief Mipot module 3200150xB avaliable power levels
 */
enum b_power_t {
	power_level_14dBm  = 0x00U,
	power_level_12dBm  = 0x01U,
	power_level_10dBm  = 0x02U,
	power_level_8dBm   = 0x03U,
	power_level_6dBm   = 0x04U,
	power_level_4dBm   = 0x05U,
	power_level_2dBm   = 0x06U,
	power_level_0dBm   = 0x07U
};

/*!
 * @brief Mipot module 3200150xB avaliable LoRa Datarate/Spreading Factor settings
 */
enum b_dr_sf_t {
	SF12_125kHz = 0x00U,
	SF11_125kHz = 0x01U,
	SF10_125kHz = 0x02U,
	SF9_125kHz  = 0x03U,
	SF8_125kHz  = 0x04U,
	SF7_125kHz  = 0x05U,
};

/*!
 * @brief Mipot module 3200150xB Adaptive Datarate setting
 */
enum b_adr_t {
	ADR_Disabled = 0x00U,
	ADR_Enabled  = 0x01U
};

/*!
 * @brief Mipot module 3200150xB Duty Cycle control setting
 */
enum b_dc_t {
	DC_Disabled = 0x00U,
	DC_Enabled  = 0x01U
};

/*!
 * @brief Mipot module 3200150xB Customer EUI setting
 */
enum b_customer_eui_t {
	Customer_EUI_Disabled = 0x00U,
	Customer_EUI_Enabled  = 0x01U
};

/*!
 * @brief Mipot module 3200150xB RX2 Data Rate
 */
enum b_rx2_data_rate_t {
	rx2_SF12_125kHz = 0x00U,
	rx2_SF11_125kHz = 0x01U,
	rx2_SF10_125kHz = 0x02U,
	rx2_SF9_125kHz  = 0x03U,
	rx2_SF8_125kHz  = 0x04U,
	rx2_SF7_125kHz  = 0x05U,
	rx2_SF7_250kHz  = 0x06U,
	rx2_FSK	        = 0x07U,
};

/*!
 * @brief Mipot module 3200150xB Public Network Enable setting
 */
enum b_public_network_t {
	Public_Network_Disabled = 0x00U,
	Public_Network_Enabled  = 0x01U
};

/*!
 * @brief Mipot module 3200150xB Join mode
 */
enum b_join_mode_t {
	Join_mode_ABP   = 0x00U, /* Activation by personalization */
	Join_mode_OTAA  = 0x01U  /* Over the air activation */
};

/*!
 * @brief Mipot module 3200150xB Join status
 */
enum b_join_status_t {
	Device_not_activated = 0x00U,
	Joining              = 0x01U,
	Joined               = 0x02U,
	MAC_error            = 0x03U
};

/*!
 * @brief Mipot module 3200150xB session status
 */
enum b_session_status_t {
	Session_Idle                 = 0x00U,
	Session_Busy                 = 0x01U, /* LoRa session sunning */
	Session_Device_not_activated = 0x02U,
	Session_Delayed              = 0x03U  /* LoRa session paused due to Duty-Cycle */
};

/*!
 * @brief Mipot module 3200150xB module parameters
 */
struct b_module_param_t {
	/* Timeout in ms */
    uint8_t DATA_INDICATE_TIMEOUT;

    /* Uart baudrate selection */
    enum UartBaudrate_t UartBaudrate;
};

/*!
 * @brief Mipot b message type
 */
typedef enum {
	UnconfirmedDataTransmission = 0x00U,
	ConfirmedDataTransmission   = 0x01U
}b_messageTrasmission_t;

/*!
 * @brief Mipot module 3200150xB stack parameters
 */
struct b_stack_param_t {
	/* Customer 64 bit Extended Unique Identifier */
	uint8_t Customer_DevEUI[8];

	/* 64 bit Application Extended Unique Identifier */
	uint8_t AppEUI[8];

	/* 32 bit Device Address */
	uint8_t DevAddr[4];

	/* LoRaWAN Class */
	enum b_class_t class;

	/* LoRa Datarate/Spreading Factor setting */
	enum b_dr_sf_t dr_sf;

	/* Tx Power level */
	enum b_power_t power;

	/* Enable/Disable Adaptive Datarate */
	enum b_adr_t ADR;

	/* Enable/Disable duty cycle control */
	enum b_dc_t Duty_Cycle_control;

	/* Number of uplink messages repetitions (unconfirmed messages only) */
	uint8_t Unconfirmed_TX_Repetition_number;

	/* Enable/Disable customer EUI */
	enum b_customer_eui_t Enable_Customer_EUI;

	/* RX2 Window Datarate */
	enum b_rx2_data_rate_t RX2_Data_Rate;

	/* RX2 Window frequency */
	uint32_t RX2_Frequency;

	/* Enable public/private network sync word */
	enum b_public_network_t Public_Network_Enable;

	/* 16 bytes in Little Endian Order. Needed for OTAA procedure. */
	uint8_t AppKey[16];

	/* 16 bytes in Little Endian Order. Needed for APB procedure. */
	uint8_t AppSKey[16];

	/* 16 bytes in Little Endian Order. Needed for APB procedure. */
	uint8_t NwkSKey[16];

	uint32_t uplink_cnt;
	uint32_t downlink_cnt;
};

/*!
 * @brief Mipot module 3200150xB link check result
 */
struct b_link_check_result_t {
	/* 0-254 link margin in dB of the last successfully received Link_Check_CMD */
	uint8_t Margin;

	/* Number of available gateways in range */
	uint8_t GW_Cnt;
};

/*!
 * @brief Mipot module 3200150xB network time
 */
struct b_network_time_t {
	uint32_t epoch_s; /* Seconds since Epoch */
	uint8_t epoch_fs;  /* Fractional seconds */
};

/*!
 * @brief Mipot module 3200150xB rx message structure
 */
struct b_downlink_msg_t {
	uint8_t MsgType;

	/* Reserved for future usage */
	uint8_t MulticastFlag;

	enum b_rx2_data_rate_t RxDataRate;

	/* RxSlotValue
	0 = Rx window 1
	1 = Rx window 2
	*/
	uint8_t RxSlot;

	/*
	Frame Pending status:
	0 = no downlink Frame Pending
	1 = downlink Frame Pending
	*/
	uint8_t FramePending;

	/* Indicates if an Ack is received */
	uint8_t AckReceived;

	/* Indicates if data is available */
	uint8_t RxData;

	/* 16-bit Rssi Value expressed in dBm */
	uint8_t Rssi;

	/* 8-bit Signal-to-Noise Ratio (for FSK SNR = 0) */
	uint8_t SNR;

	/* Port Number, from 1 to 223 */
	uint8_t Port;

	/* Data Message received in Downlink*/
	uint8_t Payload[200];

	/* The lenght of the message */
	uint8_t payload_lenght;
};

/*!
 * @brief Mipot module 3200150xB tx unconfirmed info
 */
struct b_unconfirmed_msg_ind_t {
	uint8_t status;
	enum b_dr_sf_t DataRate;
	enum b_power_t TxPower;
};

/*!
 * @brief Mipot module 3200150xB tx confirmed info
 */
struct b_confirmed_msg_ind_t {
	uint8_t status;
	enum b_dr_sf_t DataRate;
	enum b_power_t TxPower;
	_Bool AckReceived;
};

struct mip_b
{
    uint32_t fw_version;
    uint32_t serialno;
	enum b_join_status_t join_status;
	enum b_session_status_t session_status;
    struct b_stack_param_t stack_param;
    struct b_module_param_t module_param;
    struct b_link_check_result_t link_check_result;
    struct b_network_time_t network_time;
    struct b_downlink_msg_t downlink_msg;
    struct b_unconfirmed_msg_ind_t unconfirmed_msg_ind_info;
    struct b_confirmed_msg_ind_t confirmed_msg_ind_info;
    /* porting function pointers */
    void (*hardware_init_fn) (enum UartBaudrate_t baudrate);
    enum mip_error_t (*send_and_receive_fn) (uint8_t *tx_buff, uint16_t tx_dim, uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms);
    enum mip_error_t (*receive_fn) (uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms);
    void (*delay_ms_fn) (uint32_t ms);
    void (*hardware_reset_fn) (void);
};

#endif
