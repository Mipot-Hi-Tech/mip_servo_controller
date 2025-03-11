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

#ifndef MIP_A_DEF_H_
#define MIP_A_DEF_H_

#include <stdint.h>
#include <stdbool.h>
#include "mip_common.h"

/******************************************************************************/
/*! @name        A Series Macro Definitions                                   */
/******************************************************************************/

#define MIPA_GET_RSSI_CMD     (0x39U) /* This command get last RSSI value */
#define MIPA_SET_MODE_CMD     (0x40U) /* This command change WM-Bus Mode */
#define MIPA_SET_C_FIELD_MODE (0x41U) /* This command set C-Field WM_Bus value */
#define MIPA_TX_MSG_CMD       (0x50U) /* This command performs trasmission of WM-Bus message */
#define MIPA_RX_MSG_IND       (0x53U) /* This command indicate reception of WM-Bus message */

/*!
 * @brief Mipot module 3200150xA data address
 */
#define MIPA_WM_BUS_MODE_ADDR            (0x00U)
#define MIPA_RF_CHANNEL_ADDR             (0x01U)
#define MIPA_RF_POWER_ADDR               (0x02U)
#define MIPA_RF_AUTOSLEEP_ADDR           (0x03U)
#define MIPA_RX_WINDOW_ADDR              (0x04U)

#define MIPA_BLOCK1_FROM_MODULE_ADDR     (0x20U)
#define MIPA_RSSI_ADDR                   (0x21U)
#define MIPA_NDATA_INDICATE_TIMEOUT_ADDR (0x22U)
#define MIPA_UART_BAUDRATE_ADDR          (0x24U)

#define MIPA_C_FIELD_ADDR                (0x10U) /* C Field */
#define MIPA_MAN_ID0_FIELD_ADDR          (0x11U) /* Manufacturer ID LSB */
#define MIPA_AN_ID1_FIELD_ADDR           (0x12U) /* Manufacturer ID MSB */
#define MIPA_DEVICE_ID0_FIELD_ADDR       (0x13U) /* Device ID LSB */
#define MIPA_DEVICE_ID1_FIELD_ADDR       (0x14U) /* Device ID */
#define MIPA_DEVICE_ID2_FIELD_ADDR       (0x15U) /* Device ID */
#define MIPA_DEVICE_ID3_FIELD_ADDR       (0x16U) /* Device ID MSB */
#define MIPA_VERSION_ADDR                (0x17U) /* Version */
#define MIPA_DEVICE_TYPE_ADDR            (0x18U) /* Device Type */

/*!
 * @brief mipa Module Parameters default values
 */
#define MIPA_WMBUS_MODE_MIN_VAL          (0x00U)
#define MIPA_WMBUS_MODE_MAX_VAL          (0x0EU)
#define MIPA_WMBUS_MODE_DEF              (0x00U)

#define MIPA_RF_CHANNEL_MIN_VAL          (0x00U)
#define MIPA_RF_CHANNEL_MAX_VAL          (0x09U)
#define MIPA_RF_CHANNEL_DEF              (0x00U)

#define MIPA_POWER_MIN_VAL               (0x00U)
#define MIPA_POWER_MAX_VAL               (0x04U)
#define MIPA_POWER_DEF                   (0x04U)

#define MIPA_RF_AUTOSLEEP_MIN_VAL        (0x00U)
#define MIPA_RF_AUTOSLEEP_MAX_VAL        (0x01U)
#define MIPA_RF_AUTOSLEEP_DEF_VAL        (0x00U)

#define MIPA_UART_BAUDRATE_MIN_VAL       (0x00U)
#define MIPA_UART_BAUDRATE_MAX_VAL       (0x04U)
#define MIPA_UART_BAUDRATE_DEF_VAL       (0x04U)

#define MIPA_BLOCK1_FROM_MODULE_MIN_VAL  (0x00U)
#define MIPA_BLOCK1_FROM_MODULE_MAX_VAL  (0x01U)
#define MIPA_BLOCK1_FROM_MODULE_DEF_VAL  (0x00U)

#define MIPA_RSSI_MIN_VAL                (0x00U)
#define MIPA_RSSI_MAX_VAL                (0x01U)
#define MIPA_RSSI_DEF_VAL                (0x00U)

#define MIPA_RX_WINDOW_DEF_VAL           (0x04U)
#define MIPA_NDATA_IND_TIMEOUT_DEF_VAL   (0x05U)

/*!
 * @brief Mipot module 3200150xA avaliable WM Bus modes
 */
enum a_wmbusmode_t {
	S2_Short_preamble   = 0x00U,
	S2_Long_preamble    = 0x01U,
	S1                  = 0x02U,
	S1_m                = 0x03U,
	T1_meter            = 0x04U,
	T2_meter            = 0x05U,
	T2_other            = 0x06U,
	R2_meter            = 0x07U,
	R2_other            = 0x08U,
	C1_meter_Frame_A    = 0x09U,
	C1_meter_Frame_B    = 0x0AU,
	C2_meter_Frame_A    = 0x0BU,
	C2_meter_Frame_B    = 0x0CU,
	C2_other_Frame_A    = 0x0DU,
	C2_other_Frame_B    = 0x0EU
};

/*!
 * @brief Mipot module 3200150xA avaliable rf channels (Used only in R2 mode)
 */
enum a_frequency_channel_t {
	ch0__868_030_MHz  = 0x00U,
	ch1__868_090_MHz  = 0x01U,
	ch2__868_150_MHz  = 0x02U,
	ch3__868_210_MHz  = 0x03U,
	ch4__868_270_MHz  = 0x04U,
	ch5__868_330_MHz  = 0x05U,
	ch6__868_390_MHz  = 0x06U,
	ch7__868_450_MHz  = 0x07U,
	ch8__868_510_MHz  = 0x08U,
	ch9__868_570_MHz  = 0x09U,
};

/*!
 * @brief Mipot module 3200150xA avaliable power levels
 */
enum a_power_t {
	power_level_0dBm   = 0x00U,
	power_level_5dBm   = 0x01U,
	power_level_7dBm   = 0x02U,
	power_level_10dBm  = 0x03U,
	power_level_12dBm  = 0x04U,
};

/*!
 * @brief Mipot module 3200150xA RF Autosleep mode
 */
enum a_rfautosleep_t {
	autosleep_disable = 0x00U,
	autosleep_enable  = 0x01U
};

/*!
 * @brief Enable management of WM-Bus from from module
 */
enum a_block1_t {
	block1_from_module_disable  = 0x00U,
	block1_from_module_enable   = 0x01U
};

/*!
 * @brief Mipot module 3200150xA avaliable rssi modes
 */
enum a_rssi_t {
	rssi_disable = 0x00U,
	rssi_enable  = 0x01U
};

/*!
 * @brief Mipot module 3200150xA command option (RAM-FLASH)
 */
enum a_mem_t {
	mem_type_RAM     = 0x00U, /* Setup will be lost at POR */
	mem_type_EEPROM  = 0xFFU  /* Setup will be save in EEPROM memory */
};

/*!
 * @brief Mipot module 3200150xD radio parameters
 */
struct a_radio_phy_t
{
	/* WM-Bus Mode */
    enum a_wmbusmode_t wmbusmode;

    /* RF Channel (Used only in R2 mode) */
    enum a_frequency_channel_t frequency_channel;

    /* Radio Frenquency output power */
    enum a_power_t power;

    /* Configure sleep */
    enum a_rfautosleep_t rfautosleep;

    /* RX Window (ms) */
    uint8_t rx_window;
};

/*!
 * @brief Mipot module 3200150xD module parameters
 */
struct a_module_param_t
{
	/* Enable management of WM-Bus from Module */
	enum a_block1_t block1;

	/* Enable RSSI Indication in communication frame */
	enum a_rssi_t rssi;

	/* Timeout in ms */
	uint8_t ndata_indicate_timeout;

    /* Uart baudrate selection */
    enum UartBaudrate_t UartBaudrate;
};

/*!
 * @brief Mipot module 3200150xD medium access parameters
 */
struct a_medium_access_param_t
{
	/* C Field */
	uint8_t  c_field;

	/* Manufacturer ID */
	uint16_t manufacturer_id;

	/* Device ID */
	uint32_t device_id;

	/* Version */
	uint8_t version;

	/* Device Type */
	uint8_t device_type;
};

/*!
 * @brief Structure containing the received data
 */
struct a_rx_data_t {
    uint8_t Block1[9];
    /* Received message info */
    uint8_t  last_rx_msg[350];
	uint16_t last_rx_msg_len;
};

struct mip_a
{
    uint32_t fw_version;
    uint32_t serialno;
    uint8_t rssi;
    struct a_radio_phy_t radio_phy;
    struct a_module_param_t module_param;
    struct a_medium_access_param_t medium_access_param;
    struct a_rx_data_t rx_data;
    /* porting function pointers */
    void (*hardware_init_fn) (enum UartBaudrate_t baudrate);
    enum mip_error_t (*send_and_receive_fn) (uint8_t *tx_buff, uint16_t tx_dim, uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms);
    enum mip_error_t (*receive_fn) (uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms);
    void (*hardware_reset_fn) (void);
    void (*delay_ms_fn) (uint32_t ms);
};

#endif /* MIP_A_DEF_H_ */
