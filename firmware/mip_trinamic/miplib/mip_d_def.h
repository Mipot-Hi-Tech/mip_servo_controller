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

#ifndef MIP_D_DEF_H_
#define MIP_D_DEF_H_

#include <stdint.h>
#include <stdbool.h>
#include "mip_common.h"

/******************************************************************************/
/*! @name        D Series Macro Definitions                                   */
/******************************************************************************/

#define MIPD_TX_MSG_CMD                 (0x50U) /* This command performs the transmission of a radio frame. */
#define MIPD_TX_MSG_IND                 (0x52U) /* This command indicates the end of a transmission session. */
#define MIPD_RX_MSG_IND                 (0x53U) /* This command indicates the reception of radio frames. */
#define MIPD_SET_AES_KEY_CMD            (0x58U) /* This command performs an EEPROM data write. */
#define MIPD_SET_IV_CMD                 (0x59U) /* This command performs an EEPROM data write. */

/*!
 * @brief mipot module 3200150xD data address
 */
#define MIPD_POWER_ADDR                 (0x00U)
#define MIPD_FREQ_ADDR                  (0x01U)
#define MIPD_BANDWIDTH_ADDR             (0x02U)
#define MIPD_SPREADING_FACTOR_ADDR      (0x03U)
#define MIPD_CODE_RATE_ADDR             (0x04U)
#define MIPD_DATA_INDICATE_TIMEOUT_ADDR (0x05U)
#define MIPD_UART_BAUDRATE_ADDR         (0x06U)
#define MIPD_APP_ENABLE_AES_ADDR        (0x07U)

/*!
 * @brief mipd Stack Parameters default values
 */
#define MIPD_POWER_MIN_VAL               (0x02U)
#define MIPD_POWER_MAX_VAL               (0x0EU)
#define MIPD_POWER_DEF                   (0x0EU)

#define MIPD_CHANNEL_FREQUENCY_MIN_VAL   (0x00U)
#define MIPD_CHANNEL_FREQUENCY_MAX_VAL   (0x4AU)
#define MIPD_CHANNEL_FREQUENCY_DEF       (0x02U)

#define MIPD_CHANNEL_BANDWIDTH_MIN_VAL   (0x00U)
#define MIPD_CHANNEL_BANDWIDTH_MAX_VAL   (0x02U)
#define MIPD_CHANNEL_BANDWIDTH_DEF       (0x00U)

#define MIPD_SPREADING_FACTOR_MIN_VAL    (0x06U)
#define MIPD_SPREADING_FACTOR_MAX_VAL    (0x0CU)
#define MIPD_SPREADING_FACTOR_DEF        (0x0AU)

#define MIPD_CODE_RATE_MIN_VAL           (0x01U)
#define MIPD_CODE_RATE_MAX_VAL           (0x04U)
#define MIPD_CODE_RATE_DEF               (0x01U)

#define MIPD_UartBaudrate_MIN_VAL        (0x00U)
#define MIPD_UartBaudrate_MAX_VAL        (0x04U)
#define MIPD_UartBaudrate_DEF_VAL        (0x00U)

#define MIPD_AppEnAes_MIN_VAL            (0x00U)
#define MIPD_AppEnAes_MAX_VAL            (0x01U)
#define MIPD_AppEnAes_DEF_VAL            (0x00U)

/*!
 * @brief Mipot module 3200150xD avaliable power levels
 */
enum d_power_t {
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
 * @brief Mipot module 3200150xD avaliable tx-bandwidth
 */
enum d_bandwidth_t {
	TX_bandwidth_125_kHz = 0x00U,
	TX_bandwidth_250_kHz = 0x01U,
	TX_bandwidth_500_kHz = 0x02U,
};

/*!
 * @brief Mipot module 3200150xD avaliable spreading factor expressed in chips
 */
enum d_spreading_factor_t {
	SF_64_chips   = 0x06U,
	SF_128_chips  = 0x07U,
	SF_256_chips  = 0x08U,
	SF_512_chips  = 0x09U,
	SF_1024_chips = 0x0AU,
	SF_2048_chips = 0x0BU,
	SF_4096_chips = 0x0CU
};


/*!
 * @brief Mipot module 3200150xD avaliable code rates
 */
enum d_code_rate_t {
	code_rate__4_5  = 0x01U,
	code_rate__4_6  = 0x02U,
	code_rate__4_7  = 0x03U,
	code_rate__4_8  = 0x04U
};

/*!
 * @brief Mipot module 3200150xD Frequency index
 */
enum d_frequency_channel_t {
	ch0__868_075_MHz  = 0x00U,
	ch1__868_100_MHz  = 0x01U,
	ch2__868_125_MHz  = 0x02U,
	ch3__868_150_MHz  = 0x03U,
	ch4__868_175_MHz  = 0x04U,
	ch5__868_200_MHz  = 0x05U,
	ch6__868_225_MHz  = 0x06U,
	ch7__868_250_MHz  = 0x07U,
	ch8__868_275_MHz  = 0x08U,
	ch9__868_300_MHz  = 0x09U,
	ch10__868_325_MHz = 0x0AU,
	ch11__868_350_MHz = 0x0BU,
	ch12__868_375_MHz = 0x0CU,
	ch13__868_400_MHz = 0x0DU,
	ch14__868_425_MHz = 0x0EU,
	ch15__868_450_MHz = 0x0FU,
	ch16__868_475_MHz = 0x10U,
	ch17__868_500_MHz = 0x11U,
	ch18__868_525_MHz = 0x12U,
	ch19__868_550_MHz = 0x13U,
	ch20__868_575_MHz = 0x14U,
	ch21__868_600_MHz = 0x15U,
	ch22__868_625_MHz = 0x16U,
	ch23__868_650_MHz = 0x17U,
	ch24__868_675_MHz = 0x18U,
	ch25__868_700_MHz = 0x19U,
	ch26__868_725_MHz = 0x1AU,
	ch27__868_750_MHz = 0x1BU,
	ch28__868_775_MHz = 0x1CU,
	ch29__868_800_MHz = 0x1DU,
	ch30__868_825_MHz = 0x1EU,
	ch31__868_850_MHz = 0x1FU,
	ch32__868_875_MHz = 0x20U,
	ch33__868_900_MHz = 0x21U,
	ch34__868_925_MHz = 0x22U,
	ch35__868_950_MHz = 0x23U,
	ch36__868_975_MHz = 0x24U,
	ch37__869_000_MHz = 0x25U,
	ch38__869_025_MHz = 0x26U,
	ch39__869_050_MHz = 0x27U,
	ch40__869_075_MHz = 0x28U,
	ch41__869_100_MHz = 0x29U,
	ch42__869_125_MHz = 0x2AU,
	ch43__869_150_MHz = 0x2BU,
	ch44__869_175_MHz = 0x2CU,
	ch45__869_200_MHz = 0x2DU,
	ch46__869_225_MHz = 0x2EU,
	ch47__869_250_MHz = 0x2FU,
	ch48__869_275_MHz = 0x30U,
	ch49__869_300_MHz = 0x31U,
	ch50__869_325_MHz = 0x32U,
	ch51__869_350_MHz = 0x33U,
	ch52__869_375_MHz = 0x34U,
	ch53__869_400_MHz = 0x35U,
	ch54__869_425_MHz = 0x36U,
	ch55__869_450_MHz = 0x37U,
	ch56__869_475_MHz = 0x38U,
	ch57__869_500_MHz = 0x39U,
	ch58__869_525_MHz = 0x3AU,
	ch59__869_550_MHz = 0x3BU,
	ch60__869_575_MHz = 0x3CU,
	ch61__869_600_MHz = 0x3DU,
	ch62__869_625_MHz = 0x3EU,
	ch63__869_650_MHz = 0x3FU,
	ch64__869_675_MHz = 0x40U,
	ch65__869_700_MHz = 0x41U,
	ch66__869_725_MHz = 0x42U,
	ch67__869_750_MHz = 0x43U,
	ch68__869_775_MHz = 0x44U,
	ch69__869_800_MHz = 0x45U,
	ch70__869_825_MHz = 0x46U,
	ch71__869_850_MHz = 0x47U,
	ch72__869_875_MHz = 0x48U,
	ch73__869_900_MHz = 0x49U,
	ch74__869_925_MHz = 0x4AU
};

/*!
 * @brief Mipot module 3200150xD Application AES Key Enable/Disable
 */
enum d_AppEnAes_t {
	AppEnAes_Disabled = 0x00U,
	AppEnAes_Enabled  = 0x01U
};

/*!
 * @brief Mipot module 3200150xD Radio Parameters
 */
struct d_radio_phy_t {
	/* Power expressed in dBm */
    enum d_power_t power;

    /* Channel frequency selection */
    enum d_frequency_channel_t frequency_channel;

    /* TX bandwidth */
    enum d_bandwidth_t bandwidth;

    /* Spreading factor expressed in chips */
    enum d_spreading_factor_t spreading_factor;

    /* Code rate */
    enum d_code_rate_t code_rate;
};

/*!
 * @brief Mipot module 3200150xD module parameters
 */
struct d_module_param_t {
	/* Timeout in ms */
    uint8_t DATA_INDICATE_TIMEOUT;

    /* Uart baudrate selection */
    enum UartBaudrate_t UartBaudrate;

    /* Application AES Key Enable/Disable */
    enum d_AppEnAes_t AppEnAes;
};

/*!
 * @brief Mipot module 3200150xD received data information
 */
struct d_rx_data_info_t{
    /* 16-bit Rssi Value expressed in dBm */
    uint8_t RssiLSB;
    uint8_t RssiMSB;
    /* 8-bit Signal-to-Noise Ratio */
    uint8_t SNR;
    /* Rx message information */
    uint8_t last_rx_msg[240];
	uint8_t last_rx_msg_len;
	uint32_t rx_msg_num;
};

/*!
 * @brief Mipot module 3200150xD transmitted data information
 */
struct d_tx_data_info_t{
	/* Time on Air, total time needed for the transmission, in ms. */
	uint32_t on_air_time;
	uint32_t tx_msg_num;
};

struct mip_d
{
    uint32_t fw_version;
    uint32_t serialno;
    uint8_t AESKey[16];
    uint8_t InitVector[16];
    struct d_radio_phy_t radio_phy;
    struct d_module_param_t module_param;
    struct d_rx_data_info_t rx_data_info;
    struct d_tx_data_info_t tx_data_info;
    /* porting function pointers */
    void (*hardware_init_fn) (enum UartBaudrate_t baudrate);
    enum mip_error_t (*send_and_receive_fn) (uint8_t *tx_buff, uint16_t tx_dim, uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms);
    enum mip_error_t (*receive_fn) (uint8_t *rx_buff, uint16_t *rx_dim, uint32_t timeout_ms);
    void (*hardware_reset_fn) (void);
    void (*delay_ms_fn) (uint32_t ms);
};

#endif
