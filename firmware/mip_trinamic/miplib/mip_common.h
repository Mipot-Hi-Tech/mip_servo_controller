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

#ifndef MIP_COMMON_H_
#define MIP_COMMON_H_

#include <stdint.h>
#include <stddef.h>

/******************************************************************************/
/*! @name        Common Macro Definitions                */
/******************************************************************************/

#define MIPB_DEVICE_FW_VERSION     (0x10000000)
#define MIPC_DEVICE_FW_VERSION     (0x20000000)
#define MIPD_DEVICE_FW_VERSION     (0x00000000)
#define MIPA_DEVICE_FW_VERSION     (0x30000000)
#define MIPF_DEVICE_FW_VERSION     (0xA0000000)

#define MIP_RET_SUCCESS            (0x00U)
#define MIP_HEADER                 (0xAAU) /* A serial communication with MIP module, start always with a header */
#define MIP_RESET_CMD              (0x30U) /* Module software reset command */
#define MIP_FACTORY_RESET_CMD      (0x31U) /* Restore EEPROM to factory default values command  */
#define MIP_EEPROM_WRITE_CMD       (0x32U) /* Write EEPROM parameter command */
#define MIP_EEPROM_READ_CMD        (0x33U) /* Read EEPROM parameter command */
#define MIP_GET_FW_VERSION_CMD     (0x34U) /* Get firmware version command */
#define MIP_GET_SERIALNO_CMD       (0x35U) /* Get serial number command */
#define MIP_SLEEP_CMD              (0x70U) /* Enable/Disable sleep mode */

#define DATA_IND_TIMEOUT_MIN_VAL   (0x01U)
#define DATA_IND_TIMEOUT_MAX_VAL   (0x255U)
#define DATA_IND_TIMEOUT_DEF_VAL   (0x05U)

#define MIP_UART_TX_TIMEOUT         300U
#define MIP_UART_RX_TIMEOUT         50U
#define MIP_UART_END_OF_MSG_TIMEOUT 5U

#define MIP_DELAY_RESET             100U
#define MIP_DELAY_FACTORY_RESET     300U
#define MIP_EEPROM_WRITE_TIMEOUT    30U

/*!
 * @brief Mipot module avaliable uart baud rates
 */
enum UartBaudrate_t {
	UartBaudrate_9600   = 0x00U,
	UartBaudrate_19200  = 0x01U,
	UartBaudrate_38400  = 0x02U,
	UartBaudrate_57600  = 0x03U,
	UartBaudrate_115200 = 0x04U
};

/*!
 * @brief Mipot module error codes
 */
enum mip_error_t {
	no_error                 = 0x00U,
	tx_timeout               = 0x01U,
	rx_timeout               = 0x02U,
	invalid_address          = 0x03U,
	unknown_error            = 0x04U,
	failure                  = 0x05U,
	device_busy              = 0x06U,
	payload_size_error       = 0x07U,
	device_not_activated     = 0x08U,
	parameter_out_of_range   = 0x09U,
	operation_not_supported  = 0x0AU,
	crc_invalid              = 0x0BU,
	rx_message_len_error     = 0x0CU,
	tx_message_len_error     = 0x0DU,
	null_ptr                 = 0x0EU,
	End_Node_in_silent_state = 0x0FU,
	Channel_Blocked_by_dc    = 0x10U,
	data_outside_range       = 0x11U,
	LoRaMac_notin_idle_state = 0x12U,
	invalid_parameter        = 0x13U,
	port_num_not_supported   = 0x14U,
	lenght_not_supported     = 0x15U,
	error                    = 0x16U
};

/*******************************************************************************
 * API
 ******************************************************************************/
uint8_t mip_generate_checksum(const uint8_t *buff, uint16_t len);
enum mip_error_t mip_validate_response(const uint8_t *buff, uint16_t len);

#endif
