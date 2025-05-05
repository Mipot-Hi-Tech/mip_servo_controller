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

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "mip_c.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MIPC_BUFF_SZ   260U
#define MIPC_DELAY_MSG 50U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t mipc_tx_buff[MIPC_BUFF_SZ] = {0};
uint8_t mipc_rx_buff[MIPC_BUFF_SZ] = {0};

/*******************************************************************************
 * Extern
 ******************************************************************************/
 
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Validate the device pointer for null conditions.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_null_ptr_check(const struct mip_c *const dev)
{
	enum mip_error_t retval = no_error;
    if ( (dev == NULL) ||
    	 (dev->hardware_init_fn == NULL) ||
    	 (dev->send_and_receive_fn == NULL) ||
		 (dev->receive_fn == NULL) ||
		 (dev->delay_ms_fn == NULL) ||
		 (dev->hardware_reset_fn == NULL) )
    {
        retval = null_ptr;
    }
    return retval;
}

/*!
 * @brief This command performs a module Reset.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_reset(const struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIP_RESET_CMD;
	mipc_tx_buff[2] = 0x00;
	mipc_tx_buff[3] = mip_generate_checksum(mipc_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 4, mipc_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIP_DELAY_RESET);
	return retval;
}

/*!
 * @brief This command restores EEPROM factory default values.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_factory_reset(const struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIP_FACTORY_RESET_CMD;
	mipc_tx_buff[2] = 0x00;
	mipc_tx_buff[3] = mip_generate_checksum(mipc_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 4, mipc_rx_buff, &rx_msg_len, MIP_DELAY_FACTORY_RESET);
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief Get 32-bit firmware version.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_get_fw_version(struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIP_GET_FW_VERSION_CMD;
	mipc_tx_buff[2] = 0x00;
	mipc_tx_buff[3] = mip_generate_checksum(mipc_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 4, mipc_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			dev->fw_version =  ((uint32_t) mipc_rx_buff[6] << 24); /* MSB */
			dev->fw_version |= ((uint32_t) mipc_rx_buff[5] << 16);
			dev->fw_version |= ((uint32_t) mipc_rx_buff[4] << 8);
			dev->fw_version |= ((uint32_t) mipc_rx_buff[3] );      /* LSB */
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief Get unique 32-bit Serial Number.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_get_serial_no(struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIP_GET_SERIALNO_CMD;
	mipc_tx_buff[2] = 0x00;
	mipc_tx_buff[3] = mip_generate_checksum(mipc_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 4, mipc_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			dev->serialno  = ((uint32_t) mipc_rx_buff[6] << 24); /* MSB */
			dev->serialno |= ((uint32_t) mipc_rx_buff[5] << 16);
			dev->serialno |= ((uint32_t) mipc_rx_buff[4] << 8);
			dev->serialno |= ((uint32_t) mipc_rx_buff[3] );      /* LSB */
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs an EEPROM data write.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_eeprom_write_stack_parameters(const struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0]  = MIP_HEADER;
	mipc_tx_buff[1]  = MIP_EEPROM_WRITE_CMD;
	mipc_tx_buff[2]  = 0x0A; /* lenght */
	mipc_tx_buff[3]  = MIPC_DEVICE_TYPE_ADDR; /* start address */
	mipc_tx_buff[4]  = dev->stack_param.device_type;
	mipc_tx_buff[5]  = dev->stack_param.UnconfirmedTxNumber;
	mipc_tx_buff[6]  = dev->stack_param.ConfirmedTxNumber;
	mipc_tx_buff[7]  = dev->stack_param.EndNodePairingReqPayload;
	mipc_tx_buff[8]  = dev->stack_param.EndNodePairingMstAddress[0];
	mipc_tx_buff[9]  = dev->stack_param.EndNodePairingMstAddress[1];
	mipc_tx_buff[10] = dev->stack_param.EndNodePairingMstAddress[2];
	mipc_tx_buff[11] = dev->stack_param.EndNodePairingMstAddress[3];
	mipc_tx_buff[12] = dev->stack_param.EndNodeMstTblIdx;
	mipc_tx_buff[13] = mip_generate_checksum(mipc_tx_buff, 13);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 14, mipc_rx_buff, &rx_msg_len, MIP_EEPROM_WRITE_TIMEOUT);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipc_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = invalid_address;
			}
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command reads EEPROM data.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_eeprom_read_stack_parameters(const struct mip_c *const dev, struct c_stack_param_t *const config)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIP_EEPROM_READ_CMD;
	mipc_tx_buff[2] = 0x02;
	mipc_tx_buff[3] = MIPC_DEVICE_TYPE_ADDR; /* Start Address */
	mipc_tx_buff[4] = 0x09; /* Number of bytes */
	mipc_tx_buff[5] = mip_generate_checksum(mipc_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 6, mipc_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipc_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
			else
			{
				config->device_type                 = mipc_rx_buff[4];
				config->UnconfirmedTxNumber         = mipc_rx_buff[5];
				config->ConfirmedTxNumber           = mipc_rx_buff[6];
				config->EndNodePairingReqPayload    = mipc_rx_buff[7];
				config->EndNodePairingMstAddress[0] = mipc_rx_buff[8];
				config->EndNodePairingMstAddress[1] = mipc_rx_buff[9];
				config->EndNodePairingMstAddress[2] = mipc_rx_buff[10];
				config->EndNodePairingMstAddress[3] = mipc_rx_buff[11];
				config->EndNodeMstTblIdx            = mipc_rx_buff[12];
			}
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs an EEPROM data write.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_eeprom_write_radio_phy_param(const struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIP_EEPROM_WRITE_CMD;
	mipc_tx_buff[2] = 0x04; /* lenght */
	mipc_tx_buff[3] = MIPC_POWER_ADDR; /* start address */
	mipc_tx_buff[4] = dev->radio_phy_param.power;
	mipc_tx_buff[5] = dev->radio_phy_param.frequency_channel;
	mipc_tx_buff[6] = dev->radio_phy_param.RSSI_Th;
	mipc_tx_buff[7] = mip_generate_checksum(mipc_tx_buff, 7);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 8, mipc_rx_buff, &rx_msg_len, MIP_EEPROM_WRITE_TIMEOUT);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipc_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = invalid_address;
			}
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command reads EEPROM data.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_eeprom_read_radio_phy_param(const struct mip_c *const dev, struct c_radio_phy_t *const config)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIP_EEPROM_READ_CMD;
	mipc_tx_buff[2] = 0x02;
	mipc_tx_buff[3] = MIPC_POWER_ADDR; /* Start Address */
	mipc_tx_buff[4] = 0x03; /* Number of bytes */
	mipc_tx_buff[5] = mip_generate_checksum(mipc_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 6, mipc_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipc_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
			else
			{
				config->power             = mipc_rx_buff[4];
				config->frequency_channel = mipc_rx_buff[5];
				config->RSSI_Th           = mipc_rx_buff[6];
			}
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs an EEPROM data write.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_eeprom_write_module_parameters(const struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIP_EEPROM_WRITE_CMD;
	mipc_tx_buff[2] = 0x04; /* lenght */
	mipc_tx_buff[3] = MIPC_DATA_INDICATE_TIMEOUT_ADDR; /* start address */
	mipc_tx_buff[4] = dev->module_param.DATA_INDICATE_TIMEOUT;
	mipc_tx_buff[5] = dev->module_param.UartBaudrate;
	mipc_tx_buff[6] = dev->module_param.AppEnAes;
	mipc_tx_buff[7] = mip_generate_checksum(mipc_tx_buff, 7);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 8, mipc_rx_buff, &rx_msg_len, MIP_EEPROM_WRITE_TIMEOUT);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipc_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = invalid_address;
			}
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command reads EEPROM data.
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_eeprom_read_module_parameters(const struct mip_c *const dev, struct c_module_param_t *const config)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIP_EEPROM_READ_CMD;
	mipc_tx_buff[2] = 0x02;
	mipc_tx_buff[3] = MIPC_DATA_INDICATE_TIMEOUT_ADDR; /* Start Address */
	mipc_tx_buff[4] = 0x03; /* Number of bytes */
	mipc_tx_buff[5] = mip_generate_checksum(mipc_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 6, mipc_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipc_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
			else
			{
				config->DATA_INDICATE_TIMEOUT = mipc_rx_buff[4];
				config->UartBaudrate          = mipc_rx_buff[5];
				config->AppEnAes              = mipc_rx_buff[6];
			}
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs an EEPROM data write.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_enable_pairing(const struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	if (dev->stack_param.device_type != master)
		return operation_not_supported;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIPC_ENABLE_PAIRING_CMD;
	mipc_tx_buff[2] = 0x01;
	mipc_tx_buff[3] = 0x01;
	mipc_tx_buff[4] = mip_generate_checksum(mipc_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 5, mipc_rx_buff, &rx_msg_len, 10);
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief Gets the module activation retval.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_get_activation_status(struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	if (dev->stack_param.device_type != end_node)
		return operation_not_supported;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIPC_GET_ACTIVATION_STATUS_CMD;
	mipc_tx_buff[2] = 0x00;
	mipc_tx_buff[3] = mip_generate_checksum(mipc_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 4, mipc_rx_buff, &rx_msg_len, 10);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipc_rx_buff[3] == 0x01)
			{
				dev->end_node_data.is_paired = true;
				dev->end_node_data.master_serial_number[3] = mipc_rx_buff[7];
				dev->end_node_data.master_serial_number[2] = mipc_rx_buff[6];
				dev->end_node_data.master_serial_number[1] = mipc_rx_buff[5];
				dev->end_node_data.master_serial_number[0] = mipc_rx_buff[4];
			}
			else
				retval = device_not_activated;
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief Requests a pairing to a network.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_pairing_req_cmd(struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	if (dev->stack_param.device_type != end_node)
		return operation_not_supported;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIPC_PAIRING_REQ_CMD;
	mipc_tx_buff[2] = 0x00;
	mipc_tx_buff[3] = mip_generate_checksum(mipc_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 4, mipc_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if(mipc_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = device_busy;
			}
		}
	}
	if(retval != no_error)
		return retval;
	/* PAIRING_CONFIRM_IND message is coming from module */
	retval = mipc_handle_IND_messages(dev, 20000);
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs the transmission of radio frames.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_tx_msg_cmd(c_messageTrasmission_t tx_typ, uint32_t dest_id, const uint8_t *msg, uint8_t msg_len, struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	uint8_t idx = 0;
	uint8_t i,crc;
	if ( (msg_len == 0) || (msg_len > 26) )
		return tx_message_len_error;
	mipc_tx_buff[idx++] = MIP_HEADER;
	mipc_tx_buff[idx++] = MIPC_TX_MSG_CMD;
	mipc_tx_buff[idx++] = 5 + msg_len;  /* lenght */
	mipc_tx_buff[idx++] = tx_typ;       /* options */
	mipc_tx_buff[idx++] = (uint8_t)(dest_id);
	mipc_tx_buff[idx++] = (uint8_t)(dest_id >> 8);
	mipc_tx_buff[idx++] = (uint8_t)(dest_id >> 16);
	mipc_tx_buff[idx++] = (uint8_t)(dest_id >> 24);
	for(i=0; i<msg_len; i++)
	{
		mipc_tx_buff[idx++] = msg[i];
	}
	crc = mip_generate_checksum(mipc_tx_buff, idx);
	mipc_tx_buff[idx++] = crc;
	retval = dev->send_and_receive_fn(mipc_tx_buff, idx, mipc_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if(mipc_rx_buff[3] != no_error)
			{
				if(mipc_rx_buff[3] == 0x01)
					retval = device_busy;
				else if(mipc_rx_buff[3] == 0x02)
					retval = device_not_activated;
				else if(mipc_rx_buff[3] == 0x03)
					retval = payload_size_error;
				else
					retval = unknown_error;
				return retval;
			}
		}
	}
	if(retval != no_error)
		return retval;
	/* MIPC_TX_MSG_CONFIRMED_IND or MIPC_TX_MSG_IND is coming from the module */
	retval = mipc_handle_IND_messages(dev, 2000);
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief Get from MASTER module the size of Network Table.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_get_network_table_size(struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	if (dev->stack_param.device_type != master)
		return operation_not_supported;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIPC_GET_NETWORK_TABLE_SIZE_CMD;
	mipc_tx_buff[2] = 0x00;
	mipc_tx_buff[3] = mip_generate_checksum(mipc_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 4, mipc_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			dev->master_data.ROUTING_SIZE = mipc_rx_buff[3];
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief Returns a Network Table row from index.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_get_network_row_size(uint8_t idx, struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	if (dev->stack_param.device_type != master)
		return operation_not_supported;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIPC_GET_NETWORK_TABLE_ROW_CMD;
	mipc_tx_buff[2] = 0x01;
	mipc_tx_buff[3] = idx;
	mipc_tx_buff[4] = mip_generate_checksum(mipc_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 5, mipc_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			dev->master_data.enode[idx].enode_serial_number  = ((uint32_t) mipc_rx_buff[6] << 24);
			dev->master_data.enode[idx].enode_serial_number |= ((uint32_t) mipc_rx_buff[5] << 16);
			dev->master_data.enode[idx].enode_serial_number |= ((uint32_t) mipc_rx_buff[4] << 8);
			dev->master_data.enode[idx].enode_serial_number |= ((uint32_t) mipc_rx_buff[3] );
			dev->master_data.enode[idx].custom_pairing_payload_byte = mipc_rx_buff[7];
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command enable the NDATA_INDICATE pin of the mip module
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_enable_ndata_indicate_pin(struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = 0x32;
	mipc_tx_buff[2] = 0x02;
	mipc_tx_buff[3] = 0x91;
	mipc_tx_buff[4] = 0x01;
	mipc_tx_buff[5] = mip_generate_checksum(mipc_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 6, mipc_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command initiates from END NODE a link check procedure between END NODE and MASTER.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_link_check_request(struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	if (dev->stack_param.device_type != end_node)
		return operation_not_supported;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIPC_LINK_CHECK_REQ_CMD;
	mipc_tx_buff[2] = 0x03;
	mipc_tx_buff[3] = dev->link_check.power;
	mipc_tx_buff[4] = dev->link_check.MessageNumber;
	mipc_tx_buff[5] = dev->link_check.MessageTh;
	mipc_tx_buff[6] = mip_generate_checksum(mipc_tx_buff, 6);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 7, mipc_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if(mipc_rx_buff[3] != no_error)
			{
				if(mipc_rx_buff[3] == 0x01)
					retval = device_busy;
				else if(mipc_rx_buff[3] == 0x02)
					retval = parameter_out_of_range;
				else
					retval = unknown_error;
			}
		}
	}
	if(retval != no_error)
		return retval;
	/* MIPC_LINK_CHECK_ANS_IND message is coming from module */
	retval = mipc_handle_IND_messages(dev, 20000);
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief Deletes a Network Table row from module Serial Number.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_delete_en_device(uint8_t idx, struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	if (dev->stack_param.device_type != master)
		return operation_not_supported;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIPC_DEL_END_DEVICE_CMD;
	mipc_tx_buff[2] = 0x00;
	mipc_tx_buff[3] = mip_generate_checksum(mipc_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 4, mipc_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipc_rx_buff[3] != MIP_RET_SUCCESS)
				retval = failure;
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief Deletes whole Network Table.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_delete_all_en_device(struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	uint8_t i;
	if (dev->stack_param.device_type != master)
		return operation_not_supported;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIPC_DEL_ALL_EN_DEVICE_CMD;
	mipc_tx_buff[2] = 0x00;
	mipc_tx_buff[3] = mip_generate_checksum(mipc_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 4, mipc_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipc_rx_buff[3] != MIP_RET_SUCCESS)
				retval = failure;
			else
			{
				for(i=0; i<MIPC_ENODE_NUM_MAX; i++)
				{
					dev->master_data.enode[i].custom_pairing_payload_byte = 0;
					dev->master_data.enode[i].enode_serial_number         = 0;
				}
				dev->master_data.ROUTING_SIZE = 0;
			}
		}
	}
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return retval;
}

/*!
 * @brief After this command, the module enters sleep mode.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval Success.
 */
enum mip_error_t mipc_set_sleep(const struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIP_SLEEP_CMD;
	mipc_tx_buff[2] = 0x01;
	mipc_tx_buff[3] = 0x00;
	mipc_tx_buff[4] = mip_generate_checksum(mipc_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 5, mipc_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return no_error;
}

/*!
 * @brief After this command, the module exits sleep mode.
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval Success.
 */
enum mip_error_t mipc_wakeup(const struct mip_c *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipc_tx_buff[0] = MIP_HEADER;
	mipc_tx_buff[1] = MIP_SLEEP_CMD;
	mipc_tx_buff[2] = 0x01;
	mipc_tx_buff[3] = 0x01;
	mipc_tx_buff[4] = mip_generate_checksum(mipc_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipc_tx_buff, 5, mipc_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIPC_DELAY_MSG);
	return no_error;
}

/*!
 * @brief Handle all possible IND command, coming from the module
 * @param[in] dev : Structure instance of mip_c.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipc_handle_IND_messages(struct mip_c *const dev, uint32_t timeout)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	uint32_t enode_sn;
	uint8_t payload_len;
	uint8_t i;
	retval = dev->receive_fn(mipc_rx_buff, &rx_msg_len, timeout);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipc_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			switch(mipc_rx_buff[1])
			{
				case MIPC_PAIRING_CONFIRM_IND:
				{
					if(mipc_rx_buff[3] == no_error)
					{
						dev->end_node_data.is_paired = true;
						dev->end_node_data.master_serial_number[3] = mipc_rx_buff[4];
						dev->end_node_data.master_serial_number[2] = mipc_rx_buff[5];
						dev->end_node_data.master_serial_number[1] = mipc_rx_buff[6];
						dev->end_node_data.master_serial_number[0] = mipc_rx_buff[7];
						dev->end_node_data.idx = mipc_rx_buff[8];
					}
					else
					{
						dev->end_node_data.is_paired = false;
						dev->end_node_data.master_serial_number[3] = 0;
						dev->end_node_data.master_serial_number[2] = 0;
						dev->end_node_data.master_serial_number[1] = 0;
						dev->end_node_data.master_serial_number[0] = 0;
						dev->end_node_data.idx = 0;
						retval = unknown_error;
					}
					break;
				}

				case MIPC_RX_MSG_IND:
				{
					dev->rx_data.RssiLSB   = mipc_rx_buff[4];
					dev->rx_data.RssiMSB   = mipc_rx_buff[5];
					dev->rx_data.SNR       = mipc_rx_buff[6];
					dev->rx_data.SRC_ID[0] = mipc_rx_buff[7];
					dev->rx_data.SRC_ID[1] = mipc_rx_buff[8];
					dev->rx_data.SRC_ID[2] = mipc_rx_buff[9];
					dev->rx_data.SRC_ID[3] = mipc_rx_buff[10];
					payload_len = mipc_rx_buff[2] - 8;
					for(i=0; i<payload_len; i++)
					{
						dev->rx_data.last_rx_msg[i] = mipc_rx_buff[11+i];
					}
					dev->rx_data.last_rx_msg_len = payload_len;
					dev->rx_data.rx_msg_num++;
					break;
				}

				case MIPC_DEVICE_PAIRING_IND:
				{
					enode_sn  = ((uint32_t) mipc_rx_buff[6] << 24);
					enode_sn |= ((uint32_t) mipc_rx_buff[5] << 16);
					enode_sn |= ((uint32_t) mipc_rx_buff[4] << 8);
					enode_sn |= ((uint32_t) mipc_rx_buff[3] );
					for(i=0; i<MIPC_ENODE_NUM_MAX; i++)
					{
						if(dev->master_data.enode[i].enode_serial_number == 0)
							break;
					}
					dev->master_data.enode[i].enode_serial_number         = enode_sn;
					dev->master_data.enode[i].custom_pairing_payload_byte = mipc_rx_buff[7];
					dev->master_data.ROUTING_SIZE++;
					break;
				}

				case MIPC_TX_MSG_CONFIRMED_IND:
				{
					if(mipc_rx_buff[3] == no_error)
					{
						dev->confirmed_msg_ind.SessionTxTime  = ((uint32_t) mipc_rx_buff[7]  << 24);
						dev->confirmed_msg_ind.SessionTxTime |= ((uint32_t) mipc_rx_buff[6]  << 16);
						dev->confirmed_msg_ind.SessionTxTime |= ((uint32_t) mipc_rx_buff[5]  << 8 );
						dev->confirmed_msg_ind.SessionTxTime |= ((uint32_t) mipc_rx_buff[4]       );
						dev->confirmed_msg_ind.AckReceived    = mipc_rx_buff[8];
						dev->confirmed_msg_ind.NbRetries      = mipc_rx_buff[9];
					}
					break;
				}

				case MIPC_TX_MSG_IND:
				{
					if(mipc_rx_buff[3] == no_error)
					{
						dev->unconfirmed_msg_ind.SessionTxTime =  ((uint32_t) mipc_rx_buff[7] << 24);
						dev->unconfirmed_msg_ind.SessionTxTime |= ((uint32_t) mipc_rx_buff[6] << 16);
						dev->unconfirmed_msg_ind.SessionTxTime |= ((uint32_t) mipc_rx_buff[5] << 8);
						dev->unconfirmed_msg_ind.SessionTxTime |= ((uint32_t) mipc_rx_buff[4] );
					}
					break;
				}

				case MIPC_LINK_CHECK_ANS_IND:
				{
					if(mipc_rx_buff[3] == 0x00)
						dev->link_check.result = LinkCheckKO;
					else
						dev->link_check.result = LinkCheckOK;
					dev->link_check.MessageReceived = mipc_rx_buff[4];
					break;
				}

				case MIPC_TX_SESSION_ABORT_IND:
				{
					dev->end_node_data.SessionTxTime =  ((uint32_t) mipc_rx_buff[6] << 24);
					dev->end_node_data.SessionTxTime |= ((uint32_t) mipc_rx_buff[5] << 16);
					dev->end_node_data.SessionTxTime |= ((uint32_t) mipc_rx_buff[4] << 8);
					dev->end_node_data.SessionTxTime |= ((uint32_t) mipc_rx_buff[3] );
					break;
				}

				default:
				{
					retval = unknown_error;
					break;
				}
			}
		}
	}
	return retval;
}
