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

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "mip_a.h"
#include "mip_a_def.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MIPA_BUFF_SZ 350
#define MIPA_DELAY_MSG 50

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t mipa_tx_buff[MIPA_BUFF_SZ] = {0};
uint8_t mipa_rx_buff[MIPA_BUFF_SZ] = {0};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Validate the device pointer for null conditions.
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_null_ptr_check(const struct mip_a *const dev)
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
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_reset(const struct mip_a *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIP_RESET_CMD;
	mipa_tx_buff[2] = 0x00;
	mipa_tx_buff[3] = mip_generate_checksum(mipa_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 4, mipa_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIP_DELAY_RESET);
	return retval;
}

/*!
 * @brief This command restores EEPROM factory default values.
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_factory_reset(const struct mip_a *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIP_FACTORY_RESET_CMD;
	mipa_tx_buff[2] = 0x00;
	mipa_tx_buff[3] = mip_generate_checksum(mipa_tx_buff, 3);
	/* After a factory reset cmd response from mipa is coming after 140 ms */
	retval = dev->send_and_receive_fn(mipa_tx_buff, 4, mipa_rx_buff, &rx_msg_len, 200);
	dev->delay_ms_fn(MIP_DELAY_FACTORY_RESET);
	return retval;
}

/*!
 * @brief Get 32-bit firmware version.
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_get_fw_version(struct mip_a *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIP_GET_FW_VERSION_CMD;
	mipa_tx_buff[2] = 0x00;
	mipa_tx_buff[3] = mip_generate_checksum(mipa_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 4, mipa_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			dev->fw_version =  ((uint32_t) mipa_rx_buff[6] << 24);  /* MSB */
			dev->fw_version |= ((uint32_t) mipa_rx_buff[5] << 16);
			dev->fw_version |= ((uint32_t) mipa_rx_buff[4] << 8);
			dev->fw_version |= ((uint32_t) mipa_rx_buff[3] );       /* LSB */
		}
	}
	dev->delay_ms_fn(MIPA_DELAY_MSG);
	return retval;
}

/*!
 * @brief Get unique 32-bit Serial Number.
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_get_serial_no(struct mip_a *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIP_GET_SERIALNO_CMD;
	mipa_tx_buff[2] = 0x00;
	mipa_tx_buff[3] = mip_generate_checksum(mipa_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 4, mipa_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			dev->serialno  = ((uint32_t) mipa_rx_buff[6] << 24);  /* MSB */
			dev->serialno |= ((uint32_t) mipa_rx_buff[5] << 16);
			dev->serialno |= ((uint32_t) mipa_rx_buff[4] << 8);
			dev->serialno |= ((uint32_t) mipa_rx_buff[3] );       /* LSB */
		}
	}
	dev->delay_ms_fn(MIPA_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command gets the rssi of the last received message.
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_get_rssi(struct mip_a *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIPA_GET_RSSI_CMD;
	mipa_tx_buff[2] = 0x00;
	mipa_tx_buff[3] = mip_generate_checksum(mipa_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 4, mipa_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			dev->rssi = mipa_rx_buff[3];
		}
	}
	dev->delay_ms_fn(MIPA_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command changes the wmbus mode of the module.
 * @param[in] dev         : Structure instance of mip_a.
 * @param[in] wmbus_mode  : The wmbus mode to setup.
 * @param[in] memory_area : The memory area.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_set_mode(struct mip_a *const dev, enum a_wmbusmode_t wmbus_mode, enum a_mem_t memory_area)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIPA_SET_MODE_CMD;
	mipa_tx_buff[2] = 0x02;
	mipa_tx_buff[3] = memory_area;
	mipa_tx_buff[4] = wmbus_mode;
	mipa_tx_buff[5] = mip_generate_checksum(mipa_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 6, mipa_rx_buff, &rx_msg_len, MIP_EEPROM_WRITE_TIMEOUT);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval != no_error)
		{
			retval = failure;
		}
	}
	dev->delay_ms_fn(MIPA_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command enable the NDATA_INDICATE pin of the mip module
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_enable_ndata_indicate_pin(const struct mip_a *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = 0x32;
	mipa_tx_buff[2] = 0x02;
	mipa_tx_buff[3] = 0x91;
	mipa_tx_buff[4] = 0x01;
	mipa_tx_buff[5] = mip_generate_checksum(mipa_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 6, mipa_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIPA_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs an EEPROM data write.
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_eeprom_write_radio_phy_param(const struct mip_a *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIP_EEPROM_WRITE_CMD;
	mipa_tx_buff[2] = 0x06;                  /* lenght */
	mipa_tx_buff[3] = MIPA_WM_BUS_MODE_ADDR; /* start address */
	mipa_tx_buff[4] = dev->radio_phy.wmbusmode;
	mipa_tx_buff[5] = dev->radio_phy.frequency_channel;
	mipa_tx_buff[6] = dev->radio_phy.power;
	mipa_tx_buff[7] = dev->radio_phy.rfautosleep;
	mipa_tx_buff[8] = dev->radio_phy.rx_window;
	mipa_tx_buff[9] = mip_generate_checksum(mipa_tx_buff, 9);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 10, mipa_rx_buff, &rx_msg_len, MIP_EEPROM_WRITE_TIMEOUT);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipa_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
		}
	}
	dev->delay_ms_fn(MIPA_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command reads EEPROM data.
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_eeprom_read_radio_phy_param(const struct mip_a *const dev, struct a_radio_phy_t *const config)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIP_EEPROM_READ_CMD;
	mipa_tx_buff[2] = 0x02;
	mipa_tx_buff[3] = MIPA_WM_BUS_MODE_ADDR; /* Start Address */
	mipa_tx_buff[4] = 0x05;                  /* Number of bytes */
	mipa_tx_buff[5] = mip_generate_checksum(mipa_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 6, mipa_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipa_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
			else
			{
				config->wmbusmode         = mipa_rx_buff[4];
				config->frequency_channel = mipa_rx_buff[5];
				config->power             = mipa_rx_buff[6];
				config->rfautosleep       = mipa_rx_buff[7];
				config->rx_window         = mipa_rx_buff[8];
			}
		}
	}
	dev->delay_ms_fn(MIPA_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs an EEPROM data write.
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_eeprom_write_module_parameters(const struct mip_a *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIP_EEPROM_WRITE_CMD;
	mipa_tx_buff[2] = 0x04;                         /* lenght */
	mipa_tx_buff[3] = MIPA_BLOCK1_FROM_MODULE_ADDR; /* start address */
	mipa_tx_buff[4] = dev->module_param.block1;
	mipa_tx_buff[5] = dev->module_param.rssi;
	mipa_tx_buff[6] = dev->module_param.ndata_indicate_timeout;
	mipa_tx_buff[7] = mip_generate_checksum(mipa_tx_buff, 7);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 8, mipa_rx_buff, &rx_msg_len, MIP_EEPROM_WRITE_TIMEOUT);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipa_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
		}
	}
	if (no_error != retval)
		return retval;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIP_EEPROM_WRITE_CMD;
	mipa_tx_buff[2] = 0x02;                    /* lenght */
	mipa_tx_buff[3] = MIPA_UART_BAUDRATE_ADDR; /* start address */
	mipa_tx_buff[4] = dev->module_param.UartBaudrate;
	mipa_tx_buff[5] = mip_generate_checksum(mipa_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 6, mipa_rx_buff, &rx_msg_len, MIP_EEPROM_WRITE_TIMEOUT);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipa_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
		}
	}
	dev->delay_ms_fn(MIPA_DELAY_MSG);
	return retval;
}

/*!
 * @brief Get the module parameters from device EEPROM.
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_eeprom_read_module_parameters(const struct mip_a *const dev, struct a_module_param_t *const config)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIP_EEPROM_READ_CMD;
	mipa_tx_buff[2] = 0x02;
	mipa_tx_buff[3] = MIPA_BLOCK1_FROM_MODULE_ADDR; /* Start Address */
	mipa_tx_buff[4] = 0x03;                         /* Number of bytes */
	mipa_tx_buff[5] = mip_generate_checksum(mipa_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 6, mipa_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipa_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
			else
			{
				config->block1                 = mipa_rx_buff[4];
				config->rssi                   = mipa_rx_buff[5];
				config->ndata_indicate_timeout = mipa_rx_buff[6];
			}
		}
	}
	if(no_error != retval)
		return retval;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIP_EEPROM_READ_CMD;
	mipa_tx_buff[2] = 0x02;
	mipa_tx_buff[3] = MIPA_UART_BAUDRATE_ADDR; /* Start Address */
	mipa_tx_buff[4] = 0x01;                    /* Number of bytes */
	mipa_tx_buff[5] = mip_generate_checksum(mipa_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 6, mipa_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipa_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
			else
			{
				config->UartBaudrate = mipa_rx_buff[4];
			}
		}
	}
	dev->delay_ms_fn(MIPA_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs an EEPROM data write.
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_eeprom_write_wmbus_medium_access_parameters(const struct mip_a *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0]  = MIP_HEADER;
	mipa_tx_buff[1]  = MIP_EEPROM_WRITE_CMD;
	mipa_tx_buff[2]  = 0x0A; /* lenght */
	mipa_tx_buff[3]  = 0x10; /* start address */
	mipa_tx_buff[4]  = dev->medium_access_param.c_field;
	mipa_tx_buff[5]  = (uint8_t) (dev->medium_access_param.manufacturer_id );
	mipa_tx_buff[6]  = (uint8_t) (dev->medium_access_param.manufacturer_id >> 8);
	mipa_tx_buff[7]  = (uint8_t) (dev->medium_access_param.device_id      );
	mipa_tx_buff[8]  = (uint8_t) (dev->medium_access_param.device_id >> 8 );
	mipa_tx_buff[9]  = (uint8_t) (dev->medium_access_param.device_id >> 16);
	mipa_tx_buff[10] = (uint8_t) (dev->medium_access_param.device_id >> 24);
	mipa_tx_buff[11] = dev->medium_access_param.version;
	mipa_tx_buff[12] = dev->medium_access_param.device_type;
	mipa_tx_buff[13] = mip_generate_checksum(mipa_tx_buff, 13);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 14, mipa_rx_buff, &rx_msg_len, MIP_EEPROM_WRITE_TIMEOUT);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipa_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
		}
	}
	dev->delay_ms_fn(MIPA_DELAY_MSG);
	return retval;
}

/*!
 * @brief Get the module parameters from device EEPROM.
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_eeprom_read_wmbus_medium_access_parameters(const struct mip_a *const dev, struct a_medium_access_param_t *const config)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipa_tx_buff[0] = MIP_HEADER;
	mipa_tx_buff[1] = MIP_EEPROM_READ_CMD;
	mipa_tx_buff[2] = 0x02;
	mipa_tx_buff[3] = MIPA_C_FIELD_ADDR; /* Start Address */
	mipa_tx_buff[4] = 0x09;              /* Number of bytes */
	mipa_tx_buff[5] = mip_generate_checksum(mipa_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipa_tx_buff, 6, mipa_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipa_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
			else
			{
				config->c_field          = mipa_rx_buff[4];
				config->manufacturer_id  = ((uint16_t) mipa_rx_buff[5] ) & 0x00FF;
				config->manufacturer_id |= ((uint16_t) mipa_rx_buff[6] << 8);
				config->device_id        = ((uint32_t) mipa_rx_buff[7] ) & 0x000000FF;
				config->device_id       |= ((uint32_t) mipa_rx_buff[8]  << 8);
				config->device_id       |= ((uint32_t) mipa_rx_buff[9]  << 16);
				config->device_id       |= ((uint32_t) mipa_rx_buff[10] << 24);
				config->version          = mipa_rx_buff[11];
				config->device_type      = mipa_rx_buff[12];
			}
		}
	}
	return retval;
}

/*!
 * @brief This command performs the transmission of radio frames.
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_tx_msg_cmd(const uint8_t *msg, uint8_t msg_len, struct mip_a *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	uint8_t idx = 0;
	uint8_t i,crc;
	if ( (msg_len == 0) || (msg_len > 240) )
		return tx_message_len_error;
	mipa_tx_buff[idx++] = MIP_HEADER;
	mipa_tx_buff[idx++] = MIPA_TX_MSG_CMD;
	mipa_tx_buff[idx++] = msg_len;
	for(i=0; i<msg_len; i++)
	{
		mipa_tx_buff[idx++] = msg[i];
	}
	crc = mip_generate_checksum(mipa_tx_buff, idx);
	mipa_tx_buff[idx++] = crc;
	retval = dev->send_and_receive_fn(mipa_tx_buff, idx, mipa_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipa_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
		}
	}
	return retval;
}

/*!
 * @brief This command performs the reception of WM-BUS frame
 * @param[in] dev : Structure instance of mip_a.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipa_rx_msg_IND(struct mip_a *const dev, uint32_t timeout)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	uint8_t i;
	retval = dev->receive_fn(mipa_rx_buff, &rx_msg_len, timeout);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipa_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if(mipa_rx_buff[1] == MIPA_RX_MSG_IND)
			{
				if(dev->module_param.rssi == rssi_enable)
				{
					for(i=0; i<mipa_rx_buff[2]-10; i++)
					{
						dev->rx_data.last_rx_msg[i] = mipa_rx_buff[12+i];
					}
					dev->rx_data.last_rx_msg_len = mipa_rx_buff[2]-10;
					dev->rssi = mipa_rx_buff[rx_msg_len-2];
				}
				else
				{
					for(i=0; i<mipa_rx_buff[2]-9; i++)
					{
						dev->rx_data.last_rx_msg[i] = mipa_rx_buff[12+i];
					}
					dev->rx_data.last_rx_msg_len = mipa_rx_buff[2]-9;
				}
				for(i=0;i<9;i++)
				{
					dev->rx_data.Block1[i] = mipa_rx_buff[3+i];
				}
			}
		}
	}
	return retval;
}
