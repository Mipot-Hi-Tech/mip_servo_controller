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
#include "mip_b.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MIPB_BUFF_SZ   260U
#define MIPB_DELAY_MSG 50U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t mipb_tx_buff[MIPB_BUFF_SZ] = {0};
uint8_t mipb_rx_buff[MIPB_BUFF_SZ] = {0};

/*******************************************************************************
 * Extern
 ******************************************************************************/
 
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Validate the device pointer for null conditions.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_null_ptr_check(const struct mip_b *const dev)
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
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_reset(const struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIP_RESET_CMD;
	mipb_tx_buff[2] = 0x00;
	mipb_tx_buff[3] = mip_generate_checksum(mipb_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 4, mipb_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIP_DELAY_RESET);
	return retval;
}

/*!
 * @brief This command restores EEPROM factory default values.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_factory_reset(const struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIP_FACTORY_RESET_CMD;
	mipb_tx_buff[2] = 0x00;
	mipb_tx_buff[3] = mip_generate_checksum(mipb_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 4, mipb_rx_buff, &rx_msg_len, MIP_DELAY_FACTORY_RESET);
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief Get 32-bit firmware version.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_get_fw_version(struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIP_GET_FW_VERSION_CMD;
	mipb_tx_buff[2] = 0x00;
	mipb_tx_buff[3] = mip_generate_checksum(mipb_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 4, mipb_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			dev->fw_version =  ((uint32_t) mipb_rx_buff[6] << 24);  /* MSB */
			dev->fw_version |= ((uint32_t) mipb_rx_buff[5] << 16);
			dev->fw_version |= ((uint32_t) mipb_rx_buff[4] << 8);
			dev->fw_version |= ((uint32_t) mipb_rx_buff[3] );       /* LSB */
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief Get unique 32-bit Serial Number.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_get_serial_no(struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIP_GET_SERIALNO_CMD;
	mipb_tx_buff[2] = 0x00;
	mipb_tx_buff[3] = mip_generate_checksum(mipb_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 4, mipb_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			dev->serialno  = ((uint32_t) mipb_rx_buff[6] << 24);  /* MSB */
			dev->serialno |= ((uint32_t) mipb_rx_buff[5] << 16);
			dev->serialno |= ((uint32_t) mipb_rx_buff[4] << 8);
			dev->serialno |= ((uint32_t) mipb_rx_buff[3] );       /* LSB */
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command enable the NDATA_INDICATE pin of the mip module
 * @param[in] dev : Structure instance of mip_d.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_enable_ndata_indicate_pin(const struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = 0x32;
	mipb_tx_buff[2] = 0x02;
	mipb_tx_buff[3] = 0x91;
	mipb_tx_buff[4] = 0x01;
	mipb_tx_buff[5] = mip_generate_checksum(mipb_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 6, mipb_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs an EEPROM data write.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_eeprom_write_network_parameters(const struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0]  = MIP_HEADER;
	mipb_tx_buff[1]  = MIP_EEPROM_WRITE_CMD;
	mipb_tx_buff[2]  = 0x15; /* lenght */
	mipb_tx_buff[3]  = 0x00; /* start address */
	mipb_tx_buff[4]  = dev->stack_param.Customer_DevEUI[0];
	mipb_tx_buff[5]  = dev->stack_param.Customer_DevEUI[1];
	mipb_tx_buff[6]  = dev->stack_param.Customer_DevEUI[2];
	mipb_tx_buff[7]  = dev->stack_param.Customer_DevEUI[3];
	mipb_tx_buff[8]  = dev->stack_param.Customer_DevEUI[4];
	mipb_tx_buff[9]  = dev->stack_param.Customer_DevEUI[5];
	mipb_tx_buff[10] = dev->stack_param.Customer_DevEUI[6];
	mipb_tx_buff[11] = dev->stack_param.Customer_DevEUI[7];
	mipb_tx_buff[12] = dev->stack_param.AppEUI[0];
	mipb_tx_buff[13] = dev->stack_param.AppEUI[1];
	mipb_tx_buff[14] = dev->stack_param.AppEUI[2];
	mipb_tx_buff[15] = dev->stack_param.AppEUI[3];
	mipb_tx_buff[16] = dev->stack_param.AppEUI[4];
	mipb_tx_buff[17] = dev->stack_param.AppEUI[5];
	mipb_tx_buff[18] = dev->stack_param.AppEUI[6];
	mipb_tx_buff[19] = dev->stack_param.AppEUI[7];
	mipb_tx_buff[20] = dev->stack_param.DevAddr[0];
	mipb_tx_buff[21] = dev->stack_param.DevAddr[1];
	mipb_tx_buff[22] = dev->stack_param.DevAddr[2];
	mipb_tx_buff[23] = dev->stack_param.DevAddr[3];
	mipb_tx_buff[24] = mip_generate_checksum(mipb_tx_buff, 24);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 25, mipb_rx_buff, &rx_msg_len, MIP_EEPROM_WRITE_TIMEOUT);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipb_rx_buff[3] != no_error)
			{
				if (mipb_rx_buff[3] == 0x01)
				{
					retval = data_outside_range;
				}
				else if (mipb_rx_buff[3] == 0x02)
				{
					retval = LoRaMac_notin_idle_state;
				}
				else
				{
					retval = unknown_error;
				}
			}
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs an EEPROM data write.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_eeprom_write_stack_parameters(const struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0]  = MIP_HEADER;
	mipb_tx_buff[1]  = MIP_EEPROM_WRITE_CMD;
	mipb_tx_buff[2]  = 0x10; /* lenght */
	mipb_tx_buff[3]  = 0x20; /* start address */
	mipb_tx_buff[4]  = dev->stack_param.class;
	mipb_tx_buff[5]  = dev->stack_param.dr_sf;
	mipb_tx_buff[6]  = dev->stack_param.power;
	mipb_tx_buff[7]  = dev->stack_param.ADR;
	mipb_tx_buff[8]  = dev->stack_param.Duty_Cycle_control;
	mipb_tx_buff[9]  = dev->stack_param.Unconfirmed_TX_Repetition_number;
	mipb_tx_buff[10] = dev->stack_param.Enable_Customer_EUI;
	mipb_tx_buff[11] = dev->stack_param.RX2_Data_Rate;
	mipb_tx_buff[12] = (uint8_t) (dev->stack_param.RX2_Frequency >> 24);
	mipb_tx_buff[13] = (uint8_t) (dev->stack_param.RX2_Frequency >> 16);
	mipb_tx_buff[14] = (uint8_t) (dev->stack_param.RX2_Frequency >> 8 );
	mipb_tx_buff[15] = (uint8_t) (dev->stack_param.RX2_Frequency      );
	mipb_tx_buff[16] = 0x00; /* LinkCheck Timeout RESERVED */
	mipb_tx_buff[17] = 0x00; /* LinkCheck Timeout RESERVED */
	mipb_tx_buff[18] = dev->stack_param.Public_Network_Enable;
	mipb_tx_buff[19] = mip_generate_checksum(mipb_tx_buff, 19);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 20, mipb_rx_buff, &rx_msg_len, MIP_EEPROM_WRITE_TIMEOUT);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipb_rx_buff[3] != no_error)
			{
				if (mipb_rx_buff[3] == 0x01)
				{
					retval = data_outside_range;
				}
				else if (mipb_rx_buff[3] == 0x02)
				{
					retval = LoRaMac_notin_idle_state;
				}
				else
				{
					retval = unknown_error;
				}
			}
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command reads EEPROM data.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_eeprom_read_network_parameters(const struct mip_b *const dev, struct b_stack_param_t *const config)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIP_EEPROM_READ_CMD;
	mipb_tx_buff[2] = 0x02;
	mipb_tx_buff[3] = 0x00; /* Start Address */
	mipb_tx_buff[4] = 0x14; /* Number of bytes */
	mipb_tx_buff[5] = mip_generate_checksum(mipb_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 6, mipb_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipb_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
			else
			{
				config->Customer_DevEUI[0] = mipb_rx_buff[4];
				config->Customer_DevEUI[1] = mipb_rx_buff[5];
				config->Customer_DevEUI[2] = mipb_rx_buff[6];
				config->Customer_DevEUI[3] = mipb_rx_buff[7];
				config->Customer_DevEUI[4] = mipb_rx_buff[8];
				config->Customer_DevEUI[5] = mipb_rx_buff[9];
				config->Customer_DevEUI[6] = mipb_rx_buff[10];
				config->Customer_DevEUI[7] = mipb_rx_buff[11];
				config->AppEUI[0]          = mipb_rx_buff[12];
				config->AppEUI[1]          = mipb_rx_buff[13];
				config->AppEUI[2]          = mipb_rx_buff[14];
				config->AppEUI[3]          = mipb_rx_buff[15];
				config->AppEUI[4]          = mipb_rx_buff[16];
				config->AppEUI[5]          = mipb_rx_buff[17];
				config->AppEUI[6]          = mipb_rx_buff[18];
				config->AppEUI[7]          = mipb_rx_buff[19];
				config->DevAddr[0]         = mipb_rx_buff[20];
				config->DevAddr[1]         = mipb_rx_buff[21];
				config->DevAddr[2]         = mipb_rx_buff[22];
				config->DevAddr[3]         = mipb_rx_buff[23];
			}
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}


/*!
 * @brief This command reads EEPROM data.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_eeprom_read_stack_parameters(const struct mip_b *const dev, struct b_stack_param_t *const config)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIP_EEPROM_READ_CMD;
	mipb_tx_buff[2] = 0x02;
	mipb_tx_buff[3] = 0x20; /* Start Address */
	mipb_tx_buff[4] = 0x0F; /* Number of bytes */
	mipb_tx_buff[5] = mip_generate_checksum(mipb_tx_buff, 5);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 6, mipb_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if (mipb_rx_buff[3] != MIP_RET_SUCCESS)
			{
				retval = failure;
			}
			else
			{
				config->class                 = mipb_rx_buff[4];
				config->dr_sf                 = mipb_rx_buff[5];
				config->power                 = mipb_rx_buff[6];
				config->ADR                   = mipb_rx_buff[7];
				config->Duty_Cycle_control    = mipb_rx_buff[8];
				config->Unconfirmed_TX_Repetition_number= mipb_rx_buff[9];
				config->Enable_Customer_EUI   = mipb_rx_buff[10];
				config->RX2_Data_Rate         = mipb_rx_buff[11];
				config->RX2_Frequency         =  ((uint32_t) mipb_rx_buff[12] << 24);
				config->RX2_Frequency        |=  ((uint32_t) mipb_rx_buff[13] << 16);
				config->RX2_Frequency        |=  ((uint32_t) mipb_rx_buff[14] << 8 );
				config->RX2_Frequency        |=  ((uint32_t) mipb_rx_buff[15] );
				config->Public_Network_Enable = mipb_rx_buff[18];
			}
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief Get unique 32-bit Serial Number.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_get_deveui(struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_GET_DEV_EUI_CMD;
	mipb_tx_buff[2] = 0x00;
	mipb_tx_buff[3] = mip_generate_checksum(mipb_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 4, mipb_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			dev->stack_param.Customer_DevEUI[0] = mipb_rx_buff[4];
			dev->stack_param.Customer_DevEUI[1] = mipb_rx_buff[5];
			dev->stack_param.Customer_DevEUI[2] = mipb_rx_buff[6];
			dev->stack_param.Customer_DevEUI[3] = mipb_rx_buff[7];
			dev->stack_param.Customer_DevEUI[4] = mipb_rx_buff[8];
			dev->stack_param.Customer_DevEUI[5] = mipb_rx_buff[9];
			dev->stack_param.Customer_DevEUI[6] = mipb_rx_buff[10];
			dev->stack_param.Customer_DevEUI[7] = mipb_rx_buff[11];
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs the network activation.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_join(enum b_join_mode_t join_mode, struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_JOIN_CMD;
	mipb_tx_buff[2] = 0x01;
	mipb_tx_buff[3] = join_mode;
	mipb_tx_buff[4] = mip_generate_checksum(mipb_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 5, mipb_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if(mipb_rx_buff[3] == 0x01)
			{
				retval = invalid_parameter;
			}
			else if(mipb_rx_buff[3] == 0x02)
			{
				retval = device_busy;
			}
			else
				;
		}
	}
	if(retval != no_error)
		return retval;
	/* JOIN_IND message is coming from module */
	retval = mipb_handle_IND_messages(dev,20000);
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief Gets the module activation status.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_get_activation_status(struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_GET_ACTIVATION_STATUS_CMD;
	mipb_tx_buff[2] = 0x00;
	mipb_tx_buff[3] = mip_generate_checksum(mipb_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 4, mipb_rx_buff, &rx_msg_len, 10);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if(mipb_rx_buff[3] == 0x00)
			{
				dev->join_status = Device_not_activated;
			}
			else if(mipb_rx_buff[3] == 0x01)
			{
				dev->join_status = Joining;
			}
			else if(mipb_rx_buff[3] == 0x02)
			{
				dev->join_status = Joined;
			}
			else
			{
				dev->join_status = MAC_error;
			}
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs the EEPROM data write.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_set_app_key(const uint8_t *appkey, struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	uint8_t idx = 0;
	uint8_t i,crc;
	mipb_tx_buff[idx++] = MIP_HEADER;
	mipb_tx_buff[idx++] = MIPB_SET_APP_KEY_CMD;
	mipb_tx_buff[idx++] = 0x10;
	for(i=0; i<16; i++)
	{
		mipb_tx_buff[idx++]        = appkey[i];
		dev->stack_param.AppKey[i] = appkey[i];
	}
	crc = mip_generate_checksum(mipb_tx_buff, idx);
	mipb_tx_buff[idx++] = crc;
	retval = dev->send_and_receive_fn(mipb_tx_buff, idx, mipb_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if(mipb_rx_buff[3] != no_error)
			{
				retval = unknown_error;
			}
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs the EEPROM data write.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_set_app_session_key(const uint8_t *appskey, struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	uint8_t idx = 0;
	uint8_t i,crc;
	mipb_tx_buff[idx++] = MIP_HEADER;
	mipb_tx_buff[idx++] = MIPB_SET_APP_KEY_CMD;
	mipb_tx_buff[idx++] = 0x10;
	for(i=0; i<16; i++)
	{
		mipb_tx_buff[idx++]         = appskey[i];
		dev->stack_param.AppSKey[i] = appskey[i];
	}
	crc = mip_generate_checksum(mipb_tx_buff, idx);
	mipb_tx_buff[idx++] = crc;
	retval = dev->send_and_receive_fn(mipb_tx_buff, idx, mipb_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if(mipb_rx_buff[3] != no_error)
			{
				retval = unknown_error;
			}
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs the EEPROM data write.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_set_nwk_session_key(const uint8_t *nwkskey, struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	uint8_t idx = 0;
	uint8_t i,crc;
	mipb_tx_buff[idx++] = MIP_HEADER;
	mipb_tx_buff[idx++] = MIPB_SET_APP_KEY_CMD;
	mipb_tx_buff[idx++] = 0x10;
	for(i=0; i<16; i++)
	{
		mipb_tx_buff[idx++]         = nwkskey[i];
		dev->stack_param.NwkSKey[i] = nwkskey[i];
	}
	crc = mip_generate_checksum(mipb_tx_buff, idx);
	mipb_tx_buff[idx++] = crc;
	retval = dev->send_and_receive_fn(mipb_tx_buff, idx, mipb_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if(mipb_rx_buff[3] != no_error)
			{
				retval = unknown_error;
			}
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command performs the transmission of radio frames.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_tx_msg_cmd(b_messageTrasmission_t tx_typ, uint8_t port, const uint8_t *msg, uint8_t msg_len, struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	uint8_t idx = 0;
	uint8_t i,crc;
	mipb_tx_buff[idx++] = MIP_HEADER;
	mipb_tx_buff[idx++] = MIPB_TX_MSG_CMD;
	mipb_tx_buff[idx++] = 2 + msg_len;
	mipb_tx_buff[idx++] = tx_typ;
	mipb_tx_buff[idx++] = port;
	for(i=0; i<msg_len; i++)
	{
		mipb_tx_buff[idx++] = msg[i];
	}
	crc = mip_generate_checksum(mipb_tx_buff, idx);
	mipb_tx_buff[idx++] = crc;
	/* SEND THE THE TRASMISSION COMMAND TO THE MODULE */
	retval = dev->send_and_receive_fn(mipb_tx_buff, idx, mipb_rx_buff, &rx_msg_len, 25);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if(mipb_rx_buff[3] != no_error)
			{
				if(mipb_rx_buff[3] == 0x01)
					retval = device_busy;
				else if(mipb_rx_buff[3] == 0x02)
					retval = device_not_activated;
				else if(mipb_rx_buff[3] == 0x03)
					retval = Channel_Blocked_by_dc;
				else if(mipb_rx_buff[3] == 0x04)
					retval = port_num_not_supported;
				else if(mipb_rx_buff[3] == 0x05)
					retval = lenght_not_supported;
				else if(mipb_rx_buff[3] == 0x06)
					retval = End_Node_in_silent_state;
				else if(mipb_rx_buff[3] == 0x07)
					retval = error;
				else
					retval = unknown_error;
				return retval;
			}
		}
	}
	if(retval != no_error)
		return retval;
	/* TX_MSG_IND IS COMING FROM THE MODULE */
	retval = mipb_handle_IND_messages(dev,20000);
	if(retval != no_error)
		return retval;
	/* DOWNLINK MSG IS COMING FROM THE MODULE, IF AVALIABLE */
	retval = mipb_handle_IND_messages(dev,200);
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command gets the module current status.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_get_session_status(struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_GET_SESSION_STATUS_CMD;
	mipb_tx_buff[2] = 0x00;
	mipb_tx_buff[3] = mip_generate_checksum(mipb_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 4, mipb_rx_buff, &rx_msg_len, 10);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			if(mipb_rx_buff[3] == 0x00)
			{
				dev->session_status = Session_Idle;
			}
			else if(mipb_rx_buff[3] == 0x01)
			{
				dev->session_status = Session_Busy;
			}
			else if(mipb_rx_buff[3] == 0x03)
			{
				dev->session_status = Session_Delayed;
			}
			else
			{
				dev->session_status = Session_Device_not_activated;
			}
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}


/*!
 * @brief This command will set next transmission DR.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_set_next_dr_cmd(enum b_rx2_data_rate_t dr, struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_SET_NEXT_DR_CMD;
	mipb_tx_buff[2] = 0x01;
	mipb_tx_buff[3] = dr;
	mipb_tx_buff[4] = mip_generate_checksum(mipb_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 5, mipb_rx_buff, &rx_msg_len, 10);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command will set the battery level required for DevretvalReq frame used in LoRaWAN class A protocol.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_set_battery_level_cmd(uint8_t batt_lvl, struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_SET_BATTERY_LEVEL_CMD;
	mipb_tx_buff[2] = 0x01;
	mipb_tx_buff[3] = batt_lvl;
	mipb_tx_buff[4] = mip_generate_checksum(mipb_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 5, mipb_rx_buff, &rx_msg_len, 10);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command will get the battery level value.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_get_battery_level_cmd(uint8_t *batt_lvl, struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_GET_BATTERY_LVL_CMD;
	mipb_tx_buff[2] = 0x00;
	mipb_tx_buff[3] = mip_generate_checksum(mipb_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 4, mipb_rx_buff, &rx_msg_len, 10);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			*batt_lvl = mipb_rx_buff[3];
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command will set the uplink counter in RAM memory.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_set_uplink_cnt_cmd(struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_SET_UPLINK_CNT_CMD;
	mipb_tx_buff[2] = 0x04;
	mipb_tx_buff[3] = (uint8_t)(dev->stack_param.uplink_cnt);
	mipb_tx_buff[4] = (uint8_t)(dev->stack_param.uplink_cnt >> 8);
	mipb_tx_buff[5] = (uint8_t)(dev->stack_param.uplink_cnt >> 16);
	mipb_tx_buff[6] = (uint8_t)(dev->stack_param.uplink_cnt >> 24);
	mipb_tx_buff[7] = mip_generate_checksum(mipb_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 5, mipb_rx_buff, &rx_msg_len, 10);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command will get the uplink counter in RAM memory.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_get_uplink_cnt_cmd(struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_GET_UPLINK_CNT_CMD;
	mipb_tx_buff[2] = 0x00;
	mipb_tx_buff[3] = mip_generate_checksum(mipb_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 4, mipb_rx_buff, &rx_msg_len, 10);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if (retval == no_error)
		{
			dev->stack_param.uplink_cnt =  ((uint32_t) mipb_rx_buff[6] << 24);
			dev->stack_param.uplink_cnt |= ((uint32_t) mipb_rx_buff[5] << 16);
			dev->stack_param.uplink_cnt |= ((uint32_t) mipb_rx_buff[4] << 8);
			dev->stack_param.uplink_cnt |= ((uint32_t) mipb_rx_buff[3] );
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command will set the downlink counter in RAM memory.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_set_downlink_cnt_cmd(struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_SET_DOWNLINK_CNT_CMD;
	mipb_tx_buff[2] = 0x04;
	mipb_tx_buff[3] = (uint8_t)(dev->stack_param.downlink_cnt);
	mipb_tx_buff[4] = (uint8_t)(dev->stack_param.downlink_cnt >> 8);
	mipb_tx_buff[5] = (uint8_t)(dev->stack_param.downlink_cnt >> 16);
	mipb_tx_buff[6] = (uint8_t)(dev->stack_param.downlink_cnt >> 24);
	mipb_tx_buff[7] = mip_generate_checksum(mipb_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 5, mipb_rx_buff, &rx_msg_len, 10);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command will get the downlink counter from RAM memory.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_get_downlink_cnt_cmd(struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_GET_DOWNLINK_CNT_CMD;
	mipb_tx_buff[2] = 0x00;
	mipb_tx_buff[3] = mip_generate_checksum(mipb_tx_buff, 3);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 4, mipb_rx_buff, &rx_msg_len, 10);
	if (retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if (retval == no_error)
		{
			dev->stack_param.downlink_cnt =  ((uint32_t) mipb_rx_buff[6] << 24);
			dev->stack_param.downlink_cnt |= ((uint32_t) mipb_rx_buff[5] << 16);
			dev->stack_param.downlink_cnt |= ((uint32_t) mipb_rx_buff[4] << 8);
			dev->stack_param.downlink_cnt |= ((uint32_t) mipb_rx_buff[3] );
		}
	}
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command will start a link check with available gateways.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_link_check_request(struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_LINK_CHECK_REQUEST_CMD;
	mipb_tx_buff[2] = 0x01;
	mipb_tx_buff[3] = 0x01;
	mipb_tx_buff[4] = mip_generate_checksum(mipb_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 5, mipb_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
	}
	if (retval != no_error)
		return retval;
	/* LINK CHECK REQUEST IND IS COMING FROM THE MODULE */
	retval = mipb_handle_IND_messages(dev,20000);
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief This command requests the network time and date.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_device_time_request_cmd(struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIPB_DEVICE_TIME_REQUEST_CMD;
	mipb_tx_buff[2] = 0x01;
	mipb_tx_buff[3] = 0x01;
	mipb_tx_buff[4] = mip_generate_checksum(mipb_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 5, mipb_rx_buff, &rx_msg_len, 5);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
	}
	if (retval != no_error)
		return retval;
	/* DEVICE TIME REQUEST IND IS COMING FROM THE MODULE */
	retval = mipb_handle_IND_messages(dev,20000);
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return retval;
}

/*!
 * @brief After this command, the module enters sleep mode.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval Success.
 */
enum mip_error_t mipb_set_sleep(const struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIP_SLEEP_CMD;
	mipb_tx_buff[2] = 0x01;
	mipb_tx_buff[3] = 0x00;
	mipb_tx_buff[4] = mip_generate_checksum(mipb_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 5, mipb_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return no_error;
}

/*!
 * @brief After this command, the module exits sleep mode.
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval Success.
 */
enum mip_error_t mipb_wakeup(const struct mip_b *const dev)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	mipb_tx_buff[0] = MIP_HEADER;
	mipb_tx_buff[1] = MIP_SLEEP_CMD;
	mipb_tx_buff[2] = 0x01;
	mipb_tx_buff[3] = 0x01;
	mipb_tx_buff[4] = mip_generate_checksum(mipb_tx_buff, 4);
	retval = dev->send_and_receive_fn(mipb_tx_buff, 5, mipb_rx_buff, &rx_msg_len, 5);
	dev->delay_ms_fn(MIPB_DELAY_MSG);
	return no_error;
}

/*!
 * @brief Handle all possible IND command, coming from the module
 * @param[in] dev : Structure instance of mip_b.
 * @return Result of API execution retval.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mipb_handle_IND_messages(struct mip_b *const dev, uint32_t timeout)
{
	enum mip_error_t retval;
	uint16_t rx_msg_len;
	uint8_t payload_len;
	uint8_t i;
	retval = dev->receive_fn(mipb_rx_buff, &rx_msg_len, timeout);
	if(retval == no_error)
	{
		retval = mip_validate_response(mipb_rx_buff, rx_msg_len);
		if(retval == no_error)
		{
			switch(mipb_rx_buff[1])
			{
				case MIPB_JOIN_IND:
				{
					if(mipb_rx_buff[3] == 0x00)
						dev->join_status = Joined;
					else
						dev->join_status = Device_not_activated;
					break;
				}

				case MIPB_TX_MSG_CONFIRMED_IND:
				{
					dev->confirmed_msg_ind_info.status      = mipb_rx_buff[3];
					dev->confirmed_msg_ind_info.DataRate    = mipb_rx_buff[4];
					dev->confirmed_msg_ind_info.TxPower     = mipb_rx_buff[5];
					dev->confirmed_msg_ind_info.AckReceived = mipb_rx_buff[6];
					break;
				}

				case MIPB_TX_MSG_UNCONFIRMED_IND:
				{
					dev->unconfirmed_msg_ind_info.status   = mipb_rx_buff[3];
					dev->unconfirmed_msg_ind_info.DataRate = mipb_rx_buff[4];
					dev->unconfirmed_msg_ind_info.TxPower  = mipb_rx_buff[5];
					break;
				}

				case MIPB_RX_MSG_IND:
				{
					if(mipb_rx_buff[3] == no_error)
					{
						dev->downlink_msg.MsgType       = mipb_rx_buff[4];
						dev->downlink_msg.MulticastFlag = mipb_rx_buff[5];
						dev->downlink_msg.RxDataRate    = mipb_rx_buff[6];
						dev->downlink_msg.RxSlot        = mipb_rx_buff[7];
						dev->downlink_msg.FramePending  = mipb_rx_buff[8];
						dev->downlink_msg.AckReceived   = mipb_rx_buff[9];
						dev->downlink_msg.RxData        = mipb_rx_buff[10];
						dev->downlink_msg.Rssi          = mipb_rx_buff[11];
						dev->downlink_msg.SNR           = mipb_rx_buff[12];
						dev->downlink_msg.Port          = mipb_rx_buff[13];
						if(dev->downlink_msg.RxData != 0)
						{
							/* Data is available */
							for(i=0; i<mipb_rx_buff[2]-12; i++)
							{
								dev->downlink_msg.Payload[i] = mipb_rx_buff[15 + i];
							}
							dev->downlink_msg.payload_lenght = mipb_rx_buff[2]-12;
						}
					}
					break;
				}

				case MIPB_LINK_CHECK_REQUEST_IND:
				{
					dev->link_check_result.Margin = mipb_rx_buff[3];
					dev->link_check_result.GW_Cnt = mipb_rx_buff[4];
					break;
				}

				case MIPB_DEVICE_TIME_REQUEST_IND:
				{
					dev->network_time.epoch_fs =  mipb_rx_buff[7];
					dev->network_time.epoch_s  =  ((uint32_t) mipb_rx_buff[6] << 24);
					dev->network_time.epoch_s |=  ((uint32_t) mipb_rx_buff[5] << 16);
					dev->network_time.epoch_s |=  ((uint32_t) mipb_rx_buff[4] << 8);
					dev->network_time.epoch_s |=  ((uint32_t) mipb_rx_buff[3] );
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
