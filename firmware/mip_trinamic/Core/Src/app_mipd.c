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
#include "app_mipd.h"
#include <string.h>
#include "trinamic_tim.h"
#include "trinamic_gpio.h"
#include "trinamic_pulse_gen.h"
#include "mip_d.h"
#include "mip_d_def.h"
#include "cortexcomm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ipcc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MIPD_TASK_PRIOTITY 3

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void MipdInit(struct mip_d *const dev);
static void MipdApp(void *pvParameters);
static void AnalyzeRadioMessage(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct mip_d mipd;
struct d_radio_phy_t mipd_radio_py;
struct d_module_param_t mipd_config;
TaskHandle_t mipd_device_task_handle;
uint8_t app_ndata_indicate_event;

extern uint8_t ndata_indicate_event;
extern uint32_t mot0_steps_shadow;
extern uint32_t mot1_steps_shadow;
extern uint32_t mot2_steps_shadow;

/*******************************************************************************
 * Code
 ******************************************************************************/
void MipdTask(void)
{
	enum mip_error_t retval;
	(void)MipdInit(&mipd);
	mipd.delay_ms_fn(100);

	retval  = mipd_get_fw_version(&mipd);
    retval += mipd_get_serial_no(&mipd);

	retval += mipd_eeprom_write_module_parameters(&mipd);
	retval += mipd_eeprom_read_module_parameters(&mipd, &mipd_config);

	retval += mipd_eeprom_write_radio_phy_param(&mipd);
	retval += mipd_eeprom_read_radio_phy_param(&mipd, &mipd_radio_py);
	if(retval != no_error)
		return;
	(void)xTaskCreate(MipdApp,"MipdApp",100, NULL,  MIPD_TASK_PRIOTITY, &mipd_device_task_handle);
}

static void MipdApp(void *pvParameters)
{
	enum mip_error_t retval;
	const TickType_t xDelayTxMsg   = pdMS_TO_TICKS(100);
	app_ndata_indicate_event = ndata_indicate_event;
	for(;;)
	{
		if(app_ndata_indicate_event != ndata_indicate_event)
		{
			retval = mipd_receive_message(&mipd);
			if( no_error == retval )
			{
				(void)AnalyzeRadioMessage();
			}
			app_ndata_indicate_event = ndata_indicate_event;
		}
		(void)vTaskDelay(xDelayTxMsg);
	}
	(void)vTaskDelete(NULL);
}

static void AnalyzeRadioMessage(void)
{
	if(9 == mipd.rx_data_info.last_rx_msg_len)
	{
		if( mipd.rx_data_info.last_rx_msg[0] == TRINAMIC_MOT0_ID)
		{
			/* Set steps for MOT0 */
			mot0_steps_shadow =  ((uint32_t)mipd.rx_data_info.last_rx_msg[1] << 24);  /* MSB */
			mot0_steps_shadow |= ((uint32_t)mipd.rx_data_info.last_rx_msg[2] << 16);
			mot0_steps_shadow |= ((uint32_t)mipd.rx_data_info.last_rx_msg[3] << 8);
			mot0_steps_shadow |= ((uint32_t)mipd.rx_data_info.last_rx_msg[4] );       /* LSB */
		}
		else if( mipd.rx_data_info.last_rx_msg[0] == TRINAMIC_MOT1_ID)
		{
			/* Set steps for MOT1 */
			mot1_steps_shadow =  ((uint32_t)mipd.rx_data_info.last_rx_msg[1] << 24);  /* MSB */
			mot1_steps_shadow |= ((uint32_t)mipd.rx_data_info.last_rx_msg[2] << 16);
			mot1_steps_shadow |= ((uint32_t)mipd.rx_data_info.last_rx_msg[3] << 8);
			mot1_steps_shadow |= ((uint32_t)mipd.rx_data_info.last_rx_msg[4] );       /* LSB */
		}
		else if( mipd.rx_data_info.last_rx_msg[0] == TRINAMIC_MOT2_ID)
		{
			/* Set steps for MOT2 */
			mot2_steps_shadow =  ((uint32_t)mipd.rx_data_info.last_rx_msg[1] << 24);  /* MSB */
			mot2_steps_shadow |= ((uint32_t)mipd.rx_data_info.last_rx_msg[2] << 16);
			mot2_steps_shadow |= ((uint32_t)mipd.rx_data_info.last_rx_msg[3] << 8);
			mot2_steps_shadow |= ((uint32_t)mipd.rx_data_info.last_rx_msg[4] );       /* LSB */
		}
		else
		{
			;
		}
	}
}

static void MipdInit(struct mip_d *const dev)
{
	dev->hardware_init_fn                   = NULL;
	dev->send_and_receive_fn                = MipTransmitAndReceiveData;
	dev->receive_fn                         = MipReceiveData;
	dev->hardware_reset_fn                  = NULL;
	dev->delay_ms_fn                        = Delay_ms;
	dev->radio_phy.power                    = MIPD_POWER_DEF;
	dev->radio_phy.frequency_channel        = MIPD_CHANNEL_FREQUENCY_DEF;
	dev->radio_phy.bandwidth                = MIPD_CHANNEL_BANDWIDTH_DEF;
	dev->radio_phy.spreading_factor         = MIPD_SPREADING_FACTOR_DEF;
	dev->radio_phy.code_rate                = MIPD_CODE_RATE_DEF;
	dev->module_param.DATA_INDICATE_TIMEOUT = DATA_IND_TIMEOUT_DEF_VAL;
	dev->module_param.UartBaudrate          = MIPD_UartBaudrate_DEF_VAL;
	dev->module_param.AppEnAes              = MIPD_AppEnAes_DEF_VAL;
}
