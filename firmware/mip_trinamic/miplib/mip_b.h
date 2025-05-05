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

#ifndef MIP_B_H_
#define MIP_B_H_

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "mip_b_def.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
enum mip_error_t mipb_null_ptr_check(const struct mip_b *const dev);
enum mip_error_t mipb_reset(const struct mip_b *const dev);
enum mip_error_t mipb_factory_reset(const struct mip_b *const dev);
enum mip_error_t mipb_get_fw_version(struct mip_b *const dev);
enum mip_error_t mipb_get_serial_no(struct mip_b *const dev);
enum mip_error_t mipb_enable_ndata_indicate_pin(const struct mip_b *const dev);
enum mip_error_t mipb_eeprom_write_network_parameters(const struct mip_b *const dev);
enum mip_error_t mipb_eeprom_write_stack_parameters(const struct mip_b *const dev);
enum mip_error_t mipb_eeprom_read_network_parameters(const struct mip_b *const dev, struct b_stack_param_t *const config);
enum mip_error_t mipb_eeprom_read_stack_parameters(const struct mip_b *const dev, struct b_stack_param_t *const config);
enum mip_error_t mipb_get_deveui(struct mip_b *const dev);
enum mip_error_t mipb_join(enum b_join_mode_t join_mode, struct mip_b *const dev);
enum mip_error_t mipb_get_activation_status(struct mip_b *const dev);
enum mip_error_t mipb_set_app_key(const uint8_t *appkey, struct mip_b *const dev);
enum mip_error_t mipb_set_app_session_key(const uint8_t *appskey, struct mip_b *const dev);
enum mip_error_t mipb_set_nwk_session_key(const uint8_t *nwkskey, struct mip_b *const dev);
enum mip_error_t mipb_tx_msg_cmd(b_messageTrasmission_t tx_typ, uint8_t port, const uint8_t *msg, uint8_t msg_len, struct mip_b *const dev);
enum mip_error_t mipb_get_session_status(struct mip_b *const dev);
enum mip_error_t mipb_set_next_dr_cmd(enum b_rx2_data_rate_t dr, struct mip_b *const dev);
enum mip_error_t mipb_set_battery_level_cmd(uint8_t batt_lvl, struct mip_b *const dev);
enum mip_error_t mipb_get_battery_level_cmd(uint8_t *batt_lvl, struct mip_b *const dev);
enum mip_error_t mipb_set_uplink_cnt_cmd(struct mip_b *const dev);
enum mip_error_t mipb_get_uplink_cnt_cmd(struct mip_b *const dev);
enum mip_error_t mipb_set_downlink_cnt_cmd(struct mip_b *const dev);
enum mip_error_t mipb_get_downlink_cnt_cmd(struct mip_b *const dev);
enum mip_error_t mipb_link_check_request(struct mip_b *const dev);
enum mip_error_t mipb_device_time_request_cmd(struct mip_b *const dev);
enum mip_error_t mipb_reset_abp_cmd(struct mip_b *const dev);
enum mip_error_t mipb_rekey_otaa_cmd(struct mip_b *const dev);
enum mip_error_t mipb_set_sleep(const struct mip_b *const dev);
enum mip_error_t mipb_wakeup(const struct mip_b *const dev);
enum mip_error_t mipb_handle_IND_messages(struct mip_b *const dev, uint32_t timeout);

#endif /* MIP_B_H_ */
