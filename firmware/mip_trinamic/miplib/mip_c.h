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

#ifndef INC_MIP_C_H_
#define INC_MIP_C_H_

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "mip_c_def.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
/* MASTER - NED NODE COMMON */
enum mip_error_t mipc_null_ptr_check(const struct mip_c *const dev);
enum mip_error_t mipc_reset(const struct mip_c *const dev);
enum mip_error_t mipc_factory_reset(const struct mip_c *const dev);
enum mip_error_t mipc_get_fw_version(struct mip_c *const dev);
enum mip_error_t mipc_get_serial_no(struct mip_c *const dev);
enum mip_error_t mipc_eeprom_write_stack_parameters(const struct mip_c *const dev);
enum mip_error_t mipc_eeprom_read_stack_parameters(const struct mip_c *const dev, struct c_stack_param_t *const config);
enum mip_error_t mipc_eeprom_write_radio_phy_param(const struct mip_c *const dev);
enum mip_error_t mipc_eeprom_read_radio_phy_param(const struct mip_c *const dev, struct c_radio_phy_t *const config);
enum mip_error_t mipc_eeprom_write_module_parameters(const struct mip_c *const dev);
enum mip_error_t mipc_eeprom_read_module_parameters(const struct mip_c *const dev, struct c_module_param_t *const config);
enum mip_error_t mipc_tx_msg_cmd(c_messageTrasmission_t tx_typ, uint32_t dest_id, const uint8_t *msg, uint8_t msg_len, struct mip_c *const dev);
enum mip_error_t mipc_enable_ndata_indicate_pin(struct mip_c *const dev);
enum mip_error_t mipc_set_sleep(const struct mip_c *const dev);
enum mip_error_t mipc_wakeup(const struct mip_c *const dev);
enum mip_error_t mipc_handle_IND_messages(struct mip_c *const dev, uint32_t timeout);
/* MASTER SPECIFIC */
enum mip_error_t mipc_enable_pairing(const struct mip_c *const dev);
enum mip_error_t mipc_get_network_table_size(struct mip_c *const dev);
enum mip_error_t mipc_get_network_row_size(uint8_t idx, struct mip_c *const dev);
enum mip_error_t mipc_delete_en_device(uint8_t idx, struct mip_c *const dev);
enum mip_error_t mipc_delete_all_en_device(struct mip_c *const dev);
/* END NODE SPECIFIC */
enum mip_error_t mipc_link_check_request(struct mip_c *const dev);
enum mip_error_t mipc_get_activation_status(struct mip_c *const dev);
enum mip_error_t mipc_pairing_req_cmd(struct mip_c *const dev);

#endif
