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

#ifndef MIP_A_H_
#define MIP_A_H_

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "mip_a_def.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
enum mip_error_t mipa_null_ptr_check(const struct mip_a *const dev);
enum mip_error_t mipa_reset(const struct mip_a *const dev);
enum mip_error_t mipa_factory_reset(const struct mip_a *const dev);
enum mip_error_t mipa_get_fw_version(struct mip_a *const dev);
enum mip_error_t mipa_get_serial_no(struct mip_a *const dev);
enum mip_error_t mipa_get_rssi(struct mip_a *const dev);
enum mip_error_t mipa_set_mode(struct mip_a *const dev, enum a_wmbusmode_t wmbus_mode, enum a_mem_t memory_area);
enum mip_error_t mipa_enable_ndata_indicate_pin(const struct mip_a *const dev);
enum mip_error_t mipa_eeprom_write_radio_phy_param(const struct mip_a *const dev);
enum mip_error_t mipa_eeprom_read_radio_phy_param(const struct mip_a *const dev, struct a_radio_phy_t *const config);
enum mip_error_t mipa_eeprom_write_module_parameters(const struct mip_a *const dev);
enum mip_error_t mipa_eeprom_read_module_parameters(const struct mip_a *const dev, struct a_module_param_t *const config);
enum mip_error_t mipa_eeprom_write_wmbus_medium_access_parameters(const struct mip_a *const dev);
enum mip_error_t mipa_eeprom_read_wmbus_medium_access_parameters(const struct mip_a *const dev, struct a_medium_access_param_t *const config);
enum mip_error_t mipa_tx_msg_cmd(const uint8_t *msg, uint8_t msg_len, struct mip_a *const dev);
enum mip_error_t mipa_rx_msg_IND(struct mip_a *const dev, uint32_t timeout);

#endif /* MIP_A_H_ */
