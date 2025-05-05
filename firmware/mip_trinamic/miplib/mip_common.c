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
#include "mip_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

 /*******************************************************************************
 * Extern
 ******************************************************************************/
 
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief This function generates the checksum of the command that need to be sent to the Mipot module.
 * @param[in] *buff : The Tx buffer containing the command.
 * @param[in]   len : Lenght of the command.
 * @return Calcolated checksum.
 */
uint8_t mip_generate_checksum(const uint8_t *buff, uint16_t len)
{
  uint8_t n, crc;
  crc = 0;
  for (n=0; n<len; n++)
    crc += buff[n];
  crc = (crc ^ 0xFF) + 1;
  return crc;
}

/*!
 * @brief This function perform a check of the received command based on the crc.
 * @param[in] *buff : The Rx buffer containing the command.
 * @param[in]   len : Lenght of the command.
 * @retval   0 -> Success.
 * @retval > 0 -> Fail.
 */
enum mip_error_t mip_validate_response(const uint8_t *buff, uint16_t len)
{
	enum mip_error_t ret = crc_invalid;
	uint8_t crc;
	if(len)
	{
		crc = mip_generate_checksum(buff, len-1);
		if (crc == buff[len-1])
		{
			ret = no_error;
		}
	}
	return ret;
}
