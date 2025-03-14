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
#include "trinamic_conf.h"
#include "trinamic_gpio.h"
#include "trinamic_uart.h"
#include "trinamic_tim.h"
#include "tmc2130.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
TMC2130_t mot0;
TMC2130_t mot1;
TMC2130_t mot2;
bool mot0_init_status = false;
bool mot1_init_status = false;
bool mot2_init_status = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
void TrinamicInit(void)
{
	/* Set default configuration for MOT0 */
	(void)TMC2130_SetDefaults(&mot0);
	/* Set default configuration for MOT1 */
	(void)TMC2130_SetDefaults(&mot1);
	/* Set default configuration for MOT2 */
	(void)TMC2130_SetDefaults(&mot2);

	mot0.config.motor.id = TRINAMIC_MOT0_ID; /* MOT0 */
	mot1.config.motor.id = TRINAMIC_MOT1_ID; /* MOT1 */
	mot2.config.motor.id = TRINAMIC_MOT2_ID; /* MOT2 */
	(void)TrinamicMotXEnable(TRINAMIC_MOT0_ID);
	(void)TrinamicMotXEnable(TRINAMIC_MOT1_ID);
	(void)TrinamicMotXEnable(TRINAMIC_MOT2_ID);

	(void)Delay_ms(100);
	(void)LPUART_TxPolling("\r\nBEGIN STEPPER INIT PROCEDURE\r\n");


	/* Init MOT0 */
	mot0_init_status = TMC2130_Init(&mot0);
	if(true == mot0_init_status)
	{
		(void)LPUART_TxPolling("mot0 init SUCCESS\r\n");
	}
	else
	{
		(void)LPUART_TxPolling("mot0 init FAILURE\r\n");
		(void)TrinamicMotXDisable(TRINAMIC_MOT0_ID);
	}

	/* Init MOT1 */
	mot1_init_status = TMC2130_Init(&mot1);
	if(true == mot1_init_status)
	{
		(void)LPUART_TxPolling("mot1 init SUCCESS\r\n");
	}
	else
	{
		(void)LPUART_TxPolling("mot1 init FAILURE\r\n");
		(void)TrinamicMotXDisable(TRINAMIC_MOT1_ID);
	}

	/* Init MOT2 */
	mot2_init_status = TMC2130_Init(&mot2);
	if(true == mot2_init_status)
	{
		(void)LPUART_TxPolling("mot2 init SUCCESS\r\n");
	}
	else
	{
		(void)LPUART_TxPolling("mot2 init FAILURE\r\n");
		(void)TrinamicMotXDisable(TRINAMIC_MOT2_ID);
	}
}
