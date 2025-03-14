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
#include <FreeRTOS_CLI.h>
#include "sys_task.h"
#include "app_cli.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "trinamic_uart.h"
#include "trinamic_dac.h"
#include "trinamic_pulse_gen.h"
#include "trinamic_gpio.h"
#include "trinamic_diagnosis.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MAX_INPUT_LENGTH 50
#define configCOMMAND_INT_MAX_OUTPUT_SIZE 200
#define USING_OTHER_TERMINAL 1

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void AppCli(void *pvParameters);
static void vRegisterCLICommands(void);
static void handleBackspace(uint8_t *cInputIndex, char *pcInputString);
static void handleNewline(const char *const pcInputString, char *cOutputBuffer, uint8_t *cInputIndex);
static void handleCharacterInput(uint8_t *cInputIndex, char *pcInputString);
static void cliWrite(const char *str);
/* Commands */
static BaseType_t CliGetTrinamicMipFwVersion(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t CliMot0SetSteps(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t CliMot1SetSteps(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t CliMot2SetSteps(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t CliSetTrinamic_AIN_IREF_Voltage(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t CliTrinamic_ReadAllRegistrs(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

/*******************************************************************************
 * Variables
 ******************************************************************************/
const CLI_Command_Definition_t xCommandList[] = {
	{
		.pcCommand                   = "fw",
		.pcHelpString                = "fw: Get the current firmware version\r\n",
		.pxCommandInterpreter        = CliGetTrinamicMipFwVersion,
		.cExpectedNumberOfParameters = 0
	},
	{
		.pcCommand                   = "mot0",
		.pcHelpString                = "mot0: [STEPS] [DIR]\r\nExample: mot0 1000 0\r\n\r\n",
		.pxCommandInterpreter        = CliMot0SetSteps,
		.cExpectedNumberOfParameters = 2
	},
	{
		.pcCommand                   = "mot1",
		.pcHelpString                = "mot1: [STEPS] [DIR]\r\nExample: mot1 10000 1\r\n\r\n",
		.pxCommandInterpreter        = CliMot1SetSteps,
		.cExpectedNumberOfParameters = 2
	},
	{
		.pcCommand                   = "mot2",
		.pcHelpString                = "mot2: [STEPS] [DIR]\r\nExample: mot2 500 1\r\n\r\n",
		.pxCommandInterpreter        = CliMot2SetSteps,
		.cExpectedNumberOfParameters = 2
	},
	{
		.pcCommand                   = "dac",
		.pcHelpString                = "dac: [VAL] Set the voltage @ all steppers AIN_IREF pins \r\n\r\n",
		.pxCommandInterpreter        = CliSetTrinamic_AIN_IREF_Voltage,
		.cExpectedNumberOfParameters = 1
	},
	{
		.pcCommand                   = "rread",
		.pcHelpString                = "rread: [MOTOR ID]\r\nExample: rread 1\r\n\r\n",
		.pxCommandInterpreter        =  CliTrinamic_ReadAllRegistrs,
		.cExpectedNumberOfParameters = 1
	},
    {
        .pcCommand = NULL
    }
};

typedef enum{
	COMMAND_LINE_INTERFACE_INIT = 0,
	COMMAND_LINE_INTERFACE_IDLE,
}cli_t;

const char * cli_prompt = "\r\nmip> ";
const char * wrong      = "ERR";
const char * success    = "SUCCESS";
const char * mip_fail   = "MIP MODULE NOT PRESENT!";
/* CLI escape sequences */
uint8_t backspace[] = "\b \b";
uint8_t backspace_tt[] = " \b";

char cOutputBuffer[configCOMMAND_INT_MAX_OUTPUT_SIZE];
char pcInputString[MAX_INPUT_LENGTH];

extern char cRxedChar;
extern uint32_t mot0_steps_shadow;
extern uint32_t mot1_steps_shadow;
extern uint32_t mot2_steps_shadow;
extern TIM_HandleTypeDef htim2;
extern uint8_t dac_output_voltage;

/*******************************************************************************
 * Code
 ******************************************************************************/
void vTaskCli(void)
{
	(void)xTaskCreate(AppCli,"AppCli",250, NULL, CLI_TASK_PRIORITY, &cli_task_handle);
}

/*-----------------------------------------------------------*/

static void AppCli(void *pvParameters)
{
    uint8_t cInputIndex = 0;
    uint32_t receivedValue;
    cli_t cli = COMMAND_LINE_INTERFACE_INIT;
    (void)vRegisterCLICommands();
    for (;;)
    {
    	switch(cli)
    	{
			case COMMAND_LINE_INTERFACE_INIT:
			{
				(void)LL_LPUART_EnableIT_RXNE_RXFNE(LPUART1);
				cli = COMMAND_LINE_INTERFACE_IDLE;
				break;
			}

			case COMMAND_LINE_INTERFACE_IDLE:
			{
		        xTaskNotifyWait(pdFALSE, 0, &receivedValue, portMAX_DELAY);
		        cRxedChar = receivedValue & 0xFF;
		        (void)cliWrite((char *)&cRxedChar);
		        if (cRxedChar == '\r' || cRxedChar == '\n')
		        {
		        	(void)handleNewline(pcInputString, cOutputBuffer, &cInputIndex);
		        }
		        else
		        {
		        	(void)handleCharacterInput(&cInputIndex, pcInputString);
		        }
				break;
			}

			default:
			{
				cli = COMMAND_LINE_INTERFACE_INIT;
				break;
			}
    	}
    }
    (void)vTaskDelete(NULL);
}

/*-----------------------------------------------------------*/

static void cliWrite(const char *str)
{
	(void)LPUART_TxPolling(str);
}

/*-----------------------------------------------------------*/

static void handleCharacterInput(uint8_t *cInputIndex, char *pcInputString)
{
    if (cRxedChar == '\r')
    {
        return;
    }
    else if (cRxedChar == (uint8_t)0x08 || cRxedChar == (uint8_t)0x7F)
    {
    	(void)handleBackspace(cInputIndex, pcInputString);
    }
    else
    {
        if (*cInputIndex < MAX_INPUT_LENGTH)
        {
            pcInputString[*cInputIndex] = cRxedChar;
            (*cInputIndex)++;
        }
    }
}

/*-----------------------------------------------------------*/

static void handleNewline(const char *const pcInputString, char *cOutputBuffer, uint8_t *cInputIndex)
{
	(void)cliWrite("\r\n");
    BaseType_t xMoreDataToFollow;
    do
    {
        xMoreDataToFollow = FreeRTOS_CLIProcessCommand(pcInputString, cOutputBuffer, configCOMMAND_INT_MAX_OUTPUT_SIZE);
        (void)cliWrite(cOutputBuffer);
    } while (xMoreDataToFollow != pdFALSE);
    (void)cliWrite(cli_prompt);
    *cInputIndex = 0;
    memset((void*)pcInputString, 0x00, MAX_INPUT_LENGTH);
	LPUART_ClearRxBuff();
}

/*-----------------------------------------------------------*/

static void handleBackspace(uint8_t *cInputIndex, char *pcInputString)
{
    if (*cInputIndex > 0)
    {
        (*cInputIndex)--;
        pcInputString[*cInputIndex] = '\0';
#if USING_VS_CODE_TERMINAL
        cliWrite((char *)backspace);
#elif USING_OTHER_TERMINAL
        (void)cliWrite((char *)backspace_tt);
#endif
    }
    else
    {
#if USING_OTHER_TERMINAL
        uint8_t right[] = "\x1b\x5b\x43";
        (void)cliWrite((char *)right);
#endif
    }
}

/*-----------------------------------------------------------*/

static void vRegisterCLICommands(void)
{
    for (int i = 0; xCommandList[i].pcCommand != NULL; i++)
    {
    	(void)FreeRTOS_CLIRegisterCommand(&xCommandList[i]);
    }
}

/*-----------------------------------------------------------*/

static BaseType_t CliGetTrinamicMipFwVersion(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)pcCommandString;
	(void)xWriteBufferLen;
	sprintf(pcWriteBuffer, "fwver: %u", FW_VERSION);
	strcpy(pcWriteBuffer, success);
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static BaseType_t CliMot0SetSteps(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)xWriteBufferLen;
	const char *pcParameter1;
	const char *pcParameter2;
	BaseType_t xParameter1StringLength;
	BaseType_t xParameter2StringLength;
	uint32_t steps;
	uint32_t dir;
	/* Get the step field */
	pcParameter1      = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);
	steps             = strtol(pcParameter1, NULL, 10);
	mot0_steps_shadow = steps;
	/* Get the dir field */
	pcParameter2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameter2StringLength);
	dir = strtol(pcParameter2, NULL, 10);
	if(dir == 0)
	{
		(void)HAL_GPIO_WritePin(DRV0_DIR_Port, DRV0_DIR_Pin, GPIO_PIN_RESET);
	}
	else
	{
		(void)HAL_GPIO_WritePin(DRV0_DIR_Port, DRV0_DIR_Pin, GPIO_PIN_SET);
	}
	strcpy(pcWriteBuffer, success);
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static BaseType_t CliMot1SetSteps(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)xWriteBufferLen;
	const char *pcParameter1;
	const char *pcParameter2;
	BaseType_t xParameter1StringLength;
	BaseType_t xParameter2StringLength;
	uint32_t steps;
	uint32_t dir;
	/* Get the step field */
	pcParameter1      = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);
	steps             = strtol(pcParameter1, NULL, 10);
	mot1_steps_shadow = steps;
	/* Get the dir field */
	pcParameter2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameter2StringLength);
	dir = strtol(pcParameter2, NULL, 10);
	if(dir == 0)
	{
		(void)HAL_GPIO_WritePin(DRV1_DIR_Port, DRV1_DIR_Pin, GPIO_PIN_RESET);
	}
	else
	{
		(void)HAL_GPIO_WritePin(DRV1_DIR_Port, DRV1_DIR_Pin, GPIO_PIN_SET);
	}
	strcpy(pcWriteBuffer, success);
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static BaseType_t CliMot2SetSteps(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)xWriteBufferLen;
	const char *pcParameter1;
	const char *pcParameter2;
	BaseType_t xParameter1StringLength;
	BaseType_t xParameter2StringLength;
	uint32_t steps;
	uint32_t dir;
	/* Get the step field */
	pcParameter1      = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);
	steps             = strtol(pcParameter1, NULL, 10);
	mot2_steps_shadow = steps;
	/* Get the dir field */
	pcParameter2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameter2StringLength);
	dir = strtol(pcParameter2, NULL, 10);
	if(dir == 0)
	{
		(void)HAL_GPIO_WritePin(DRV2_DIR_Port, DRV2_DIR_Pin, GPIO_PIN_RESET);
	}
	else
	{
		(void)HAL_GPIO_WritePin(DRV2_DIR_Port, DRV2_DIR_Pin, GPIO_PIN_SET);
	}
	strcpy(pcWriteBuffer, success);
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static BaseType_t CliSetTrinamic_AIN_IREF_Voltage(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)xWriteBufferLen;
	const char *pcParameter1;
	BaseType_t xParameter1StringLength;
	uint32_t dac_val;
	pcParameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);
	dac_val      = strtol(pcParameter1, NULL, 10);
	if ( (dac_val >= 0) && (dac_val <= 255) )
	{
		dac_output_voltage = (uint8_t) dac_val;
		DACUpdate();
		strcpy(pcWriteBuffer, success);
	}
	else
	{
		strcpy(pcWriteBuffer, wrong);
	}
	return pdFALSE;
}

/*-----------------------------------------------------------*/

static BaseType_t CliTrinamic_ReadAllRegistrs(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)pcCommandString;
	(void)xWriteBufferLen;
	const char *pcParameter1;
	BaseType_t xParameter1StringLength;
	uint32_t motor_id;
	pcParameter1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);
	motor_id = strtol(pcParameter1, NULL, 10);
	if( (TRINAMIC_MOT0_ID == motor_id) || (TRINAMIC_MOT1_ID == motor_id) || (TRINAMIC_MOT2_ID == motor_id) )
	{
		TrinamicReadAllResisters(motor_id);
		strcpy(pcWriteBuffer, success);
	}
	else
	{
		strcpy(pcWriteBuffer, wrong);
	}
	return pdFALSE;
}
