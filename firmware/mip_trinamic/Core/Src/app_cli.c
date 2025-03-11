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

/*******************************************************************************
 * Variables
 ******************************************************************************/
const CLI_Command_Definition_t xCommandList[] = {
	{
		.pcCommand                   = "fw",
		.pcHelpString                = "fw:Get the current firmware version\r\n",
		.pxCommandInterpreter        = CliGetTrinamicMipFwVersion,
		.cExpectedNumberOfParameters = 0
	},
	{
		.pcCommand                   = "mot0",
		.pcHelpString                = "mot0: Set the steps number\r\n",
		.pxCommandInterpreter        = CliMot0SetSteps,
		.cExpectedNumberOfParameters = 1
	},
	{
		.pcCommand                   = "mot1",
		.pcHelpString                = "mot1: Set the steps number\r\n",
		.pxCommandInterpreter        = CliMot1SetSteps,
		.cExpectedNumberOfParameters = 1
	},
	{
		.pcCommand                   = "mot2",
		.pcHelpString                = "mot2: Set the steps number\r\n",
		.pxCommandInterpreter        = CliMot2SetSteps,
		.cExpectedNumberOfParameters = 1
	},
	{
		.pcCommand                   = "dac",
		.pcHelpString                = "dac: Set the voltage @ all steppers AIN_IREF pins \r\n",
		.pxCommandInterpreter        = CliSetTrinamic_AIN_IREF_Voltage,
		.cExpectedNumberOfParameters = 1
	},
    {
        .pcCommand = NULL
    }
};

const char * cli_prompt = "\r\ntrinamic_mip> ";
const char * wrong      = "ERR";
const char * success    = "SUCCESS";
const char * mip_fail   = "MIP MODULE NOT PRESENT!";
/* CLI escape sequences */
uint8_t backspace[] = "\b \b";
uint8_t backspace_tt[] = " \b";
char cOutputBuffer[configCOMMAND_INT_MAX_OUTPUT_SIZE];
char pcInputString[MAX_INPUT_LENGTH];

extern char cRxedChar;
extern uint32_t mot0_steps;
extern uint32_t mot0_steps_shadow;
extern uint32_t mot1_steps;
extern uint32_t mot1_steps_shadow;
extern uint32_t mot2_steps;
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

static void AppCli(void *pvParameters)
{
    uint8_t cInputIndex = 0;
    uint32_t receivedValue;
    (void)vRegisterCLICommands();
    for (;;)
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
    }
    (void)vTaskDelete(NULL);
}

static void cliWrite(const char *str)
{
	(void)LPUART_TxPolling(str);
}

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
}

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

static void vRegisterCLICommands(void)
{
    for (int i = 0; xCommandList[i].pcCommand != NULL; i++)
    {
    	(void)FreeRTOS_CLIRegisterCommand(&xCommandList[i]);
    }
}

static BaseType_t CliGetTrinamicMipFwVersion(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)pcCommandString;
	(void)xWriteBufferLen;
	sprintf(pcWriteBuffer, "fwver: %u", FW_VERSION);
	return pdFALSE;
}

static BaseType_t CliMot0SetSteps(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)xWriteBufferLen;
	const char *pcParameter1;
	BaseType_t xParameter1StringLength;
	uint32_t steps;
	pcParameter1      = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);
	steps             = strtol(pcParameter1, NULL, 10);
	mot0_steps_shadow = steps;
	strcpy(pcWriteBuffer, success);
	return pdFALSE;
}

static BaseType_t CliMot1SetSteps(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)xWriteBufferLen;
	const char *pcParameter1;
	BaseType_t xParameter1StringLength;
	uint32_t steps;
	pcParameter1      = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);
	steps             = strtol(pcParameter1, NULL, 10);
	mot1_steps_shadow = steps;
	strcpy(pcWriteBuffer, success);
	return pdFALSE;
}

static BaseType_t CliMot2SetSteps(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)xWriteBufferLen;
	const char *pcParameter1;
	BaseType_t xParameter1StringLength;
	uint32_t steps;
	pcParameter1      = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameter1StringLength);
	steps             = strtol(pcParameter1, NULL, 10);
	mot2_steps_shadow = steps;
	strcpy(pcWriteBuffer, success);
	return pdFALSE;
}

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
