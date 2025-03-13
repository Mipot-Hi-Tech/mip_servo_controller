/*******************************************************************************
 * Included files
 *****************************************************************************/
#include <stdio.h>
#include "trinamic_diagnosis.h"
#include "tmc2130.h"
#include "trinamic_uart.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void ReadDrvStatus(TMC2130_t *driver);

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern TMC2130_t mot0;
extern TMC2130_t mot1;
extern TMC2130_t mot2;
extern bool mot0_init;
extern bool mot1_init;
extern bool mot2_init;

/*******************************************************************************
 * Code
 ******************************************************************************/
void TrinamicReadDrvStatus(void)
{
	if (true == mot0_init)
	{
		(void)LPUART_TxPolling("Checking MOT0\r\n");
		(void)ReadDrvStatus(&mot0);
	}
	if( true == mot1_init)
	{
		(void)LPUART_TxPolling("Checking MOT1\r\n");
		(void)ReadDrvStatus(&mot1);
	}
	if( true == mot2_init)
	{
		(void)LPUART_TxPolling("Checking MOT2\r\n");
		(void)ReadDrvStatus(&mot2);
	}
}

static void ReadDrvStatus(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->drv_status);
	if(0 == stat)
	{
		if(driver->drv_status.reg.olb != 0)
		{
			(void)LPUART_TxPolling("DETECTED Open Load @ phase B\r\n");
		}
		if(driver->drv_status.reg.ola != 0)
		{
			(void)LPUART_TxPolling("DETECTED Open Load @ phase A\r\n");
		}
		if(driver->drv_status.reg.otpw != 0)
		{
			(void)LPUART_TxPolling("DETECTED Overtemperature pre-warning \r\n");
		}
		if(driver->drv_status.reg.otpw != 0)
		{
			(void)LPUART_TxPolling("DETECTED Overtemperature limit \r\n");
		}
		if(driver->drv_status.reg.stallGuard != 0)
		{
			(void)LPUART_TxPolling("DETECTED motor stall \r\n");
		}
		sprintf(buff, "DRVSTATUS: 0x%08X\r\n\r\n", (unsigned int)driver->drv_status.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}
