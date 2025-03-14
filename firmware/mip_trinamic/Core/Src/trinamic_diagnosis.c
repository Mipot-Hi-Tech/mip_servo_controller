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
#include <stdio.h>
#include "trinamic_gpio.h"
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
static void ReadGCONF(TMC2130_t *driver);
static void ReadGSTAT(TMC2130_t *driver);
static void ReadIHOLD_IRUN(TMC2130_t *driver);
static void ReadTPOWER_DOWN(TMC2130_t *driver);
static void ReadTSTEP(TMC2130_t *driver);
static void ReadTPWMTHRS(TMC2130_t *driver);
static void ReadTCOOLTHRS(TMC2130_t *driver);
static void ReadTHIGH(TMC2130_t *driver);
static void ReadXDIRECT(TMC2130_t *driver);
static void ReadVDCMIN(TMC2130_t *driver);
static void ReadMSCNT(TMC2130_t *driver);
static void ReadMSCURACT(TMC2130_t *driver);
static void ReadLOST_STEPS(TMC2130_t *driver);
static void ReadCHOPCONF(TMC2130_t *driver);
static void ReadCOOLCONF(TMC2130_t *driver);

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern TMC2130_t mot0;
extern TMC2130_t mot1;
extern TMC2130_t mot2;
extern bool mot0_init_status;
extern bool mot1_init_status;
extern bool mot2_init_status;

/*******************************************************************************
 * Code
 ******************************************************************************/
uint8_t TrinamicReadAllResisters(uint8_t mot_id)
{
	uint8_t retval = 0;
	if(TRINAMIC_MOT0_ID == mot_id)
	{
		if(true == mot0_init_status)
		{
			(void)LPUART_TxPolling("Checking MOT0\r\n");
			(void)ReadDrvStatus(&mot0);
			(void)ReadGCONF(&mot0);
			(void)ReadGSTAT(&mot0);
			(void)ReadIHOLD_IRUN(&mot0);
			(void)ReadTPOWER_DOWN(&mot0);
			(void)ReadTSTEP(&mot0);
			(void)ReadTPWMTHRS(&mot0);
			(void)ReadTCOOLTHRS(&mot0);
			(void)ReadTHIGH(&mot0);
			(void)ReadXDIRECT(&mot0);
			(void)ReadVDCMIN(&mot0);
			(void)ReadMSCNT(&mot0);
			(void)ReadMSCURACT(&mot0);
			(void)ReadLOST_STEPS(&mot0);
			(void)ReadCHOPCONF(&mot0);
			(void)ReadCOOLCONF(&mot0);
		}
		else
		{
			(void)LPUART_TxPolling("MOT0 not initialized\r\n");
			retval++;
		}
	}
	if(TRINAMIC_MOT1_ID == mot_id)
	{
		if(true == mot1_init_status)
		{
			(void)LPUART_TxPolling("Checking MOT1\r\n");
			(void)ReadDrvStatus(&mot1);
			(void)ReadGCONF(&mot1);
			(void)ReadGSTAT(&mot1);
			(void)ReadIHOLD_IRUN(&mot1);
			(void)ReadTPOWER_DOWN(&mot1);
			(void)ReadTSTEP(&mot1);
			(void)ReadTPWMTHRS(&mot1);
			(void)ReadTCOOLTHRS(&mot1);
			(void)ReadTHIGH(&mot1);
			(void)ReadXDIRECT(&mot1);
			(void)ReadVDCMIN(&mot1);
			(void)ReadMSCNT(&mot1);
			(void)ReadMSCURACT(&mot1);
			(void)ReadLOST_STEPS(&mot1);
			(void)ReadCHOPCONF(&mot1);
			(void)ReadCOOLCONF(&mot1);
		}
		else
		{
			(void)LPUART_TxPolling("MOT1 not initialized\r\n");
			retval++;
		}
	}
	if(TRINAMIC_MOT2_ID == mot_id)
	{
		if(true == mot2_init_status)
		{
			(void)LPUART_TxPolling("Checking MOT2\r\n");
			(void)ReadDrvStatus(&mot2);
			(void)ReadGCONF(&mot2);
			(void)ReadGSTAT(&mot2);
			(void)ReadIHOLD_IRUN(&mot2);
			(void)ReadTPOWER_DOWN(&mot2);
			(void)ReadTSTEP(&mot2);
			(void)ReadTPWMTHRS(&mot2);
			(void)ReadTCOOLTHRS(&mot2);
			(void)ReadTHIGH(&mot2);
			(void)ReadXDIRECT(&mot2);
			(void)ReadVDCMIN(&mot2);
			(void)ReadMSCNT(&mot2);
			(void)ReadMSCURACT(&mot2);
			(void)ReadLOST_STEPS(&mot2);
			(void)ReadCHOPCONF(&mot2);
			(void)ReadCOOLCONF(&mot2);
		}
		else
		{
			(void)LPUART_TxPolling("MOT2 not initialized\r\n");
			retval++;
		}
	}
	return retval;
}

/*-----------------------------------------------------------*/

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
		sprintf(buff, "DRVSTATUS: 0x%08X\r\n", (unsigned int)driver->drv_status.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadGCONF(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->gconf);
	if(0 == stat)
	{
		sprintf(buff, "GCONF: 0x%08X\r\n", (unsigned int)driver->gconf.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadGSTAT(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->gstat);
	if(0 == stat)
	{
		if(driver->gstat.reg.reset != 0)
		{
			(void)LPUART_TxPolling("IC RESET Detected\r\n");
		}
		if(driver->gstat.reg.drv_err != 0)
		{
			(void)LPUART_TxPolling("IC SHUT DOWN Detected\r\n");
		}
		if(driver->gstat.reg.uv_cp != 0)
		{
			(void)LPUART_TxPolling("Charge pump UNDERVOLTAGE Detected\r\n");
		}
		sprintf(buff, "GSTAT: 0x%08X\r\n", (unsigned int)driver->gstat.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadIHOLD_IRUN(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->ihold_irun);
	if(0 == stat)
	{
		sprintf(buff, "IHOLD_IRUN: 0x%08X\r\n", (unsigned int)driver->ihold_irun.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadTPOWER_DOWN(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->tpowerdown);
	if(0 == stat)
	{
		sprintf(buff, "TPOWER_DOWN: 0x%08X\r\n", (unsigned int)driver->tpowerdown.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadTSTEP(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->tstep);
	if(0 == stat)
	{
		sprintf(buff, "TPOWER_DOWN: 0x%08X\r\n", (unsigned int)driver->tstep.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadTPWMTHRS(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->tpwmthrs);
	if(0 == stat)
	{
		sprintf(buff, "TPWMTHRS: 0x%08X\r\n", (unsigned int)driver->tpwmthrs.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadTCOOLTHRS(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->tcoolthrs);
	if(0 == stat)
	{
		sprintf(buff, "TCOOLTHRS: 0x%08X\r\n", (unsigned int)driver->tcoolthrs.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadTHIGH(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->thigh);
	if(0 == stat)
	{
		sprintf(buff, "THIGH: 0x%08X\r\n", (unsigned int)driver->thigh.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadXDIRECT(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->xdirect);
	if(0 == stat)
	{
		sprintf(buff, "XDIRECT: 0x%08X\r\n", (unsigned int)driver->xdirect.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadVDCMIN(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->vdcmin);
	if(0 == stat)
	{
		sprintf(buff, "VDCMIN: 0x%08X\r\n", (unsigned int)driver->vdcmin.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadMSCNT(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->mscnt);
	if(0 == stat)
	{
		sprintf(buff, "MSCNT: 0x%08X\r\n", (unsigned int)driver->mscnt.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadMSCURACT(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->mscuract);
	if(0 == stat)
	{
		sprintf(buff, "MSCURACT: 0x%08X\r\n", (unsigned int)driver->mscuract.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadLOST_STEPS(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->lost_steps);
	if(0 == stat)
	{
		sprintf(buff, "LOST_STEPS: 0x%08X\r\n", (unsigned int)driver->lost_steps.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadCHOPCONF(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->chopconf);
	if(0 == stat)
	{
		sprintf(buff, "CHOPCONF: 0x%08X\r\n", (unsigned int)driver->chopconf.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}

/*-----------------------------------------------------------*/

static void ReadCOOLCONF(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
	char buff[50];
	stat = tmc_spi_read(driver->config.motor, (TMC_spi_datagram_t *)&driver->coolconf);
	if(0 == stat)
	{
		sprintf(buff, "COOLCONF: 0x%08X\r\n", (unsigned int)driver->coolconf.reg.value);
		(void)LPUART_TxPolling(buff);
	}
}
