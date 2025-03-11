/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "app_diagnosis.h"
#include "sys_task.h"
#include "tmc2130.h"
#include "trinamic_uart.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void AppDiagnosis(void *pvParameters);
static void TrinamicReadDrvStatus(TMC2130_t *driver);

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
void vTaskStepperDiagnosis(void)
{
	if ( (true == mot0_init) || (true == mot1_init) || (true == mot2_init) )
	{
		/* Run task only if one of the steppers has completed initialization */
		(void)LPUART_TxPolling("Creating stepper diagnosis task\r\n");
		(void)xTaskCreate(AppDiagnosis,"AppDiagnosis",250, NULL, DIAGNOSIS_TASK_PRIORITY, &stepper_diagnosis_task_handle);
	}
	else
	{
		(void)LPUART_TxPolling("Skipping stepper diagnosis task\r\n");
	}
}

static void AppDiagnosis(void *pvParameters)
{
	const TickType_t xDelayDiagnosis = pdMS_TO_TICKS(10000);
    for (;;)
    {
    	if (true == mot0_init)
    	{
    		(void)LPUART_TxPolling("Checking MOT0\r\n");
    		(void)TrinamicReadDrvStatus(&mot0);
    	}
    	if( true == mot1_init)
    	{
    		(void)LPUART_TxPolling("Checking MOT1\r\n");
    		(void)TrinamicReadDrvStatus(&mot1);
    	}
    	if( true == mot2_init)
    	{
    		(void)LPUART_TxPolling("Checking MOT2\r\n");
    		(void)TrinamicReadDrvStatus(&mot2);
    	}
    	(void)vTaskDelay(xDelayDiagnosis);
    }
    (void)vTaskDelete(NULL);
}

static void TrinamicReadDrvStatus(TMC2130_t *driver)
{
	TMC_spi_status_t stat;
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
			(void)LPUART_TxPolling("DETECTED motor srall \r\n");
		}
	}
}
