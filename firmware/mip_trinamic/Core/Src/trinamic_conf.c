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
bool mot0_init = false;
bool mot1_init = false;
bool mot2_init = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
void TrinamicInit(void)
{
	mot0.config.motor.id = TRINAMIC_MOT0_ID; /* MOT0 */
	mot1.config.motor.id = TRINAMIC_MOT1_ID; /* MOT1 */
	mot2.config.motor.id = TRINAMIC_MOT2_ID; /* MOT2 */

	/* Set default configuration for MOT0 */
	(void)TMC2130_SetDefaults(&mot0);
	(void)TrinamicMotXEnable(TRINAMIC_MOT0_ID);

	/* Set default configuration for MOT1 */
	(void)TMC2130_SetDefaults(&mot1);
	(void)TrinamicMotXEnable(TRINAMIC_MOT1_ID);

	/* Set default configuration for MOT2 */
	(void)TMC2130_SetDefaults(&mot2);
	(void)TrinamicMotXEnable(TRINAMIC_MOT2_ID);

	(void)Delay_ms(100);
	(void)LPUART_TxPolling("\r\nBEGIN STEPPER INIT PROCEDURE\r\n");


	/* Init MOT0 */
	mot0_init = TMC2130_Init(&mot0);
	if(true == mot0_init)
	{
		(void)LPUART_TxPolling("mot0 init SUCCESS\r\n");
	}
	else
	{
		(void)LPUART_TxPolling("mot0 init FAILURE\r\n");
		(void)TrinamicMotXDisable(TRINAMIC_MOT0_ID);
	}

	/* Init MOT1 */
	mot1_init = TMC2130_Init(&mot1);
	if(true == mot1_init)
	{
		(void)LPUART_TxPolling("mot1 init SUCCESS\r\n");
	}
	else
	{
		(void)LPUART_TxPolling("mot1 init FAILURE\r\n");
		(void)TrinamicMotXDisable(TRINAMIC_MOT1_ID);
	}

	/* Init MOT2 */
	mot2_init = TMC2130_Init(&mot2);
	if(true == mot2_init)
	{
		(void)LPUART_TxPolling("mot2 init SUCCESS\r\n");
	}
	else
	{
		(void)LPUART_TxPolling("mot2 init FAILURE\r\n");
		(void)TrinamicMotXDisable(TRINAMIC_MOT2_ID);
	}

}
