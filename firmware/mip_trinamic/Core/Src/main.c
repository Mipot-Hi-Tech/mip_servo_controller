/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "stm32_lpm.h"
#include "utilities_def.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "trinamic_spi.h"
#include "trinamic_gpio.h"
#include "trinamic_pulse_gen.h"
#include "trinamic_uart.h"
#include "trinamic_tim.h"
#include "trinamic_iic.h"
#include "trinamic_conf.h"
#include "trinamic_dac.h"
#include "app_diagnosis.h"
#include "app_cli.h"
#include "ipcc.h"
#include "app_mipd.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void SystemClock_Config(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void vApplicationIdleHook(void)
{
	/* User adds here LOW POWER features */
	;
}

int main(void)
{

	/* Set default state from potential system reset: Clear Stop2 flag of CPU1 */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOP2);
	/* Boot CPU2 */
	(void)HAL_PWREx_ReleaseCore(PWR_CORE_CPU2);
	/* Wait until cpu2 is ready to receive commands */
	while (*cpu2InitDone != CPU2_INITIALISED)
	{
		__asm("NOP");
	}
	/* Low Level clock configuration */
	(void)HAL_Init();
	(void)SystemClock_Config();
	(void)TIM16_Init();
	Delay_ms(1000);
	(void)IPCC_Init();
	(void)TIM2_Init();
	(void)LPUART_Init();
	(void)SPI2_Init();
	(void)I2C1_Init();
	(void)DAC_Init();
	(void)TrinamicGPIO_Init();
	(void)TrinamicInit();
	/* Starting Tasks */
	(void)vTaskCli();
	(void)MipdTask();
	/* Enable FreeRTOS */
	(void)vTaskStartScheduler();
	for(;;);
}

static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/* Configure the main internal regulator output voltage */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* MSI is enabled after System reset, activate PLL with MSI as source */
	RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI |
									  RCC_OSCILLATORTYPE_HSE |RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_PWR;
	RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11; /*System Clock set at 48 MHz*/

	/*Activate PLL, HSE clock source, System Clock 48 MHz*/
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 12;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
	CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
	CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_ENABLE_Msk);

	/**Initializes the CPU, AHB and APB busses clocks*/

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
								 RCC_CLOCKTYPE_HCLK2 | RCC_CLOCKTYPE_PCLK1 |
								 RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_HCLK3);
	RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
	CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
	CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_ENABLE_Msk);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_TICKINT_Msk);
	CLEAR_BIT(SysTick->CTRL,SysTick_CTRL_ENABLE_Msk);
}
