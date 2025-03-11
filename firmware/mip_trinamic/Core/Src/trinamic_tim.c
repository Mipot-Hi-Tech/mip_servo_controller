/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "trinamic_tim.h"
#include "trinamic_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
//#define TIM16_GPIO_DEBUG

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
TIM_HandleTypeDef htim16;
volatile uint32_t tim16_cnt = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
void TIM16_Init(void)
{
	/* Enable clock @ TIM16 */
	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	__NOP();
	__NOP();

	/* General purpose timer generating irq @ 1ms */
	htim16.Instance               = TIM16;
	htim16.Init.Prescaler         = 0;
	htim16.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim16.Init.Period            = 47999;
	htim16.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	(void)HAL_TIM_Base_Init(&htim16);
	(void)HAL_NVIC_EnableIRQ(TIM16_IRQn);
	(void)HAL_TIM_Base_Start_IT(&htim16);
}

uint32_t GetTick(void)
{
	return tim16_cnt;
}

void Delay_ms(uint32_t ms)
{
	uint32_t ticks =  GetTick();
	while( (GetTick() - ticks) < ms);
}

void TIM16_IRQHandler(void)
{
	__HAL_TIM_CLEAR_IT(&htim16 ,TIM_IT_UPDATE);
#ifdef TIM16_GPIO_DEBUG
	(void)HAL_GPIO_TogglePin(GPIO0_NWAKE_Port, GPIO0_NWAKE_Pin);
#endif
	 tim16_cnt++;
}
