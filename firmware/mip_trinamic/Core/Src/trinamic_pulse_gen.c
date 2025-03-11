/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "trinamic_pulse_gen.h"
#include "trinamic_gpio.h"
#include <stdbool.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PULSE_GEN_GPIO_SPEED GPIO_SPEED_FREQ_HIGH
#define PULSE_GEN_PULL_TYPE  GPIO_PULLDOWN
#define TIM2_GPIO_DEBUG
#define TIM2_PERIOD 47999

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void TIM2_GPIO_Init(void);
static inline void HandleMot0StepPulseGeneration(void);
static inline void HandleMot1StepPulseGeneration(void);
static inline void HandleMot2StepPulseGeneration(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
TIM_HandleTypeDef htim2;
volatile uint32_t tim2_cnt   = 0;
volatile uint32_t mot0_steps        = 0;
volatile uint32_t mot1_steps        = 0;
volatile uint32_t mot2_steps        = 0;
volatile uint32_t mot0_steps_shadow = 0;
volatile uint32_t mot1_steps_shadow = 0;
volatile uint32_t mot2_steps_shadow = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
void TIM2_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_SlaveConfigTypeDef sSlaveConfig       = {0};
	TIM_MasterConfigTypeDef sMasterConfig     = {0};
	TIM_OC_InitTypeDef sConfigOC              = {0};

	/* Enable clock @ TIM2 */
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	__NOP();
	__NOP();

	/* TIM2 CONFIGURATION */
	htim2.Instance               = TIM2;
	htim2.Init.Prescaler         = 0;
	htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim2.Init.Period            = TIM2_PERIOD;
	htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	(void)HAL_TIM_Base_Init(&htim2);

	/* TIM2 clocked @ 48 MHz */
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	(void)HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
	(void)HAL_TIM_PWM_Init(&htim2);

	sSlaveConfig.SlaveMode    = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	(void)HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
	(void)HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

	sConfigOC.OCMode     = TIM_OCMODE_PWM1;
	sConfigOC.Pulse      = TIM2_PERIOD-1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	(void)HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
	(void)HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
	(void)HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

	(void)TIM2_GPIO_Init();
	(void)HAL_NVIC_EnableIRQ(TIM2_IRQn);
	(void)HAL_TIM_Base_Start_IT(&htim2);
	(void)HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_1); /* Routed to DRV2_STEP GPIO */
	(void)HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_3); /* Routed to DRV0_STEP GPIO */
	(void)HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_4); /* Routed to DRV1_STEP GPIO */
	__HAL_TIM_ENABLE(&htim2);
}

void TIM2_IRQHandler(void)
{
#ifdef TIM2_GPIO_DEBUG
	//(void)HAL_GPIO_WritePin(GPIO0_NWAKE_Port, GPIO0_NWAKE_Pin,1);
	(void)HAL_GPIO_TogglePin(GPIO0_NWAKE_Port, GPIO0_NWAKE_Pin);
	//(void)HAL_GPIO_WritePin(GPIO0_NWAKE_Port, GPIO0_NWAKE_Pin,0);
#endif
	HandleMot0StepPulseGeneration();
	HandleMot1StepPulseGeneration();
	HandleMot2StepPulseGeneration();
	tim2_cnt++;
	__HAL_TIM_CLEAR_IT(&htim2 ,TIM_IT_UPDATE);
}

static inline void HandleMot0StepPulseGeneration(void)
{
	if(mot0_steps != 0)
	{
		mot0_steps--;
		if(mot0_steps == 0)
		{
			/* Disable output on TIM_CHANNEL_3 */
			TIM2->CCER &= ~TIM_CCER_CC3E;
		}
	}
	if( (mot0_steps != mot0_steps_shadow) && (mot0_steps_shadow != 0) )
	{
		mot0_steps        = mot0_steps_shadow;
		mot0_steps_shadow = 0;
		/* Enable output on TIM_CHANNEL_3 */
		TIM2->CCER |= TIM_CCER_CC3E;
	}
}

static inline void HandleMot1StepPulseGeneration(void)
{
	if(mot1_steps != 0)
	{
		mot1_steps--;
		if(mot1_steps == 0)
		{
			/* Disable output on TIM_CHANNEL_4 */
			TIM2->CCER &= ~TIM_CCER_CC4E;
		}
	}
	if( (mot1_steps != mot1_steps_shadow) && (mot1_steps_shadow != 0) )
	{
		mot1_steps        = mot1_steps_shadow;
		mot1_steps_shadow = 0;
		/* Enable output on TIM_CHANNEL_4 */
		TIM2->CCER |= TIM_CCER_CC4E;
	}
}

static inline void HandleMot2StepPulseGeneration(void)
{
	if(mot2_steps != 0)
	{
		mot2_steps--;
		if(mot2_steps == 0)
		{
			/* Disable output on TIM_CHANNEL_1 */
			TIM2->CCER &= ~TIM_CCER_CC1E;
		}
	}
	if( (mot2_steps != mot2_steps_shadow) && (mot2_steps_shadow != 0) )
	{
		mot2_steps        = mot2_steps_shadow;
		mot2_steps_shadow = 0;
		/* Enable output on TIM_CHANNEL_1 */
		TIM2->CCER |= TIM_CCER_CC1E;
	}
}

static void TIM2_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /* Configure GPIO pin : DRV0_STEP_Pin -> TIM2 CHANNEL 3 */
    GPIO_InitStruct.Pin       = DRV0_STEP_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = PULSE_GEN_PULL_TYPE;
    GPIO_InitStruct.Speed     = PULSE_GEN_GPIO_SPEED;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    (void)HAL_GPIO_Init(DRV0_STEP_Port, &GPIO_InitStruct);

    /* Configure GPIO pin : DRV1_STEP_Pin -> TIM2 CHANNEL 4 */
    GPIO_InitStruct.Pin       = DRV1_STEP_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = PULSE_GEN_PULL_TYPE;
    GPIO_InitStruct.Speed     = PULSE_GEN_GPIO_SPEED;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    (void)HAL_GPIO_Init(DRV1_STEP_Port, &GPIO_InitStruct);

    /* Configure GPIO pin : DRV2_STEP_Pin -> TIM2 CHANNEL 1 */
    GPIO_InitStruct.Pin       = DRV2_STEP_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = PULSE_GEN_PULL_TYPE;
    GPIO_InitStruct.Speed     = PULSE_GEN_GPIO_SPEED;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    (void)HAL_GPIO_Init(DRV2_STEP_Port, &GPIO_InitStruct);
}
