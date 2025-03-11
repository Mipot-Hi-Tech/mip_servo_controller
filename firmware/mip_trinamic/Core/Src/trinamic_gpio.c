/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "trinamic_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define NWAKE0_MOD_PUSHPULL
#define DIAG_PIN_INPUT_TYPE GPIO_MODE_IT_RISING
#define DIAG_PIN_PULL_TYPE  GPIO_NOPULL

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void I2cSclRelease(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Configuration on GPIO Pins
 * @param[in] None.
 * @return    None.
 * @retval    Void.
 */
void TrinamicGPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	(void)HAL_GPIO_WritePin(DRV0_SPI_NSS_Port, DRV0_SPI_NSS_Pin, GPIO_PIN_SET);
	(void)HAL_GPIO_WritePin(DRV1_SPI_NSS_Port, DRV1_SPI_NSS_Pin, GPIO_PIN_SET);
	(void)HAL_GPIO_WritePin(DRV2_SPI_NSS_Port, DRV2_SPI_NSS_Pin, GPIO_PIN_SET);

	/* Configure GPIO pin : DRV0_SPI_NSS_Pin */
	GPIO_InitStruct.Pin   = DRV0_SPI_NSS_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP ;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	(void)HAL_GPIO_Init(DRV0_SPI_NSS_Port, &GPIO_InitStruct);

	/* Configure GPIO pin : DRV1_SPI_NSS_Pin */
	GPIO_InitStruct.Pin   = DRV1_SPI_NSS_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP ;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	(void)HAL_GPIO_Init(DRV1_SPI_NSS_Port, &GPIO_InitStruct);

	/* Configure GPIO pin : DRV2_SPI_NSS_Pin */
	GPIO_InitStruct.Pin   = DRV2_SPI_NSS_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP ;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	(void)HAL_GPIO_Init(DRV2_SPI_NSS_Port, &GPIO_InitStruct);

	(void)HAL_GPIO_WritePin(DRV0_DIR_Port, DRV0_DIR_Pin, GPIO_PIN_SET);
	(void)HAL_GPIO_WritePin(DRV1_DIR_Port, DRV1_DIR_Pin, GPIO_PIN_SET);
	(void)HAL_GPIO_WritePin(DRV2_DIR_Port, DRV2_DIR_Pin, GPIO_PIN_SET);

	/* Configure GPIO pin : DRV0_DIR_Pin */
	GPIO_InitStruct.Pin   = DRV0_DIR_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP ;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	(void)HAL_GPIO_Init(DRV0_DIR_Port, &GPIO_InitStruct);

	/* Configure GPIO pin : DRV1_DIR_Pin */
	GPIO_InitStruct.Pin   = DRV1_DIR_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP ;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	(void)HAL_GPIO_Init(DRV1_DIR_Port, &GPIO_InitStruct);

	/* Configure GPIO pin : DRV2_DIR_Pin */
	GPIO_InitStruct.Pin   = DRV2_DIR_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP ;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	(void)HAL_GPIO_Init(DRV2_DIR_Port, &GPIO_InitStruct);

	/* Disable the Driver */
	(void)HAL_GPIO_WritePin(DRV0_EN_Port, DRV0_EN_Pin, GPIO_PIN_SET);
	(void)HAL_GPIO_WritePin(DRV1_EN_Port, DRV1_EN_Pin, GPIO_PIN_SET);
	(void)HAL_GPIO_WritePin(DRV2_EN_Port, DRV2_EN_Pin, GPIO_PIN_SET);

	/* Configure GPIO pin : DRV0_EN_Pin */
	GPIO_InitStruct.Pin   = DRV0_EN_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP ;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	(void)HAL_GPIO_Init(DRV0_EN_Port, &GPIO_InitStruct);

	/* Configure GPIO pin : DRV1_EN_Pin */
	GPIO_InitStruct.Pin   = DRV1_EN_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP ;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	(void)HAL_GPIO_Init(DRV1_EN_Port, &GPIO_InitStruct);

	/* Configure GPIO pin : DRV2_EN_Pin */
	GPIO_InitStruct.Pin   = DRV2_EN_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP ;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	(void)HAL_GPIO_Init(DRV2_EN_Port, &GPIO_InitStruct);

	/* Configure GPIO pins : DRV0_DIAGNOSTIC_Pin */
	GPIO_InitStruct.Pin   = DRV0_DIAGNOSTIC_Pin;
	GPIO_InitStruct.Mode  = DIAG_PIN_INPUT_TYPE;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	(void)HAL_GPIO_Init(DRV0_DIAGNOSTIC_Port, &GPIO_InitStruct);

	/* Configure GPIO pins : DRV1_DIAGNOSTIC_Pin */
	GPIO_InitStruct.Pin   = DRV0_DIAGNOSTIC_Pin;
	GPIO_InitStruct.Mode  = DIAG_PIN_INPUT_TYPE;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	(void)HAL_GPIO_Init(DRV1_DIAGNOSTIC_Port, &GPIO_InitStruct);

	/* Configure GPIO pin : DRV2_DIAGNOSTIC_Pin */
	GPIO_InitStruct.Pin   = DRV0_DIAGNOSTIC_Pin;
	GPIO_InitStruct.Mode  = DIAG_PIN_INPUT_TYPE;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	(void)HAL_GPIO_Init(DRV2_DIAGNOSTIC_Port, &GPIO_InitStruct);

	/* Configure GPIO pin : GPIO0_NWAKE_Pin */
#ifdef NWAKE0_MOD_PUSHPULL
	(void)HAL_GPIO_WritePin(GPIO0_NWAKE_Port, GPIO0_NWAKE_Pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin   = GPIO0_NWAKE_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	(void)HAL_GPIO_Init(GPIO0_NWAKE_Port, &GPIO_InitStruct);
#else
	GPIO_InitStruct.Pin   = GPIO0_NWAKE_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	HAL_GPIO_Init(GPIO0_NWAKE_Port, &GPIO_InitStruct);
#endif
    /* Configure GPIO pin : IIC_SDA_Pin IIC_SCL_Pin */
	(void)I2cSclRelease();
	/* Configure GPIO pin : SPI2_MOSI_Pin, SPI2_MISO_Pin, SPI2_CLK_Pin */
	/* EXTI interrupt init*/
	//HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
	//HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void TrinamicNSS_Select(uint8_t driver)
{
	/* Drive NSS Pin ON selected stepper driver */
	if (driver == TRINAMIC_MOT0_ID)
	{
		(void)HAL_GPIO_WritePin(DRV0_SPI_NSS_Port, DRV0_SPI_NSS_Pin, GPIO_PIN_RESET);
		(void)HAL_GPIO_WritePin(DRV1_SPI_NSS_Port, DRV1_SPI_NSS_Pin, GPIO_PIN_SET);
		(void)HAL_GPIO_WritePin(DRV2_SPI_NSS_Port, DRV2_SPI_NSS_Pin, GPIO_PIN_SET);
	}
	else if (driver == TRINAMIC_MOT1_ID)
	{
		(void)HAL_GPIO_WritePin(DRV0_SPI_NSS_Port, DRV0_SPI_NSS_Pin, GPIO_PIN_SET);
		(void)HAL_GPIO_WritePin(DRV1_SPI_NSS_Port, DRV1_SPI_NSS_Pin, GPIO_PIN_RESET);
		(void)HAL_GPIO_WritePin(DRV2_SPI_NSS_Port, DRV2_SPI_NSS_Pin, GPIO_PIN_SET);
	}
	else if (driver == TRINAMIC_MOT2_ID)
	{
		(void)HAL_GPIO_WritePin(DRV0_SPI_NSS_Port, DRV0_SPI_NSS_Pin, GPIO_PIN_SET);
		(void)HAL_GPIO_WritePin(DRV1_SPI_NSS_Port, DRV1_SPI_NSS_Pin, GPIO_PIN_SET);
		(void)HAL_GPIO_WritePin(DRV2_SPI_NSS_Port, DRV2_SPI_NSS_Pin, GPIO_PIN_RESET);
	}
	else
	{
		;
	}
}

void TrinamicNSS_Idle(void)
{
	(void)HAL_GPIO_WritePin(DRV0_SPI_NSS_Port, DRV0_SPI_NSS_Pin, GPIO_PIN_SET);
	(void)HAL_GPIO_WritePin(DRV1_SPI_NSS_Port, DRV1_SPI_NSS_Pin, GPIO_PIN_SET);
	(void)HAL_GPIO_WritePin(DRV2_SPI_NSS_Port, DRV2_SPI_NSS_Pin, GPIO_PIN_SET);
}

void TrinamicMotXEnable(uint8_t driver)
{
	/* From TMC2130 TMC2130 DATASHEET (Rev. 1.16 / 2023-MAR-08) PAR 2.2 PAGE 11 */
	/* The power stage becomes switched off (all motor outputs floating) when this pin becomes driven to a high level. */
	if (driver == TRINAMIC_MOT0_ID)
	{
		(void)HAL_GPIO_WritePin(DRV0_EN_Port, DRV0_EN_Pin, GPIO_PIN_RESET);
	}
	else if (driver == TRINAMIC_MOT1_ID)
	{
		(void)HAL_GPIO_WritePin(DRV2_EN_Port, DRV2_EN_Pin, GPIO_PIN_RESET);

	}
	else if (driver == TRINAMIC_MOT2_ID)
	{
		(void)HAL_GPIO_WritePin(DRV2_EN_Port, DRV2_EN_Pin, GPIO_PIN_RESET);
	}
	else
	{
		;
	}
}

void TrinamicMotXDisable(uint8_t driver)
{
	/* From TMC2130 TMC2130 DATASHEET (Rev. 1.16 / 2023-MAR-08) PAR 2.2 PAGE 11 */
	/* The power stage becomes switched off (all motor outputs floating) when this pin becomes driven to a high level. */
	if (driver == TRINAMIC_MOT0_ID)
	{
		(void)HAL_GPIO_WritePin(DRV0_EN_Port, DRV0_EN_Pin, GPIO_PIN_SET);
	}
	else if (driver == TRINAMIC_MOT1_ID)
	{
		(void)HAL_GPIO_WritePin(DRV2_EN_Port, DRV2_EN_Pin, GPIO_PIN_SET);

	}
	else if (driver == TRINAMIC_MOT2_ID)
	{
		(void)HAL_GPIO_WritePin(DRV2_EN_Port, DRV2_EN_Pin, GPIO_PIN_SET);
	}
	else
	{
		;
	}
}

static void I2cSclRelease(void)
{
	uint8_t i;
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	(void)HAL_GPIO_WritePin(IIC_SCL_Port, IIC_SCL_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = IIC_SCL_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    (void)HAL_GPIO_Init(IIC_SCL_Port, &GPIO_InitStruct);
    for(i=0;i<9;i++)
    {
    	(void)HAL_GPIO_WritePin(IIC_SCL_Port, IIC_SCL_Pin, GPIO_PIN_SET);
    	(void)HAL_GPIO_WritePin(IIC_SCL_Port, IIC_SCL_Pin, GPIO_PIN_RESET);
    }
}
