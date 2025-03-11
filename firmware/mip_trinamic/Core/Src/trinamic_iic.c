/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "trinamic_iic.h"
#include "trinamic_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void I2C1_GPIO_Init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
I2C_HandleTypeDef hi2c1;

/*******************************************************************************
 * Code
 ******************************************************************************/

void I2C1_Init(void)
{
	/* Enable clock @ I2C1 */
	RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
	__NOP();
	__NOP();

	/* I2C1 CONFIGURATION */
	hi2c1.Instance              = I2C1;
	hi2c1.Init.Timing           = 0x00100D14;
	hi2c1.Init.OwnAddress1      = 0;
	hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2      = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
	(void)HAL_I2C_Init(&hi2c1);
	(void)HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
	(void)HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
	(void)I2C1_GPIO_Init();

	/* Start up */
	I2C1->CR1 |=  I2C_CR1_PE;
}

static void I2C1_GPIO_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configure GPIO pin : IIC_SCL_Pin	 */
	GPIO_InitStruct.Pin        = IIC_SCL_Pin;
	GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate  = LL_GPIO_AF_4;
	(void)LL_GPIO_Init(IIC_SCL_Port, &GPIO_InitStruct);

	/* Configure GPIO pin : IIC_SDA_Pin */
	GPIO_InitStruct.Pin        = IIC_SDA_Pin;
	GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_UP;
	GPIO_InitStruct.Alternate  = LL_GPIO_AF_4;
	(void)LL_GPIO_Init(IIC_SDA_Port, &GPIO_InitStruct);
}
