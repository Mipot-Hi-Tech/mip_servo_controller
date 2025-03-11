#ifndef CORE_INC_TRINAMIC_GPIO_H_
#define CORE_INC_TRINAMIC_GPIO_H_

/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "main.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define TRINAMIC_MOT0_ID 0
#define TRINAMIC_MOT1_ID 1
#define TRINAMIC_MOT2_ID 2

/* MOT 0 */
#define DRV0_STEP_Pin			GPIO_PIN_10
#define DRV0_STEP_Port 			GPIOB

#define DRV0_DIR_Pin			GPIO_PIN_6
#define DRV0_DIR_Port  			GPIOA

#define DRV0_EN_Pin				GPIO_PIN_5
#define DRV0_EN_Port  			GPIOA

#define DRV0_DIAGNOSTIC_Pin		GPIO_PIN_2
#define DRV0_DIAGNOSTIC_Port  	GPIOB

#define DRV0_SPI_NSS_Pin		GPIO_PIN_4
#define DRV0_SPI_NSS_Port  		GPIOA

/* MOT 1 */
#define DRV1_STEP_Pin			GPIO_PIN_11
#define DRV1_STEP_Port 			GPIOB

#define DRV1_DIR_Pin			GPIO_PIN_14
#define DRV1_DIR_Port  			GPIOB

#define DRV1_EN_Pin		        GPIO_PIN_4
#define DRV1_EN_Port	        GPIOB

#define DRV1_DIAGNOSTIC_Pin		GPIO_PIN_3
#define DRV1_DIAGNOSTIC_Port	GPIOA

#define DRV1_SPI_NSS_Pin		GPIO_PIN_12
#define DRV1_SPI_NSS_Port  		GPIOB

/* MOT 2 */
#define DRV2_EN_Pin		        GPIO_PIN_1
#define DRV2_EN_Port	        GPIOA

#define DRV2_STEP_Pin		    GPIO_PIN_0
#define DRV2_STEP_Port	        GPIOA

#define DRV2_DIR_Pin			GPIO_PIN_5
#define DRV2_DIR_Port 			GPIOB

#define DRV2_DIAGNOSTIC_Pin		GPIO_PIN_15
#define DRV2_DIAGNOSTIC_Port	GPIOA

#define DRV2_SPI_NSS_Pin		GPIO_PIN_12
#define DRV2_SPI_NSS_Port  		GPIOA

/* SPI */
#define SPI2_MOSI_Pin			GPIO_PIN_3
#define SPI2_MOSI_Port  		GPIOC

#define SPI2_MISO_Pin			GPIO_PIN_2
#define SPI2_MISO_Port  		GPIOC

#define SPI2_CLK_Pin			GPIO_PIN_9
#define SPI2_CLK_Port  		    GPIOA

/* FTDI */
#define MOD_TX_Pin				GPIO_PIN_0
#define MOD_TX_Port				GPIOC

#define MOD_RX_Pin				GPIO_PIN_1
#define MOD_RX_Port				GPIOC

#define MOD_NWAKE_Pin			GPIO_PIN_7
#define MOD_NWAKE_Port			GPIOA

/* IIC */
#define IIC_SCL_Pin				GPIO_PIN_8
#define IIC_SCL_Port			GPIOB

#define IIC_SDA_Pin				GPIO_PIN_9
#define IIC_SDA_Port			GPIOB

/* GPIO DEBUG */
#define GPIO0_NWAKE_Pin			GPIO_PIN_7
#define GPIO0_NWAKE_Port		GPIOA

/* BATTERY MEASUREMENT & DAC */
#define AIN_IREF_MIP_Pin		GPIO_PIN_10
#define AIN_IREF_MIP_Port  		GPIOA

#define VEXT_SENSE_Pin			GPIO_PIN_13
#define VEXT_SENSE_Port  		GPIOB

/*******************************************************************************
* API
******************************************************************************/
void TrinamicGPIO_Init(void);
void TrinamicNSS_Select(uint8_t driver);
void TrinamicNSS_Idle(void);
void TrinamicMotXEnable(uint8_t driver);
void TrinamicMotXDisable(uint8_t driver);

#endif
