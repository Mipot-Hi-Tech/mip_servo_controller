/*******************************************************************************
 * Included files
 *****************************************************************************/
#include "trinamic_uart.h"
#include "trinamic_gpio.h"
#include "sys_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void LPUART1_GPIO_Init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
char   tx_buff[UART_TX_BUFF_SIZE];
uint32_t tx_buff_write_index = 0;
uint32_t tx_buff_read_index  = 0;

char   rx_buff[UART_RX_BUFF_SIZE];
uint32_t rx_buff_write_index = 0;
uint32_t rx_buff_read_index  = 0;

char cRxedChar = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
void LPUART_Init(void)
{
	LL_LPUART_InitTypeDef LPUART_InitStruct = {0};

	/* Init LPUART1 Pins */
	(void)LPUART1_GPIO_Init();

	/* Enable clock @ LPUART1 */
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
	__NOP();
	__NOP();

	/* LPUART CONFIGURATION */
	LPUART_InitStruct.PrescalerValue      = LL_LPUART_PRESCALER_DIV2;
	LPUART_InitStruct.BaudRate            = LPUART1_BAUDRATE_DEF;
	LPUART_InitStruct.DataWidth           = LL_LPUART_DATAWIDTH_8B;
	LPUART_InitStruct.StopBits            = LL_LPUART_STOPBITS_1;
	LPUART_InitStruct.Parity              = LL_LPUART_PARITY_NONE;
	LPUART_InitStruct.TransferDirection   = LL_LPUART_DIRECTION_TX_RX;
	LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;
	(void)LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
	(void)LL_LPUART_SetTXFIFOThreshold(LPUART1, LL_LPUART_FIFOTHRESHOLD_1_8);
	(void)LL_LPUART_SetRXFIFOThreshold(LPUART1, LL_LPUART_FIFOTHRESHOLD_1_8);
	(void)LL_LPUART_SetTXRXSwap(LPUART1, LL_LPUART_TXRX_SWAPPED);
	(void)LL_LPUART_RequestRxDataFlush(LPUART1);
	(void)LL_LPUART_ClearFlag_TXFE(LPUART1);
	(void)LL_LPUART_ClearFlag_TC(LPUART1);
	(void)LL_LPUART_EnableIT_RXNE_RXFNE(LPUART1);
	(void)LL_LPUART_Enable(LPUART1);

	/* LPUART1 interrupt Init */
	(void)NVIC_EnableIRQ(LPUART1_IRQn);
	(void)NVIC_SetPriority(LPUART1_IRQn, 6);
}

void LPUART1_IRQHandler(void)
{
	/* Check for RXFIFO not empty flag */
	if( (LPUART1->ISR & USART_ISR_RXNE_RXFNE) != 0)
	{
		if(rx_buff_write_index <= UART_RX_BUFF_SIZE-1)
		{
			rx_buff[rx_buff_write_index] = (char)LPUART1->RDR;
			xTaskNotifyFromISR(cli_task_handle, (uint32_t)rx_buff[rx_buff_write_index], eSetValueWithOverwrite, pdFALSE);
			rx_buff_write_index++;
		}
		else
			LPUART1->RDR;
	}
}

static void LPUART1_GPIO_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* Configure GPIO pin : MOD_TX_Pin */
	GPIO_InitStruct.Pin        = MOD_TX_Pin;
	GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate  = LL_GPIO_AF_8;
	(void)LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* Configure GPIO pin : MOD_RX_Pin */
	GPIO_InitStruct.Pin        = MOD_RX_Pin;
	GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate  = LL_GPIO_AF_8;
	(void)LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void LPUART_TxPolling(const char *buff)
{
	while(*buff != 0)
	{
		LPUART1->TDR = *buff;
		while( (LPUART1->ISR & USART_ISR_TXE_TXFNF) == 0);
		buff++;
	}
}
