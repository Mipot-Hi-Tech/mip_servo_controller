/*
 * tmc_interface.c - SPI/UART interface dummy functions for Trinamic stepper drivers
 *
 * v0.0.1 / 2020-02-04 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2021, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission..

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <string.h>
#include <stdio.h>
#include "common.h"
#include "trinamic_gpio.h"
#include "trinamic_tim.h"

extern SPI_HandleTypeDef hspi2;

__attribute__((weak)) TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
	uint8_t spi_tx_buff[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t spi_rx_buff[5] = {0x00, 0x00, 0x00, 0x00, 0x00};

	spi_tx_buff[0]  = datagram->addr.value;
	spi_tx_buff[0] |= 0x80;

	spi_tx_buff[1] = datagram->payload.data[0];
	spi_tx_buff[2] = datagram->payload.data[1];
	spi_tx_buff[3] = datagram->payload.data[2];
	spi_tx_buff[4] = datagram->payload.data[3];
	TrinamicNSS_Select(driver.id);
	(void)HAL_SPI_TransmitReceive(&hspi2, spi_tx_buff, spi_rx_buff, 5, 10);
	TrinamicNSS_Idle();

	Delay_ms(2);

	TrinamicNSS_Select(driver.id);
	spi_tx_buff[0] = 0x6F;
	spi_tx_buff[1] = 0x00;
	spi_tx_buff[2] = 0x00;
	spi_tx_buff[3] = 0x00;
	spi_tx_buff[4] = 0x00;
	(void)HAL_SPI_TransmitReceive(&hspi2, spi_tx_buff, spi_rx_buff, 5, 10);
	TrinamicNSS_Idle();

    return 0;
}

__attribute__((weak)) TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
	uint8_t spi_tx_buff[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t spi_rx_buff[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	spi_tx_buff[0] = datagram->addr.value;
	spi_tx_buff[1] = datagram->payload.data[0];
	spi_tx_buff[2] = datagram->payload.data[1];
	spi_tx_buff[3] = datagram->payload.data[2];
	spi_tx_buff[4] = datagram->payload.data[3];

	TrinamicNSS_Select(driver.id);
	(void)HAL_SPI_TransmitReceive(&hspi2, spi_tx_buff, spi_rx_buff, 5, 100);
	TrinamicNSS_Idle();

	Delay_ms(2);

	TrinamicNSS_Select(driver.id);
	(void)HAL_SPI_TransmitReceive(&hspi2, spi_tx_buff, spi_rx_buff, 5, 100);
	datagram->addr.value      = spi_rx_buff[0];
	datagram->payload.data[0] = spi_rx_buff[1];
	datagram->payload.data[1] = spi_rx_buff[2];
	datagram->payload.data[2] = spi_rx_buff[3];
	datagram->payload.data[3] = spi_rx_buff[4];
	TrinamicNSS_Idle();
    return 0;
}

__attribute__((weak)) void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *datagram)
{

}

__attribute__((weak)) TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *datagram)
{
    return NULL;
}
