#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "nrf52_radio.h"

#include "led.h"
#include "dw1000_hal.h"

SPIConfig spi_cfg = { .end_cb = NULL, .ssport = IOPORT1, .sspad = SPI_SS,
        .freq = NRF5_SPI_FREQ_8MBPS, .sckpad = SPI_SCK, .mosipad = SPI_MOSI,
        .misopad = SPI_MISO, .lsbfirst = false, .mode = 2};

void send_message() {
	size_t data_size = 10;

	uint8_t sendtx[4] = {0x2,0x0,0x0,0x2};
	
	uint8_t data[10] = {0xde,0xad,0xbe,0xef,0xef,0xbe,0xad,0xde,0x0,0x0};

	dw_write(DW_REG_INFO.TX_BUFFER, data, data_size, 0);

	dw_write(DW_REG_INFO.SYS_CTRL, sendtx, 4, 0);

	toggle_led(blue);
}

void recv_message() {
	uint8_t sendrx[4] = {0x0,0x1,0x1,0x0};
	
	uint8_t received = 0;
	uint8_t data[10];

	uint32_t count=0;

	while (!received && count<100) {

		dw_write(DW_REG_INFO.SYS_CTRL, sendrx, 4, 0);

		sys_status_t statusrx;

		dw_read(DW_REG_INFO.SYS_STATUS, &statusrx, DW_REG_INFO.SYS_STATUS.size, 0);

		received=statusrx.RXDFR;
		count++;
		chThdSleepMilliseconds(1);
	}

	sys_status_t statusrx2;
	dw_read(DW_REG_INFO.SYS_STATUS, &statusrx2, DW_REG_INFO.SYS_STATUS.size, 0);
	statusrx2.RXDFR = 1;
	dw_write(DW_REG_INFO.SYS_STATUS, &statusrx2, DW_REG_INFO.SYS_STATUS.size, 0);

	uint8_t rx2[10]={0,0,0,0,0,0,0,0,0,0};
	size_t rx2_size = 10;

	dw_read(DW_REG_INFO.RX_BUFFER, rx2, rx2_size, 0);

	if (received)
		toggle_led(green);
	else 
		toggle_led(red2);

}

static THD_WORKING_AREA(waRadioThread, 256);
static THD_FUNCTION(RadioThread, arg) {
    (void)arg;

	 while (true) {	
		recv_message();
		chThdSleepMilliseconds(100);
    }
}

void chibi_power_on(void)
{
	palSetPad(IOPORT1, DW_RST);
}

void chibi_power_off(void)
{
	palClearPad(IOPORT1, DW_RST);
}

void chibi_spi_lock(void)
{
	spiAcquireBus(&SPID1);
}

void chibi_spi_unlock(void)
{
	spiReleaseBus(&SPID1);
}

void chibi_spi_set_cs(void)
{
	spiSelect(&SPID1);
}

void chibi_spi_clear_cs(void)
{
	spiUnselect(&SPID1);
}

void chibi_spi_send(size_t count, const uint8_t* buf)
{
	spiSend(&SPID1, count, buf);
}

void chibi_spi_recv(size_t count, const uint8_t* buf)
{
	spiReceive(&SPID1, count, buf);
}

int main(void) {
    halInit();
    chSysInit();

    leds_off(ALL_LEDS);

	dw_set_power_on(chibi_power_on);
	dw_set_power_off(chibi_power_off);

	_dw_power_off();
	chThdSleepMilliseconds(10);
	_dw_power_on();

	spiStart(&SPID1, &spi_cfg);

	//chThdCreateStatic(waRadioThread, sizeof(waRadioThread), NORMALPRIO, RadioThread, NULL);

	dw_set_spi_lock(chibi_spi_lock);
	dw_set_spi_unlock(chibi_spi_unlock);
	dw_set_spi_set_cs(chibi_spi_set_cs);
	dw_set_spi_clear_cs(chibi_spi_clear_cs);
	dw_set_spi_send(chibi_spi_send);
	dw_set_spi_recv(chibi_spi_recv);

	dev_id_t dev_test;

	dw_read(DW_REG_INFO.DEV_ID, &dev_test, DW_REG_INFO.DEV_ID.size, 0);

	while (true) {	
		send_message();
		chThdSleepMilliseconds(100);
	}
}
