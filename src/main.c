#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "ch.h"
#include "hal.h"

#include "nrf52_radio.h"

#include "led.h"
#include "dw1000_hal.h"

SPIConfig spi_cfg = { .end_cb = NULL, .ssport = IOPORT1, .sspad = SPI_SS,
        .freq = NRF5_SPI_FREQ_8MBPS, .sckpad = SPI_SCK, .mosipad = SPI_MOSI,
        .misopad = SPI_MISO, .lsbfirst = false, .mode = 2};

void send_message() {
	uint8_t header[3] = {0b10001001, 0X00, 0X00};
	size_t header_size = 1;
	size_t data_size = 10;

	uint8_t sendtx[4] = {0x2,0x0,0x0,0x2};
	
	uint8_t data[10] = {0xde,0xad,0xbe,0xef,0xef,0xbe,0xad,0xde,0x0,0x0};
	uint8_t message[20];

	spiAcquireBus(&SPID1);
	spiSelect(&SPID1);

	memcpy(message, header, header_size);
	memcpy(message+header_size, data, data_size);
	spiSend(&SPID1, header_size+data_size, message);

	spiUnselect(&SPID1);
	spiReleaseBus(&SPID1);

	header[0] = 0b10001101;

	spiAcquireBus(&SPID1);
	spiSelect(&SPID1);

	memcpy(message, header, header_size);
	memcpy(message+header_size, sendtx, 4);
	spiSend(&SPID1, header_size+4, message);

	spiUnselect(&SPID1);
	spiReleaseBus(&SPID1);

	toggle_led(blue);
}

void recv_message() {
	size_t header_size = 1;
	size_t data_size = 10;

	uint8_t sendrx[4] = {0x0,0x1,0x1,0x0};
	
	uint8_t received = 0;
	uint8_t data[10];



	uint32_t count=0;
	uint8_t header[3] = {0b10001101, 0X00, 0X00};

	while (!received && count<100) {

		uint8_t message[20];

		header[0] = 0b10001101;

		spiAcquireBus(&SPID1);
		spiSelect(&SPID1);

		memcpy(message, header, header_size);
		memcpy(message+header_size, sendrx, 4);
		spiSend(&SPID1, header_size+4, message);

		spiUnselect(&SPID1);
		spiReleaseBus(&SPID1);

		header[0] = 0xF;
		uint8_t rx[5];
		size_t rx_size = 5;

		spiAcquireBus(&SPID1);
		spiSelect(&SPID1);

		spiSend(&SPID1, header_size, &header);
		spiReceive(&SPID1, rx_size, rx);

		spiUnselect(&SPID1);
		spiReleaseBus(&SPID1);

		received=((rx[1]>>5)&0x1);
		count++;
		chThdSleepMilliseconds(1);
	}

	header[0] = 0x11;
	header_size = 1;


	uint8_t rx2[10]={0,0,0,0,0,0,0,0,0,0};
	size_t rx2_size = 10;

	spiAcquireBus(&SPID1);
	spiSelect(&SPID1);

	spiSend(&SPID1, header_size, &header);
	spiReceive(&SPID1, rx2_size, rx2);

	spiUnselect(&SPID1);
	spiReleaseBus(&SPID1);

	if (received)
		toggle_led(green);
	else 
		toggle_led(red2);

}

static THD_WORKING_AREA(waRadioThread, 256);
static THD_FUNCTION(RadioThread, arg) {
    (void)arg;

	 while (true) {	
		send_message();
		chThdSleepMilliseconds(20);
		recv_message();
		chThdSleepMilliseconds(20);
    }
}

int main(void) {
    halInit();
    chSysInit();

    leds_off(ALL_LEDS);

	dw_power_off();
	chThdSleepMilliseconds(10);
	dw_power_on();

	spiStart(&SPID1, &spi_cfg);

	// send_message();
	// chThdSleepMilliseconds(20);

	chThdCreateStatic(waRadioThread, sizeof(waRadioThread), NORMALPRIO, RadioThread, NULL);


	//uint16_t header  = 0b0000100000000000;
	uint8_t header[3] = {0, 0X00, 0X00};
	size_t header_size = 1;
	size_t data_size = 10;

	uint8_t sendtx[4] = {0x0,0x0,0x0,0x1};
	
	uint8_t data[10] = {0xde,0xad,0xbe,0xef,0xef,0xbe,0xad,0xde,0x0,0x0};

	uint8_t read = 1;

	dev_id_t rx;
	uint8_t rx2[4];
	size_t rx_size = 4;

	spiAcquireBus(&SPID1);
	spiSelect(&SPID1);

	spiSend(&SPID1, header_size, &header);
	spiReceive(&SPID1, DW_REG_INFO.DEV_ID.size, rx.reg);

	spiUnselect(&SPID1);
	spiReleaseBus(&SPID1);

    while (true) {	
		//send_message();
		chThdSleepMilliseconds(20);
		//recv_message();
		toggle_led(red1);
		chThdSleepMilliseconds(20);
    }
}
