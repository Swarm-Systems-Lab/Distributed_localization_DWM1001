#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "nrf52_radio.h"

#include "led.h"
#include "dist_loc.h"

thread_t* comm_thread;

void TXFRS_handler(void)
{
	chEvtSignal(comm_thread, MTXFRS_E);
	//chEvtBroadcast(&message_evt);
}

void RXFCG_handler(void)
{
	chEvtSignal(comm_thread, MRXFCG_E);
	//chEvtBroadcast(&recv_evt);
}

void RXERR_handler(void)
{
	chEvtSignal(comm_thread, MRXPHE_E | MRXFCE_E | MLDEERR_E);
	//toggle_led(blue);
	//chEvtBroadcast(&nrecv_evt);
}

int main(void) {
    halInit();
    chSysInit();

    leds_off(ALL_LEDS);

	dw_reset();
	spi_hal_init();
	default_config();
	load_lde();

	irq_vector._dw_TXFRS_handler = TXFRS_handler;
	irq_vector._dw_RXFCG_handler = RXFCG_handler;
	// irq_vector._dw_RXRFTO_handler = RXERR_handler;
	irq_vector._dw_RXPHE_handler = RXERR_handler;
	irq_vector._dw_RXFCE_handler = RXERR_handler;
	irq_vector._dw_RXRFSL_handler = RXERR_handler;
	// irq_vector._dw_RXSFDTO_handler = RXERR_handler;
	// irq_vector._dw_AFFREJ_handler = RXERR_handler;
	irq_vector._dw_LDEERR_handler = RXERR_handler;
	//irq_vector._dw_LDEDONE_handler = RXFCG_handler;
	irq_vector._dw_AFFREJ_handler = RXERR_handler;

	palEnablePadEvent(IOPORT1, DW_IRQ, PAL_EVENT_MODE_RISING_EDGE);
	palSetPadCallback(IOPORT1, DW_IRQ, ISR_wrapper, NULL);
	chThdSleepMilliseconds(100);
	chThdCreateStatic(DW_IRQ_THREAD, sizeof(DW_IRQ_THREAD), NORMALPRIO+1, DW_IRQ_HANDLER, NULL);
	comm_thread = chThdCreateStatic(DW_CONTROLLER_THREAD, sizeof(DW_CONTROLLER_THREAD), NORMALPRIO, DW_CONTROLLER, NULL);

	while (true) {	
		toggle_led(red1);
		chThdSleepMilliseconds(500);
	}
}