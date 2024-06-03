#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "nrf52_radio.h"
#include "dist_loc.h"

int main(void) {
    halInit();
    chSysInit();

    leds_off(ALL_LEDS);

	palEnablePadEvent(IOPORT1, DW_IRQ, PAL_EVENT_MODE_RISING_EDGE);
	palSetPadCallback(IOPORT1, DW_IRQ, ISR_wrapper, NULL);
	chThdSleepMilliseconds(100);
	chThdCreateStatic(DW_IRQ_THREAD, sizeof(DW_IRQ_THREAD), NORMALPRIO+1, DW_IRQ_HANDLER, NULL);

	chThdCreateStatic(SYSTEM_STATUS_THREAD, sizeof(SYSTEM_STATUS_THREAD), NORMALPRIO, SYSTEM_STATUS, NULL);
	chThdSleepMilliseconds(100);
	
	chThdCreateStatic(DW_CONTROLLER_THREAD, sizeof(DW_CONTROLLER_THREAD), NORMALPRIO, DW_CONTROLLER, NULL);
	chThdCreateStatic(COMMS_THREAD, sizeof(COMMS_THREAD), NORMALPRIO, COMMS, NULL);
	chThdCreateStatic(PEER_DISCOVERY_THREAD, sizeof(PEER_DISCOVERY_THREAD), NORMALPRIO, PEER_DISCOVERY, NULL);
	chThdCreateStatic(PEER_CONNECTION_THREAD, sizeof(PEER_CONNECTION_THREAD), NORMALPRIO, PEER_CONNECTION, NULL);

	while (true) {	
		toggle_led(red1);
		chThdSleepMilliseconds(500);
	}
}