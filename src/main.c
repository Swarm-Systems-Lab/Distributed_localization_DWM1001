#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "source_seeking_app.h"
//#include "test_app.h"


static THD_WORKING_AREA(APP_THREAD, 512);

int main(void) {
    halInit();
    chSysInit();

    leds_off(ALL_LEDS);

	palEnablePadEvent(IOPORT1, DW_IRQ, PAL_EVENT_MODE_RISING_EDGE);
	palSetPadCallback(IOPORT1, DW_IRQ, ISR_wrapper, NULL);
	chThdSleepMilliseconds(100);
	chThdCreateStatic(DW_IRQ_THREAD, sizeof(DW_IRQ_THREAD), NORMALPRIO+1, DW_IRQ_HANDLER, NULL);

	//chThdCreateStatic(SYSTEM_STATUS_THREAD, sizeof(SYSTEM_STATUS_THREAD), NORMALPRIO, SYSTEM_STATUS, NULL);
	chThdSleepMilliseconds(100);
	
	chThdCreateStatic(DW_CONTROLLER_THREAD, sizeof(DW_CONTROLLER_THREAD), NORMALPRIO, DW_CONTROLLER, NULL);
	chThdCreateStatic(UART_CONTROLLER_THREAD, sizeof(UART_CONTROLLER_THREAD), NORMALPRIO, UART_CONTROLLER, NULL);
	chThdCreateStatic(APP_THREAD, sizeof(APP_THREAD), NORMALPRIO, SS, NULL);

	while (true) {	
		toggle_led(red1);
		chThdSleepMilliseconds(500);
	}
}