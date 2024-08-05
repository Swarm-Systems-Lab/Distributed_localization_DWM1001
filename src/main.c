#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "button.h"

#include "source_seeking_app.h"
//#include "test_app.h"

uint8_t switch_app_toggle = 0;
uint8_t app_toggle = 0;

void switch_app(void* args)
{
	switch_app_toggle = 1;
}

static THD_WORKING_AREA(APP_THREAD, 512);

int main(void) {
    halInit();
    chSysInit();

    leds_off(ALL_LEDS);

	palEnablePadEvent(IOPORT1, DW_IRQ, PAL_EVENT_MODE_RISING_EDGE);
	palSetPadCallback(IOPORT1, DW_IRQ, ISR_wrapper, NULL);

	// button_cb_arguments args_b = {.bt=bt1, .cb=switch_app, .cb_args=NULL};
	// bind_button_cb(&args_b, NORMALPRIO+1, "switch_app_thread");

	chThdSleepMilliseconds(100);
	chThdCreateStatic(DW_IRQ_THREAD, sizeof(DW_IRQ_THREAD), NORMALPRIO+2, DW_IRQ_HANDLER, NULL);

	//chThdCreateStatic(SYSTEM_STATUS_THREAD, sizeof(SYSTEM_STATUS_THREAD), NORMALPRIO, SYSTEM_STATUS, NULL);
	chThdSleepMilliseconds(100);
	
	chThdCreateStatic(DW_CONTROLLER_THREAD, sizeof(DW_CONTROLLER_THREAD), NORMALPRIO, DW_CONTROLLER, NULL);
	chThdCreateStatic(UART_RECEIVER_THREAD, sizeof(UART_RECEIVER_THREAD), NORMALPRIO, UART_RECEIVER, NULL);
	chThdCreateStatic(UART_SENDER_THREAD, sizeof(UART_SENDER_THREAD), NORMALPRIO, UART_SENDER, NULL);
	thread_t* app_thread_p = chThdCreateStatic(APP_THREAD, sizeof(APP_THREAD), NORMALPRIO, SS, NULL);

	while (true) {	
		toggle_led(red1);
		chThdSleepMilliseconds(500);

		// if (switch_app_toggle)
		// {
		// 	switch_app_toggle = 0;
		// 	app_toggle = !app_toggle;

		// 	chThdTerminate(app_thread_p);
		// 	chThdSleepMilliseconds(500);

		// 	if (app_toggle)
		// 		app_thread_p = chThdCreateStatic(APP_THREAD, sizeof(APP_THREAD), NORMALPRIO, SS, NULL);
		// 	else
		// 		app_thread_p = chThdCreateStatic(APP_THREAD, sizeof(APP_THREAD), NORMALPRIO, DISTANCE_FIELD, NULL);
		// }
	}
}