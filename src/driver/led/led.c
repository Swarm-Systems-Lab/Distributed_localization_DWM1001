#include "led.h"

const led LEDS[NUMBER_OF_LEDS] = {green, blue, red1, red2};
const leds_struct ALL_LEDS = {NUMBER_OF_LEDS, LEDS};

THD_FUNCTION(conditional_blinker_function, arg) {

	conditional_blink_arguments* cba = (conditional_blink_arguments*) arg;

	if(cba->b_lds.leds.num_leds > 0 && cba->b_lds.leds.num_leds < NUMBER_OF_LEDS) {
		while((cba->cond_funct)(cba->conf_funct_args)) {
			blink(cba->b_lds);
			chThdSleepMilliseconds(cba->b_lds.delay);
		}
	}
}

THD_FUNCTION(event_blinker_function, arg) {

	event_blink_arguments* eba = (event_blink_arguments*) arg;

	if(eba->b_lds.leds.num_leds > 0 && eba->b_lds.leds.num_leds < NUMBER_OF_LEDS) {

		eba->register_events_funct(eba->register_events_funct_args);

		do {
			chEvtWaitAny(ALL_EVENTS); // Wait until attached events happen.
			blink(eba->b_lds);
			chThdSleepMilliseconds(eba->b_lds.delay);
		}while(true);
	}
}

void blink(const blink_leds_struct leds) {
	leds_on(leds.leds);
	chThdSleepMilliseconds(leds.delay);
	leds_off(leds.leds);
}

thread_t* conditional_blink(const conditional_blink_arguments* args, const tprio_t prio, const char *th_name) {
	return chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(sizeof(conditional_blinker_function)),
			th_name, prio, conditional_blinker_function, (void*)args);
}

thread_t* event_blink(const event_blink_arguments* args, const tprio_t prio, const char *th_name) {
	return chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(sizeof(event_blinker_function)),
			th_name, prio, event_blinker_function, (void*)args);
}

void led_off(const led l) {
	palSetPad(IOPORT1, l);
}

void led_on(const led l) {
	palClearPad(IOPORT1, l);
}

void leds_off(const leds_struct leds) {
	int i;
	for(i = 0; i < leds.num_leds; ++i) {
		led_off(leds.l[i]);
	}
}

void leds_on(const leds_struct leds) {
	int i;
	for(i = 0; i < leds.num_leds; ++i) {
		led_on(leds.l[i]);
	}
}

void toggle_led(const led l)  {
	palTogglePad(IOPORT1, l);
}
