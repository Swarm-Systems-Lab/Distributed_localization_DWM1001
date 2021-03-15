#include "led.h"

const led LEDS[NUMBER_OF_LEDS] = {green, blue, red1, red2};

void led_off(const led l) {
	palSetPad(IOPORT1, l);
}

void led_on(const led l) {
	palClearPad(IOPORT1, l);
}

void leds_off(void) {
	int i;
	for(i = 0; i < NUMBER_OF_LEDS; ++i) {
		led_off(LEDS[i]);
	}
}

void leds_on(void) {
	int i;
	for(i = 0; i < NUMBER_OF_LEDS; ++i) {
		led_on(LEDS[i]);
	}
}

void toggle_led(const led l)  {
	palTogglePad(IOPORT1, l);
}
