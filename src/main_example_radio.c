#include "ch.h"
#include "hal.h"
#include "led.h"
#include "button.h"

void function(void* args) {
    toggle_led(blue);
}

void function2(void* args) {
    toggle_led(green);
}

int main(void) {

    halInit();
    chSysInit();

    leds_off(ALL_LEDS);

    button_cb_arguments bca = {bt1, function, NULL};
    button_cb_arguments bca2 = {bt2, function2, NULL};
    bind_button_cb(&bca, NORMALPRIO+1, "blinker buton");
    bind_button_cb(&bca2, NORMALPRIO+1, "blinker buton");

    while (true) {
       chThdSleepMilliseconds(250);
       toggle_led(red1);
    }
}
