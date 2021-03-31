#include "ch.h"
#include "hal.h"

#include "led.h"
#include "nrf52_radio.h"

int main(void) {

    halInit();
    chSysInit();

    leds_off(ALL_LEDS);

    radio_init(&radiocfg);
    radio_flush_tx();
    radio_flush_rx();
    radio_start_rx();

//    chThdCreateStatic(waRadioThread, sizeof(waRadioThread), NORMALPRIO, RadioThread, NULL);

    chThdSleep(2);

//    NRF_P0->DETECTMODE = 0;

    while (true) {
        toggle_led(blue);
        chThdSleepMilliseconds(500);
    }
}
