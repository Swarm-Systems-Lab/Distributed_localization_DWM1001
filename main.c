#include "ch.h"
#include "hal.h"

static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {

    (void)arg;
    uint8_t led1 = LED4;
    uint8_t led2 = LED2;

    chRegSetThreadName("blinker");

    while (1) {
      palTogglePad(IOPORT1, led1);
      palTogglePad(IOPORT1, led2);
      chThdSleepMilliseconds(100);
    }
}

int main(void)
{
    halInit();
    chSysInit();

    palSetPad(IOPORT1, LED4);
    palSetPad(IOPORT1, LED2);

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1,
      Thread1, NULL);

    chThdSleep(2);

//    NRF_P0->DETECTMODE = 0;

    while (true) {
        chThdSleepMilliseconds(250);
    }
}
