#include "ch.h"
#include "hal.h"

static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {

    (void)arg;
    uint8_t led = LED4;

    chRegSetThreadName("blinker");

    while (1) {
      palTogglePad(IOPORT1, led);
      chThdSleepMilliseconds(100);
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    halInit();
    chSysInit();

    palSetPad(IOPORT1, LED1);
    palClearPad(IOPORT1, LED2);
    palClearPad(IOPORT1, LED3);
    palSetPad(IOPORT1, LED4);

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1,
      Thread1, NULL);

    chThdSleep(2);

//    NRF_P0->DETECTMODE = 0;

    while (true) {
        chThdSleepMilliseconds(250);
    }
}
