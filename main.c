#include "ch.h"
#include "hal.h"

static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {

    (void)arg;
    uint8_t green = GREEN_LED_D9;
    //uint8_t blue = BLUE_LED_D10;
    //uint8_t red1 = RED_LED_D11;
    //uint8_t red2 = RED_LED_D12;

    chRegSetThreadName("blinker");

    while (1) {
      palTogglePad(IOPORT1, green);
      //palTogglePad(IOPORT1, led2);
      //palTogglePad(IOPORT1, led3);
      //palTogglePad(IOPORT1, led4);
      chThdSleepMilliseconds(100);
    }
}

int main(void)
{
    halInit();
    chSysInit();

    palSetPad(IOPORT1, GREEN_LED_D9);
    //palSetPad(IOPORT1, LED2);
    //palSetPad(IOPORT1, LED3);
    //palSetPad(IOPORT1, LED4);

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1,
      Thread1, NULL);

    chThdSleep(2);

//    NRF_P0->DETECTMODE = 0;

    while (true) {
        chThdSleepMilliseconds(250);
    }
}
