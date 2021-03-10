#include "ch.h"
#include "hal.h"

void gpt_callback(GPTDriver *gptp) {
  palTogglePad(IOPORT1, LED2);
}

/*
 * GPT configuration
 * Frequency: 31250Hz (32us period)
 * Resolution: 16 bits
 */
static const GPTConfig gpt_config = {
    .frequency  = 31250,
    .callback   = gpt_callback,
    .resolution = 16,
};

static THD_WORKING_AREA(waThread1, 64);
static THD_FUNCTION(Thread1, arg) {

    (void)arg;
    uint8_t led = LED4;

    chRegSetThreadName("blinker");

    while (1) {
    alTogglePad(IOPORT1, led);
    chThdSleepMilliseconds(100);
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{

    halInit();
    chSysInit();
    shellInit();

    sdStart(&SD1, &serial_config);

    palSetPad(IOPORT1, LED1);
    palClearPad(IOPORT1, LED2);
    palClearPad(IOPORT1, LED3);
    palSetPad(IOPORT1, LED4);

    gptStart(&GPTD1, &gpt_config);
    gptStartContinuous(&GPTD1, 31250);

    chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO+1,
      Thread1, NULL);

    chThdSleep(2);

//    NRF_P0->DETECTMODE = 0;

    while (true) {
        chThdSleepMilliseconds(250);
    }
}
