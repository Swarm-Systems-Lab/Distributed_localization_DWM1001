#include "ch.h"
#include "hal.h"
#include "led.h"
#include "button.h"
#include "nrf52_radio.h"

void function(void* args) {
    toggle_led(blue);
}

void function2(void* args) {
    toggle_led(blue);
}

int main(void) {

    halInit();
    chSysInit();

    leds_off(ALL_LEDS);

    // Init radio with the ESB protocol
    nrf52_address_t nrf52_address = {NRF52_RADIO_BASE_ADDR_P0,
        NRF52_RADIO_BASE_ADDR_P1,
        NRF52_RADIO_PIPE_PREFIXES,
        NRF52_RADIO_NUM_PIPES,
        NRF52_RADIO_ADDR_LENGTH,
        NRF52_RADIO_RX_PIPES,
        NRF52_RADIO_RF_CHANNEL};

    nrf52_retransmit_t nrf52_retransmit = {NRF52_RADIO_RETRANSMIT_DELAY,
        NRF52_RADIO_RETRANSMIT_COUNT};

    nrf52_config_t nrf52_conf_init =
      {NRF52_PROTOCOL_ESB, RADIO_MODE, NRF52_BITRATE_1MBPS,
      NRF52_CRC_OFF, NRF52_TX_POWER_0DBM, NRF52_TXMODE_MANUAL,
      false, nrf52_retransmit, 1, nrf52_address};

    if(NRF52_SUCCESS == radio_init(&nrf52_conf_init))
        led_on(green);

    button_cb_arguments bca = {bt1, function, NULL};
    button_cb_arguments bca2 = {bt2, function2, NULL};
    bind_button_cb(&bca, NORMALPRIO+1, "blinker buton");
    bind_button_cb(&bca2, NORMALPRIO+1, "blinker buton");

    while (true) {
       chThdSleepMilliseconds(250);
       toggle_led(red1);
    }
}
