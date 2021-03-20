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
    nrf52_address_t nrf52_address = {RADIO_ESB_BASE_ADDR_P0,
        RADIO_ESB_BASE_ADDR_P1,
        RADIO_ESB_PIPE_PREFIXES,
        RADIO_ESB_NUM_PIPES,
        RADIO_ESB_ADDR_LENGTH,
        RADIO_ESB_RX_PIPES,
        RADIO_ESB_RF_CHANNEL};

    nrf52_retransmit_t nrf52_retransmit = {RADIO_ESB_RETRANSMIT_DELAY,
        RADIO_ESB_RETRANSMIT_COUNT};

    nrf52_config_t nrf52_conf_init =
      {NRF52_PROTOCOL_ESB, RADIO_ESB_MODE, NRF52_BITRATE_1MBPS,
      NRF52_CRC_8BIT, NRF52_TX_POWER_0DBM, NRF52_TXMODE_AUTO,
      false, nrf52_retransmit, RADIO_ESB_STATIC_PAYLOAD_LENGTH, nrf52_address};

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
