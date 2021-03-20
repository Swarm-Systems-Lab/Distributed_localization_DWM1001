#include "ch.h"
#include "hal.h"
#include "led.h"
#include "button.h"
#include "nrf52_radio.h"

void radio_test_send_msg(void* args) {
    uint8_t data = 0xFE;

    nrf52_payload_t tx_msg;
    tx_msg.length = 1;
    tx_msg.pipe = 1;
    tx_msg.noack = 1;
   // tx_msg.pid = 1;
    tx_msg.data[0] = data;

    nrf52_error_t error = radio_write_payload(&tx_msg);
    switch(error){
      case NRF52_SUCCESS:
          toggle_led(blue);
          break;
      case NRF52_ERROR_INVALID_LENGTH:
          toggle_led(red2);
          break;
      case NRF52_INVALID_STATE:
          toggle_led(green);
          break;
      default:
          break;
    }
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

    nrf52_config_t nrf52_conf_init = {NRF52_PROTOCOL_ESB_DPL,
        RADIO_ESB_MODE,
        NRF52_BITRATE_1MBPS,
        NRF52_CRC_8BIT,
        NRF52_TX_POWER_0DBM,
        NRF52_TXMODE_AUTO,
        true,
        nrf52_retransmit,
        RADIO_ESB_STATIC_PAYLOAD_LENGTH,
        nrf52_address};

    if(NRF52_SUCCESS == radio_init(&nrf52_conf_init))
        led_on(green);

    button_cb_arguments bca = {bt1, radio_test_send_msg, NULL};
    button_cb_arguments bca2 = {bt2, radio_test_send_msg, NULL};
    bind_button_cb(&bca, NORMALPRIO+1, "blinker buton");
    bind_button_cb(&bca2, NORMALPRIO+1, "blinker buton");

    while (true) {
       chThdSleepMilliseconds(250);
       toggle_led(red1);
    }
}
