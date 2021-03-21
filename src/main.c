#include "ch.h"
#include "hal.h"
#include "led.h"
#include "button.h"
#include "nrf52_radio.h"

static nrf52_payload_t tx_msg;
static nrf52_payload_t rx_msg;

void radio_test_send_msg(void* args) {
    uint8_t data = 0xFE;

    tx_msg.length = 1;
    tx_msg.pipe = 1;
    tx_msg.noack = 1;
    //tx_msg.pid = 0xCA;
    tx_msg.data[0] = data;

    radio_stop_rx();
    radio_write_payload(&tx_msg);
    radio_start_tx();
}

static THD_WORKING_AREA(waRadioListenerThread, 128);
static THD_FUNCTION(RadioListenerThread, arg) {

    event_listener_t radio_listener;
    chEvtRegisterMask(&RFD1.eventsrc, &radio_listener, EVENT_MASK(0));

    chRegSetThreadName("radioListener");

    while(true){
      chEvtWaitAny(ALL_EVENTS);

      eventflags_t flags = chEvtGetAndClearFlags(&radio_listener);

      if(flags & NRF52_EVENT_TX_FAILED){
        radio_start_rx();
        toggle_led(red2);
      }

      if(flags & NRF52_EVENT_TX_SUCCESS){
        radio_start_rx();
        toggle_led(green);
      }

      if(flags & NRF52_EVENT_RX_RECEIVED){
        radio_read_rx_payload(&rx_msg);
        led_on(blue);
      }
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
        NRF52_TXMODE_MANUAL_START,
        true,
        nrf52_retransmit,
        RADIO_ESB_STATIC_PAYLOAD_LENGTH,
        nrf52_address};

    radio_init(&nrf52_conf_init);
    radio_flush_tx();
    radio_flush_rx();
    radio_start_rx();

    chThdCreateStatic(waRadioListenerThread, sizeof(waRadioListenerThread),
                                  NORMALPRIO, RadioListenerThread, NULL);

    button_cb_arguments bca = {bt1, radio_test_send_msg, NULL};
    button_cb_arguments bca2 = {bt2, radio_test_send_msg, NULL};
    bind_button_cb(&bca, NORMALPRIO+1, "blinker buton");
    bind_button_cb(&bca2, NORMALPRIO+1, "blinker buton");

    while (true) {
       chThdSleepMilliseconds(250);
       toggle_led(red1);
    }
}
