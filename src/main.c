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
    tx_msg.pid = 1;
    tx_msg.data[0] = data;

    radio_write_payload(&tx_msg);
}

static THD_WORKING_AREA(waRadioListenerThread, 128);
static THD_FUNCTION(RadioListenerThread, arg) {
    event_listener_t radio_listener;
    chEvtRegisterMaskWithFlags(&RFD1.eventsrc,
                           &radio_listener,
                           EVENT_MASK(0),
                           NRF52_EVENT_RX_RECEIVED);
    while(true){
      eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
      if(evt & EVENT_MASK(0)) {
          eventflags_t flags = chEvtGetAndClearFlags(&radio_listener);
          if(flags & NRF52_EVENT_RX_RECEIVED){
            nrf52_payload_t rx_msg;
            radio_read_rx_payload(&rx_msg);
              if(rx_msg.data[0] == 0xFE)
                toggle_led(green);
          }
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
        NRF52_TXMODE_AUTO,
        true,
        nrf52_retransmit,
        RADIO_ESB_STATIC_PAYLOAD_LENGTH,
        nrf52_address};

    radio_init(&nrf52_conf_init);

    (void) chThdCreateStatic(waRadioListenerThread, sizeof(waRadioListenerThread),
                                  NORMALPRIO + 3, RadioListenerThread, NULL);

    button_cb_arguments bca = {bt1, radio_test_send_msg, NULL};
    button_cb_arguments bca2 = {bt2, radio_test_send_msg, NULL};
    bind_button_cb(&bca, NORMALPRIO+1, "blinker buton");
    bind_button_cb(&bca2, NORMALPRIO+1, "blinker buton");

    while (true) {
       chThdSleepMilliseconds(250);
       toggle_led(red1);
    }
}
