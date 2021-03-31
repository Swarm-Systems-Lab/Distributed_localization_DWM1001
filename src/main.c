#include "ch.h"
#include "hal.h"

#include "led.h"
#include "nrf52_radio.h"

int main(void) {

    halInit();
    chSysInit();

//  Leds off
    leds_off(ALL_LEDS);
//  LED SUMMARY
//  Blue for the heartbeep
//  Green for PCK_RECEIVED between internal radios

//  Internal radio nrf52 init
    radio_init(&radiocfg);
    radio_flush_tx();
    radio_flush_rx();
    radio_start_rx();

    chThdSleep(2);

    while (true) {
        toggle_led(blue);
        chThdSleepMilliseconds(500);

        // example of dummy transmission (it is configured so that there is ACK)
        uint8_t neighborh_id = 0; // Destination between 0 and 7, we reserver 0 for broadcast
        tx_payload.pipe = neighborh_id;
        tx_payload.noack = 0;
        tx_payload.data[0] = 0x03; // Packet ID
        tx_payload.data[1] = 0x33; // Payload
        tx_payload.length = 2;

        radio_stop_rx();
        // tx_payload and rx_payload are GLOBAL and they are not thread-safe now
        radio_write_payload(&tx_payload);
        radio_start_tx(); // Either fail or success TX (with or w/o ACK), the radio_start_rx is called afterward
    }
}
