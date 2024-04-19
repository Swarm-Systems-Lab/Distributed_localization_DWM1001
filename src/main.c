#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "nrf52_radio.h"

#include "led.h"
#include "dist_loc.h"

// SPIConfig spi_cfg = { .end_cb = NULL, .ssport = IOPORT1, .sspad = SPI_SS,
//         .freq = NRF5_SPI_FREQ_8MBPS, .sckpad = SPI_SCK, .mosipad = SPI_MOSI,
//         .misopad = SPI_MISO, .lsbfirst = false, .mode = 2};

//thread_reference_t irq_evt = NULL;
thread_t* message_thread;
thread_t* recv_thread;
thread_t* comm_thread;
event_source_t message_evt;
event_source_t recv_evt;
event_source_t nrecv_evt;
static mutex_t comm_mutex;
uint64_t time_tx_g;

// static void irq_hand_wrapper(void * arg)
// {
// 	(void)arg;
// 	chSysLockFromISR();
// 	chThdResumeI(&irq_evt, MSG_OK);
// 	chSysUnlockFromISR();
// }

// void _dw_reset(void)
// {
// 	_dw_power_off();
// 	chThdSleepMilliseconds(10);
// 	_dw_power_on();
// }

uint8_t recv_message() {
	sys_state_t state;
	uint8_t respond = 0;
	sys_status_t status;
	chMtxLock(&comm_mutex);
	dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);

	int count = 0;
	while (state.TX_STATE && count < 1000)
	{
		dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);
		count++;
	}
	if (count < 1000)
	{
		dx_time_t time;
		dw_clear_register(time.reg, DW_REG_INFO.DX_TIME.size);
		dw_start_rx(time);
		eventmask_t evt = chEvtWaitOneTimeout(EVENT_MASK(1) | EVENT_MASK(3), TIME_MS2I(600));
		//dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);
		//dw_read(DW_REG_INFO.SYS_STATUS, status.reg, DW_REG_INFO.SYS_STATUS.size, 0);
		if (!evt || evt == EVENT_MASK(3)){
			dw_transceiver_off();
			chThdSleepMilliseconds(1);
		}
		//dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);
		// uint8_t rx2[10]={0,0,0,0,0,0,0,0,0,0};
		// size_t rx2_size = 10;
		//dw_read(DW_REG_INFO.RX_BUFFER, rx2, rx2_size, 0);

		if (evt == EVENT_MASK(1))
		{
			toggle_led(green);
			respond = 1;
		}
		if (evt == EVENT_MASK(3))
		{
			toggle_led(red2);
			//chThdSleepMilliseconds(500);
		}
	}
	chMtxUnlock(&comm_mutex);

	return respond;		
}

void send_message() {
	uint8_t data[10] = {0xde,0xad,0xbe,0xef,0xef,0xbe,0xad,0xde,0x0,0x0};
	tx_fctrl_t tx_ctrl;
	sys_state_t state;
	chMtxLock(&comm_mutex);
	dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);
	int count = 0;
	while (state.RX_STATE && count < 1000)
	{
		dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);
		count++;
	}
	tx_time_t time_send;
	if (count < 1000)
	{
		for (int i = 0; i < 5; i++)
			tx_ctrl.reg[i] = 0;
		tx_ctrl.TFLEN = 12;
		tx_ctrl.TXBR = 0b10;
		tx_ctrl.TXPRF = 0b1;
		tx_ctrl.TXPSR = 0b1;
		tx_ctrl.PE = 0b1;
		dx_time_t time;
		dw_clear_register(time.reg, DW_REG_INFO.DX_TIME.size);
		ack_resp_t_t w4r;
		dw_clear_register(w4r.reg, DW_REG_INFO.ACK_RESP_T.size);
		dw_start_tx(tx_ctrl, data, time, w4r);
		for (int i = 0; i < DW_REG_INFO.TX_TIME.size; i++)
			time_send.reg[i] = 0;
		dw_read(DW_REG_INFO.TX_TIME, time_send.reg, DW_REG_INFO.TX_TIME.size, 0);
		uint64_t time_tx = 0;
		time_tx &= (uint64_t)time_send.TX_STAMP[0];
		time_tx &= (uint64_t)time_send.TX_STAMP[1] << 8;
		time_tx &= (uint64_t)time_send.TX_STAMP[2] << 16;
		time_tx &= (uint64_t)time_send.TX_STAMP[3] << 24;
		time_tx &= (uint64_t)time_send.TX_STAMP[4] << 32;
		time_tx_g = time_tx;
		eventmask_t evt = chEvtWaitOneTimeout(EVENT_MASK(0), TIME_MS2I(20));
		if (!evt){
			dw_transceiver_off();
			chThdSleepMilliseconds(3);
		}
	}
	chMtxUnlock(&comm_mutex);
}

static THD_WORKING_AREA(warecvThread, 256);
static THD_FUNCTION(recvThread, arg) {
    (void)arg;

	sys_mask_t recv_mask;
	recv_mask.mask = 0U;
	recv_mask.MRXFCG = 0b1;
	// recv_mask.MRXRFTO = 0b1;
	recv_mask.MRXPHE = 0b1;
	recv_mask.MRXFCE = 0b1;
	// recv_mask.MRXRFSL = 0b1;
	recv_mask.MLDEERR = 0b1;
	event_listener_t recv_l;
	chEvtRegisterMask(&recv_evt, &recv_l, EVENT_MASK(1));
	chEvtRegisterMask(&nrecv_evt, &recv_l, EVENT_MASK(3));
	chMtxLock(&comm_mutex);
	dw_set_irq(recv_mask);
	// uint8_t fwto[2] = {0xFF, 0xFE};
	// dw_write(DW_REG_INFO.RX_FWTO, fwto, DW_REG_INFO.RX_FWTO.size, 0);
	// dw_read(DW_REG_INFO.RX_FWTO, fwto, DW_REG_INFO.RX_FWTO.size, 0);
	chMtxUnlock(&comm_mutex);

	 while (true) {
		recv_message();
		// chMtxLock(&comm_mutex);
		// dw_reset();
		// spiStart(&SPID1, &spi_cfg);
		// chThdSleepMilliseconds(100);
		// chMtxUnlock(&comm_mutex);
    }
}

// static THD_WORKING_AREA(wairqThread, 256);
// static THD_FUNCTION(irqThread, arg) {
//     (void)arg;

// 	 while (true) {	
// 		chSysLock();
//    		chThdSuspendS(&irq_evt);
// 		chSysUnlock();
// 		//palDisablePadEvent(IOPORT1, DW_IRQ);
// 		_dw_irq_handler();
// 		//palEnablePadEvent(IOPORT1, DW_IRQ, PAL_EVENT_MODE_RISING_EDGE);
//     }
// }

static THD_WORKING_AREA(wasendThread, 256);
static THD_FUNCTION(sendThread, arg) {
    (void)arg;

	sys_mask_t sent_mask;
	sent_mask.mask = 0U;
	sent_mask.MTXFRS = 0b1;
	event_listener_t send_l;
	chEvtRegisterMask(&message_evt, &send_l, EVENT_MASK(0));
	chMtxLock(&comm_mutex);
	dw_set_irq(sent_mask);
	chMtxUnlock(&comm_mutex);

	 while (true) {	
		send_message();
		chThdSleepMilliseconds(500);
		toggle_led(blue);
    }
}

void TXFRS_handler(void)
{
	chEvtSignal(comm_thread, MTXFRS_E);
	//chEvtBroadcast(&message_evt);
}

void RXFCG_handler(void)
{
	chEvtSignal(comm_thread, MRXFCG_E);
	//chEvtBroadcast(&recv_evt);
}

void RXERR_handler(void)
{
	chEvtSignal(comm_thread, MRXPHE_E | MRXFCE_E | MLDEERR_E);
	//toggle_led(blue);
	//chEvtBroadcast(&nrecv_evt);
}

int main(void) {
    halInit();
    chSysInit();

    leds_off(ALL_LEDS);

	dw_reset();
	spi_hal_init();

	irq_vector._dw_TXFRS_handler = TXFRS_handler;
	irq_vector._dw_RXFCG_handler = RXFCG_handler;
	// irq_vector._dw_RXRFTO_handler = RXERR_handler;
	irq_vector._dw_RXPHE_handler = RXERR_handler;
	irq_vector._dw_RXFCE_handler = RXERR_handler;
	// irq_vector._dw_RXRFSL_handler = RXERR_handler;
	// irq_vector._dw_RXSFDTO_handler = RXERR_handler;
	// irq_vector._dw_AFFREJ_handler = RXERR_handler;
	irq_vector._dw_LDEERR_handler = RXERR_handler;

	// chMtxObjectInit(&comm_mutex);

	palEnablePadEvent(IOPORT1, DW_IRQ, PAL_EVENT_MODE_RISING_EDGE);
	palSetPadCallback(IOPORT1, DW_IRQ, ISR_wrapper, NULL);
	chThdSleepMilliseconds(100);
	chThdCreateStatic(DW_IRQ_THREAD, sizeof(DW_IRQ_THREAD), NORMALPRIO+1, DW_IRQ_HANDLER, NULL);
	comm_thread = chThdCreateStatic(DW_CONTROLLER_THREAD, sizeof(DW_CONTROLLER_THREAD), NORMALPRIO, DW_CONTROLLER, NULL);
	// message_thread = chThdCreateStatic(wasendThread, sizeof(wasendThread), NORMALPRIO-2, sendThread, NULL);
	// recv_thread = chThdCreateStatic(warecvThread, sizeof(warecvThread), NORMALPRIO-3, recvThread, NULL);

	while (true) {	
		toggle_led(red1);
		chThdSleepMilliseconds(500);
	}
}