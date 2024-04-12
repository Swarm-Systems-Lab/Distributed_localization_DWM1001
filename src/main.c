#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "nrf52_radio.h"

#include "led.h"
#include "dw1000_hal.h"

SPIConfig spi_cfg = { .end_cb = NULL, .ssport = IOPORT1, .sspad = SPI_SS,
        .freq = NRF5_SPI_FREQ_8MBPS, .sckpad = SPI_SCK, .mosipad = SPI_MOSI,
        .misopad = SPI_MISO, .lsbfirst = false, .mode = 2};

thread_reference_t irq_evt = NULL;
thread_t* message_thread;
thread_t* recv_thread;
event_source_t message_evt;
event_source_t recv_evt;
event_source_t nrecv_evt;
static mutex_t comm_mutex;

static void irq_hand_wrapper(void * arg)
{
	(void)arg;
	chSysLockFromISR();
	chThdResumeI(&irq_evt, MSG_OK);
	chSysUnlockFromISR();
}

void _dw_reset(void)
{
	_dw_power_off();
	chThdSleepMilliseconds(10);
	_dw_power_on();
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
	if (count < 1000)
	{
		for (int i = 0; i < 5; i++)
			tx_ctrl.reg[i] = 0;
		tx_ctrl.TFLEN = 12;
		tx_ctrl.TXBR = 0b10;
		tx_ctrl.TXPRF = 0b1;
		tx_ctrl.TXPSR = 0b1;
		tx_ctrl.PE = 0b1;
		dw_start_tx(tx_ctrl, data);
		eventmask_t evt = chEvtWaitOneTimeout(EVENT_MASK(0), TIME_MS2I(600));
		if (!evt){
			sys_ctrl_t ctrl;
			ctrl.mask = 0;
			ctrl.TRXOFF = 1;
			dw_write(DW_REG_INFO.SYS_CTRL, ctrl.reg, DW_REG_INFO.SYS_CTRL.size, 0);
			chThdSleepMilliseconds(1);
		}
	}
	chMtxUnlock(&comm_mutex);

}

void recv_message() {
	sys_state_t state;
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
		dw_start_rx();
		eventmask_t evt = chEvtWaitOneTimeout(EVENT_MASK(1) | EVENT_MASK(3), TIME_MS2I(600));
		dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);
		dw_read(DW_REG_INFO.SYS_STATUS, status.reg, DW_REG_INFO.SYS_STATUS.size, 0);
		if (!evt || evt == EVENT_MASK(3)){
			sys_ctrl_t ctrl;
			ctrl.mask = 0;
			ctrl.TRXOFF = 1;
			dw_write(DW_REG_INFO.SYS_CTRL, ctrl.reg, DW_REG_INFO.SYS_CTRL.size, 0);
			chThdSleepMilliseconds(1);
		}
		dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);
		uint8_t rx2[10]={0,0,0,0,0,0,0,0,0,0};
		size_t rx2_size = 10;
		dw_read(DW_REG_INFO.RX_BUFFER, rx2, rx2_size, 0);

		if (evt == EVENT_MASK(1))
		{
			toggle_led(green);
			//chThdSleepMilliseconds(500);
		}
		if (evt == EVENT_MASK(3))
		{
			toggle_led(red2);
			//chThdSleepMilliseconds(500);
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

static THD_WORKING_AREA(wairqThread, 256);
static THD_FUNCTION(irqThread, arg) {
    (void)arg;

	 while (true) {	
		chSysLock();
   		chThdSuspendS(&irq_evt);
		chSysUnlock();
		//palDisablePadEvent(IOPORT1, DW_IRQ);
		_dw_irq_handler();
		//palEnablePadEvent(IOPORT1, DW_IRQ, PAL_EVENT_MODE_RISING_EDGE);
    }
}

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

void chibi_power_on(void)
{
	palSetPad(IOPORT1, DW_RST);
}

void chibi_power_off(void)
{
	palClearPad(IOPORT1, DW_RST);
}

void chibi_spi_lock(void)
{
	spiAcquireBus(&SPID1);
}

void chibi_spi_unlock(void)
{
	spiReleaseBus(&SPID1);
}

void chibi_spi_set_cs(void)
{
	spiSelect(&SPID1);
}

void chibi_spi_clear_cs(void)
{
	spiUnselect(&SPID1);
}

void chibi_spi_send(size_t count, const uint8_t* buf)
{
	spiSend(&SPID1, count, buf);
}

void chibi_spi_recv(size_t count, const uint8_t* buf)
{
	spiReceive(&SPID1, count, buf);
}

void callback1(void)
{
	chEvtSignal(message_thread, EVENT_MASK(0));
	//chEvtBroadcast(&message_evt);
}

void callback2(void)
{
	chEvtSignal(recv_thread, EVENT_MASK(1));
	//chEvtBroadcast(&recv_evt);
}

void callback3(void)
{
	chEvtSignal(recv_thread, EVENT_MASK(3));
	//toggle_led(blue);
	//chEvtBroadcast(&nrecv_evt);
}

int main(void) {
    halInit();
    chSysInit();

    leds_off(ALL_LEDS);

	dw_set_power_on(chibi_power_on);
	dw_set_power_off(chibi_power_off);

	_dw_power_off();
	chThdSleepMilliseconds(10);
	_dw_power_on();

	spiStart(&SPID1, &spi_cfg);

	dw_set_spi_lock(chibi_spi_lock);
	dw_set_spi_unlock(chibi_spi_unlock);
	dw_set_spi_set_cs(chibi_spi_set_cs);
	dw_set_spi_clear_cs(chibi_spi_clear_cs);
	dw_set_spi_send(chibi_spi_send);
	dw_set_spi_recv(chibi_spi_recv);

	irq_vector._dw_TXFRS_handler = callback1;
	irq_vector._dw_RXFCG_handler = callback2;
	// irq_vector._dw_RXRFTO_handler = callback3;
	irq_vector._dw_RXPHE_handler = callback3;
	irq_vector._dw_RXFCE_handler = callback3;
	// irq_vector._dw_RXRFSL_handler = callback2;
	// irq_vector._dw_RXSFDTO_handler = callback2;
	// irq_vector._dw_AFFREJ_handler = callback2;
	irq_vector._dw_LDEERR_handler = callback3;

	chMtxObjectInit(&comm_mutex);

	// sys_mask_t sent_mask;
	// sent_mask.mask = 0U;
	// sent_mask.MTXFRS = 0b1;
	// dw_set_irq(sent_mask);

	// size_t data_size = 10;
	// uint8_t sendtx[4] = {0x2,0x0,0x0,0x2};
	// uint8_t data[10] = {0xde,0xad,0xbe,0xef,0xef,0xbe,0xad,0xde,0x0,0x0};

	// event_listener_t irq_lst;

	// chEvtRegisterMask(&irq_evt,
    //                 &irq_lst,
    //                 EVENT_MASK(2));

	//TODO poner otra hebra para listener irq

	// dw_write(DW_REG_INFO.TX_BUFFER, data, data_size, 0);
	// dw_write(DW_REG_INFO.SYS_CTRL, sendtx, 4, 0);

	sys_cfg_t cfdf;
	cfdf.mask = 0;
	cfdf.HIRQ_POL = 1;
	cfdf.DIS_DRXB = 1;
	cfdf.RXWTOE = 1;

	//dw_write(DW_REG_INFO.SYS_CFG, cfdf.reg, DW_REG_INFO.SYS_CFG.size, 0);

	palEnablePadEvent(IOPORT1, DW_IRQ, PAL_EVENT_MODE_RISING_EDGE);
	palSetPadCallback(IOPORT1, DW_IRQ, irq_hand_wrapper, NULL);
	chThdSleepMilliseconds(100);
	chThdCreateStatic(wairqThread, sizeof(wairqThread), NORMALPRIO+1, irqThread, NULL);
	message_thread = chThdCreateStatic(wasendThread, sizeof(wasendThread), NORMALPRIO-2, sendThread, NULL);
	recv_thread = chThdCreateStatic(warecvThread, sizeof(warecvThread), NORMALPRIO-3, recvThread, NULL);

	while (true) {	
		toggle_led(red1);
		chThdSleepMilliseconds(500);
	}
}
