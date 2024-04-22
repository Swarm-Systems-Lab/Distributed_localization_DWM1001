/**
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "dist_loc.h"

thread_reference_t irq_evt = NULL;

SPIConfig spi_cfg = {.end_cb = NULL, .ssport = IOPORT1, .sspad = SPI_SS,
        .freq = NRF5_SPI_FREQ_2MBPS, .sckpad = SPI_SCK, .mosipad = SPI_MOSI,
        .misopad = SPI_MISO, .lsbfirst = false, .mode = 2};

void ISR_wrapper(void * arg)
{
	(void)arg;

	chSysLockFromISR();
	chThdResumeI(&irq_evt, MSG_OK);
	chSysUnlockFromISR();
}

THD_FUNCTION(DW_IRQ_HANDLER, arg)
{
    (void)arg;

	 while (true) {	
		chSysLock();
   		chThdSuspendS(&irq_evt);
		chSysUnlock();
		_dw_irq_handler();
    }
}

void spi1_lock(void)
{
	spiAcquireBus(&SPID1);
}

void spi1_unlock(void)
{
	spiReleaseBus(&SPID1);
}

void spi1_set_cs(void)
{
	spiSelect(&SPID1);
}

void spi1_clear_cs(void)
{
	spiUnselect(&SPID1);
}

void spi1_send(size_t count, const uint8_t* buf)
{
	spiSend(&SPID1, count, buf);
}

void spi1_recv(size_t count, const uint8_t* buf)
{
	spiReceive(&SPID1, count, buf);
}

void _dw_power_on(void)
{
	palSetPad(IOPORT1, DW_RST);
	spiStart(&SPID1, &spi_cfg);
}

void _dw_power_off(void)
{
	spiStop(&SPID1);
	palClearPad(IOPORT1, DW_RST);
}

void dw_reset(void)
{
	spi1_lock();	
	_dw_power_off();
	chThdSleepMilliseconds(3);
	spi_cfg.freq = NRF5_SPI_FREQ_2MBPS;
	_dw_power_on();
	spi1_unlock();
}

THD_FUNCTION(DW_CONTROLLER, arg)
{
	(void)arg;
	// sys_cfg_t cfdf;
	// cfdf.mask = 0;
	// cfdf.HIRQ_POL = 1;
	// cfdf.DIS_DRXB = 1;
	// cfdf.RXWTOE = 1;
	// dw_write(DW_REG_INFO.SYS_CFG, cfdf.reg, DW_REG_INFO.SYS_CFG.size, 0);
	// uint8_t fwto[2] = {0xFF, 0xFE};
	// dw_write(DW_REG_INFO.RX_FWTO, fwto, DW_REG_INFO.RX_FWTO.size, 0);
	// dw_read(DW_REG_INFO.RX_FWTO, fwto, DW_REG_INFO.RX_FWTO.size, 0);

	sys_state_t state;

	sys_mask_t irq_mask;

	irq_mask.mask = 0U;
	irq_mask.MTXFRS = 0b1;
	irq_mask.MRXFCG = 0b1;
	// irq_mask.MRXRFTO = 0b1;
	irq_mask.MRXPHE = 0b1;
	irq_mask.MRXFCE = 0b1;
	// irq_mask.MRXRFSL = 0b1;
	irq_mask.MLDEERR = 0b1;

	tx_fctrl_t tx_ctrl;
	dx_time_t dx_time;
	ack_resp_t_t w4r;

	uint8_t data[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t recv[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	dw_clear_register(tx_ctrl.reg, sizeof(tx_ctrl.reg));

	uint64_t id = get_hardware_id();
	uint64_t rx_time = 0;
	uint64_t tx_time = 0;
	uint64_t m_rx_time = 0;
	uint64_t m_tx_time = 0;
	uint64_t delay_tx = 0;
	int64_t rt_init = 0;
	int64_t rt_resp = 0;
	uint64_t sys_time = 0;
	uint8_t sys_time_raw[5] = {0,0,0,0,0};

	tx_ctrl.TFLEN = 22;
	tx_ctrl.TXBR = 0b10;
	tx_ctrl.TXPRF = 0b1;
	tx_ctrl.TXPSR = 0b1;
	tx_ctrl.PE = 0b1;
	
	dw_set_irq(irq_mask);

	uint16_t tx_ant_d = 0;
	uint16_t rx_ant_d = 0;
	eventmask_t evt = 0;

	// dw_write(DW_REG_INFO.LDE_CTRL, &rx_ant_d, 2, 0x1804);
	// dw_write(DW_REG_INFO.TX_ANTD, &tx_ant_d, 2, 0);

	// tx_ant_d = 0;
	// rx_ant_d = 0;

	// dw_read(DW_REG_INFO.TX_ANTD, &tx_ant_d, 2, 0);
	// dw_read(DW_REG_INFO.LDE_CTRL, &rx_ant_d, 2, 0x1804);

	double tof = 0.0;
	double distance = 0.0;
	float clock_offset_r = 0.0;

	if (id == 588618085864310691)
	{
		dw_clear_register(dx_time.reg, sizeof(dx_time.reg));
		// Initiator
		while (true)
		{
			dw_clear_register(w4r.reg, sizeof(w4r.reg));
			w4r.ACK_TIM = 1;
			dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);
			dw_start_tx(tx_ctrl, data, dx_time, w4r);
			evt = chEvtWaitOneTimeout(MRXPHE_E | MRXFCE_E | MLDEERR_E | MRXFCG_E, TIME_MS2I(30));
			if (evt == MRXFCG_E)
			{
				toggle_led(green);
				tx_time = dw_get_tx_time() + tx_ant_d;
				rx_time = dw_get_rx_time() + rx_ant_d;
				dw_read(DW_REG_INFO.RX_BUFFER, recv, sizeof(recv), 0);
				// m_tx_time |= (uint64_t)recv[0];
				// m_tx_time |= (uint64_t)recv[1] << 8;
				// m_tx_time |= (uint64_t)recv[2] << 16;
				// m_tx_time |= (uint64_t)recv[3] << 24;
				// m_tx_time |= (uint64_t)recv[4] << 32;
				memcpy(&m_tx_time, recv, 5);

				// m_rx_time |= (uint64_t)recv[8];
				// m_rx_time |= (uint64_t)recv[9] << 8;
				// m_rx_time |= (uint64_t)recv[10] << 16;
				// m_rx_time |= (uint64_t)recv[11] << 24;
				// m_rx_time |= (uint64_t)recv[12] << 32;
				memcpy(&m_rx_time, recv+8, 5);

				rt_init = rx_time - tx_time;
				rt_resp = m_tx_time - m_rx_time;

				clock_offset_r = dw_get_car_int() * ((998.4e6/2.0/1024.0/131072.0) * (-1.0e6/6489.6e6) / 1.0e6);
				tof = ((rt_init - rt_resp * (1.0f - clock_offset_r)) / 2.0f) * (1.0/499.2e6/128.0);
				distance = tof * 299702547;
				distance -= 149;
				distance *= 64;
			}
			else
				dw_soft_reset_rx();
			//dw_soft_reset_rx();
			evt = 0;
			state = dw_transceiver_off();
			chThdSleepMilliseconds(100);
			dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);

			// dw_reset();
			// set_fast_spi_freq();
			// dw_set_irq(irq_mask);
		}
	}
	else
	{
		// Responder
		dw_clear_register(w4r.reg, sizeof(w4r.reg));
		while (true)
		{
			dw_clear_register(dx_time.reg, sizeof(dx_time.reg));
			dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);
			dw_start_rx(dx_time);
			evt = chEvtWaitOneTimeout(MRXPHE_E | MRXFCE_E | MLDEERR_E | MRXFCG_E, TIME_MS2I(30));
			if (evt == MRXFCG_E)
			{
				toggle_led(green);
				rx_time = dw_get_rx_time() + rx_ant_d;
				delay_tx = (uint64_t)rx_time + (uint64_t)(65536*2500);
				// tx_time = (uint64_t)((uint64_t)delay_tx & 0xFFFFFE00) + (uint64_t)tx_ant_d;
				dx_time.time = (uint32_t)(delay_tx >> 8);
				// sys_time = 0;
				// dw_read(DW_REG_INFO.SYS_TIME, sys_time_raw, DW_REG_INFO.SYS_TIME.size, 0);
				// sys_time |= (uint64_t)sys_time_raw[0];
				// sys_time |= (uint64_t)sys_time_raw[1] << 8;
				// sys_time |= (uint64_t)sys_time_raw[2] << 16;
				// sys_time |= (uint64_t)sys_time_raw[3] << 24;
				// sys_time |= (uint64_t)sys_time_raw[4] << 32;
				// dx_time.time = (uint32_t)((sys_time +(65536*5000)) >> 8);
				// m_tx_time = ((sys_time+(65536*5000)) & 0xFFFFFE00) + 16456;
				memcpy(data, &delay_tx, 8);
				memcpy(data+8, &rx_time, 8);
				dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);
				dw_start_tx(tx_ctrl, data, dx_time, w4r);
				evt = chEvtWaitOneTimeout(MTXFRS_E, TIME_MS2I(30));
				if (!evt)
					dw_soft_reset_rx();
			}
			else
				dw_soft_reset_rx();
			// dw_soft_reset_rx();
			evt = 0;
			state = dw_transceiver_off();
			chThdSleepMilliseconds(80);
			dw_read(DW_REG_INFO.SYS_STATE, state.reg, DW_REG_INFO.SYS_STATE.size, 0);

			// dw_reset();
			// set_fast_spi_freq();
			// chThdSleepMilliseconds(10);
			// dw_set_irq(irq_mask);
		}

	}

	// memcpy(data, &id, sizeof(id));

	// while (true)
	// {
	// 	dw_clear_register(tx_ctrl.reg, sizeof(tx_ctrl.reg));
	// 	dw_clear_register(dx_time.reg, sizeof(dx_time.reg));
	// 	dw_clear_register(w4r.reg, sizeof(w4r.reg));
	// 	tx_ctrl.TFLEN = 12;
	// 	tx_ctrl.TXBR = 0b10;
	// 	tx_ctrl.TXPRF = 0b1;
	// 	tx_ctrl.TXPSR = 0b1;
	// 	tx_ctrl.PE = 0b1;
	// 	dw_start_tx(tx_ctrl, data, dx_time, w4r);
	// 	evt = chEvtWaitOneTimeout(MTXFRS_E, TIME_MS2I(20));
	// 	if (evt)
	// 		toggle_led(blue);

	// 	state = dw_transceiver_off();
	// 	chThdSleepMilliseconds(3);

	// 	// SEND OVER

	// 	dw_start_rx(dx_time);
	// 	eventmask_t evt = chEvtWaitOneTimeout(MRXPHE_E | MRXFCE_E | MLDEERR_E | MRXFCG_E, TIME_MS2I(600));

	// 	if (evt)
	// 	{
	// 		if (evt == MRXFCG_E)
	// 		{
	// 			toggle_led(green);
	// 			dw_read(DW_REG_INFO.RX_BUFFER, recv, sizeof(recv), 0);
	// 		}
	// 		else
	// 			toggle_led(red2);
	// 	}

	// 	state = dw_transceiver_off();
	// 	chThdSleepMilliseconds(3);
	// }
}

void set_fast_spi_freq(void)
{
	spi1_lock();	
	spiStop(&SPID1);
	spi_cfg.freq = NRF5_SPI_FREQ_8MBPS;
	spiStart(&SPID1, &spi_cfg);
	spi1_unlock();	
}

void set_slow_spi_freq(void)
{
	spi1_lock();	
	spiStop(&SPID1);
	spi_cfg.freq = NRF5_SPI_FREQ_2MBPS;
	spiStart(&SPID1, &spi_cfg);
	spi1_unlock();	
}

uint64_t get_hardware_id(void)
{
	uint64_t id = 0;
	uint32_t part_id = 0;
	uint32_t lot_id = 0;

	dw_command_read_OTP(0x6);
	spi1_lock();
	chThdSleepMicroseconds(1);
	spi1_unlock();
	dw_read(DW_REG_INFO.OTP_IF, &part_id, sizeof(part_id), 0xA);
	dw_command_read_OTP(0x7);
	spi1_lock();
	chThdSleepMicroseconds(1);
	spi1_unlock();
	dw_read(DW_REG_INFO.OTP_IF, &lot_id, sizeof(lot_id), 0xA);

	id = (uint64_t)part_id | ((uint64_t)lot_id << 32);

	return id;
}

void spi_hal_init(void)
{
	set_fast_spi_freq();
	dw_set_spi_lock(spi1_lock);
	dw_set_spi_unlock(spi1_unlock);
	dw_set_spi_set_cs(spi1_set_cs);
	dw_set_spi_clear_cs(spi1_clear_cs);
	dw_set_spi_send(spi1_send);
	dw_set_spi_recv(spi1_recv);
}