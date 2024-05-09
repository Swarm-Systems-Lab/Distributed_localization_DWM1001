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

frame_control_t def_frame_ctrl = {.mask=0x9841};
// MHR.frame_control.frame_type = FT_DATA;
// MHR.frame_control.sec_en = 0b0;
// MHR.frame_control.frame_pending = 0b0;
// MHR.frame_control.ack_req = 0b0;
// MHR.frame_control.pan_id_compress = 0b1;
// MHR.frame_control.dest_addr_mode = SHORT_16;
// MHR.frame_control.frame_version = 0x1;
// MHR.frame_control.src_addr_mode = SHORT_16;

uint16_t neighbours[3] = {0,0,0};
uint16_t neighbours_3wh[3] = {0,0,0};

SPIConfig spi_cfg = 
{
	.end_cb = NULL,
	.ssport = IOPORT1,
	.sspad = SPI_SS,
    .freq = NRF5_SPI_FREQ_2MBPS,
	.sckpad = SPI_SCK,
	.mosipad = SPI_MOSI,
    .misopad = SPI_MISO,
	.lsbfirst = false,
	.mode = 2
};

SerialConfig serial_cfg = 
{
  .speed = 115200,
  .tx_pad  = UART_TX,
  .rx_pad  = UART_RX,
};

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

void loc_init_fun(uint16_t addr)
{
	sys_state_t state;
	tx_fctrl_t tx_ctrl;
	dx_time_t dx_time;
	ack_resp_t_t w4r;
	uint64_t rx_time = 0;
	uint64_t tx_time = 0;
	uint64_t m_rx_time = 0;
	uint64_t m_tx_time = 0;
	int64_t rt_init = 0;
	int64_t rt_resp = 0;
	eventmask_t evt = 0;
	panadr_t panadr;
	uint8_t data[29] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t recv[29] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	double tof = 0.0;
	double distance = 0.0;
	float clock_offset_r = 0.0;
	float distances[20] = {0,0,0,0,0};
	uint8_t cnt = 0;

	dw_read(DW_REG_INFO.PAN_ADR, panadr.reg, DW_REG_INFO.PAN_ADR.size, 0);
	encode_MHR(def_frame_ctrl, data, 0x0, panadr.pan_id, addr, panadr.short_addr);
	dw_clear_register(tx_ctrl.reg, sizeof(tx_ctrl.reg));
	tx_ctrl.TFLEN = sizeof(data)+2;
	tx_ctrl.TXBR = BR_6_8MBPS;
	tx_ctrl.TXPRF = PRF_16MHZ;
	tx_ctrl.TXPL = PL_128;

	w4r.mask = 0;
	dw_clear_register(dx_time.reg, sizeof(dx_time.reg));
	w4r.ACK_TIM = 1;
	dw_start_tx(tx_ctrl, data, dx_time, w4r);
	evt = chEvtWaitOneTimeout(MRXPHE_E | MRXFCE_E | MLDEERR_E | MRXFCG_E, TIME_MS2I(30));
	if (evt == MRXFCG_E)
	{
		toggle_led(green);
		tx_time = dw_get_tx_time();
		rx_time = dw_get_rx_time();
		dw_read(DW_REG_INFO.RX_BUFFER, recv, sizeof(recv), 0);
		memcpy(&m_tx_time, recv+9, 5);
		memcpy(&m_rx_time, recv+17, 5);

		rt_init = (rx_time) - (tx_time);
		rt_resp = (m_tx_time) - (m_rx_time);

		clock_offset_r = dw_get_car_int() * ((998.4e6/2.0/1024.0/131072.0) * (-1.0e6/6489.6e6) / 1.0e6);
		
		//rt_resp *= (1.0f - clock_offset_r);

		tof = (rt_init - rt_resp)/2.0f;
		tof = tof * (1.0f/(float)(499.2e6*128.0));

		distance = tof * 299702547;
		distance *= 100;

		if (distance > 0 && distance < 1000)
			distances[cnt] = distance;

		cnt++;
		if (cnt == 10)
			cnt = 0;

		distance = 0;

		for (int i = 0; i < 10; i++)
			distance += distances[i];

		distance /= 10;

		chprintf((BaseSequentialStream*)&SD1, "Distance: %dcm\n", (int)distance);
	}
	else
		dw_soft_reset_rx();
	evt = 0;
	state = dw_transceiver_off();
	chThdSleepMilliseconds(50);
}

void loc_resp_fun(uint16_t addr)
{
	sys_state_t state;
	tx_fctrl_t tx_ctrl;
	dx_time_t dx_time;
	ack_resp_t_t w4r;
	uint64_t rx_time = 0;
	uint64_t tx_time = 0;
	uint64_t delay_tx = 0;
	eventmask_t evt = 0;
	uint16_t tx_ant_d;
	panadr_t panadr;

	dw_read(DW_REG_INFO.TX_ANTD, &tx_ant_d, DW_REG_INFO.TX_ANTD.size, 0);

	uint8_t data[29] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t recv[29] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	dw_read(DW_REG_INFO.PAN_ADR, panadr.reg, DW_REG_INFO.PAN_ADR.size, 0);
	encode_MHR(def_frame_ctrl, data, 0x0, panadr.pan_id, addr, panadr.short_addr);
	dw_clear_register(tx_ctrl.reg, sizeof(tx_ctrl.reg));
	tx_ctrl.TFLEN = sizeof(data)+2;
	tx_ctrl.TXBR = BR_6_8MBPS;
	tx_ctrl.TXPRF = PRF_16MHZ;
	tx_ctrl.TXPL = PL_128;

	w4r.mask = 0;
	dw_clear_register(dx_time.reg, sizeof(dx_time.reg));
	dw_start_rx(dx_time);
	evt = chEvtWaitOneTimeout(MRXPHE_E | MRXFCE_E | MLDEERR_E | MRXFCG_E, TIME_MS2I(31));
	if (evt == MRXFCG_E)
	{
		toggle_led(green);
		dw_read(DW_REG_INFO.RX_BUFFER, recv, sizeof(recv), 0);
		rx_time = dw_get_rx_time();
		delay_tx = (uint64_t)rx_time + (uint64_t)(65536*3000);
		tx_time = (uint64_t)delay_tx + (uint64_t)tx_ant_d;
		dx_time.time32 = (uint32_t)(delay_tx >> 8);
		memcpy(data+9, &tx_time, 8);
		memcpy(data+17, &rx_time, 8);
		dw_start_tx(tx_ctrl, data, dx_time, w4r);
		evt = chEvtWaitOneTimeout(MTXFRS_E, TIME_MS2I(30));
		if (!evt)
			dw_soft_reset_rx();
	}
	else
		dw_soft_reset_rx();
	evt = 0;
	state = dw_transceiver_off();
	chThdSleepMilliseconds(51);
}

void loc_disc_fun(void)
{

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
	tx_fctrl_t tx_ctrl;
	dx_time_t dx_time;
	ack_resp_t_t w4r;
	sys_mask_t irq_mask;
	MHR_16_t MHR;
	panadr_t panadr;

	irq_mask.mask = 0U;
	irq_mask.MTXFRS = 0b1;
	irq_mask.MRXFCG = 0b1;
	//irq_mask.MLDEDONE = 0b1;
	// irq_mask.MRXRFTO = 0b1;
	irq_mask.MRXPHE = 0b1;
	irq_mask.MRXFCE = 0b1;
	irq_mask.MRXRFSL = 0b1;
	irq_mask.MLDEERR = 0b1;
	irq_mask.MAFFREJ = 0b1;
	
	sdStart(&SD1, &serial_cfg);

	uint64_t id = get_hardware_id();
	srand(id);
	dw_write(DW_REG_INFO.PAN_ADR, &id, 2, 0);
	uint16_t tx_ant_d = 63520;
	uint16_t rx_ant_d = 63520;
	eventmask_t evt = 0;
	
	dw_set_irq(irq_mask);

	dw_write(DW_REG_INFO.LDE_CTRL, &rx_ant_d, DW_SUBREG_INFO.LDE_RXANTD.size, DW_SUBREG_INFO.LDE_RXANTD.offset);
	dw_write(DW_REG_INFO.TX_ANTD, &tx_ant_d, DW_REG_INFO.TX_ANTD.size, 0);

	uint8_t data[10] = {0,0,0,0,0,0,0,0,0,0};
	uint8_t recv[10] = {0,0,0,0,0,0,0,0,0,0};

	uint16_t addrs[3] = {0,0,0};
	uint8_t end[3] = {0,0,0};

	dw_read(DW_REG_INFO.PAN_ADR, addrs, 2, 0);
	uint8_t finish[3] = {0,0,0};

	while (!(finish[0] == 0x1 && finish[1] == 0x1 && finish[2] == 0x1) && !(end[0] == 0x1 && end[1] == 0x1 && end[2] == 0x1))
	{
		dw_clear_register(dx_time.reg, sizeof(dx_time.reg));
		dw_start_rx(dx_time);
		evt = chEvtWaitOneTimeout(MRXPHE_E | MRXFCE_E | MLDEERR_E | MRXFCG_E, TIME_MS2I(300+(id&0x3F)));
		if (evt == MRXFCG_E)
		{
			toggle_led(green);
			dw_read(DW_REG_INFO.RX_BUFFER, recv, sizeof(recv), 0);
			MHR = decode_MHR(recv);
			if (addrs[1] == 0)
				addrs[1] = MHR.src_addr;
			if (addrs[1] != 0 && addrs[1] != MHR.src_addr)
			{
				addrs[2] = MHR.src_addr;
			}

			if (recv[9] == 0xD)
			{
				for (int i = 1; i < 3; i++)
				{
					if (addrs[i] == MHR.src_addr)
						end[i] = 0x1;
				}
			}

			if (recv[9] == 0xC)
			{
				for (int i = 1; i < 3; i++)
				{
					if (addrs[i] == MHR.src_addr)
						finish[i] = 0x1;
				}
			}
			dw_clear_register(recv, sizeof(recv));
		}
		else
		{
			dw_soft_reset_rx();
			state = dw_transceiver_off();
			dw_read(DW_REG_INFO.PAN_ADR, panadr.reg, DW_REG_INFO.PAN_ADR.size, 0);
			encode_MHR(def_frame_ctrl, data, 0x0, panadr.pan_id, 0xFFFF, panadr.short_addr);
			dw_clear_register(tx_ctrl.reg, sizeof(tx_ctrl.reg));
			tx_ctrl.TFLEN = sizeof(data)+2;
			tx_ctrl.TXBR = BR_6_8MBPS;
			tx_ctrl.TXPRF = PRF_16MHZ;
			tx_ctrl.TXPL = PL_128;
			w4r.mask = 0;
			dw_clear_register(dx_time.reg, sizeof(dx_time.reg));
			dw_start_tx(tx_ctrl, data, dx_time, w4r);
			dw_clear_register(data, sizeof(data));
			evt = chEvtWaitOneTimeout(MTXFRS_E, TIME_MS2I(30));
			if (!evt)
				dw_soft_reset_rx();
			chThdSleepMilliseconds(50);
		}
		evt = 0;
		state = dw_transceiver_off();
		chThdSleepMilliseconds(51);

		if (addrs[0] != 0x0 && addrs[1] != 0x0 && addrs[2] != 0x0)
		{
			end[0] = 0x1;
			data[9] = 0xD;
		}

		if ((addrs[0] != 0x0 && addrs[1] != 0x0 && addrs[2] != 0x0) && (end[0] == 0x1 && end[1] == 0x1 && end[2] == 0x1))
		{
			finish[0] = 0x1;
			data[9] = 0xC;
		}
	}
	while (true) {	
		toggle_led(blue);
		chThdSleepMilliseconds(500);
	}
}

THD_FUNCTION(SYSTEM_STATUS, arg)
{
	loc_state_t loc_state = LOC_DISC;

	uint16_t addrs[3] = {0,0,0};

	dw_read(DW_REG_INFO.PAN_ADR, addrs, 2, 0);

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

	dw_command_read_OTP(PARTID);
	spi1_lock();
	chThdSleepMicroseconds(1);
	spi1_unlock();
	dw_read(DW_REG_INFO.OTP_IF, &part_id, sizeof(part_id), DW_SUBREG_INFO.OTP_RDAT.offset);
	dw_command_read_OTP(LOTID);
	spi1_lock();
	chThdSleepMicroseconds(1);
	spi1_unlock();
	dw_read(DW_REG_INFO.OTP_IF, &lot_id, sizeof(lot_id), DW_SUBREG_INFO.OTP_RDAT.offset);

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

void load_lde(void)
{
	pmsc_ctrl0_t pmsc_ctrl0;
	otp_ctrl_t otp_ctrl;

	pmsc_ctrl0.mask = 0x0200;
	pmsc_ctrl0.ADCCE = 0b1;
	pmsc_ctrl0.SYSCLKS = 0b10; //125 MHz
	otp_ctrl.mask = 0;
	otp_ctrl.LDELOAD = 0b1;

	dw_write(DW_REG_INFO.PMSC, pmsc_ctrl0.reg, 2, DW_SUBREG_INFO.PMSC_CTRL0.offset);
	dw_write(DW_REG_INFO.OTP_IF, otp_ctrl.reg, DW_SUBREG_INFO.OTP_CTRL.size, DW_SUBREG_INFO.OTP_CTRL.offset);
	spi1_lock();
	chThdSleepMicroseconds(150);
	spi1_unlock();
	pmsc_ctrl0.mask = 0x0200;
	dw_write(DW_REG_INFO.PMSC, pmsc_ctrl0.reg, 2, DW_SUBREG_INFO.PMSC_CTRL0.offset);
}

uint64_t load_ldotune(void)
{
	// TODO Check array sizes and otp address magic number
	ldotune_t ldotune;
	uint64_t ldotune64 = 0;
	dw_command_read_OTP(LDOTUNE0);
	spi1_lock();
	chThdSleepMicroseconds(1);
	spi1_unlock();
	dw_read(DW_REG_INFO.OTP_IF, ldotune.reg, 4, DW_SUBREG_INFO.OTP_RDAT.offset);
	
	if (!ldotune.reg[0])
		return 0;

	dw_command_read_OTP(LDOTUNE1);
	spi1_lock();
	chThdSleepMicroseconds(1);
	spi1_unlock();
	dw_read(DW_REG_INFO.OTP_IF, ldotune.reg+4, 1, DW_SUBREG_INFO.OTP_RDAT.offset);

	memcpy(&ldotune64, ldotune.reg, DW_SUBREG_INFO.LDO_TUNE.size);

	return ldotune64;
}