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

#include "dw1000_ch.h"

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

thread_reference_t irq_evt;
thread_t* dw_thread;

mailbox_t dw_controller;
msg_t dw_controller_msg;

mailbox_t dw_controller_resp;
msg_t dw_controller_resp_msg;

panadr_t panadr_own;
tx_antd_t tx_antd;
uint16_t rx_ant_d;

rx_finfo_t rx_finfo;

uint32_t recv_tmo_usec;

dw1000_resp_t dw1000_resp = 
{
	.state = DW_NO_RESP,
	.recvd_size = 0, .rx_time = 0,
	.tx_time = 0,
	.recv_buf = { 0 }
};

uint8_t recv_tmo_cnt;


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

void dw_soft_reset_rx(void)
{
	pmsc_ctrl0_t pmsc_ctrl_sr;
	_dw_spi_hal_set._dw_spi_lock();
	_dw_spi_transaction(1, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, DW_SUBREG_INFO.PMSC_CTRL0.size, DW_SUBREG_INFO.PMSC_CTRL0.offset);
	// Todo check reserved bits as 1 maybe not write whole register
	pmsc_ctrl_sr.mask |= 0x00300200;
	pmsc_ctrl_sr.SOFTRESET &= 0b1110; // Clear bit 28
	_dw_spi_transaction(0, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, DW_SUBREG_INFO.PMSC_CTRL0.size, DW_SUBREG_INFO.PMSC_CTRL0.offset);
	pmsc_ctrl_sr.SOFTRESET |= 0b0001; // Set bit 28
	chThdSleepMilliseconds(1);
	_dw_spi_transaction(0, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, DW_SUBREG_INFO.PMSC_CTRL0.size, DW_SUBREG_INFO.PMSC_CTRL0.offset);
	_dw_spi_hal_set._dw_spi_unlock();
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
	dw_read(DW_REG_INFO.OTP_IF, (uint8_t*)(&part_id), sizeof(part_id), DW_SUBREG_INFO.OTP_RDAT.offset);
	dw_command_read_OTP(LOTID);
	spi1_lock();
	chThdSleepMicroseconds(1);
	spi1_unlock();
	dw_read(DW_REG_INFO.OTP_IF, (uint8_t*)(&lot_id), sizeof(lot_id), DW_SUBREG_INFO.OTP_RDAT.offset);

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

void set_irq_vector(void)
{
	irq_vector._dw_CPLOCK_handler		= CPLOCK_handler;
	irq_vector._dw_ESYNCR_handler		= ESYNCR_handler;
	irq_vector._dw_AAT_handler			= AAT_handler;
	irq_vector._dw_TXFRB_handler		= TXFRB_handler;
	irq_vector._dw_TXPRS_handler		= TXPRS_handler;
	irq_vector._dw_TXPHS_handler		= TXPHS_handler;
	irq_vector._dw_TXFRS_handler		= TXFRS_handler;
	irq_vector._dw_RXPRD_handler		= RXPRD_handler;
	irq_vector._dw_RXFSDD_handler		= RXFSDD_handler;
	irq_vector._dw_LDEDONE_handler		= LDEDONE_handler;
	irq_vector._dw_RXPHD_handler		= RXPHD_handler;
	irq_vector._dw_RXPHE_handler		= RXPHE_handler;
	irq_vector._dw_RXDFR_handler		= RXDFR_handler;
	irq_vector._dw_RXFCG_handler		= RXFCG_handler;
	irq_vector._dw_RXFCE_handler		= RXFCE_handler;
	irq_vector._dw_RXRFSL_handler		= RXRFSL_handler;
	irq_vector._dw_RXRFTO_handler		= RXRFTO_handler;
	irq_vector._dw_LDEERR_handler		= LDEERR_handler;
	irq_vector._dw_RXOVRR_handler		= RXOVRR_handler;
	irq_vector._dw_RXPTO_handler		= RXPTO_handler;
	irq_vector._dw_GPIOIRQ_handler		= GPIOIRQ_handler;
	irq_vector._dw_SLP2INIT_handler		= SLP2INIT_handler;
	irq_vector._dw_RFPLL_LL_handler		= RFPLLLL_handler;
	irq_vector._dw_CLKPLL_LL_handler	= CPLLLL_handler;
	irq_vector._dw_RXSFDTO_handler		= RXSFDTO_handler;
	irq_vector._dw_HPDWARN_handler		= HPDWARN_handler;
	irq_vector._dw_TXBERR_handler		= TXBERR_handler;
	irq_vector._dw_AFFREJ_handler		= AFFREJ_handler;
}


void dw_setup(void)
{
	dw_reset();
	spi_hal_init();
	default_config();
	load_lde();

	sys_mask_t irq_mask;

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

	dw_set_irq(irq_mask);

	// sys_cfg_t cfdf;
	// cfdf.mask = 0;
	// cfdf.HIRQ_POL = 1;
	// cfdf.DIS_DRXB = 1;
	// cfdf.RXWTOE = 1;
	// dw_write(DW_REG_INFO.SYS_CFG, cfdf.reg, DW_REG_INFO.SYS_CFG.size, 0);
	// uint8_t fwto[2] = {0xFF, 0xFE};
	// dw_write(DW_REG_INFO.RX_FWTO, fwto, DW_REG_INFO.RX_FWTO.size, 0);
	// dw_read(DW_REG_INFO.RX_FWTO, fwto, DW_REG_INFO.RX_FWTO.size, 0);

	uint64_t id = get_hardware_id();
	srand(id);
	dw_write(DW_REG_INFO.PAN_ADR, (uint8_t*)(&id), 2, 0);
	dw_read(DW_REG_INFO.PAN_ADR, panadr_own.reg, DW_REG_INFO.PAN_ADR.size, 0);

	tx_antd = 0;
	rx_ant_d = 0;

	// switch (panadr_own.short_addr)
	// {
	// 	case 5923:
	// 		tx_antd = 15918;
	// 		rx_ant_d = 15918;
	// 		break;
	// 	case 7090:
	// 		tx_antd = 15918;
	// 		rx_ant_d = 15918;
	// 		break;
	// 	case 1955:
	// 		tx_antd = 0;
	// 		rx_ant_d = 0;
	// 		break;
	// }
	dw_write(DW_REG_INFO.LDE_CTRL, (uint8_t*)(&rx_ant_d), DW_SUBREG_INFO.LDE_RXANTD.size, DW_SUBREG_INFO.LDE_RXANTD.offset);
	dw_write(DW_REG_INFO.TX_ANTD, (uint8_t*)(&tx_antd), DW_REG_INFO.TX_ANTD.size, 0);
}

void read_frame(void)
{
	dw_read(DW_REG_INFO.RX_FINFO, rx_finfo.reg, DW_REG_INFO.RX_FINFO.size, 0);

	dw1000_resp.recvd_size = dw_get_recv_size(rx_finfo);
	dw1000_resp.rx_time = dw_get_rx_time();
	dw_read(DW_REG_INFO.RX_BUFFER, dw1000_resp.recv_buf, dw1000_resp.recvd_size, 0);
}

uint16_t dw_get_addr(void)
{
	return panadr_own.short_addr;
}

uint16_t dw_get_panid(void)
{
	return panadr_own.pan_id;
}

void CPLOCK_handler(void)
{
	return;
}

void ESYNCR_handler(void)
{
	return;
}

void AAT_handler(void)
{
	return;
}

void TXFRB_handler(void)
{
	return;
}


void TXPRS_handler(void)
{
	return;
}


void TXPHS_handler(void)
{
	return;
}


void TXFRS_handler(void)
{
	dw1000_resp.tx_time = dw_get_tx_time();
	toggle_led(blue);
	chEvtSignal(dw_thread, MTXFRS_E);
}

void RXPRD_handler(void)
{
	return;
}


void RXFSDD_handler(void)
{
	return;
}


void LDEDONE_handler(void)
{
	return;
}


void RXPHD_handler(void)
{
	return;
}


void RXPHE_handler(void)
{
	dw_soft_reset_rx();
	chEvtSignal(dw_thread, MRXPHE_E);
}


void RXDFR_handler(void)
{
	return;
}


void RXFCG_handler(void)
{
	read_frame();
	toggle_led(green);
	chEvtSignal(dw_thread, MRXFCG_E);
}

void RXFCE_handler(void)
{
	dw_soft_reset_rx();
	chEvtSignal(dw_thread, MRXFCE_E);
}


void RXRFSL_handler(void)
{
	dw_soft_reset_rx();
	chEvtSignal(dw_thread, MRXRFSL_E);;
}


void RXRFTO_handler(void)
{
	return;
}


void LDEERR_handler(void)
{
	return;
}


void RXOVRR_handler(void)
{
	return;
}


void RXPTO_handler(void)
{
	return;
}


void GPIOIRQ_handler(void)
{
	return;
}


void SLP2INIT_handler(void)
{
	return;
}


void RFPLLLL_handler(void)
{
	return;
}


void CPLLLL_handler(void)
{
	return;
}


void RXSFDTO_handler(void)
{
	return;
}


void HPDWARN_handler(void)
{
	return;
}


void TXBERR_handler(void)
{
	return;
}


void AFFREJ_handler(void)
{
	return;
}

dw_ctrl_req_t calculate_distance(void)
{
	twr_header_t twr_header;
	uint64_t tx_time = dw_get_tx_time();
	uint64_t rx_time = dw1000_resp.rx_time;
	int64_t rt_init, rt_resp;

	// Get time from response
	memcpy(&twr_header, dw1000_resp.recv_buf+sizeof(MHR_16_t), sizeof(twr_header));

	rt_init = (rx_time) - (tx_time);
	rt_resp = (twr_header.tx_time) - (twr_header.rx_time);

	float clock_offset_r = dw_get_car_int() * CAR_INT_CTE;
	rt_resp *= (1.0f - clock_offset_r);

	double tof = (rt_init - rt_resp)/2.0f;
	tof *= (1.0f/(float)DW_TIME_U);

	float distance = (float)tof * C_ATM;

	if (distance > -0.1 && distance < 300)
	{
		memcpy(dw1000_resp.recv_buf+sizeof(MHR_16_t), &distance, sizeof(distance));
		dw1000_resp.recvd_size = sizeof(MHR_16_t) + sizeof(distance);
		return DW_CTRL_YIELD;
	}

	return DW_RESET;
}

uint64_t build_twr_resp(uint8_t* buffer, MHR_16_t header)
{
	twr_header_t twr_header;
	uint64_t delay_tx = dw1000_resp.rx_time + (uint64_t)(65536*2800);
	uint64_t tx_time = (uint64_t)delay_tx +(uint64_t)tx_antd;

	header.dest_addr = header.src_addr;
	header.src_addr = dw_get_addr();

	twr_header.m_type = MT_TWR_RESP;
	twr_header.tx_time = tx_time;
	twr_header.rx_time = dw1000_resp.rx_time;

	memcpy(buffer, &header, sizeof(header));
	memcpy(buffer + sizeof(header), &twr_header, sizeof(twr_header));

	return delay_tx;
}

twr_state_t respond_twr(twr_state_t twr_state)
{
	twr_header_t twr_header;
	MHR_16_t header;
	uint64_t delay_tx;
	uint8_t resp_message_size = sizeof(MHR_16_t) + sizeof(twr_header) + 1;
	uint8_t resp_message[resp_message_size];
	dx_time_t dx_time;
	ack_resp_t_t w4r;
	tx_fctrl_t tx_ctrl;
	eventmask_t evt = 0;

	if (twr_state == TWR_SEND_INIT)
		return TWR_RESP_RECVD;

	header = decode_MHR(dw1000_resp.recv_buf);
	memcpy(&twr_header, dw1000_resp.recv_buf+sizeof(MHR_16_t), sizeof(twr_header));

	if (twr_header.m_type != MT_TWR_INIT)
		return TWR_FAIL;
	
	memset(tx_ctrl.reg, 0, sizeof(tx_ctrl.reg));
	memset(dx_time.reg, 0, sizeof(dx_time.reg));

	tx_ctrl.TXBR = BR_6_8MBPS;
	tx_ctrl.TXPRF = PRF_16MHZ;
	tx_ctrl.TXPL = PL_128;
	tx_ctrl.TFLEN = resp_message_size+2; // TODO magic
	tx_ctrl.TR = 0b1;
	w4r.mask = 0;

	delay_tx = build_twr_resp(resp_message, header);

	dx_time.time32 = (uint32_t)(delay_tx >> 8);
	dw_start_tx(tx_ctrl, resp_message, dx_time, w4r);
	evt = chEvtWaitOneTimeout(MTXFRS_E, TX_TIMEOUT);
	if (evt != MTXFRS_E)
		return TWR_FAIL;
	return TWR_SEND_RESP;
}

dw_ctrl_req_t handle_sstwr(twr_state_t state)
{
	switch (state)
	{
		case TWR_NO_TWR:
			break;
		case TWR_SEND_INIT:
			return DW_SEND_W4R;
			break;
		case TWR_SEND_RESP:
			return DW_CTRL_YIELD;
			break;
		case TWR_RESP_RECVD:
			return calculate_distance();
			break;
		case TWR_FAIL:
			return DW_RESET;
			break;
	}
}

THD_FUNCTION(DW_CONTROLLER, arg)
{
	(void)arg;

	dw_setup();

	dx_time_t dx_time;
	tx_fctrl_t tx_ctrl;
	ack_resp_t_t w4r;
	eventmask_t evt = 0;
	memset(tx_ctrl.reg, 0, sizeof(tx_ctrl.reg));
	tx_ctrl.TXBR = BR_6_8MBPS;
	tx_ctrl.TXPRF = PRF_16MHZ;
	tx_ctrl.TXPL = PL_128;

	dw_ctrl_req_t current_state;
	dw_ctrl_req_t dw_ctrl_req = DW_CTRL_YIELD;
	dw_ctrl_req_t last_state = DW_RESET;
	uint8_t err_cnt = 0;
	uint8_t rst_cnt = 0;

	twr_state_t twr_state = TWR_NO_TWR;

	dw1000_cmd_t* dw1000_cmd = NULL;

	chMBObjectInit(&dw_controller, &dw_controller_msg, 1);
	chMBObjectInit(&dw_controller_resp, &dw_controller_resp_msg, 1);

	dw_thread = chThdGetSelfX();

	set_irq_vector();


	// TODO This sleep makes LDE work, may not enough time to start lde on setup
	chThdSleepMilliseconds(100);

	while (true)
	{
		if (dw1000_cmd != NULL)
			tx_ctrl.TFLEN = dw1000_cmd->size+2; // TODO magic

		w4r.mask = 0;
		memset(dx_time.reg, 0, sizeof(dx_time.reg));

		current_state = dw_ctrl_req;

		switch (dw_ctrl_req)
		{
			case DW_RECV:
				dw_start_rx(dx_time);
				evt = chEvtWaitOneTimeout(MRXFCG_E | MRXERR_E, dw1000_cmd->tmo);
				if (evt == MRXFCG_E)
				{
					dw_ctrl_req = DW_CTRL_YIELD;
					dw1000_resp.state = DW_RECV_OK;
					if (rx_finfo.RNG)
					{
						twr_state = respond_twr(twr_state);
						dw_ctrl_req = handle_sstwr(twr_state);
					}
				}
				else if (evt == 0)
				{
					dw_ctrl_req = DW_RECV_TMO;
					dw1000_resp.state = DW_RECV_TMOUT;
				}
				else
				{
					err_cnt++;
					dw_soft_reset_rx();
				}
				break;
			case DW_SEND:
				dw_start_tx(tx_ctrl, dw1000_cmd->send_buf, dx_time, w4r);
				evt = chEvtWaitOneTimeout(MTXFRS_E, TX_TIMEOUT);
				if (evt == MTXFRS_E)
				{
					dw1000_resp.state = DW_SEND_OK;
					dw_ctrl_req = DW_CTRL_YIELD;
				}
				else
					dw_ctrl_req = DW_TRX_ERR;
				break;
			case DW_SEND_W4R:
				w4r.W4R_TIM = dw1000_cmd->wait;
				w4r.ACK_TIM = 1;
				dw_start_tx(tx_ctrl, dw1000_cmd->send_buf, dx_time, w4r);
				evt = chEvtWaitOneTimeout(MTXFRS_E, TX_TIMEOUT);
				if (evt != MTXFRS_E)
					dw_ctrl_req = DW_TRX_ERR;
				else
				{
					evt = chEvtWaitOneTimeout(MRXFCG_E | MRXERR_E, dw1000_cmd->tmo);
					if (evt == MRXFCG_E)
					{
						dw1000_resp.state = DW_SEND_W4R_OK;
						dw_ctrl_req = DW_CTRL_YIELD;
						if (rx_finfo.RNG)
						{
							twr_state = respond_twr(twr_state);
							dw_ctrl_req = handle_sstwr(twr_state);
						}
					}
					else if (evt == 0)
					{
						dw1000_resp.state = DW_SEND_W4R_TMO;
						dw_ctrl_req = DW_RECV_TMO;
					}
					else
						dw_ctrl_req = DW_TRX_ERR;
					dw_soft_reset_rx();
				}
				break;
			case DW_SEND_DLY:
				dx_time.time32 = dw1000_cmd->dly;
				dw_start_tx(tx_ctrl, dw1000_cmd->send_buf, dx_time, w4r);
				evt = chEvtWaitOneTimeout(MTXFRS_E, TX_TIMEOUT);
				if (evt == MTXFRS_E)
				{
					dw1000_resp.state = DW_SEND_DLY_OK;
					dw_ctrl_req = DW_CTRL_YIELD;
				}
				else
					dw_ctrl_req = DW_TRX_ERR;
				break;
			case DW_TRX_ERR:
				dw_soft_reset_rx();
				chThdSleepMilliseconds(1);
				dw_transceiver_off();
				err_cnt++;
				dw_ctrl_req = last_state;
				break;
			case DW_CTRL_YIELD:
				if (last_state != DW_RESET)
					chMBPostAheadTimeout(&dw_controller_resp, (msg_t)&dw1000_resp, TIME_INFINITE);
					
				dw_soft_reset_rx();
				dw_transceiver_off();

				if (chMBFetchTimeout(&dw_controller, (msg_t*)&dw1000_cmd, TIME_INFINITE) == MSG_OK)
				{
					dw_ctrl_req = dw1000_cmd->dw_ctrl_req;

					tx_ctrl.TR = 0b0;
				}
				break;
			case DW_RECV_TMO:
				dw1000_resp.recvd_size = 0;
				dw1000_resp.rx_time = 0;
				dw1000_resp.tx_time = 0;
				memset(dw1000_resp.recv_buf, 0, sizeof(dw1000_resp.recv_buf));
				dw_ctrl_req = DW_CTRL_YIELD;
				break;
			case DW_SSTWR:
				twr_state = TWR_SEND_INIT;
				tx_ctrl.TR = 0b1;
				dw_ctrl_req = handle_sstwr(twr_state);
				break;
			case DW_DSTWR:
				dw_ctrl_req = DW_CTRL_YIELD;
				break;
			case DW_3DSTWR:
				dw_ctrl_req = DW_CTRL_YIELD;
				break;
			case DW_RESET:
				dw_setup();
				chThdSleepMilliseconds(100);
				rst_cnt++;
				dw_ctrl_req = DW_CTRL_YIELD;
				dw1000_resp.state = DW_SYS_ERR;
				dw1000_resp.recvd_size = 0;
				dw1000_resp.rx_time = 0;
				dw1000_resp.tx_time = 0;
				memset(dw1000_resp.recv_buf, 0, sizeof(dw1000_resp.recv_buf));
				chMBPostAheadTimeout(&dw_controller_resp, (msg_t)&dw1000_resp, TIME_INFINITE);
				break;
			default:
				break;
		}
		
		last_state = current_state;

		if (err_cnt > DW_ERR_THRESH)
		{
			dw_ctrl_req = DW_RESET;
			err_cnt = 0;
		}
	}
}