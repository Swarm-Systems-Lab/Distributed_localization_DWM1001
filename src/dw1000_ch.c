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
	//irq_mask.MLDEERR = 0b1;
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

	switch (panadr_own.short_addr)
	{
		case 5923:
			tx_antd = 15918;
			rx_ant_d = 15918;
			break;
		case 7090:
			tx_antd = 15918;
			rx_ant_d = 15918;
			break;
		case 1955:
			tx_antd = 14535;
			rx_ant_d = 18500;
			break;
	}
	dw_write(DW_REG_INFO.LDE_CTRL, (uint8_t*)(&rx_ant_d), DW_SUBREG_INFO.LDE_RXANTD.size, DW_SUBREG_INFO.LDE_RXANTD.offset);
	dw_write(DW_REG_INFO.TX_ANTD, (uint8_t*)(&tx_antd), DW_REG_INFO.TX_ANTD.size, 0);
}

void read_frame(void)
{
	dw1000_resp.recvd_size = dw_get_recv_size();
	dw1000_resp.rx_time = dw_get_rx_time();
	dw_read(DW_REG_INFO.RX_BUFFER, dw1000_resp.recv_buf, dw1000_resp.recvd_size, 0);
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

// int8_t respond_if_twr(void)
// {
// 	recvd_type = recv_buf[sizeof(recvd_header)];
// 	if (recvd_type != MT_D_INIT || twr_state != TWR_REQ_RECVD)
// 		return 0;

// 	dx_time_t dx_time;
// 	ack_resp_t_t w4r;
// 	tx_fctrl_t tx_ctrl;
// 	eventmask_t evt = 0;
// 	memset(tx_ctrl.reg, 0, sizeof(tx_ctrl.reg));
// 	memset(send_buf, 0, sizeof(send_buf));
// 	tx_ctrl.TXBR = BR_6_8MBPS;
// 	tx_ctrl.TXPRF = PRF_16MHZ;
// 	tx_ctrl.TXPL = PL_128;
// 	w4r.mask = 0;
// 	memset(dx_time.reg, 0, sizeof(dx_time.reg));

// 	uint8_t lde_stat = 0xd;

// 	recvd_header = decode_MHR(recv_buf);

// 	uint64_t rx_time = 0;
// 	memcpy(&rx_time, recv_info.dw_rx_time.RX_STAMP, sizeof(recv_info.dw_rx_time.RX_STAMP));
// 	if (!rx_time)
// 	{
// 		memcpy(&rx_time, recv_info.dw_rx_time.RX_RAWST, sizeof(recv_info.dw_rx_time.RX_RAWST));
// 		rx_time -= rx_ant_d;
// 		memcpy(send_buf+sizeof(rx_time)+sizeof(rx_time), &lde_stat, sizeof(lde_stat));
// 	}
// 	uint64_t delay_tx = (uint64_t)rx_time + (uint64_t)(65536*4000);
// 	uint64_t tx_time = (uint64_t)delay_tx +(uint64_t)tx_antd;

// 	memcpy(send_buf, &tx_time, sizeof(tx_time));
// 	memcpy(send_buf+sizeof(tx_time), &rx_time, sizeof(rx_time));

// 	send_msg_meta.size = sizeof(tx_time)+sizeof(rx_time)+sizeof(lde_stat);
// 	send_msg_meta.seq_ack_num = 0;
// 	send_msg_meta.type = MT_D_RESP;
// 	send_msg_meta.addr = recvd_header.src_addr;

// 	prepare_message();
// 	tx_ctrl.TFLEN = send_size+2; // TODO magic

// 	dx_time.time32 = (uint32_t)(delay_tx >> 8);
// 	dw_start_tx(tx_ctrl, send_buf, dx_time, w4r);
// 	evt = chEvtWaitOneTimeout(MTXFRS_E, TX_TIMEOUT);
// 	if (evt != MTXFRS_E)
// 	{
// 		twr_state = TWR_FAIL;
// 		return -1;
// 	}
// 	return 1;
		
// }

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
	dw_ctrl_req_t dw_ctrl_req;
	dw_ctrl_req_t last_state = DW_RESET;
	uint8_t err_cnt = 0;
	uint8_t rst_cnt = 0;
	int8_t twr_ret;

	dw1000_cmd_t* dw1000_cmd;

	chMBObjectInit(&dw_controller, &dw_controller_msg, 1);
	chMBObjectInit(&dw_controller_resp, &dw_controller_resp_msg, 1);

	dw_thread = chThdGetSelfX();

	set_irq_vector();

	//barrier();

	while (true)
	{
		tx_ctrl.TFLEN = dw1000_cmd->size+2; // TODO magic
		w4r.mask = 0;
		memset(dx_time.reg, 0, sizeof(dx_time.reg));

		current_state = dw_ctrl_req;

		switch (dw_ctrl_req)
		{
			case DW_RECV:
				dw_start_rx(dx_time);
				evt = chEvtWaitOneTimeout(MRXFCG_E | MRXERR_E, TIME_US2I(dw1000_cmd->tmo));
				if (evt == MRXFCG_E)
				{
					dw_ctrl_req = DW_CTRL_YIELD;
					dw1000_resp.state = DW_RECV_OK;
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
					evt = chEvtWaitOneTimeout(MRXFCG_E | MRXERR_E, TIME_US2I(dw1000_cmd->tmo));
					if (evt == MRXFCG_E)
					{
						dw1000_resp.state = DW_SEND_W4R_OK;
						// twr_ret = respond_if_twr();
						// if (twr_ret < 0)
						// 	dw_ctrl_req = DW_TRX_ERR;
						dw_ctrl_req = DW_CTRL_YIELD;
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
				chThdSleepMilliseconds(3);
				dw_ctrl_req = last_state;
				break;
			case DW_CTRL_YIELD:
				if (last_state != DW_RESET)
					chMBPostAheadTimeout(&dw_controller_resp, (msg_t)&dw1000_resp, TIME_INFINITE);

				if (chMBFetchTimeout(&dw_controller, (msg_t*)&dw1000_cmd, TIME_INFINITE) == MSG_OK)
				{
					dw_ctrl_req = dw1000_cmd->dw_ctrl_req;

					dw_soft_reset_rx();
					chThdSleepMilliseconds(1);
					dw_transceiver_off();
					chThdSleepMilliseconds(3);
					// chThdSleepMilliseconds(TRX_RST_TM);
					// state = dw_transceiver_off();
				}
				break;
			case DW_RECV_TMO:
				dw1000_resp.recvd_size = 0;
				dw1000_resp.rx_time = 0;
				dw1000_resp.tx_time = 0;
				memset(dw1000_resp.recv_buf, 0, sizeof(dw1000_resp.recv_buf));
				dw_ctrl_req = DW_CTRL_YIELD;
				break;
			case DW_RESET:
				dw_setup();
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