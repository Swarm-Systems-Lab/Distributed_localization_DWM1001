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

	// MHR.frame_control.frame_type = FT_DATA;
	// MHR.frame_control.sec_en = 0b0;
	// MHR.frame_control.frame_pending = 0b0;
	// MHR.frame_control.ack_req = 0b0;
	// MHR.frame_control.pan_id_compress = 0b1;
	// MHR.frame_control.dest_addr_mode = SHORT_16;
	// MHR.frame_control.frame_version = 0x1;
	// MHR.frame_control.src_addr_mode = SHORT_16;
frame_control_t def_frame_ctrl = {.mask=0x9841};

panadr_t panadr_own;
uint8_t recv_buf[128];
uint8_t send_buf[128];

address_list_t neighbours;
double distances_neigh[NEIGHBOUR_NUM];

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

int32_t _send_message(uint16_t addr, uint8_t* message, size_t size, uint8_t w4r_on, uint32_t w_time, uint32_t dly_time)
{
	uint8_t frame[size+9];
	// Todo remove 9 magic and panadr should be global variable set at config
	panadr_t panadr;
	tx_fctrl_t tx_ctrl;
	dx_time_t dx_time;
	ack_resp_t_t w4r;
	eventmask_t evt = 0;
	memset(tx_ctrl.reg, 0, sizeof(tx_ctrl.reg));
	memset(dx_time.reg, 0, sizeof(dx_time.reg));
	w4r.mask = 0;
	if (w4r_on)
	{
		w4r.mask = w_time; // TODO check if bitfield works
		w4r.ACK_TIM = 1;
	}

	// TODO time should be the delay??
	dx_time.time32 = dly_time;

	dw_read(DW_REG_INFO.PAN_ADR, panadr.reg, DW_REG_INFO.PAN_ADR.size, 0);
	encode_MHR(def_frame_ctrl, frame, 0x0, panadr.pan_id, addr, panadr.short_addr);
	memcpy(frame+9, message, size);
	tx_ctrl.TFLEN = sizeof(frame)+2;
	tx_ctrl.TXBR = BR_6_8MBPS;
	tx_ctrl.TXPRF = PRF_16MHZ;
	tx_ctrl.TXPL = PL_128;

	dw_start_tx(tx_ctrl, frame, dx_time, w4r);
	evt = chEvtWaitOneTimeout(MTXFRS_E, TIME_MS2I(8));
	if (!evt)
		return -1;
	
	return evt;
}

int32_t send_message(uint16_t addr, uint8_t* message, size_t size)
{	
	return _send_message(addr, message, size, 0, 0, 0);
}

int32_t send_message_w4r(uint16_t addr, uint8_t* message, size_t size, uint32_t time, uint16_t* recv_addr, size_t* recv_size)
{
	eventmask_t evt = 0;
	int32_t code = _send_message(addr, message, size, 1, time, 0);
	//TODO check code
	evt = chEvtWaitOneTimeout(MRXPHE_E | MRXFCE_E | MLDEERR_E | MRXFCG_E, TIME_US2I(60000));
	if (evt == MRXFCG_E)
		get_message(recv_addr, recv_size);
	else if (!evt)
		return 0;
	dw_soft_reset_rx();
	return evt;
}

int32_t send_message_delay(uint16_t addr, uint8_t* message, size_t size, uint32_t time)
{
	return _send_message(addr, message, size, 0, 0, time);
}

void get_message(uint16_t* addr, size_t* size)
{
	rx_finfo_t rx_finfo;
	rx_finfo.mask = 0;
	MHR_16_t MHR;

	dw_read(DW_REG_INFO.RX_FINFO, rx_finfo.reg, DW_REG_INFO.RX_FINFO.size, 0);
	dw_read(DW_REG_INFO.RX_BUFFER, recv_buf, rx_finfo.RXFLEN, 0);
	MHR = decode_MHR(recv_buf);
	*addr = MHR.src_addr;
	*size = rx_finfo.RXFLEN-9;
	memmove(recv_buf, recv_buf+9, *size);
}

int32_t recv_message(uint16_t* addr, size_t* size, uint32_t timeout)
{
	dx_time_t dx_time;
	eventmask_t evt = 0;

	memset(dx_time.reg, 0, sizeof(dx_time.reg));
	*size = 0;

	dw_start_rx(dx_time);
	evt = chEvtWaitOneTimeout(MRXPHE_E | MRXFCE_E | MLDEERR_E | MRXFCG_E, TIME_US2I(timeout));
	if (evt == MRXFCG_E)
		get_message(addr, size);
	else if (!evt)
		return 0;
	dw_soft_reset_rx();
	return evt;
}

void init_neigh(void)
{
	neighbours.last_p = 0;
	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
	{
		neighbours.addrs[i] = 0;
	}
}

int8_t search_addr(uint16_t addr)
{
	if (addr == 0 || addr == panadr_own.short_addr)
		return -2;
	
	if (neighbours.last_p > 0)
	{
		for (size_t i = 0; i < neighbours.last_p; i++)
		{
			if (addr == neighbours.addrs[i])
				return i;
		}
	}

	return -1;
}

int8_t insert_addr(uint16_t addr)
{
	if (neighbours.last_p >= NEIGHBOUR_NUM)
		return -2;

	if (search_addr(addr) == -1)
	{
		neighbours.addrs[neighbours.last_p] = addr;
		neighbours.last_p++;
		return neighbours.last_p-1;
	}
	else
		return -1;
}

int32_t recv_disc(void)
{
	size_t size = 0;
	uint32_t rand_wait = (rand() & 0xFFFF) + 4000;
	uint16_t addr;
	int32_t suc = recv_message(&addr, &size, rand_wait);
	//uint8_t connected = search_conn(src_addr);
	uint8_t connected = 1;
	
	if (suc == 0)
		return 0;

	if (suc < 0 || size <= 0)
		return -1;
	
	if (suc == MRXFCG_E)
	{
		if (recv_buf[0] == MT_BROADCAST)
		{
			//toggle_led(green);
			int8_t suc;
			suc = insert_addr(addr);
			if (suc == -2)
				return 4; // list is full
			for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
			{
				suc = insert_addr(((uint16_t*)(recv_buf+1))[i]);
				if (suc == -1)
					return 1; // no new address
			}
			return 1;
		}
		else
			return 2;

		if (recv_buf[0] == 2)
			return 2;
		
		if (connected)
			// Normal exchange
			return 3;
	}
	return suc;
}

double loc_init_fun(uint16_t addr)
{
	sys_state_t state;
	uint64_t rx_time = 0;
	uint64_t tx_time = 0;
	uint64_t m_rx_time = 0;
	uint64_t m_tx_time = 0;
	int64_t rt_init = 0;
	int64_t rt_resp = 0;
	eventmask_t evt = 0;
	uint16_t recv_addr;
	size_t recv_size;

	uint8_t data[2] = {MT_LOC_REQ,0};
	double tof = 0.0;
	float clock_offset_r = 0.0;

	evt = send_message_w4r(addr, data, sizeof(data), 0, &recv_addr, &recv_size);
	if (evt == MRXFCG_E && recv_buf[0] == MT_LOC_RESP)
	{
		toggle_led(green);
		tx_time = dw_get_tx_time();
		rx_time = dw_get_rx_time();
		memcpy(&m_tx_time, recv_buf+1, 5);
		memcpy(&m_rx_time, recv_buf+9, 5);

		rt_init = (rx_time) - (tx_time);
		rt_resp = (m_tx_time) - (m_rx_time);

		clock_offset_r = dw_get_car_int() * ((998.4e6/2.0/1024.0/131072.0) * (-1.0e6/6489.6e6) / 1.0e6);
		rt_resp *= (1.0f - clock_offset_r);

		tof = (rt_init - rt_resp)/2.0f;
		tof = tof * (1.0f/(float)(499.2e6*128.0));

		// chprintf((BaseSequentialStream*)&SD1, "Distance: %dcm\n", (int)distance);
	}
	else
		dw_soft_reset_rx();
	evt = 0;
	state = dw_transceiver_off();
	chThdSleepMilliseconds(40);
	return tof;
}

void loc_resp_fun()
{
	sys_state_t state;
	uint32_t d_time;
	uint64_t rx_time = 0;
	uint64_t tx_time = 0;
	uint64_t delay_tx = 0;
	eventmask_t evt = 0;
	tx_antd_t tx_ant_d;
	uint16_t src_addr;
	size_t size_recv;

	dw_read(DW_REG_INFO.TX_ANTD, (uint8_t*)(&tx_ant_d), DW_REG_INFO.TX_ANTD.size, 0);

	uint8_t data[17] = {MT_LOC_RESP,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	int32_t suc = recv_message(&src_addr, &size_recv, 51000);
	if (suc == MRXFCG_E && recv_buf[0] == MT_LOC_REQ)
	{
		toggle_led(green);
		rx_time = dw_get_rx_time();
		delay_tx = (uint64_t)rx_time + (uint64_t)(65536*3000);
		tx_time = (uint64_t)delay_tx + (uint64_t)tx_ant_d;
		d_time = (uint32_t)(delay_tx >> 8);
		memcpy(data+1, &tx_time, 8);
		memcpy(data+9, &rx_time, 8);
		send_message_delay(src_addr, data, sizeof(data), d_time);
	}
	evt = 0;
	state = dw_transceiver_off();
	chThdSleepMilliseconds(41);
}

void loc_disc_fun(void)
{
	init_neigh();
	disc_state_t disc_state = DISC_INIT;
	uint8_t disc_to_cnt = 0;
	uint8_t message[7] = {MT_BROADCAST,0,0,0,0,0,0};
	uint8_t recv_b[10];
	uint16_t src_addr;
	size_t size;
	uint8_t ttl_rx = 0;
	sys_state_t state;
	int32_t suc;

	while (disc_state != DISC_3WH)
	{
		memcpy(message+1, neighbours.addrs, sizeof(neighbours.addrs));
		switch (disc_state)
		{
			case DISC_INIT:
				disc_to_cnt = 0;
				disc_state = DISC_WAIT_RX;
				break;
			case DISC_BROAD:
				message[0] = 1;
				suc = send_message(0xFFFF, message, sizeof(message));
				if (suc < 0)
					disc_state = DISC_TX_ERR;
				else 
					disc_state = DISC_WAIT_RX;
				break;
			case DISC_WAIT_RX:
				suc = recv_disc();
				switch (suc)
				{
					case 0:
						disc_to_cnt++;
						disc_state = DISC_BROAD;
						state = dw_transceiver_off();
						chThdSleepMilliseconds(10);
						break;
					case 1:
						disc_state = DISC_WAIT_RX;
						break;
					case 2:
						disc_state = DISC_3WH;
						break;
					case 4:
						disc_state = DISC_3WH; // Will be DISC_ST_CONN
						toggle_led(blue);
						break;
					default:
						disc_state = DISC_TX_ERR;
						break;
				}
				break;
			case DISC_3WH:
				break;
			case DISC_IDLE:
				break;
			case DISC_TX_ERR:
				state = dw_transceiver_off();
				chThdSleepMilliseconds(10);
				disc_state = DISC_INIT;
				break;
			case DISC_RX_ERR:
				state = dw_transceiver_off();
				chThdSleepMilliseconds(10);
				disc_state = DISC_INIT;
				break;
		}

		if (disc_to_cnt > 8)
		{
			disc_state = DISC_3WH;
		}
	}
}

void get_distance_to(uint16_t addr)
{
	uint8_t pos = search_addr(addr);
	if (pos >= 0 && pos < NEIGHBOUR_NUM)
	{
		double tof = loc_init_fun(addr);
		if (tof > 0.0 && tof < 1e-5)
		{
			double distance = tof * 299702547;
			distance *= 100;
			distances_neigh[pos] = distances_neigh[pos]*NEIGHBOUR_NUM/(NEIGHBOUR_NUM+1) + distance/(NEIGHBOUR_NUM+1);
		}
	}
}

// {
// 	uint32_t dirty_regs;

// 	uint8_t recv_buf[128];
// 	uint8_t send_buf[128];
// }

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
	//irq_mask.MLDEERR = 0b1;
	irq_mask.MAFFREJ = 0b1;
	
	sdStart(&SD1, &serial_cfg);

	uint64_t id = get_hardware_id();
	srand(id);
	dw_write(DW_REG_INFO.PAN_ADR, (uint8_t*)(&id), 2, 0);
	dw_read(DW_REG_INFO.PAN_ADR, panadr_own.reg, DW_REG_INFO.PAN_ADR.size, 0);
	uint16_t tx_ant_d = 63520;
	uint16_t rx_ant_d = 63520;
	eventmask_t evt = 0;
	
	dw_set_irq(irq_mask);

	dw_config_t t;
	int sizeconfig = sizeof(t);
	chprintf((BaseSequentialStream*)&SD1, "size: %d\n", sizeconfig);


	dw_write(DW_REG_INFO.LDE_CTRL, (uint8_t*)(&rx_ant_d), DW_SUBREG_INFO.LDE_RXANTD.size, DW_SUBREG_INFO.LDE_RXANTD.offset);
	dw_write(DW_REG_INFO.TX_ANTD, (uint8_t*)(&tx_ant_d), DW_REG_INFO.TX_ANTD.size, 0);

	while (true) {	
		//if (neighbours.addrs[0] == 0)
		loc_disc_fun();
		// if (panadr_own.short_addr < neighbours.addrs[0])
		// 	get_distance_to(neighbours.addrs[0]);
		// else
		// {
		// 	chprintf((BaseSequentialStream*)&SD1, "AAAAAAAAAAAAAAAA\n");
		// 	loc_resp_fun();
		// }
		for (int i = 0; i < NEIGHBOUR_NUM; i++)
			chprintf((BaseSequentialStream*)&SD1, "neigh: %d\n", neighbours.addrs[i]);
		
		//chprintf((BaseSequentialStream*)&SD1, "Distance: %dcm\n", (int)distances_neigh[0]);
	}
}

THD_FUNCTION(SYSTEM_STATUS, arg)
{
	// loc_state_t loc_state = LOC_DISC;

	// uint16_t addrs[3] = {0,0,0};

	// dw_read(DW_REG_INFO.PAN_ADR, addrs, 2, 0);

	loc_disc_fun();
	get_distance_to(neighbours.addrs[0]);
	get_distance_to(neighbours.addrs[1]);
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