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

semaphore_t barrier_sem;
mutex_t barrier_mutex;
uint8_t barrier_cnt = 0;
uint8_t barrier_num;

mutex_t dw_mutex;

thread_t* dw_thread;
thread_t* peer_conn_thread;
thread_t* peer_disc_thread;
thread_t* comm_thread;

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

peer_connection_t broad_peer = {.peer_addr = 0xFFFF, .ttl = 0, .seq_num = 0, .ack_num = 0};

panadr_t panadr_own;
dw_rod_info_t recv_info;

uint8_t recv_size;
uint32_t recv_tmo_usec;

uint8_t send_size;
uint32_t send_wtime;
uint32_t send_dlytime;

uint8_t recv_buf[128];
uint8_t send_buf[128];

address_list_t neighbours;
double distances_neigh[NEIGHBOUR_NUM];

loc_state_t loc_state = LOC_STANDBY;

dw_ctrl_req_t dw_ctrl_req = DW_CTRL_YIELD;

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

void barrier(void)
{
	uint8_t release = 1;
	chMtxLock(&barrier_mutex);
	barrier_cnt++;
	if (barrier_cnt == barrier_num)
	{
		for (uint8_t i = 0; i < barrier_num; i++)
			chSemSignal(&barrier_sem);
		barrier_cnt = 0;
		release = 0;
	}	
	chMtxUnlock(&barrier_mutex);
	if (release)
		chSemWait(&barrier_sem);
}

void barrier_init(uint8_t n)
{
	barrier_num = n;
	chMtxObjectInit(&barrier_mutex);
	chSemObjectInit(&barrier_sem, 0);
	barrier_cnt = 0;
}

peer_connection_t* search_peer(uint16_t addr)
{
	return &broad_peer;
}

void read_frame(void)
{
	_dw_spi_transaction(1, DW_REG_INFO.RX_FINFO.id, recv_info.dw_rx_finfo.reg, DW_REG_INFO.RX_FINFO.size, 0);
	_dw_spi_transaction(1, DW_REG_INFO.RX_TIME.id, recv_info.dw_rx_time.reg, DW_REG_INFO.RX_TIME.size, 0);
	_dw_spi_transaction(1, DW_REG_INFO.RX_BUFFER.id, recv_buf, recv_info.dw_rx_finfo.RXFLEN, 0);
	recv_size = recv_info.dw_rx_finfo.RXFLEN; // TODO 2 magic number FCS
}

int32_t get_message(message_meta_t* msg_meta)
{
	if (recv_size == 0)
		return 1;		// RECV timeout

	MHR_16_t recv_header = decode_MHR(recv_buf);
	msg_meta->peer = search_peer(recv_header.src_addr);
	msg_meta->seq_num = recv_header.seq_num;

	msg_meta->type = recv_buf[sizeof(recv_header)];
	msg_meta->size = recv_size - (sizeof(recv_header)+1);

	memmove(recv_buf, recv_buf+sizeof(recv_header)+1, sizeof(recv_buf)-sizeof(recv_header)-1);

	if (msg_meta->type == msg_meta->expected_message && msg_meta->peer == msg_meta->expected_peer)
	{
		if (recv_header.dest_addr == 0xFFFF || recv_header.dest_addr == panadr_own.short_addr)
			return 1;
	}
	return 0;
}

void prepare_message(message_meta_t* msg_meta)
{
	MHR_16_t send_header;
	send_header.frame_control = def_frame_ctrl;
	send_header.dest_addr = msg_meta->peer->peer_addr;
	send_header.src_addr = panadr_own.short_addr;
	send_header.dest_pan_id = panadr_own.pan_id;
	send_header.seq_num = msg_meta->peer->seq_num;
	memmove(send_buf+sizeof(send_header)+1, send_buf, sizeof(send_buf)-sizeof(send_header)-1);
	memcpy(send_buf, &send_header, sizeof(send_header));
	send_buf[sizeof(send_header)] = msg_meta->type;
	send_size = msg_meta->size + sizeof(send_header)+1;
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

// double loc_init_fun(uint16_t addr)
// {
// 	sys_state_t state;
// 	uint64_t rx_time = 0;
// 	uint64_t tx_time = 0;
// 	uint64_t m_rx_time = 0;
// 	uint64_t m_tx_time = 0;
// 	int64_t rt_init = 0;
// 	int64_t rt_resp = 0;
// 	eventmask_t evt = 0;
// 	uint16_t recv_addr;
// 	size_t recv_size2;

// 	uint8_t data[2] = {MT_LOC_REQ,0};
// 	double tof = 0.0;
// 	float clock_offset_r = 0.0;

// 	evt = send_message_w4r(addr, data, sizeof(data), 0, &recv_addr, &recv_size2);
// 	if (evt == MRXFCG_E && recv_buf[0] == MT_LOC_RESP)
// 	{
// 		toggle_led(green);
// 		tx_time = dw_get_tx_time();
// 		rx_time = dw_get_rx_time();
// 		memcpy(&m_tx_time, recv_buf+1, 5);
// 		memcpy(&m_rx_time, recv_buf+9, 5);

// 		rt_init = (rx_time) - (tx_time);
// 		rt_resp = (m_tx_time) - (m_rx_time);

// 		clock_offset_r = dw_get_car_int() * ((998.4e6/2.0/1024.0/131072.0) * (-1.0e6/6489.6e6) / 1.0e6);
// 		rt_resp *= (1.0f - clock_offset_r);

// 		tof = (rt_init - rt_resp)/2.0f;
// 		tof = tof * (1.0f/(float)(499.2e6*128.0));

// 		// chprintf((BaseSequentialStream*)&SD1, "Distance: %dcm\n", (int)distance);
// 	}
// 	else
// 		dw_soft_reset_rx();
// 	evt = 0;
// 	state = dw_transceiver_off();
// 	chThdSleepMilliseconds(40);
// 	return tof;
// }

// void loc_resp_fun()
// {
// 	sys_state_t state;
// 	uint32_t d_time;
// 	uint64_t rx_time = 0;
// 	uint64_t tx_time = 0;
// 	uint64_t delay_tx = 0;
// 	eventmask_t evt = 0;
// 	tx_antd_t tx_ant_d;
// 	uint16_t src_addr;
// 	size_t size_recv;

// 	dw_read(DW_REG_INFO.TX_ANTD, (uint8_t*)(&tx_ant_d), DW_REG_INFO.TX_ANTD.size, 0);

// 	uint8_t data[17] = {MT_LOC_RESP,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// 	int32_t suc = recv_message(&src_addr, &size_recv, 51000);
// 	if (suc == MRXFCG_E && recv_buf[0] == MT_LOC_REQ)
// 	{
// 		toggle_led(green);
// 		rx_time = dw_get_rx_time();
// 		delay_tx = (uint64_t)rx_time + (uint64_t)(65536*3000);
// 		tx_time = (uint64_t)delay_tx + (uint64_t)tx_ant_d;
// 		d_time = (uint32_t)(delay_tx >> 8);
// 		memcpy(data+1, &tx_time, 8);
// 		memcpy(data+9, &rx_time, 8);
// 		send_message_delay(src_addr, data, sizeof(data), d_time);
// 	}
// 	evt = 0;
// 	state = dw_transceiver_off();
// 	chThdSleepMilliseconds(41);
// }

THD_FUNCTION(PEER_DISCOVERY, arg)
{
	(void)arg;

	barrier_init(3);

	peer_disc_thread = chThdGetSelfX();
	init_neigh();
	disc_state_t disc_state = DISC_INIT;
	uint8_t disc_to_cnt = 0;
	message_meta_t send_broadcast = 
	{
		.message = COMM_SEND_CMD,
		.peer = &broad_peer,
		.seq_num = 0,
		.type = MT_BROADCAST,
		.expected_peer = NULL,
		.expected_message = MT_BROADCAST,
		.size = NEIGHBOUR_NUM*sizeof(uint16_t)
	};

	message_meta_t recv_broadcast = 
	{
		.message = COMM_RECV_CMD,
		.peer = &broad_peer,
		.seq_num = 0,
		.type = MT_BROADCAST,
		.expected_peer = &broad_peer,
		.expected_message = MT_BROADCAST,
		.size = NEIGHBOUR_NUM*sizeof(uint16_t)
	};

	uint32_t rand_wait;
	uint8_t ttl_rx = 0;
	sys_state_t state;
	int32_t msg;
	int8_t suc;

	barrier();

	while (disc_state != DISC_3WH)
	{
		rand_wait = (rand() & 0xFFFF) + 40000;
		memcpy(send_buf, neighbours.addrs, sizeof(neighbours.addrs));
		switch (disc_state)
		{
			case DISC_INIT:
				disc_to_cnt = 0;
				disc_state = DISC_WAIT_RX;
				break;
			case DISC_BROAD:
				send_wtime = 0;
				send_dlytime = 0;
				chMsgSend(comm_thread, (int32_t)&send_broadcast);
				chMsgWait();
				if (chMsgGet(comm_thread) == COMM_END)
					disc_state = DISC_WAIT_RX;
				else 
					disc_state = DISC_TX_ERR;

				chMsgRelease(comm_thread, MSG_OK);
				break;
			case DISC_WAIT_RX:
				recv_tmo_usec = rand_wait;
				chMsgSend(comm_thread, (int32_t)&recv_broadcast);
				chMsgWait();
				msg = chMsgGet(comm_thread);
				chMsgRelease(comm_thread, MSG_OK);
				if (msg == COMM_TMO_RESP)
				{
					disc_to_cnt++;
					disc_state = DISC_BROAD;
				}
				if (msg == COMM_END)
				{
					msg >>= 16;
					toggle_led(green);
					suc = insert_addr(msg);
					if (suc == -2)
						disc_state = DISC_INIT;
					for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
					{
						suc = insert_addr(((uint16_t*)(recv_buf+1))[i]);
						if (suc == -1)
							break; // no new address
					}
				}
				break;
			case DISC_3WH:
				break;
			case DISC_IDLE:
				break;
			case DISC_TX_ERR:
				disc_state = DISC_INIT;
				break;
			case DISC_RX_ERR:
				disc_state = DISC_INIT;
				break;
		}

		if (disc_to_cnt > 8)
		{
			disc_state = DISC_INIT;
		}
	}
}

// void get_distance_to(uint16_t addr)
// {
// 	uint8_t pos = search_addr(addr);
// 	if (pos >= 0 && pos < NEIGHBOUR_NUM)
// 	{
// 		double tof = loc_init_fun(addr);
// 		if (tof > 0.0 && tof < 1e-5)
// 		{
// 			double distance = tof * 299702547;
// 			distance *= 100;
// 			distances_neigh[pos] = distances_neigh[pos]*NEIGHBOUR_NUM/(NEIGHBOUR_NUM+1) + distance/(NEIGHBOUR_NUM+1);
// 		}
// 	}
// }

THD_FUNCTION(COMMS, arg)
{
	(void)arg;

	comm_thread = chThdGetSelfX();
	comm_state_t comm_state = COMM_IDLE;
	comm_state_t current_state = COMM_IDLE;
	comm_state_t last_state = COMM_IDLE;
	uint8_t wrong_message_cnt = 0;
	eventmask_t evt = 0;
	int8_t message_ret;
	uint8_t w4r_dly_flag = 0;
	message_meta_t* cmd_msg = 0;

	thread_t* caller_thread = NULL;

	barrier();

	while (true)
	{
		current_state = comm_state;
		switch (comm_state)
		{
			case COMM_RECV:
				chMtxLock(&dw_mutex);
				dw_ctrl_req = DW_RECV;
				if (w4r_dly_flag)
					dw_ctrl_req = DW_SEND_W4R;
				chMtxUnlock(&dw_mutex);
				chEvtSignal(dw_thread, DW_COMM_OK_E);
				evt = chEvtWaitOneTimeout(DW_COMM_OK_E | DW_COMM_F_E, CH_TIMEOUT);
				if (evt == DW_COMM_OK_E)
				{
					message_ret = get_message(cmd_msg);
					if (message_ret)
						comm_state = COMM_IDLE;
					else
					{
						wrong_message_cnt++;
		 				comm_state = COMM_RECV;
					}
				}
				else
					comm_state = COMM_ERR;
				break;
			case COMM_SEND:
				chMtxLock(&dw_mutex);
				dw_ctrl_req = DW_SEND;
				if (w4r_dly_flag)
					dw_ctrl_req = DW_SEND_DLY;
				chMtxUnlock(&dw_mutex);
				prepare_message(cmd_msg);
				chEvtSignal(dw_thread, DW_COMM_OK_E);
				evt = chEvtWaitOneTimeout(DW_COMM_OK_E | DW_COMM_F_E, CH_TIMEOUT);
				if (evt == DW_COMM_OK_E)
					comm_state = COMM_IDLE;
				else
					comm_state = COMM_ERR;
				break;
			case COMM_ERR:
				// if (!evt)
				// 	// danger thread no responding reset mcu?
				if (evt == DW_COMM_F_E)
					chMsgSend(caller_thread, COMM_ERR_RESP);
				comm_state = COMM_IDLE;
				break;
			case COMM_IDLE:
				if (caller_thread)
				{
					if (recv_size == 0 && last_state == COMM_RECV)
						chMsgSend(caller_thread, COMM_TMO_RESP);
					else
						chMsgSend(caller_thread, COMM_END);
				}
				caller_thread = chMsgWait();
				cmd_msg = (message_meta_t*)chMsgGet(caller_thread);
				switch (cmd_msg->message)
				{
					case COMM_RECV_CMD:
						comm_state = COMM_RECV;
						w4r_dly_flag = 0;
						break;
					case COMM_SEND_CMD:
						comm_state = COMM_SEND;
						w4r_dly_flag = 0;
						break;
					case COMM_SEND_W4R_CMD:
						comm_state = COMM_RECV;
						w4r_dly_flag = 1;
						break;
					case COMM_SEND_DLY_CMD:
						comm_state = COMM_SEND;
						w4r_dly_flag = 1;
						break;
					default:
						cmd_msg = NULL;
						break;
				}
				if (cmd_msg == NULL)
					chMsgRelease(caller_thread, MSG_RESET);
				else
					chMsgRelease(caller_thread, MSG_OK);
				break;
			default:
				break;
		}

		last_state = current_state;

		if (wrong_message_cnt > 100)
		{
			chMsgSend(caller_thread, COMM_ERR_RESP);
			wrong_message_cnt = 0;
		}
	}
}

// THD_FUNCTION(PEER_CONNECTION, arg)
// {
// 	(void)arg;
// 	uint8_t res = create_conn(addr);
	

// }

THD_FUNCTION(DW_CONTROLLER, arg)
{
	(void)arg;

	dw_reset();
	spi_hal_init();
	default_config();
	load_lde();

	chMtxObjectInit(&dw_mutex);
	dw_thread = chThdGetSelfX();
	set_irq_vector();

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

	sdStart(&SD1, &serial_cfg);

	uint64_t id = get_hardware_id();
	srand(id);
	dw_write(DW_REG_INFO.PAN_ADR, (uint8_t*)(&id), 2, 0);
	dw_read(DW_REG_INFO.PAN_ADR, panadr_own.reg, DW_REG_INFO.PAN_ADR.size, 0);

	dx_time_t dx_time;
	tx_fctrl_t tx_ctrl;
	ack_resp_t_t w4r;
	eventmask_t evt = 0;
	sys_state_t state;
	memset(tx_ctrl.reg, 0, sizeof(tx_ctrl.reg));
	tx_ctrl.TXBR = BR_6_8MBPS;
	tx_ctrl.TXPRF = PRF_16MHZ;
	tx_ctrl.TXPL = PL_128;

	dw_ctrl_req_t current_state;
	dw_ctrl_req_t last_state = DW_RESET;
	uint8_t err_cnt = 0;
	uint8_t rst_cnt = 0;

	uint8_t message[7] = {MT_BROADCAST,0,0,0,0,0,0};

	barrier();

	while (true)
	{
		tx_ctrl.TFLEN = send_size+2; // TODO magic
		w4r.mask = 0;
		memset(dx_time.reg, 0, sizeof(dx_time.reg));

		current_state = dw_ctrl_req;

		switch (dw_ctrl_req)
		{
			case DW_RECV:
				dw_start_rx(dx_time);
				evt = chEvtWaitOneTimeout(MRXFCG_E | MRXERR_E, TIME_US2I(recv_tmo_usec));
				if (evt == MRXFCG_E)
					dw_ctrl_req = DW_CTRL_YIELD;
				else if (evt == 0)
					dw_ctrl_req = DW_RECV_TMO;
				else
					err_cnt++;
				dw_soft_reset_rx();
				break;
			case DW_SEND:
				dw_start_tx(tx_ctrl, send_buf, dx_time, w4r);
				evt = chEvtWaitOneTimeout(MTXFRS_E, TX_TIMEOUT);
				if (evt == MTXFRS_E)
					dw_ctrl_req = DW_CTRL_YIELD;
				else
					dw_ctrl_req = DW_TRX_ERR;
				break;
			case DW_SEND_W4R:
				w4r.W4R_TIM = send_wtime;
				w4r.ACK_TIM = 1;
				dw_start_tx(tx_ctrl, send_buf, dx_time, w4r);
				evt = chEvtWaitOneTimeout(MTXFRS_E, TX_TIMEOUT);
				if (evt != MTXFRS_E)
					dw_ctrl_req = DW_TRX_ERR;
				else
				{
					evt = chEvtWaitOneTimeout(MRXFCG_E | MRXERR_E, recv_tmo_usec);
					if (evt == MRXFCG_E)
						dw_ctrl_req = DW_CTRL_YIELD;
					else if (evt == 0)
						dw_ctrl_req = DW_RECV_TMO;
					else
						dw_ctrl_req = DW_TRX_ERR;
				}
				break;
			case DW_SEND_DLY:
				dx_time.time32 = send_dlytime;
				dw_start_tx(tx_ctrl, send_buf, dx_time, w4r);
				evt = chEvtWaitOneTimeout(MTXFRS_E, TX_TIMEOUT);
				if (evt == MTXFRS_E)
					dw_ctrl_req = DW_CTRL_YIELD;
				else
					dw_ctrl_req = DW_TRX_ERR;
				break;
			case DW_TRX_ERR:
				state = dw_transceiver_off();
				err_cnt++;
				chThdSleepMilliseconds(TRX_RST_TM);
				dw_ctrl_req = last_state;
				break;
			case DW_CTRL_YIELD:
				if (last_state != DW_RESET)
				{
					chEvtSignal(comm_thread, DW_COMM_OK_E);
					chMtxUnlock(&dw_mutex);
				}
				chEvtWaitOne(DW_COMM_OK_E);
				chMtxLock(&dw_mutex);
				chThdSleepMilliseconds(TRX_RST_TM);
				state = dw_transceiver_off();
				break;
			case DW_RECV_TMO:
				memset(recv_buf, 0, sizeof(recv_buf));
				recv_size = 0;
				dw_ctrl_req = DW_CTRL_YIELD;
				break;
			case DW_RESET:
				dw_reset();
				spi_hal_init();
				default_config();
				load_lde();
				set_irq_vector();
				rst_cnt = 0;
				dw_ctrl_req = DW_CTRL_YIELD;
				chEvtSignal(comm_thread, DW_COMM_F_E);
				break;
			default:
				break;
		}
		
		last_state = current_state;

		if (err_cnt > DW_ERR_THRESH)
		{
			rst_cnt++;
			err_cnt = 0;
		}
	}

}

// THD_FUNCTION(DIS_LOC, arg)
// {
// 	(void)arg;
// 	// sys_cfg_t cfdf;
// 	// cfdf.mask = 0;
// 	// cfdf.HIRQ_POL = 1;
// 	// cfdf.DIS_DRXB = 1;
// 	// cfdf.RXWTOE = 1;
// 	// dw_write(DW_REG_INFO.SYS_CFG, cfdf.reg, DW_REG_INFO.SYS_CFG.size, 0);
// 	// uint8_t fwto[2] = {0xFF, 0xFE};
// 	// dw_write(DW_REG_INFO.RX_FWTO, fwto, DW_REG_INFO.RX_FWTO.size, 0);
// 	// dw_read(DW_REG_INFO.RX_FWTO, fwto, DW_REG_INFO.RX_FWTO.size, 0);

// 	sys_state_t state;
// 	sys_mask_t irq_mask;
// 	MHR_16_t MHR;
// 	panadr_t panadr;

// 	irq_mask.mask = 0U;
// 	irq_mask.MTXFRS = 0b1;
// 	irq_mask.MRXFCG = 0b1;
// 	//irq_mask.MLDEDONE = 0b1;
// 	// irq_mask.MRXRFTO = 0b1;
// 	irq_mask.MRXPHE = 0b1;
// 	irq_mask.MRXFCE = 0b1;
// 	irq_mask.MRXRFSL = 0b1;
// 	//irq_mask.MLDEERR = 0b1;
// 	irq_mask.MAFFREJ = 0b1;
	
// 	sdStart(&SD1, &serial_cfg);

// 	uint64_t id = get_hardware_id();
// 	srand(id);
// 	dw_write(DW_REG_INFO.PAN_ADR, (uint8_t*)(&id), 2, 0);
// 	dw_read(DW_REG_INFO.PAN_ADR, panadr_own.reg, DW_REG_INFO.PAN_ADR.size, 0);
// 	uint16_t tx_ant_d = 63520;
// 	uint16_t rx_ant_d = 63520;
// 	eventmask_t evt = 0;
	
// 	dw_set_irq(irq_mask);

// 	dw_config_t t;
// 	int sizeconfig = sizeof(t);
// 	chprintf((BaseSequentialStream*)&SD1, "size: %d\n", sizeconfig);


// 	dw_write(DW_REG_INFO.LDE_CTRL, (uint8_t*)(&rx_ant_d), DW_SUBREG_INFO.LDE_RXANTD.size, DW_SUBREG_INFO.LDE_RXANTD.offset);
// 	dw_write(DW_REG_INFO.TX_ANTD, (uint8_t*)(&tx_ant_d), DW_REG_INFO.TX_ANTD.size, 0);

// 	while (true) {	
// 		//if (neighbours.addrs[0] == 0)
// 		loc_disc_fun();
// 		// if (panadr_own.short_addr < neighbours.addrs[0])
// 		// 	get_distance_to(neighbours.addrs[0]);
// 		// else
// 		// {
// 		// 	chprintf((BaseSequentialStream*)&SD1, "AAAAAAAAAAAAAAAA\n");
// 		// 	loc_resp_fun();
// 		// }
// 		for (int i = 0; i < NEIGHBOUR_NUM; i++)
// 			chprintf((BaseSequentialStream*)&SD1, "neigh: %d\n", neighbours.addrs[i]);
		
// 		//chprintf((BaseSequentialStream*)&SD1, "Distance: %dcm\n", (int)distances_neigh[0]);
// 	}
// }

// THD_FUNCTION(SYSTEM_STATUS, arg)
// {
// 	// loc_state_t loc_state = LOC_DISC;

// 	// uint16_t addrs[3] = {0,0,0};

// 	// dw_read(DW_REG_INFO.PAN_ADR, addrs, 2, 0);

// 	loc_disc_fun();
// 	get_distance_to(neighbours.addrs[0]);
// 	get_distance_to(neighbours.addrs[1]);
// }

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

