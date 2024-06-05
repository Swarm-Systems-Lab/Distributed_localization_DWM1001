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
thread_t* twr_thread;
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

peer_connection_t peers[NEIGHBOUR_NUM];
uint8_t current_peer_n = 0;
uint8_t current_peer_c_n = 0;
peer_info_t peers_info[NEIGHBOUR_NUM];

uint8_t twr_state = 0;

double distances_neigh[NEIGHBOUR_NUM];

panadr_t panadr_own;
tx_antd_t tx_antd;

uint32_t recv_tmo_usec;

uint8_t send_size;

int32_t send_wtime = -1;
uint32_t send_dlytime = 0;
uint8_t msg_size = 0;
uint8_t msg_seq_num = 0;
message_t send_type = 0;
uint16_t send_addr = 0;

MHR_16_t recvd_header;
message_t recvd_type;

uint8_t recv_buf[128];

dw_rod_info_t recv_info;
uint8_t recv_size;

uint8_t send_buf[128];

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

void read_frame(void)
{
	_dw_spi_transaction(1, DW_REG_INFO.RX_FINFO.id, recv_info.dw_rx_finfo.reg, DW_REG_INFO.RX_FINFO.size, 0);
	_dw_spi_transaction(1, DW_REG_INFO.RX_TIME.id, recv_info.dw_rx_time.reg, DW_REG_INFO.RX_TIME.size, 0);
	_dw_spi_transaction(1, DW_REG_INFO.RX_BUFFER.id, recv_buf, recv_info.dw_rx_finfo.RXFLEN, 0);
	recv_size = recv_info.dw_rx_finfo.RXFLEN-2; // TODO 2 magic number FCS
}

int32_t get_message(void)
{
	if (recv_size == 0)
		return 0;

	recvd_header = decode_MHR(recv_buf);
	recvd_type = recv_buf[sizeof(recvd_header)];

	if (recvd_header.frame_control.mask != def_frame_ctrl.mask)
		return 0;

	create_new_peer(recvd_header.src_addr);

	memset(recv_buf+recv_size, 0, sizeof(recv_buf)-recv_size);
	memmove(recv_buf, recv_buf+sizeof(recvd_header)+1, sizeof(recv_buf)-sizeof(recvd_header)-1);

	return 1;
}

void prepare_message(void)
{
	MHR_16_t send_header;

	peer_connection_t* peer = get_peer(send_addr);

	if (peer)
	{
		if (send_type > MT_ACK)
		{
			memcpy(peer->last_message, send_buf, msg_size);
			peer->last_message_size = msg_size;
			peer->last_message_type = send_type;
		}
		peer->last_cmd_type = send_type;
	}

	send_header.frame_control = def_frame_ctrl;
	send_header.dest_addr = send_addr;
	send_header.src_addr = panadr_own.short_addr;
	send_header.dest_pan_id = panadr_own.pan_id;
	send_header.seq_num = msg_seq_num;
	memmove(send_buf+sizeof(send_header)+1, send_buf, sizeof(send_buf)-sizeof(send_header)-1);
	memcpy(send_buf, &send_header, sizeof(send_header));
	send_buf[sizeof(send_header)] = send_type;
	send_size = msg_size + sizeof(send_header)+1;
}

void init_peers(void)
{
	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
	{
		peers[i].peer_addr = 0;
		peers[i].seq_ack_n = 0;
		peers[i].ttl = 255;
		peers[i].last_message_size = 0;
		peers[i].last_message_type = 0;
		peers_info[i].conn = peers+i;
		peers_info[i].distance = 0.0;
		peers_info[i].d_measures = 0;
		peers_info[i].peer_id = i;
	}
}

peer_connection_t* create_new_peer(uint16_t addr)
{
	uint8_t pos;

	toggle_led(green);

	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
	{
		pos = i;
		if (peers[i].peer_addr == addr)
			return NULL;
		if (peers[i].peer_addr == 0)
		{
			peers[i].peer_addr = addr;
			current_peer_n++;
			return peers+i;
		}
	}

	if (pos == NEIGHBOUR_NUM-1)
		return NULL;
}

peer_connection_t* get_peer(uint16_t addr)
{
	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
	{
		if (peers[i].peer_addr == addr)
			return peers+i;
	}
	return NULL;
}

peer_info_t* get_peer_info(uint16_t addr)
{
	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
	{
		if (peers_info[i].conn->peer_addr == addr)
			return peers_info+i;
	}
	return NULL;
}

void connect_peer(peer_connection_t* peer)
{
	peer->seq_ack_n = 0x11;
	current_peer_c_n++;
	peer->ttl = 0;
}

void disconnect_peer(peer_connection_t* peer)
{
	peer_info_t* peer_info = get_peer_info(peer->peer_addr);
	peer_info->distance = 0.0;
	peer_info->d_measures = 0;
	peer->peer_addr = 0;
	peer->seq_ack_n = 0;
	peer->last_message_size = 0;
	peer->last_message_type = 0;
	peers->ttl = 255;
	current_peer_c_n--;
	current_peer_n--;
}

peer_connection_t* get_unconn_peer(void)
{
	if ((current_peer_n-current_peer_c_n) <= 0)
		return NULL;

	uint8_t rnd_peer = rand() / (RAND_MAX/(current_peer_n-current_peer_c_n) + 1);
	int16_t tried_cnt = -1;

	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
	{
		if (peers[i].peer_addr > 0 && peers[i].ttl == 255)
			tried_cnt++;
		if (tried_cnt == rnd_peer)
			return peers+i;
	}

	return NULL;
}

peer_connection_t* get_conn_peer(void)
{
	if (current_peer_c_n == 0)
		return NULL;

	uint8_t d_measures_min = 255;
	peer_connection_t* peer_min = NULL;

	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
	{
		if (peers[i].peer_addr > 0 && peers[i].ttl < PEER_CONN_TTL)
		{
			if (peers_info[i].d_measures <= d_measures_min)
			{
				d_measures_min = peers_info[i].d_measures;
				peer_min = peers+i;
			}
		}
	}

	return peer_min;
}


void clean_recvd(void)
{
	memset(&recvd_header, 0, sizeof(recvd_header));
	memset(recv_buf, 0, recv_size);

	recvd_type = 0;
}

void clean_send(void)
{
	memset(send_buf, 0, send_size);

	send_wtime = -1;
	send_dlytime = 0;
	msg_size = 0;
	msg_seq_num = 0;
	send_type = 0;
	send_addr = 0;
}

void send_syn(void)
{
	peer_connection_t* peer = get_unconn_peer();

	if (peer)
		send_conn_msg(peer, 1, MT_SYN);
}

void send_conn_msg(peer_connection_t* peer, uint8_t size, message_t type)
{
	if (peer->ttl > PEER_CONN_TTL && send_type > MT_ACK)
		return;
	send_wtime = -1;
	send_dlytime = 0;
	msg_size = size;
	msg_seq_num = peer->seq_ack_n&0x01;
	send_type = type;
	send_addr = peer->peer_addr;

	chEvtSignal(dw_thread, DW_COMM_SEND_E);
	chEvtSignal(comm_thread, DW_COMM_SEND_E);
}

void send_ack(peer_connection_t* peer)
{
	send_wtime = -1;
	send_dlytime = 0;
	msg_size = 1;
	msg_seq_num = peer->seq_ack_n&0x10;
	send_type = MT_ACK;
	send_addr = peer->peer_addr;
	
	chEvtSignal(dw_thread, DW_COMM_SEND_E);
	chEvtSignal(comm_thread, DW_COMM_SEND_E);
}

void send_last_message(peer_connection_t* peer)
{
	memcpy(send_buf, peer->last_message, peer->last_message_size);
	send_conn_msg(peer, peer->last_message_size, peer->last_message_type);
}

void send_d_req(void)
{
	peer_connection_t* peer = get_conn_peer();

	if (peer)
		send_conn_msg(peer, 1, MT_D_REQ);

}

void compute_distance(void)
{
	toggle_led(blue);
	uint64_t m_tx_time, m_rx_time;
	uint64_t tx_time = dw_get_tx_time();
	uint64_t rx_time = 0;
	memcpy(&rx_time, recv_info.dw_rx_time.RX_STAMP, sizeof(recv_info.dw_rx_time.RX_STAMP));
	if (!rx_time)
		memcpy(&rx_time, recv_info.dw_rx_time.RX_RAWST, sizeof(recv_info.dw_rx_time.RX_RAWST));

	memcpy(&m_tx_time, recv_buf, sizeof(m_tx_time));
	memcpy(&m_rx_time, recv_buf+sizeof(m_tx_time), sizeof(m_rx_time));

	int64_t rt_init = (rx_time) - (tx_time);
	int64_t rt_resp = (m_tx_time) - (m_rx_time);

	float clock_offset_r = dw_get_car_int() * ((998.4e6/2.0/1024.0/131072.0) * (-1.0e6/6489.6e6) / 1.0e6);
	rt_resp *= (1.0f - clock_offset_r);

	double tof = (rt_init - rt_resp)/2.0f;
	tof = tof * (1.0f/(float)(499.2e6*128.0));

	peer_info_t* peer_info = get_peer_info(recvd_header.src_addr);

	if (tof > 0.0 && tof < 1e-5 && peer_info)
	{
		double distance = tof * 299702547;
		distance *= 100;
		peer_info->distance = (peer_info->distance)*NEIGHBOUR_NUM/(NEIGHBOUR_NUM+1) + distance/(NEIGHBOUR_NUM+1);
		peer_info->d_measures++;
	}
}

THD_FUNCTION(TWR, arg)
{
	(void)arg;

	msg_t recvd_msg;
	peer_connection_t* peer;
	peer_connection_t* twr_peer;
	twr_thread = chThdGetSelfX();

	barrier();

	while (true)
	{
		clean_recvd();
		chMsgWait();
		recvd_msg = chMsgGet(comm_thread);
		chMsgRelease(comm_thread, MSG_OK);
		chThdYield();
		if (recvd_msg == MSG_OK)
		{
			peer = get_peer(recvd_header.src_addr);
			if (peer == NULL)
				recvd_type == 0;
			switch (recvd_type)
			{
				case MT_D_REQ:
					if (((peer->seq_ack_n&0x10)>>4) == (recvd_header.seq_num&0x01)  && peer->ttl < PEER_CONN_TTL)
					{
						peer->seq_ack_n ^= 0x10; // Flips 4 bit (ack_num) 
						//send_d_req_ack(peer); // w4r
						send_wtime = 0;
						send_dlytime = 0;
						msg_size = 1;
						msg_seq_num = peer->seq_ack_n&0x10;
						send_type = MT_D_REQ_ACK;
						send_addr = recvd_header.src_addr;
						
						chEvtSignal(dw_thread, DW_COMM_SEND_E);
						chEvtSignal(comm_thread, DW_COMM_SEND_E);

						twr_state = 1;
						twr_peer = peer;
					}
					else
						send_ack(peer);
					break;
				case MT_D_REQ_ACK:
 					if (peer->last_message_type == MT_D_REQ  && peer->ttl < PEER_CONN_TTL)
					{
						peer->seq_ack_n ^= 0x01; // Flips 0 bit (seq_num) 
						twr_state = 1;
						twr_peer = peer;
						//send_d_init(); // w4r
						send_wtime = 0;
						send_dlytime = 0;
						msg_size = 1;
						msg_seq_num = peer->seq_ack_n&0x01;
						send_type = MT_D_INIT;
						send_addr = recvd_header.src_addr;
						
						chEvtSignal(dw_thread, DW_COMM_SEND_E);
						chEvtSignal(comm_thread, DW_COMM_SEND_E);
					}
					else
						send_ack(peer);
					break;
				case MT_D_INIT:
					if (!(peer == twr_peer && twr_state))
					{
						send_conn_msg(twr_peer, 1, MT_D_FAIL);
						twr_state = 0;
					}
					break;
				case MT_D_RESP:
 					if (peer == twr_peer && twr_state)
					{
						twr_state = 1;
						compute_distance();
						//send_d_res(); // dly
						peer_info_t* peer_info = get_peer_info(recvd_header.src_addr);
						memcpy(send_buf, &(peer_info->distance), sizeof(peer_info->distance));

						send_conn_msg(peer, sizeof(peer_info->distance), MT_D_RES);
					}
					else
						send_conn_msg(twr_peer, 1, MT_D_FAIL);
						
					twr_state = 0;
					break;
				default:
					break;
			}
		}
	}
}

THD_FUNCTION(PEER_DISCOVERY, arg)
{
	(void)arg;

	msg_t recvd_msg;
	uint8_t skip_cnt = 0;
	peer_disc_thread = chThdGetSelfX();

	barrier();

	while (true)
	{
		clean_recvd();
		chMsgWait();
		recvd_msg = chMsgGet(comm_thread);
		chMsgRelease(comm_thread, MSG_OK);
		chThdYield();
		if (twr_state)
			twr_state = 0;
		else
		{
			if (recvd_msg == MSG_OK && current_peer_n < (1<<skip_cnt) && current_peer_n < NEIGHBOUR_NUM)
			{
				skip_cnt = 0;
				send_wtime = -1;
				send_dlytime = 0;
				msg_size = 1;
				msg_seq_num = 0;
				send_type = MT_BROADCAST;
				send_addr = 0xFFFF;
				
				chEvtSignal(dw_thread, DW_COMM_SEND_E);
				chEvtSignal(comm_thread, DW_COMM_SEND_E);
			}
			else
			{
				skip_cnt++;
				if (current_peer_c_n == 0 || current_peer_c_n < current_peer_n)
					send_syn();
				else
					send_d_req();
				chThdSleepMilliseconds(10*skip_cnt);
			}
		}
	}
}

THD_FUNCTION(PEER_CONNECTION, arg)
{
	(void)arg;

	msg_t recvd_msg;
	peer_connection_t* peer;
	init_peers();
	double distance = 0;
	peer_conn_thread = chThdGetSelfX();

	barrier();

	while (true)
	{
		clean_recvd();
		chMsgWait();
		recvd_msg = chMsgGet(comm_thread);
		chMsgRelease(comm_thread, MSG_OK);
		chThdYield();
		if (recvd_msg == MSG_OK)
		{
			peer = get_peer(recvd_header.src_addr);
			if (peer == NULL)
				recvd_type == 0;
			switch (recvd_type)
			{
				case MT_SYN:
					if (peer->ttl < PEER_CONN_TTL)
						disconnect_peer(peer);
					else
					{
						peer->seq_ack_n = 0x10;
						send_conn_msg(peer, 1, MT_SYN_ACK);
					} 
					break;
				case MT_SYN_ACK:
					if (peer->ttl < PEER_CONN_TTL)
						disconnect_peer(peer);
					else
					{
						if (peer->last_cmd_type == MT_SYN)
						{
							connect_peer(peer);
							send_ack(peer);
						}
						else
							disconnect_peer(peer);
					}
					break;
				case MT_ACK:
					if ((peer->seq_ack_n&0x01) == !((recvd_header.seq_num&0x10)>>4)) // seq num == not ack_num
					{
						// Make connection
						if (peer->last_cmd_type == MT_SYN_ACK)
						{
							if (peer->ttl == 255)
							{
								connect_peer(peer);
								send_d_req();
							}
						}
						else
						{
							if (peer->ttl < PEER_CONN_TTL)
								peer->seq_ack_n ^= 0x01; // Flips 0 bit (seq_num)  
						}
						// TODO check multiple positive acks (not suppose to happen)
					}
					else
					{
						if (peer->ttl < PEER_CONN_TTL)
						{
							send_last_message(peer);
							peer->ttl++;
						}
						else
							disconnect_peer(peer);
					}
					break;
				case MT_D_FAIL:
					twr_state = 0;
					if ((peer->seq_ack_n&0x01) == ((recvd_header.seq_num&0x10)>>4))
						peer->seq_ack_n ^= 0x10; // Flips 4 bit (ack_num) 
					send_ack(peer);
					break;
				case MT_D_RES:
					// Add returned distance if sensible
					memcpy(&distance, recv_buf, sizeof(distance));
					peer_info_t* peer_info = get_peer_info(recvd_header.src_addr);
					peer_info->distance = (peer_info->distance)*NEIGHBOUR_NUM/(NEIGHBOUR_NUM+1) + distance/(NEIGHBOUR_NUM+1);
					// NO BREAK
				case MT_MCONN:
					if (((peer->seq_ack_n&0x10)>>4) == (recvd_header.seq_num&0x01)  && peer->ttl < PEER_CONN_TTL)
						peer->seq_ack_n ^= 0x10; // Flips 4 bit (ack_num) 
					else
					{
						peer->ttl++;
						if (peer->ttl >= PEER_CONN_TTL)
							disconnect_peer(peer);
					}
					send_ack(peer);
					break;
				default:
					break;
			}
		}
	}
}

void proccess_message(void)
{
	uint8_t is_timeout = 0;

	switch (recvd_type&0xF0)
	{
		case 0:
			is_timeout = 1;
			break;
		case 0x10:
			chMsgSend(peer_conn_thread, MSG_OK);
			break;
		case 0x20:
			chMsgSend(twr_thread, MSG_OK);
			break;
	}
}

THD_FUNCTION(COMMS, arg)
{
	(void)arg;

	comm_thread = chThdGetSelfX();
	comm_state_t comm_state = COMM_RECV;
	comm_state_t current_state = COMM_IDLE;
	comm_state_t last_state = COMM_IDLE;
	eventmask_t evt = 0;
	uint8_t wrong_message_cnt = 0;
	int32_t message_ret = 0;
	uint8_t w4r_dly_flag = 0;

	thread_t* caller_thread = NULL;

	barrier();

	while (true)
	{
		current_state = comm_state;
		recv_tmo_usec = (rand()&0xF000)+40000;
		switch (comm_state)
		{
			case COMM_RECV:
				chMtxLock(&dw_mutex);
				dw_ctrl_req = DW_RECV;
				if (send_wtime >= 0)
				{
					dw_ctrl_req = DW_SEND_W4R;
					prepare_message();
				}
				chMtxUnlock(&dw_mutex);
				chEvtSignal(dw_thread, DW_COMM_OK_E);
				evt = chEvtWaitOneTimeout(DW_COMM_OK_E | DW_COMM_SEND_E, CH_TIMEOUT);
				if (evt == DW_COMM_OK_E)
				{
					message_ret = get_message();
					comm_state = COMM_IDLE;
				}
				else if (evt == DW_COMM_SEND_E)
				{
					if (send_wtime < 0)
						comm_state = COMM_SEND;
				}
				else
					comm_state = COMM_ERR;
				break;
			case COMM_SEND:
				chMtxLock(&dw_mutex);
				dw_ctrl_req = DW_SEND;
				if (send_dlytime > 0)
					dw_ctrl_req = DW_SEND_DLY;
				chMtxUnlock(&dw_mutex);
				prepare_message();
				chEvtSignal(dw_thread, DW_COMM_OK_E);
				evt = chEvtWaitOneTimeout(DW_COMM_OK_E, CH_TIMEOUT);
				if (evt == DW_COMM_OK_E)
					comm_state = COMM_IDLE;
				else
					comm_state = COMM_ERR;
				break;
			case COMM_ERR:
				// if (!evt)
				// 	// danger thread no responding reset mcu?
				//chMsgSend(caller_thread, COMM_ERR_RESP);
				comm_state = COMM_IDLE;
				break;
			case COMM_IDLE:
				if (last_state == COMM_RECV)
				{
					// decide
					switch (recvd_type&0xF0)
					{
						case 0:
							chMsgSend(peer_disc_thread, MSG_OK);
							break;
						case 0x10:
							chMsgSend(peer_conn_thread, MSG_OK);
							break;
						case 0x20:
							chMsgSend(twr_thread, MSG_OK);
							break;
					}
				}
				comm_state = COMM_RECV;
				break;
			default:
				break;
		}

		last_state = current_state;

		if (wrong_message_cnt > 100)
		{
			chMsgSend(caller_thread, MSG_RESET);
			wrong_message_cnt = 0;
		}
	}
}

int8_t respond_if_twr(void)
{
	recvd_type = recv_buf[sizeof(recvd_header)];
	if (!(recvd_type == MT_D_INIT && twr_state))
		return 0;

	dx_time_t dx_time;
	ack_resp_t_t w4r;
	tx_fctrl_t tx_ctrl;
	eventmask_t evt = 0;
	memset(tx_ctrl.reg, 0, sizeof(tx_ctrl.reg));
	memset(send_buf, 0, sizeof(send_buf));
	tx_ctrl.TXBR = BR_6_8MBPS;
	tx_ctrl.TXPRF = PRF_16MHZ;
	tx_ctrl.TXPL = PL_128;
	w4r.mask = 0;
	memset(dx_time.reg, 0, sizeof(dx_time.reg));

	recvd_header = decode_MHR(recv_buf);

	uint64_t rx_time = 0;
	memcpy(&rx_time, recv_info.dw_rx_time.RX_STAMP, sizeof(recv_info.dw_rx_time.RX_STAMP));
	if (!rx_time)
		memcpy(&rx_time, recv_info.dw_rx_time.RX_RAWST, sizeof(recv_info.dw_rx_time.RX_RAWST));
	uint64_t delay_tx = (uint64_t)rx_time + (uint64_t)(65536*3000);
	uint64_t tx_time = (uint64_t)delay_tx +(uint64_t)tx_antd;

	memcpy(send_buf, &tx_time, sizeof(tx_time));
	memcpy(send_buf+sizeof(tx_time), &rx_time, sizeof(rx_time));

	msg_size = sizeof(tx_time)+sizeof(rx_time);
	msg_seq_num = 0;
	send_type = MT_D_RESP;
	send_addr = recvd_header.src_addr;

	prepare_message();
	tx_ctrl.TFLEN = send_size+2; // TODO magic

	dx_time.time32 = (uint32_t)(delay_tx >> 8);
	dw_start_tx(tx_ctrl, send_buf, dx_time, w4r);
	evt = chEvtWaitOneTimeout(MTXFRS_E, TX_TIMEOUT);
	if (evt != MTXFRS_E)
	{
		twr_state = 0;
		return -1;
	}
	return 1;
		
}

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

	uint64_t id = get_hardware_id();
	srand(id);
	dw_write(DW_REG_INFO.PAN_ADR, (uint8_t*)(&id), 2, 0);
	dw_read(DW_REG_INFO.PAN_ADR, panadr_own.reg, DW_REG_INFO.PAN_ADR.size, 0);

	tx_antd = 31900;
	uint16_t rx_ant_d = 31900;
	dw_write(DW_REG_INFO.LDE_CTRL, (uint8_t*)(&rx_ant_d), DW_SUBREG_INFO.LDE_RXANTD.size, DW_SUBREG_INFO.LDE_RXANTD.offset);
	dw_write(DW_REG_INFO.TX_ANTD, (uint8_t*)(&tx_antd), DW_REG_INFO.TX_ANTD.size, 0);

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
	int8_t twr_ret;

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
				evt = chEvtWaitOneTimeout(MRXFCG_E | MRXERR_E | DW_COMM_SEND_E, TIME_US2I(recv_tmo_usec));
				if (evt == MRXFCG_E)
					dw_ctrl_req = DW_CTRL_YIELD;
				else if (evt == 0 || evt == DW_COMM_SEND_E)
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
				clean_send();
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
					evt = chEvtWaitOneTimeout(MRXFCG_E | MRXERR_E | DW_COMM_SEND_E, TIME_US2I(recv_tmo_usec));
					if (evt == MRXFCG_E)
					{
						twr_ret = respond_if_twr();
						if (twr_ret < 0)
							dw_ctrl_req = DW_TRX_ERR;
						dw_ctrl_req = DW_CTRL_YIELD;
					}
					else if (evt == 0 || evt == DW_COMM_SEND_E)
						dw_ctrl_req = DW_RECV_TMO;
					else
						dw_ctrl_req = DW_TRX_ERR;
					dw_soft_reset_rx();
				}
				if (evt != DW_COMM_SEND_E)
					clean_send();
				break;
			case DW_SEND_DLY:
				dx_time.time32 = send_dlytime;
				dw_start_tx(tx_ctrl, send_buf, dx_time, w4r);
				evt = chEvtWaitOneTimeout(MTXFRS_E, TX_TIMEOUT);
				if (evt == MTXFRS_E)
					dw_ctrl_req = DW_CTRL_YIELD;
				else
					dw_ctrl_req = DW_TRX_ERR;
				clean_send();
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
					if (evt != DW_COMM_SEND_E)
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
				chEvtSignal(comm_thread, DW_COMM_OK_E);
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

THD_FUNCTION(SYSTEM_STATUS, arg)
{
	(void)arg;

	barrier_init(6);

	sdStart(&SD1, &serial_cfg);

	barrier();

	while (true)
	{
		for (int i = 0; i < NEIGHBOUR_NUM; i++)
		{
			peers_info[i].d_measures = 0;
			chprintf((BaseSequentialStream*)&SD1, "peer_addr: %d\n", peers[i].peer_addr);
			chprintf((BaseSequentialStream*)&SD1, "peer_seq_ack: %x\nttl: %d\n", peers[i].seq_ack_n, peers[i].ttl);
			chprintf((BaseSequentialStream*)&SD1, "d: %d\n", (int)peers_info[i].distance);
		}

		chThdSleepMilliseconds(500);
	}
}

// 	// sys_cfg_t cfdf;
// 	// cfdf.mask = 0;
// 	// cfdf.HIRQ_POL = 1;
// 	// cfdf.DIS_DRXB = 1;
// 	// cfdf.RXWTOE = 1;
// 	// dw_write(DW_REG_INFO.SYS_CFG, cfdf.reg, DW_REG_INFO.SYS_CFG.size, 0);
// 	// uint8_t fwto[2] = {0xFF, 0xFE};
// 	// dw_write(DW_REG_INFO.RX_FWTO, fwto, DW_REG_INFO.RX_FWTO.size, 0);
// 	// dw_read(DW_REG_INFO.RX_FWTO, fwto, DW_REG_INFO.RX_FWTO.size, 0);

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

