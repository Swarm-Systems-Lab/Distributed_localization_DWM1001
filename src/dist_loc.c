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
peer_info_t peers_info[NEIGHBOUR_NUM];
uint8_t current_peer_n = 0;
uint8_t current_peer_c_n = 0;

panadr_t panadr_own;
tx_antd_t tx_antd;

uint32_t recv_tmo_usec;

uint8_t send_size;

send_msg_meta_t send_msg_meta;
send_msg_meta_t send_msg_meta_def = 
{
	.wtime = -1,
	.dlytime = 0,
	.size = 0,
	.seq_ack_num = 0,
	.type = 0,
	.addr = 0
};

MHR_16_t recvd_header;
message_t recvd_type;

dw_rod_info_t recv_info;
uint8_t recv_size;

uint8_t recv_buf[128];
uint8_t send_buf[128];

dw_ctrl_req_t dw_ctrl_req = DW_CTRL_YIELD;

loc_state_t loc_state = LOC_INIT;
loc_action_t loc_action = LOC_NO_RESP;
twr_state_t twr_state = TWR_NO_TWR;

peer_connection_t* twr_peer = NULL;
uint8_t twr_peer_seq = 0;

uint8_t messages_since_broad = 0;
uint8_t recv_tmo_cnt = 0;
uint8_t twr_fail_cnt = 0;

euclidean_d_m_t euclidean_d_m;
float peer_positions[NEIGHBOUR_NUM+1][3];

virtual_timer_t comm_watchdog;

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

// Function to get the index of an address in the addrs array
int8_t _get_address_index(uint16_t addr) 
{
	for (uint8_t i = 0; i < NEIGHBOUR_NUM + 1; i++) 
	{
		if (euclidean_d_m.addrs[i] == addr) 
			return i;
	}
	return -1; // Address not found
}

// Function to get the distance between two nodes
float get_distance(uint16_t addr1, uint16_t addr2) 
{
 	int8_t index1 = _get_address_index(addr1);
  	int8_t index2 = _get_address_index(addr2);

	if (index1 == -1 || index2 == -1) 
    	return -1.0f; // Indicate invalid address

	return euclidean_d_m.distances[index1][index2];
}

// Function to set the distance between two nodes
void set_distance(uint16_t addr1, uint16_t addr2, float distance) 
{
  	int8_t index1 = _get_address_index(addr1);
  	int8_t index2 = _get_address_index(addr2);

  	if (index1 == -1 || index2 == -1)
		return; // Do nothing if address is invalid

	if (distance > 0.0)
		euclidean_d_m.distances[index1][index2] = distance;
}

// Function to initialize the matrix
void init_d_m(void) 
{
	// Set all distances to -1 (except for self-distances)
	for (uint8_t i = 0; i < NEIGHBOUR_NUM + 1; i++) 
	{
		euclidean_d_m.addrs[i] = 0;
		for (uint8_t j = 0; j < NEIGHBOUR_NUM + 1; j++) 
			euclidean_d_m.distances[i][j] = -1.0f;

		euclidean_d_m.distances[i][i] = 0.0f;
	}
}

void read_frame(void)
{
	_dw_spi_transaction(1, DW_REG_INFO.RX_FINFO.id, recv_info.dw_rx_finfo.reg, DW_REG_INFO.RX_FINFO.size, 0);
	_dw_spi_transaction(1, DW_REG_INFO.RX_TIME.id, recv_info.dw_rx_time.reg, DW_REG_INFO.RX_TIME.size, 0);
	_dw_spi_transaction(1, DW_REG_INFO.RX_BUFFER.id, recv_buf, recv_info.dw_rx_finfo.RXFLEN, 0);
	recv_size = recv_info.dw_rx_finfo.RXFLEN-2; // TODO 2 magic number FCS
}

void reset_comms(virtual_timer_t* vtp, void* args)
{
	init_peers();
	recv_tmo_usec = (rand()&0xF000)+40000;
 	send_msg_meta = send_msg_meta_def;
	recvd_type = 0;
 	memset(recv_buf, 0, sizeof(recv_buf));
 	memset(send_buf, 0, sizeof(send_buf));
	dw_ctrl_req = DW_CTRL_YIELD;
	loc_state = LOC_INIT;
	loc_action = LOC_NO_RESP;
	twr_state = TWR_NO_TWR;
	twr_peer = NULL;
	twr_peer_seq = 0;
	messages_since_broad = 0;
	recv_tmo_cnt = 0;
	twr_fail_cnt = 0;
}

int32_t get_message(void)
{
	if (recv_size == 0)
		return 0;

	recv_tmo_cnt = 0;
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
	peer_connection_t* peer = NULL;

	if (send_msg_meta.type == MT_D_REQ || send_msg_meta.type == MT_D_REQ_ACK)
		peer = get_peer(*((uint16_t*)send_buf));
	else
		peer = get_peer(send_msg_meta.addr);

	if (send_msg_meta.type != MT_BROADCAST && messages_since_broad < 255)
		messages_since_broad++;

	if (peer)
	{
		if (send_msg_meta.type > MT_ACK)
		{
			memcpy(peer->last_message, send_buf, send_msg_meta.size);
			peer->last_message_size = send_msg_meta.size;
			peer->last_message_type = send_msg_meta.type;
		}
		peer->last_cmd_type = send_msg_meta.type;
	}

	send_header.frame_control = def_frame_ctrl;
	send_header.dest_addr = send_msg_meta.addr;
	send_header.src_addr = panadr_own.short_addr;
	send_header.dest_pan_id = panadr_own.pan_id;
	send_header.seq_num = send_msg_meta.seq_ack_num;
	memmove(send_buf+sizeof(send_header)+1, send_buf, sizeof(send_buf)-sizeof(send_header)-1);
	memcpy(send_buf, &send_header, sizeof(send_header));
	send_buf[sizeof(send_header)] = send_msg_meta.type;
	send_size = send_msg_meta.size + sizeof(send_header)+1;
}

void init_peers(void)
{
	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
	{
		peers[i].peer_addr = 0;
		peers[i].seq_ack_n = 0;
		peers[i].ttl = 255;
		chVTObjectInit(&(peers[i].tmo_timer));
		peers[i].last_message_size = 0;
		peers[i].last_message_type = 0;
		peers_info[i].conn = peers+i;
		peers_info[i].calc_distance = -1.0;
		peers_info[i].recvd_distance = -1.0;
		peers_info[i].d_measures = 0;
		peers_info[i].peer_id = i;
	}
	current_peer_n = 0;
 	current_peer_c_n = 0;
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

void inc_ttl(virtual_timer_t* vtp, void* arg)
{
	disconnect_peer((peer_connection_t*)arg);
}

void connect_peer(peer_connection_t* peer)
{
	peer->seq_ack_n = 0x11;
	current_peer_c_n++;
	peer->ttl = 0;
	chVTSet(&(peer->tmo_timer), TIME_S2I(5), inc_ttl, peer);
}

void disconnect_peer(peer_connection_t* peer)
{
	peer_info_t* peer_info = get_peer_info(peer->peer_addr);
	if (peer->ttl < 255)
	{
		peer_info->calc_distance = -1.0;
		peer_info->recvd_distance = -1.0;
		peer_info->d_measures = 0;
		peer->peer_addr = 0;
		peer->seq_ack_n = 0;
		peer->last_message_size = 0;
		peer->last_message_type = 0;
		peers->ttl = 255;
		current_peer_c_n--;
		current_peer_n--;
	}
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
	send_msg_meta = send_msg_meta_def;
}

void send_syn(void)
{
	peer_connection_t* peer = get_unconn_peer();

	if (peer)
		send_conn_msg(peer, 1, MT_SYN);
}

void send_broad(void)
{
	send_msg_meta = send_msg_meta_def;

	send_msg_meta.size = 1;
	send_msg_meta.seq_ack_num = 0;
	send_msg_meta.type = MT_BROADCAST;
	send_msg_meta.addr = 0xFFFF;
}

void send_conn_msg(peer_connection_t* peer, uint8_t size, message_t type)
{
	if (peer->ttl > PEER_CONN_TTL && send_msg_meta.type > MT_ACK)
		return;
	
	send_msg_meta = send_msg_meta_def;

	send_msg_meta.size = size;
	send_msg_meta.seq_ack_num = peer->seq_ack_n&0x01;
	send_msg_meta.type = type;
	send_msg_meta.addr = peer->peer_addr;

	loc_action = LOC_RESP_BTMO;
}

void send_w4r_msg(peer_connection_t* peer, uint8_t size, message_t type)
{	
	send_msg_meta.wtime = 0;
	send_msg_meta.dlytime = 0;
	send_msg_meta.size = size;
	send_msg_meta.seq_ack_num = peer->seq_ack_n&0x01;
	send_msg_meta.type = type;
	send_msg_meta.addr = peer->peer_addr;

	loc_action = LOC_RESP_BTMO;
}

void send_ack(peer_connection_t* peer)
{
	send_msg_meta = send_msg_meta_def;

	send_msg_meta.size = 1;
	send_msg_meta.seq_ack_num = peer->seq_ack_n&0x10;
	send_msg_meta.type = MT_ACK;
	send_msg_meta.addr = peer->peer_addr;
	
	loc_action = LOC_RESP_BTMO;
}

void send_last_message(peer_connection_t* peer)
{
	memcpy(send_buf, peer->last_message, peer->last_message_size);
	send_conn_msg(peer, peer->last_message_size, peer->last_message_type);

	loc_action = LOC_RESP_BTMO;
}

void send_d_req(void)
{
	peer_connection_t* peer = get_conn_peer();

	if (peer)
	{
		send_msg_meta.wtime = 0;
		send_msg_meta.dlytime = 0;
		send_msg_meta.size = sizeof(peer->peer_addr)+sizeof(euclidean_d_m);
		send_msg_meta.seq_ack_num = peer->seq_ack_n&0x01;
		send_msg_meta.type = MT_D_REQ;
		send_msg_meta.addr = 0xFFFF;

		// TODO memory danger if neigh_num too large
		memcpy(send_buf, &(peer->peer_addr), sizeof(peer->peer_addr));
		memcpy(send_buf+sizeof(peer->peer_addr), &euclidean_d_m, sizeof(euclidean_d_m));

		loc_state = LOC_TWR;
		twr_state = TWR_REQ_SENT;
		twr_peer = peer;
		twr_peer_seq = peer->seq_ack_n;
	}
}

void send_d_req_ack(peer_connection_t* peer)
{
	send_msg_meta.wtime = 0;
	send_msg_meta.dlytime = 0;
	send_msg_meta.size = sizeof(peer->peer_addr)+sizeof(euclidean_d_m);
	send_msg_meta.seq_ack_num = peer->seq_ack_n&0x10;
	send_msg_meta.type = MT_D_REQ_ACK;
	send_msg_meta.addr = 0xFFFF;

	memcpy(send_buf, &(peer->peer_addr), sizeof(peer->peer_addr));
	memcpy(send_buf+sizeof(peer->peer_addr), &euclidean_d_m, sizeof(euclidean_d_m));
}

void process_req(void)
{
	euclidean_d_m_t* m = (euclidean_d_m_t*)(recv_buf+sizeof(panadr_own.short_addr));

	for (uint8_t i = 0; i < NEIGHBOUR_NUM+1; i++)
	{
		if (m->addrs[i] != panadr_own.short_addr)
		{
			if (m->addrs[i] == recvd_header.src_addr)
			{
				for (uint8_t j = 0; j < NEIGHBOUR_NUM+1; j++)
					set_distance(m->addrs[i], m->addrs[j], m->distances[i][j]);
			}
			// else
			// {
			// 	for (uint8_t j = 0; j < NEIGHBOUR_NUM+1; j++)
			// 		euclidean_d_m.distances[i][j] = m->distances[i][j];
			// }
		}
	}
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
		if (peer_info->calc_distance > 0)
			peer_info->calc_distance = (peer_info->calc_distance)*NEIGHBOUR_NUM/(NEIGHBOUR_NUM+1) + distance/(NEIGHBOUR_NUM+1);
		else
			peer_info->calc_distance = distance;

		peer_info->d_measures++;
	}
}

void handle_twr_fail(void)
{
	loc_state = LOC_COMM;
	twr_state = TWR_NO_TWR;
	twr_peer->seq_ack_n = twr_peer_seq;
	// EXPONENTIAL BACKOFF !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	send_conn_msg(twr_peer, 1, MT_D_FAIL);
	twr_peer = NULL;
	twr_fail_cnt++;
}

void twr_handle(peer_connection_t* peer)
{
	float distance = 0.0;

	if (twr_state != TWR_NO_TWR && peer != twr_peer)
		recvd_type == 0;

	loc_action = LOC_RESP_NOW;

	switch (recvd_type)
	{
		distance = -1.0;
		case MT_D_REQ:
			if (/*((peer->seq_ack_n&0x10)>>4) == (recvd_header.seq_num&0x01) &&*/ (twr_state == TWR_NO_TWR || twr_state == TWR_REQ_SENT))
			{
				if (twr_state == TWR_REQ_SENT && peer->peer_addr < panadr_own.short_addr)
				{
					//chprintf((BaseSequentialStream*)&SD1, "gentle\n");
					loc_action = LOC_NO_RESP;
				}
				else
				{
					loc_state = LOC_TWR;
					twr_state = TWR_REQ_RECVD;
					twr_peer = peer;
					twr_peer_seq = peer->seq_ack_n;

					send_d_req_ack(peer);
				}
			}
			else
			{
				handle_twr_fail();
				//chprintf((BaseSequentialStream*)&SD1, "twr_state: %d recvd_header.seq_num: %d peer->seq_ack_n: %d\n", twr_state, recvd_header.seq_num, peer->seq_ack_n);
			}
			break;
		case MT_D_REQ_ACK:
			if (peer->last_message_type == MT_D_REQ && twr_state == TWR_REQ_SENT)
			{
				twr_state = TWR_REQ_ACK_RECVD;
				send_w4r_msg(peer, 1, MT_D_INIT);
			}
			else
			{
				handle_twr_fail();
			}
			break;
		case MT_D_INIT:
			if (!(peer == twr_peer && twr_state == TWR_REQ_RECVD))
			{
				handle_twr_fail();
			}
			twr_state = TWR_INIT_RECVD;
			loc_action = LOC_NO_RESP;
			break;
		case MT_D_RESP:
			if (peer == twr_peer && twr_state == TWR_REQ_ACK_RECVD)
			{
				twr_state = TWR_RESP_RECVD;
				compute_distance();
				//send_d_res(); // dly
				peer_info_t* peer_info = get_peer_info(recvd_header.src_addr);
				memcpy(send_buf, &(peer_info->calc_distance), sizeof(peer_info->calc_distance));

				send_w4r_msg(peer, sizeof(peer_info->calc_distance), MT_D_RES);
				twr_state = TWR_NO_TWR;
			}
			else
			{
				handle_twr_fail();
			}						
			break;
		case MT_D_FAIL:
			loc_state = LOC_COMM;
			twr_state = TWR_NO_TWR;
			twr_peer->seq_ack_n = twr_peer_seq;
			twr_fail_cnt++;
			twr_peer = NULL;
			send_d_req();
			break;
		case MT_D_RES:
			// NO BREAK
			if (peer->last_message_type == MT_D_RESP && twr_state == TWR_INIT_RECVD)
			{
				// Add returned distance if sensible
				toggle_led(blue);
				twr_fail_cnt = 0;
				memcpy(&distance, recv_buf, sizeof(distance));
				peer_info_t* peer_info = get_peer_info(recvd_header.src_addr);
				if (peer_info->recvd_distance > 0)
					peer_info->recvd_distance = (peer_info->recvd_distance)*NEIGHBOUR_NUM/(NEIGHBOUR_NUM+1) + distance/(NEIGHBOUR_NUM+1);
				else
					peer_info->recvd_distance = distance;

				twr_state = TWR_NO_TWR;
				peer->seq_ack_n ^= 0x10; // Flips 4 bit (ack_num) 

				//send_d_res_ack(peer);
				send_msg_meta.wtime = -1;
				send_msg_meta.dlytime = 0;
				send_msg_meta.size = 1;
				send_msg_meta.seq_ack_num = peer->seq_ack_n&0x10;
				send_msg_meta.type = MT_D_RES_ACK;
				send_msg_meta.addr = peer->peer_addr;
			}
			else
			{
				peer->ttl++;
				handle_twr_fail();
				if (peer->ttl >= PEER_CONN_TTL)
					disconnect_peer(peer);
			}
			break;
		case MT_D_RES_ACK:
			if (peer->last_message_type == MT_D_RES && twr_state == TWR_NO_TWR)
			{
				peer->seq_ack_n ^= 0x01; // Flips 0 bit (seq_num) 
				twr_state = TWR_NO_TWR;
				loc_state = LOC_COMM;
				loc_action = LOC_NO_RESP;
				twr_fail_cnt = 0;
			}
			else
			{
				peer->ttl++;
				handle_twr_fail();
				if (peer->ttl >= PEER_CONN_TTL)
					disconnect_peer(peer);
			}
		default:
			break;
	}
}

void conn_handle(peer_connection_t* peer)
{
	loc_action = LOC_NO_RESP;
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
						//send_d_req();
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

void no_resp_action(void)
{
	peer_connection_t* lowest_d_peer = get_conn_peer();
	peer_info_t* lowest_d_info;
	uint8_t lowest_d_measures = 255;
	
	if (lowest_d_peer)
	{
		lowest_d_info = get_peer_info(lowest_d_peer->peer_addr);
		lowest_d_measures = lowest_d_info->d_measures;
	}
		
	double shortest_timeout = 10;
	uint8_t possible_conn_n = (current_peer_n - current_peer_c_n) > 0;
	uint8_t broad_send = messages_since_broad >= (uint8_t)(200*((float)current_peer_n/(float)NEIGHBOUR_NUM)) || recv_tmo_cnt > 5;

	if (twr_state == TWR_NO_TWR || recv_tmo_cnt > 5)
	{
		if (broad_send)
		{
			recv_tmo_usec = (rand()&0xF000)+40000;
			send_broad();
			messages_since_broad = 0;
			recv_tmo_cnt = 0;
		}
		else if (possible_conn_n)
			send_syn();
		// else if (shortest_timeout < CONN_MSG_TMO_MAX)
		// 	send_maintain();
		else if (lowest_d_measures < MIN_D_MEASURES)
			send_d_req();
		else
		{
			recv_tmo_usec = (rand()&0xF000)+40000;
			send_msg_meta = send_msg_meta_def;
		}
	}
}

void process_message(void)
{
	uint8_t is_timeout = 0;
	uint8_t peer_valid = 0;
	peer_connection_t* peer = NULL;

	loc_action = LOC_NO_RESP;

	if ((recvd_type == MT_D_REQ || recvd_type == MT_D_REQ_ACK))
	{
		process_req();
		if (*((uint16_t*)recv_buf) != panadr_own.short_addr)
		{
			recv_tmo_usec = (rand()&0xF000)+40000;
			loc_action = LOC_STOP;
			send_msg_meta = send_msg_meta_def;
		}
	}
	if(loc_action != LOC_STOP)
	{
		if (recvd_type == 0)
		{
			recv_tmo_cnt++;
			//chprintf((BaseSequentialStream*)&SD1, "timeout\n");
			if (loc_state == LOC_TWR)
				handle_twr_fail();
		}
		else
		{
			//chprintf((BaseSequentialStream*)&SD1, "recv: %d\n", recvd_type);
			peer = get_peer(recvd_header.src_addr);
			if (peer != NULL && peer->ttl < PEER_CONN_TTL)
			{
				peer_valid = 1;
				chVTReset(&(peer->tmo_timer));
				chVTSet(&(peer->tmo_timer), TIME_S2I(5), inc_ttl, peer);
			}
				
			if ((recvd_type&0xF0) == 0x20)
			{
				if (peer_valid)
					twr_handle(peer);
				else
					handle_twr_fail();
			}
			else
			{
				if ((recvd_type&0xF0) == 0x10 /*|| some_peer_recv_ack_timeout*/)
					conn_handle(peer);
				else if ((recvd_type&0xF0) == 0x30 && peer_valid)
					conn_handle(peer);
				// else
				// 	invalid_message_handle
			}
		}
	}

	// if (loc_action == LOC_ACT_ERR)
	// 	reset // how? what?

	//new_state();
	clean_recvd();

	if (loc_action == LOC_NO_RESP || recv_tmo_cnt > 5)
		no_resp_action();
	// else
	// {
	// 	if (loc_action != LOC_RESP_NOW)
	// 		resp_action();
	// }

	// chVTReset(&comm_watchdog);
	// chVTSet(&comm_watchdog, CH_TIMEOUT, reset_comms, NULL);
	//chprintf((BaseSequentialStream*)&SD1, "send: %d\n", send_msg_meta.type);
}

THD_FUNCTION(COMMS, arg)
{
	(void)arg;

	comm_thread = chThdGetSelfX();
	comm_state_t comm_state = COMM_RECV;
	eventmask_t evt = 0;
	int32_t message_ret = 0;
	recv_tmo_usec = (rand()&0xF000)+40000;

	init_peers();

	barrier();

	chMtxLock(&dw_mutex);
	chEvtWaitOne(DW_COMM_OK_E);
	chMtxUnlock(&dw_mutex);

	while (true)
	{
		switch (comm_state)
		{
			case COMM_RECV:
				chMtxLock(&dw_mutex);
				dw_ctrl_req = DW_RECV;
				if (send_msg_meta.wtime >= 0 && send_msg_meta.type != 0)
				{
					dw_ctrl_req = DW_SEND_W4R;
					prepare_message();
				}
				chMtxUnlock(&dw_mutex);
				chEvtSignal(dw_thread, DW_COMM_OK_E);
				evt = chEvtWaitOneTimeout(DW_COMM_OK_E, CH_TIMEOUT);
				if (evt == DW_COMM_OK_E)
				{
					message_ret = get_message();
					comm_state = COMM_IDLE;
				}
				else
					comm_state = COMM_ERR;
				break;
			case COMM_SEND:
				chMtxLock(&dw_mutex);
				dw_ctrl_req = DW_SEND;
				if (send_msg_meta.dlytime > 0)
					dw_ctrl_req = DW_SEND_DLY;
				chMtxUnlock(&dw_mutex);
				prepare_message();
				chEvtSignal(dw_thread, DW_COMM_OK_E);
				evt = chEvtWaitOneTimeout(DW_COMM_OK_E, CH_TIMEOUT);
				if (evt == DW_COMM_OK_E)
					comm_state = COMM_RECV;
				else
					comm_state = COMM_ERR;
				break;
			case COMM_ERR:
				// Thread not responding reset thread?
				comm_state = COMM_IDLE;
				break;
			case COMM_IDLE:
				recv_tmo_usec = 50000;
				process_message();
				comm_state = COMM_RECV;
				if (send_msg_meta.type != 0 && send_msg_meta.wtime < 0)
					comm_state = COMM_SEND;
				break;
			default:
				break;
		}
	}
}

int8_t respond_if_twr(void)
{
	recvd_type = recv_buf[sizeof(recvd_header)];
	if (recvd_type != MT_D_INIT || twr_state != TWR_REQ_RECVD)
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
	uint64_t delay_tx = (uint64_t)rx_time + (uint64_t)(65536*4000);
	uint64_t tx_time = (uint64_t)delay_tx +(uint64_t)tx_antd;

	memcpy(send_buf, &tx_time, sizeof(tx_time));
	memcpy(send_buf+sizeof(tx_time), &rx_time, sizeof(rx_time));

	send_msg_meta.size = sizeof(tx_time)+sizeof(rx_time);
	send_msg_meta.seq_ack_num = 0;
	send_msg_meta.type = MT_D_RESP;
	send_msg_meta.addr = recvd_header.src_addr;

	prepare_message();
	tx_ctrl.TFLEN = send_size+2; // TODO magic

	dx_time.time32 = (uint32_t)(delay_tx >> 8);
	dw_start_tx(tx_ctrl, send_buf, dx_time, w4r);
	evt = chEvtWaitOneTimeout(MTXFRS_E, TX_TIMEOUT);
	if (evt != MTXFRS_E)
	{
		twr_state = TWR_FAIL;
		return -1;
	}
	return 1;
		
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
	uint16_t rx_ant_d = 0;
	dw_write(DW_REG_INFO.LDE_CTRL, (uint8_t*)(&rx_ant_d), DW_SUBREG_INFO.LDE_RXANTD.size, DW_SUBREG_INFO.LDE_RXANTD.offset);
	dw_write(DW_REG_INFO.TX_ANTD, (uint8_t*)(&tx_antd), DW_REG_INFO.TX_ANTD.size, 0);
}

THD_FUNCTION(DW_CONTROLLER, arg)
{
	(void)arg;

	dw_setup();

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

	chMtxObjectInit(&dw_mutex);
	dw_thread = chThdGetSelfX();
	set_irq_vector();

	chMtxLock(&dw_mutex);

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
				clean_send();
				break;
			case DW_SEND_W4R:
				w4r.W4R_TIM = send_msg_meta.wtime;
				w4r.ACK_TIM = 1;
				dw_start_tx(tx_ctrl, send_buf, dx_time, w4r);
				evt = chEvtWaitOneTimeout(MTXFRS_E, TX_TIMEOUT);
				if (evt != MTXFRS_E)
					dw_ctrl_req = DW_TRX_ERR;
				else
				{
					evt = chEvtWaitOneTimeout(MRXFCG_E | MRXERR_E, TIME_US2I(recv_tmo_usec));
					if (evt == MRXFCG_E)
					{
						twr_ret = respond_if_twr();
						if (twr_ret < 0)
							dw_ctrl_req = DW_TRX_ERR;
						dw_ctrl_req = DW_CTRL_YIELD;
					}
					else if (evt == 0)
						dw_ctrl_req = DW_RECV_TMO;
					else
						dw_ctrl_req = DW_TRX_ERR;
					dw_soft_reset_rx();
				}
				clean_send();
				break;
			case DW_SEND_DLY:
				dx_time.time32 = send_msg_meta.dlytime;
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
				chMtxUnlock(&dw_mutex);
				chEvtSignal(comm_thread, DW_COMM_OK_E);
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
				dw_setup();
				rst_cnt++;
				dw_ctrl_req = DW_CTRL_YIELD;
				chEvtSignal(comm_thread, DW_COMM_OK_E);
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

void update_peer_pos(void)
{
	float d = peers_info[0].calc_distance;
	float d0 = get_distance(panadr_own.short_addr, peers[1].peer_addr);
	float d1 = get_distance(peers[0].peer_addr, peers[1].peer_addr);

	d = 4;
	d0 = (float)sqrt(13.0);
	d1 = (float)sqrt(13.0);

	peer_positions[1][0] = d;

	peer_positions[2][0] = (d*d+d0*d0-d1*d1)/(2*d);
	peer_positions[2][1] = (float)(sqrt((double)(d0*d0 - (peer_positions[2][0])*(peer_positions[2][0]))));
}

THD_FUNCTION(SYSTEM_STATUS, arg)
{
	(void)arg;

	barrier_init(3);

	//chVTObjectInit(&comm_watchdog);
	init_d_m();

	peer_positions[0][0] = 0;
	peer_positions[0][1] = 0;
	peer_positions[0][2] = 0;
	peer_positions[1][1] = 0;
	peer_positions[1][2] = 0;
	peer_positions[2][2] = 0;

	sdStart(&SD1, &serial_cfg);

	barrier();
	euclidean_d_m.addrs[0] = panadr_own.short_addr;

	//chVTSet(&comm_watchdog, CH_TIMEOUT, reset_comms, NULL);
	while (true)
	{
		for (uint8_t i = 1; i < NEIGHBOUR_NUM+1; i++)
		{
			euclidean_d_m.addrs[i] = peers[i-1].peer_addr;
			euclidean_d_m.distances[0][i] = peers_info[i-1].calc_distance;
			euclidean_d_m.distances[i][0] = peers_info[i-1].recvd_distance;
		}

		update_peer_pos();

		for (uint8_t i = 0; i < NEIGHBOUR_NUM; i++)
		{
			peers_info[i].d_measures = 0;
			if (peers[i].ttl >= PEER_CONN_TTL)
				disconnect_peer(peers+i);
			// chprintf((BaseSequentialStream*)&SD1, "peer_addr: %d\n", peers[i].peer_addr);
			// chprintf((BaseSequentialStream*)&SD1, "peer_seq_ack: %x\nttl: %d\n", peers[i].seq_ack_n, peers[i].ttl);
			// chprintf((BaseSequentialStream*)&SD1, "d: %d\n", (int)peers_info[i].distance);
		}

		for (uint8_t i = 1; i < NEIGHBOUR_NUM+1; i++)
		{
			chprintf((BaseSequentialStream*)&SD1, "(%d,%d,%d)\n", (int)peer_positions[0][0], (int)peer_positions[0][1], (int)peer_positions[0][2]);
			chprintf((BaseSequentialStream*)&SD1, "(%d,%d,%d)\n", (int)peer_positions[1][0], (int)peer_positions[1][1], (int)peer_positions[1][2]);
			chprintf((BaseSequentialStream*)&SD1, "(%d,%d,%d)\n", (int)peer_positions[2][0], (int)peer_positions[2][1], (int)peer_positions[2][2]);
		}

		chprintf((BaseSequentialStream*)&SD1, "\n\t| ");

		for (uint8_t j = 0; j < NEIGHBOUR_NUM+1; j++)
		{
			chprintf((BaseSequentialStream*)&SD1, "%d\t", (int)euclidean_d_m.addrs[j]);
			chprintf((BaseSequentialStream*)&SD1, "| ");
		}
		chprintf((BaseSequentialStream*)&SD1, "\n");
		chprintf((BaseSequentialStream*)&SD1, "-------------------------------------\n");

		for (uint8_t i = 0; i < NEIGHBOUR_NUM+1; i++)
		{
			chprintf((BaseSequentialStream*)&SD1, "%d\t", (int)euclidean_d_m.addrs[i]);
			chprintf((BaseSequentialStream*)&SD1, "| ");

			for (uint8_t j = 0; j < NEIGHBOUR_NUM+1; j++)
			{
				chprintf((BaseSequentialStream*)&SD1, "%d\t", (int)euclidean_d_m.distances[i][j]);
				chprintf((BaseSequentialStream*)&SD1, "| ");
			}

			chprintf((BaseSequentialStream*)&SD1, "\n");
			chprintf((BaseSequentialStream*)&SD1, "-------------------------------------\n");
		}

		chprintf((BaseSequentialStream*)&SD1, "\nXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");

		chThdSleepMilliseconds(2000);
	}
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

