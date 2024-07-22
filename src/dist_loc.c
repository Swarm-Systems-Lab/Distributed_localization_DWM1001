// /**
//  *
//  * This program is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, version 2.
//  *
//  * This program is distributed in the hope that it will be useful, but
//  * WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//  * General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with this program. If not, see <http://www.gnu.org/licenses/>.
//  *
//  */

// #include "dist_loc.h"

// semaphore_t barrier_sem;
// mutex_t barrier_mutex;
// uint8_t barrier_cnt = 0;
// uint8_t barrier_num;

// peer_connection_t peers[NEIGHBOUR_NUM];
// peer_info_t peers_info[NEIGHBOUR_NUM];
// uint8_t current_peer_n = 0;
// uint8_t current_peer_c_n = 0;

// peer_connection_t* twr_peer = NULL;
// uint8_t twr_peer_seq = 0;

// uint8_t messages_since_broad = 0;
// uint8_t twr_fail_cnt = 0;

// euclidean_d_m_t euclidean_d_m;
// float peer_positions[NEIGHBOUR_NUM+1][3];

// // void barrier(void)
// // {
// // 	uint8_t release = 1;
// // 	chMtxLock(&barrier_mutex);
// // 	barrier_cnt++;
// // 	if (barrier_cnt == barrier_num)
// // 	{
// // 		for (uint8_t i = 0; i < barrier_num; i++)
// // 			chSemSignal(&barrier_sem);
// // 		barrier_cnt = 0;
// // 		release = 0;
// // 	}	
// // 	chMtxUnlock(&barrier_mutex);
// // 	if (release)
// // 		chSemWait(&barrier_sem);
// // }

// // void barrier_init(uint8_t n)
// // {
// // 	barrier_num = n;
// // 	chMtxObjectInit(&barrier_mutex);
// // 	chSemObjectInit(&barrier_sem, 0);
// // 	barrier_cnt = 0;
// // }

// // Function to initialize the matrix
// void init_d_m(void) 
// {
// 	// Set all distances to -1 (except for self-distances)
// 	for (uint8_t i = 0; i < NEIGHBOUR_NUM + 1; i++) 
// 	{
// 		euclidean_d_m.addrs[i] = 0;
// 		for (uint8_t j = 0; j < NEIGHBOUR_NUM + 1; j++) 
// 			euclidean_d_m.distances[i][j] = MIN_DIST;

// 		euclidean_d_m.distances[i][i] = 0.0f;
// 	}
// }

// int32_t get_message(void)
// {
// 	if (recv_size == 0)
// 		return 0;

// 	recv_tmo_cnt = 0;
// 	recvd_header = decode_MHR(recv_buf);
// 	recvd_type = recv_buf[sizeof(recvd_header)];

// 	if (recvd_header.frame_control.mask != def_frame_ctrl.mask)
// 		return 0;

// 	create_new_peer(recvd_header.src_addr);

// 	memset(recv_buf+recv_size, 0, sizeof(recv_buf)-recv_size);
// 	memmove(recv_buf, recv_buf+sizeof(recvd_header)+1, sizeof(recv_buf)-sizeof(recvd_header)-1);

// 	return 1;
// }

// void prepare_message(void)
// {
// 	MHR_16_t send_header;
// 	peer_connection_t* peer = NULL;

// 	if (send_msg_meta.type == MT_D_REQ || send_msg_meta.type == MT_D_REQ_ACK)
// 		peer = get_peer(*((uint16_t*)send_buf));
// 	else
// 		peer = get_peer(send_msg_meta.addr);

// 	if (send_msg_meta.type != MT_BROADCAST && messages_since_broad < 255)
// 		messages_since_broad++;

// 	if (peer)
// 	{
// 		if (send_msg_meta.type > MT_ACK)
// 		{
// 			memcpy(peer->last_message, send_buf, send_msg_meta.size);
// 			peer->last_message_size = send_msg_meta.size;
// 			peer->last_message_type = send_msg_meta.type;
// 		}
// 		peer->last_cmd_type = send_msg_meta.type;
// 	}

// 	send_header.frame_control = def_frame_ctrl;
// 	send_header.dest_addr = send_msg_meta.addr;
// 	send_header.src_addr = panadr_own.short_addr;
// 	send_header.dest_pan_id = panadr_own.pan_id;
// 	send_header.seq_num = send_msg_meta.seq_ack_num;
// 	memmove(send_buf+sizeof(send_header)+1, send_buf, sizeof(send_buf)-sizeof(send_header)-1);
// 	memcpy(send_buf, &send_header, sizeof(send_header));
// 	send_buf[sizeof(send_header)] = send_msg_meta.type;
// 	send_size = send_msg_meta.size + sizeof(send_header)+1;
// }

// void init_peers(void)
// {
// 	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
// 	{
// 		peers[i].peer_addr = 0;
// 		peers[i].seq_ack_n = 0;
// 		peers[i].ttl = 255;
// 		chVTObjectInit(&(peers[i].tmo_timer));
// 		peers[i].last_message_size = 0;
// 		peers[i].last_message_type = 0;
// 		peers_info[i].conn = peers+i;
// 		peers_info[i].calc_distance = MIN_DIST;
// 		peers_info[i].recvd_distance = MIN_DIST;
// 		peers_info[i].d_measures = 0;
// 		peers_info[i].peer_id = i;
// 	}
// 	current_peer_n = 0;
//  	current_peer_c_n = 0;
// }

// peer_connection_t* create_new_peer(uint16_t addr)
// {
// 	toggle_led(green);

// 	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
// 	{
// 		if (peers[i].peer_addr == addr)
// 			return NULL;
// 		if (peers[i].peer_addr == 0)
// 		{
// 			peers[i].peer_addr = addr;
// 			current_peer_n++;
// 			return peers+i;
// 		}
// 	}
		
// 	return NULL;
// }

// peer_connection_t* get_peer(uint16_t addr)
// {
// 	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
// 	{
// 		if (peers[i].peer_addr == addr)
// 			return peers+i;
// 	}
// 	return NULL;
// }

// peer_info_t* get_peer_info(uint16_t addr)
// {
// 	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
// 	{
// 		if (peers_info[i].conn->peer_addr == addr)
// 			return peers_info+i;
// 	}
// 	return NULL;
// }

// void peer_tmo_cb(virtual_timer_t* vtp, void* arg)
// {
// 	disconnect_peer((peer_connection_t*)arg);
// }

// void connect_peer(peer_connection_t* peer)
// {
// 	peer->seq_ack_n = 0x11;
// 	current_peer_c_n++;
// 	peer->ttl = 0;
// 	chVTSet(&(peer->tmo_timer), TIME_S2I(5), peer_tmo_cb, peer);
// }

// void disconnect_peer(peer_connection_t* peer)
// {
// 	peer_info_t* peer_info = get_peer_info(peer->peer_addr);
// 	if (peer->ttl < 255)
// 	{
// 		peer_info->calc_distance = MIN_DIST;
// 		peer_info->recvd_distance = MIN_DIST;
// 		peer_info->d_measures = 0;
// 		peer->peer_addr = 0;
// 		peer->seq_ack_n = 0;
// 		peer->last_message_size = 0;
// 		peer->last_message_type = 0;
// 		peers->ttl = 255;
// 		current_peer_c_n--;
// 		current_peer_n--;
// 	}
// }

// peer_connection_t* get_unconn_peer(void)
// {
// 	if ((current_peer_n-current_peer_c_n) <= 0)
// 		return NULL;

// 	uint8_t rnd_peer = rand() / (RAND_MAX/(current_peer_n-current_peer_c_n));
// 	int16_t tried_cnt = -1;

// 	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
// 	{
// 		if (peers[i].peer_addr > 0 && peers[i].ttl == 255)
// 			tried_cnt++;
// 		if (tried_cnt == rnd_peer)
// 			return peers+i;
// 	}

// 	return NULL;
// }

// peer_connection_t* get_conn_peer(void)
// {
// 	if (current_peer_c_n == 0)
// 		return NULL;

// 	uint32_t d_measures_min = 1<<(sizeof(d_measures_min)-1);
// 	peer_connection_t* peer_min = NULL;

// 	for (size_t i = 0; i < NEIGHBOUR_NUM; i++)
// 	{
// 		if (peers[i].peer_addr > 0 && peers[i].ttl < PEER_CONN_TTL)
// 		{
// 			if (peers_info[i].d_measures <= d_measures_min)
// 			{
// 				d_measures_min = peers_info[i].d_measures;
// 				peer_min = peers+i;
// 			}
// 		}
// 	}

// 	return peer_min;
// }


// void clean_recvd(void)
// {
// 	memset(&recvd_header, 0, sizeof(recvd_header));
// 	memset(recv_buf, 0, recv_size);

// 	recvd_type = 0;
// }

// void clean_send(void)
// {
// 	memset(send_buf, 0, send_size);
// 	send_msg_meta = send_msg_meta_def;
// }

// void send_syn(void)
// {
// 	peer_connection_t* peer = get_unconn_peer();

// 	if (peer)
// 		send_conn_msg(peer, 1, MT_SYN);
// }

// void send_broad(void)
// {
// 	send_msg_meta = send_msg_meta_def;

// 	send_msg_meta.size = 1;
// 	send_msg_meta.seq_ack_num = 0;
// 	send_msg_meta.type = MT_BROADCAST;
// 	send_msg_meta.addr = 0xFFFF;
// }

// void send_conn_msg(peer_connection_t* peer, uint8_t size, message_t type)
// {
// 	if (peer->ttl > PEER_CONN_TTL && send_msg_meta.type > MT_ACK)
// 		return;
	
// 	send_msg_meta = send_msg_meta_def;

// 	send_msg_meta.size = size;
// 	send_msg_meta.seq_ack_num = peer->seq_ack_n&0x01;
// 	send_msg_meta.type = type;
// 	send_msg_meta.addr = peer->peer_addr;

// 	loc_action = LOC_RESP_BTMO;
// }

// void send_w4r_msg(peer_connection_t* peer, uint8_t size, message_t type)
// {	
// 	send_msg_meta.wtime = 0;
// 	send_msg_meta.dlytime = 0;
// 	send_msg_meta.size = size;
// 	send_msg_meta.seq_ack_num = peer->seq_ack_n&0x01;
// 	send_msg_meta.type = type;
// 	send_msg_meta.addr = peer->peer_addr;

// 	loc_action = LOC_RESP_BTMO;
// }

// void send_ack(peer_connection_t* peer)
// {
// 	send_msg_meta = send_msg_meta_def;

// 	send_msg_meta.size = 1;
// 	send_msg_meta.seq_ack_num = peer->seq_ack_n&0x10;
// 	send_msg_meta.type = MT_ACK;
// 	send_msg_meta.addr = peer->peer_addr;
	
// 	loc_action = LOC_RESP_BTMO;
// }

// void send_last_message(peer_connection_t* peer)
// {
// 	memcpy(send_buf, peer->last_message, peer->last_message_size);
// 	send_conn_msg(peer, peer->last_message_size, peer->last_message_type);

// 	loc_action = LOC_RESP_BTMO;
// }

// void send_d_req(void)
// {
// 	peer_connection_t* peer = get_conn_peer();

// 	if (peer)
// 	{
// 		send_msg_meta.wtime = 0;
// 		send_msg_meta.dlytime = 0;
// 		send_msg_meta.size = sizeof(peer->peer_addr)+sizeof(euclidean_d_m);
// 		send_msg_meta.seq_ack_num = peer->seq_ack_n&0x01;
// 		send_msg_meta.type = MT_D_REQ;
// 		send_msg_meta.addr = 0xFFFF;

// 		// TODO memory danger if neigh_num too large
// 		memcpy(send_buf, &(peer->peer_addr), sizeof(peer->peer_addr));
// 		memcpy(send_buf+sizeof(peer->peer_addr), &euclidean_d_m, sizeof(euclidean_d_m));

// 		loc_state = LOC_TWR;
// 		twr_state = TWR_REQ_SENT;
// 		twr_peer = peer;
// 		twr_peer_seq = peer->seq_ack_n;
// 	}
// }

// void send_d_req_ack(peer_connection_t* peer)
// {
// 	send_msg_meta.wtime = 0;
// 	send_msg_meta.dlytime = 0;
// 	send_msg_meta.size = sizeof(peer->peer_addr)+sizeof(euclidean_d_m);
// 	send_msg_meta.seq_ack_num = peer->seq_ack_n&0x10;
// 	send_msg_meta.type = MT_D_REQ_ACK;
// 	send_msg_meta.addr = 0xFFFF;

// 	memcpy(send_buf, &(peer->peer_addr), sizeof(peer->peer_addr));
// 	memcpy(send_buf+sizeof(peer->peer_addr), &euclidean_d_m, sizeof(euclidean_d_m));
// }

// void process_req(void)
// {
// 	euclidean_d_m_t* m = (euclidean_d_m_t*)(recv_buf+sizeof(panadr_own.short_addr));

// 	for (uint8_t i = 0; i < NEIGHBOUR_NUM+1; i++)
// 	{
// 		if (m->addrs[i] != panadr_own.short_addr)
// 		{
// 			if (m->addrs[i] == recvd_header.src_addr)
// 			{
// 				for (uint8_t j = 0; j < NEIGHBOUR_NUM+1; j++)
// 					set_distance(m->addrs[i], m->addrs[j], m->distances[i][j]);
// 			}
// 			// else
// 			// {
// 			// 	for (uint8_t j = 0; j < NEIGHBOUR_NUM+1; j++)
// 			// 		euclidean_d_m.distances[i][j] = m->distances[i][j];
// 			// }
// 		}
// 	}
// }

// void update_peer_distance(peer_info_t* peer_info, float dist)
// {
// 	if (dist+1e-6 < MIN_DIST || dist >= MAX_DIST)
// 		return;
	
// 	if (peer_info->calc_distance == MIN_DIST || peer_info->d_measures == 0)
// 		peer_info->calc_distance = dist;
// 	else
// 		peer_info->calc_distance = (peer_info->calc_distance)*NEIGHBOUR_NUM/(NEIGHBOUR_NUM+1) + dist/(NEIGHBOUR_NUM+1);

// 	peer_info->d_measures++;
// 	chprintf((BaseSequentialStream*)&SD1, "A:%d\n", (int)(dist*10000.0));
// }

// void compute_distance(void)
// {
// 	toggle_led(blue);
// 	uint64_t m_tx_time, m_rx_time;
// 	uint64_t tx_time = dw_get_tx_time();
// 	uint64_t rx_time = 0;
// 	memcpy(&rx_time, recv_info.dw_rx_time.RX_STAMP, sizeof(recv_info.dw_rx_time.RX_STAMP));

// 	uint8_t lde_stat = 0;
// 	memcpy(&lde_stat, recv_buf+sizeof(m_tx_time)+sizeof(m_rx_time), sizeof(lde_stat));
// 	if (!rx_time || lde_stat)
// 	{
// 		memcpy(&rx_time, recv_info.dw_rx_time.RX_RAWST, sizeof(recv_info.dw_rx_time.RX_RAWST));
// 		rx_time -= rx_ant_d;

// 		rx_time = tx_time;
// 	}

// 	memcpy(&m_tx_time, recv_buf, sizeof(m_tx_time));
// 	memcpy(&m_rx_time, recv_buf+sizeof(m_tx_time), sizeof(m_rx_time));

// 	int64_t rt_init = (rx_time) - (tx_time);
// 	int64_t rt_resp = (m_tx_time) - (m_rx_time);

// 	float clock_offset_r = dw_get_car_int() * CAR_INT_CTE;
// 	rt_resp *= (1.0f - clock_offset_r);

// 	double tof = (rt_init - rt_resp)/2.0f;
// 	tof *= (1.0f/(float)DW_TIME_U);

// 	double distance = tof * C_ATM;

// 	peer_info_t* peer_info = get_peer_info(recvd_header.src_addr);
	
// 	if (peer_info)
// 		update_peer_distance(peer_info, (float)distance);
// }

// int8_t _get_address_index(uint16_t addr)
// {
// 	for (uint8_t i = 0; i < NEIGHBOUR_NUM + 1; i++) 
// 	{
// 		if (euclidean_d_m.addrs[i] == addr) 
// 			return i;
// 	}
// 	return -1; // Address not found
// }

// float get_distance(uint16_t addr1, uint16_t addr2)
// {
//  	int8_t index1 = _get_address_index(addr1);
//   	int8_t index2 = _get_address_index(addr2);

// 	if (index1 == -1 || index2 == -1) 
//     	return -1.0f; // Indicate invalid address

// 	return euclidean_d_m.distances[index1][index2];
// }

// void set_distance(uint16_t addr1, uint16_t addr2, float distance)
// {
//   	int8_t index1 = _get_address_index(addr1);
//   	int8_t index2 = _get_address_index(addr2);

//   	if (index1 == -1 || index2 == -1)
// 		return; // Do nothing if address is invalid

// 	if (distance > MIN_DIST)
// 		euclidean_d_m.distances[index1][index2] = distance;
// }

// void handle_twr_fail(void)
// {
// 	loc_state = LOC_COMM;
// 	twr_state = TWR_NO_TWR;
// 	twr_peer->seq_ack_n = twr_peer_seq;
// 	// EXPONENTIAL BACKOFF !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 	send_conn_msg(twr_peer, 1, MT_D_FAIL);
// 	twr_peer = NULL;
// 	twr_fail_cnt++;
// }

// void twr_handle(peer_connection_t* peer)
// {
// 	float distance = 0.0;

// 	if (twr_state != TWR_NO_TWR && peer != twr_peer)
// 		recvd_type = 0;

// 	loc_action = LOC_RESP_NOW;

// 	distance = MIN_DIST;
// 	switch (recvd_type)
// 	{
// 		case MT_D_REQ:
// 			if (/*((peer->seq_ack_n&0x10)>>4) == (recvd_header.seq_num&0x01) &&*/ (twr_state == TWR_NO_TWR || twr_state == TWR_REQ_SENT))
// 			{
// 				if (twr_state == TWR_REQ_SENT && peer->peer_addr < panadr_own.short_addr)
// 				{
// 					loc_action = LOC_NO_RESP;
// 				}
// 				else
// 				{
// 					loc_state = LOC_TWR;
// 					twr_state = TWR_REQ_RECVD;
// 					twr_peer = peer;
// 					twr_peer_seq = peer->seq_ack_n;

// 					send_d_req_ack(peer);
// 				}
// 			}
// 			else
// 			{
// 				handle_twr_fail();
// 			}
// 			break;
// 		case MT_D_REQ_ACK:
// 			if (peer->last_message_type == MT_D_REQ && twr_state == TWR_REQ_SENT)
// 			{
// 				twr_state = TWR_REQ_ACK_RECVD;
// 				send_w4r_msg(peer, 1, MT_D_INIT);
// 			}
// 			else
// 			{
// 				handle_twr_fail();
// 			}
// 			break;
// 		case MT_D_INIT:
// 			if (!(peer == twr_peer && twr_state == TWR_REQ_RECVD))
// 			{
// 				handle_twr_fail();
// 			}
// 			twr_state = TWR_INIT_RECVD;
// 			loc_action = LOC_NO_RESP;
// 			break;
// 		case MT_D_RESP:
// 			if (peer == twr_peer && twr_state == TWR_REQ_ACK_RECVD)
// 			{
// 				twr_state = TWR_RESP_RECVD;
// 				compute_distance();
// 				peer_info_t* peer_info = get_peer_info(recvd_header.src_addr);
// 				memcpy(send_buf, &(peer_info->calc_distance), sizeof(peer_info->calc_distance));

// 				send_w4r_msg(peer, sizeof(peer_info->calc_distance), MT_D_RES);
// 				twr_state = TWR_NO_TWR;
// 			}
// 			else
// 			{
// 				handle_twr_fail();
// 			}						
// 			break;
// 		case MT_D_FAIL:
// 			loc_state = LOC_COMM;
// 			twr_state = TWR_NO_TWR;
// 			twr_peer->seq_ack_n = twr_peer_seq;
// 			twr_fail_cnt++;
// 			twr_peer = NULL;
// 			send_d_req();
// 			break;
// 		case MT_D_RES:
// 			// NO BREAK
// 			if (peer->last_message_type == MT_D_RESP && twr_state == TWR_INIT_RECVD)
// 			{
// 				// Add returned distance if sensible
// 				toggle_led(blue);
// 				twr_fail_cnt = 0;
// 				memcpy(&distance, recv_buf, sizeof(distance));
// 				peer_info_t* peer_info = get_peer_info(recvd_header.src_addr);
// 				if (peer_info->recvd_distance > 0)
// 					peer_info->recvd_distance = (peer_info->recvd_distance)*NEIGHBOUR_NUM/(NEIGHBOUR_NUM+1) + distance/(NEIGHBOUR_NUM+1);
// 				else
// 					peer_info->recvd_distance = distance;

// 				chprintf((BaseSequentialStream*)&SD1, "B:%d\n", (int)(distance*10000.0));

// 				twr_state = TWR_NO_TWR;
// 				peer->seq_ack_n ^= 0x10; // Flips 4 bit (ack_num) 

// 				//send_d_res_ack(peer);
// 				send_msg_meta.wtime = -1;
// 				send_msg_meta.dlytime = 0;
// 				send_msg_meta.size = 1;
// 				send_msg_meta.seq_ack_num = peer->seq_ack_n&0x10;
// 				send_msg_meta.type = MT_D_RES_ACK;
// 				send_msg_meta.addr = peer->peer_addr;
// 			}
// 			else
// 			{
// 				peer->ttl++;
// 				handle_twr_fail();
// 				if (peer->ttl >= PEER_CONN_TTL)
// 					disconnect_peer(peer);
// 			}
// 			break;
// 		case MT_D_RES_ACK:
// 			if (peer->last_message_type == MT_D_RES && twr_state == TWR_NO_TWR)
// 			{
// 				peer->seq_ack_n ^= 0x01; // Flips 0 bit (seq_num) 
// 				twr_state = TWR_NO_TWR;
// 				loc_state = LOC_COMM;
// 				loc_action = LOC_NO_RESP;
// 				twr_fail_cnt = 0;
// 			}
// 			else
// 			{
// 				peer->ttl++;
// 				handle_twr_fail();
// 				if (peer->ttl >= PEER_CONN_TTL)
// 					disconnect_peer(peer);
// 			}
// 		default:
// 			break;
// 	}
// }

// void conn_handle(peer_connection_t* peer)
// {
// 	loc_action = LOC_NO_RESP;
// 	switch (recvd_type)
// 	{
// 		case MT_SYN:
// 			if (peer->ttl < PEER_CONN_TTL)
// 				disconnect_peer(peer);
// 			else
// 			{
// 				peer->seq_ack_n = 0x10;
// 				send_conn_msg(peer, 1, MT_SYN_ACK);
// 			} 
// 			break;
// 		case MT_SYN_ACK:
// 			if (peer->ttl < PEER_CONN_TTL)
// 				disconnect_peer(peer);
// 			else
// 			{
// 				if (peer->last_cmd_type == MT_SYN)
// 				{
// 					connect_peer(peer);
// 					send_ack(peer);
// 				}
// 				else
// 					disconnect_peer(peer);
// 			}
// 			break;
// 		case MT_ACK:
// 			if ((peer->seq_ack_n&0x01) == !((recvd_header.seq_num&0x10)>>4)) // seq num == not ack_num
// 			{
// 				// Make connection
// 				if (peer->last_cmd_type == MT_SYN_ACK)
// 				{
// 					if (peer->ttl == 255)
// 					{
// 						connect_peer(peer);
// 						//send_d_req();
// 					}
// 				}
// 				else
// 				{
// 					if (peer->ttl < PEER_CONN_TTL)
// 						peer->seq_ack_n ^= 0x01; // Flips 0 bit (seq_num)  
// 				}
// 				// TODO check multiple positive acks (not suppose to happen)
// 			}
// 			else
// 			{
// 				if (peer->ttl < PEER_CONN_TTL)
// 				{
// 					send_last_message(peer);
// 					peer->ttl++;
// 				}
// 				else
// 					disconnect_peer(peer);
// 			}
// 			break;
// 		case MT_MCONN:
// 			if (((peer->seq_ack_n&0x10)>>4) == (recvd_header.seq_num&0x01)  && peer->ttl < PEER_CONN_TTL)
// 				peer->seq_ack_n ^= 0x10; // Flips 4 bit (ack_num) 
// 			else
// 			{
// 				peer->ttl++;
// 				if (peer->ttl >= PEER_CONN_TTL)
// 					disconnect_peer(peer);
// 			}
// 			send_ack(peer);
// 			break;
// 		default:
// 			break;
// 	}
// }

// loc_action_t no_resp_action(void)
// {
// 	peer_connection_t* lowest_d_peer = get_conn_peer();
// 	peer_info_t* lowest_d_info;
// 	uint32_t lowest_d_measures = 1<<(sizeof(uint32_t)-1);
	
// 	if (lowest_d_peer)
// 	{
// 		lowest_d_info = get_peer_info(lowest_d_peer->peer_addr);
// 		lowest_d_measures = lowest_d_info->d_measures;
// 	}
		
// 	double shortest_timeout = 10;
// 	uint8_t possible_conn_n = (current_peer_n - current_peer_c_n) > 0;
// 	uint8_t broad_send = messages_since_broad >= (uint8_t)(200*((float)current_peer_n/(float)NEIGHBOUR_NUM)) || recv_tmo_cnt > 5;

// 	if (twr_state == TWR_NO_TWR || recv_tmo_cnt > 5)
// 	{
// 		if (broad_send)
// 		{
// 			return LOC_SEND_BROAD;
// 			messages_since_broad = 0;
// 			recv_tmo_cnt = 0;
// 		}
// 		else if (possible_conn_n)
// 			return LOC_SEND_SYN;
// 		// else if (shortest_timeout < CONN_MSG_TMO_MAX)
// 		// 	send_maintain();
// 		else if (lowest_d_measures < MIN_D_MEASURES)
// 			return LOC_SEND_TWR;
// 	}

// 	return LOC_NO_SEND;
// }

// // void process_message(void)
// // {
// // 	uint8_t is_timeout = 0;
// // 	uint8_t peer_valid = 0;
// // 	peer_connection_t* peer = NULL;

// // 	loc_action = LOC_NO_RESP;

// // 	if ((recvd_type == MT_D_REQ || recvd_type == MT_D_REQ_ACK))
// // 	{
// // 		process_req();
// // 		if (*((uint16_t*)recv_buf) != panadr_own.short_addr)
// // 		{
// // 			recv_tmo_usec = (rand()&0xF000)+40000;
// // 			loc_action = LOC_STOP;
// // 			send_msg_meta = send_msg_meta_def;
// // 		}
// // 	}
// // 	if(loc_action != LOC_STOP)
// // 	{
// // 		if (recvd_type == 0)
// // 		{
// // 			recv_tmo_cnt++;
// // 			if (loc_state == LOC_TWR)
// // 				handle_twr_fail();
// // 		}
// // 		else
// // 		{
// // 			peer = get_peer(recvd_header.src_addr);
// // 			if (peer != NULL && peer->ttl < PEER_CONN_TTL)
// // 			{
// // 				peer_valid = 1;
// // 				chVTReset(&(peer->tmo_timer));
// // 				chVTSet(&(peer->tmo_timer), TIME_S2I(5), peer_tmo_cb, peer);
// // 			}
				
// // 			if ((recvd_type&0xF0) == 0x20)
// // 			{
// // 				if (peer_valid)
// // 					loc_action = LOC_SSTWR;
// // 			}
// // 			else
// // 			{
// // 				loc_state = LOC_COMM;
// // 				twr_state = TWR_NO_TWR;
// // 				if ((recvd_type&0xF0) == 0x10 /*|| some_peer_recv_ack_timeout*/)
// // 					conn_handle(peer);
// // 				else if ((recvd_type&0xF0) == 0x30 && peer_valid)
// // 					conn_handle(peer);
// // 				// else
// // 				// 	invalid_message_handle
// // 			}
// // 		}
// // 	}

// // 	// if (loc_action == LOC_ACT_ERR)
// // 	// 	reset // how? what?

// // 	//new_state();
// // 	clean_recvd();

// // 	if (loc_action == LOC_NO_RESP || recv_tmo_cnt > 5)
// // 		no_resp_action();
// // 	// else
// // 	// {
// // 	// 	if (loc_action != LOC_RESP_NOW)
// // 	// 		resp_action();
// // 	// }
// // }

// // THD_FUNCTION(COMMS, arg)
// // {
// // 	(void)arg;

// // 	comm_thread = chThdGetSelfX();
// // 	comm_state_t comm_state = COMM_RECV;
// // 	eventmask_t evt = 0;
// // 	recv_tmo_usec = (rand()&0xF000)+40000;

// // 	init_peers();

// // 	barrier();

// // 	chMtxLock(&dw_mutex);
// // 	chEvtWaitOne(DW_COMM_OK_E);
// // 	chMtxUnlock(&dw_mutex);

// // 	while (true)
// // 	{
// // 		switch (comm_state)
// // 		{
// // 			case COMM_RECV:
// // 				chMtxLock(&dw_mutex);
// // 				dw_ctrl_req = DW_RECV;
// // 				if (send_msg_meta.wtime >= 0 && send_msg_meta.type != 0)
// // 				{
// // 					dw_ctrl_req = DW_SEND_W4R;
// // 					prepare_message();
// // 				}
// // 				chMtxUnlock(&dw_mutex);
// // 				chEvtSignal(dw_thread, DW_COMM_OK_E);
// // 				evt = chEvtWaitOneTimeout(DW_COMM_OK_E, CH_TIMEOUT);
// // 				if (evt == DW_COMM_OK_E)
// // 				{
// // 					get_message();
// // 					comm_state = COMM_IDLE;
// // 				}
// // 				else
// // 					comm_state = COMM_ERR;
// // 				break;
// // 			case COMM_SEND:
// // 				chMtxLock(&dw_mutex);
// // 				dw_ctrl_req = DW_SEND;
// // 				if (send_msg_meta.dlytime > 0)
// // 					dw_ctrl_req = DW_SEND_DLY;
// // 				chMtxUnlock(&dw_mutex);
// // 				prepare_message();
// // 				chEvtSignal(dw_thread, DW_COMM_OK_E);
// // 				evt = chEvtWaitOneTimeout(DW_COMM_OK_E, CH_TIMEOUT);
// // 				if (evt == DW_COMM_OK_E)
// // 					comm_state = COMM_RECV;
// // 				else
// // 					comm_state = COMM_ERR;
// // 				break;
// // 			case COMM_ERR:
// // 				// Thread not responding reset thread?
// // 				comm_state = COMM_IDLE;
// // 				break;
// // 			case COMM_IDLE:
// // 				recv_tmo_usec = 50000;
// // 				process_message();
// // 				comm_state = COMM_RECV;
// // 				if (send_msg_meta.type != 0 && send_msg_meta.wtime < 0)
// // 					comm_state = COMM_SEND;
// // 				break;
// // 			default:
// // 				break;
// // 		}
// // 	}
// // }

// loc_action_t dist_loc_decide(dw_addr_t addr, dw_recv_info_t recv_info, uint8_t* recvd)
// {
// 	loc_action_t loc_action;

// 	message_t recvd_msg_type = recvd[0];

// 	uint8_t is_timeout = 0;
// 	uint8_t peer_valid = 0;
// 	peer_connection_t* peer = NULL;

// 	loc_action = LOC_NO_RESP;

// 	if ((recvd_msg_type == MT_D_REQ || recvd_msg_type == MT_D_REQ_ACK))
// 	{
// 		process_req();
// 		if (*((uint16_t*)recvd) != panadr_own.short_addr)
// 		{
// 			recv_tmo_usec = (rand()&0xF000)+40000;
// 			loc_action = LOC_STOP;
// 			send_msg_meta = send_msg_meta_def;
// 		}
// 	}
// 	if(loc_action != LOC_STOP)
// 	{
// 		if (recvd_msg_type == 0)
// 		{
// 			recv_tmo_cnt++;
// 			// if (loc_state == LOC_TWR)
// 			// 	handle_twr_fail();
// 		}
// 		else
// 		{
// 			peer = get_peer(addr);
// 			if (peer != NULL && peer->ttl < PEER_CONN_TTL)
// 			{
// 				peer_valid = 1;
// 				chVTReset(&(peer->tmo_timer));
// 				chVTSet(&(peer->tmo_timer), TIME_S2I(5), peer_tmo_cb, peer);
// 			}
				
// 			if ((recvd_msg_type&0xF0) == 0x20)
// 			{
// 				if (peer_valid)
// 					loc_action = twr_handle(peer);
// 				else
// 					handle_twr_fail();
// 			}
// 			else
// 			{
// 				loc_state = LOC_COMM;
// 				twr_state = TWR_NO_TWR;
// 				if ((recvd_msg_type&0xF0) == 0x10 /*|| some_peer_recv_ack_timeout*/)
// 					conn_handle(peer);
// 				else if ((recvd_msg_type&0xF0) == 0x30 && peer_valid)
// 					conn_handle(peer);
// 				// else
// 				// 	invalid_message_handle
// 			}
// 		}
// 	}

// 	// if (loc_action == LOC_ACT_ERR)
// 	// 	reset // how? what?

// 	//new_state();

// 	if (loc_action == LOC_NO_RESP || recv_tmo_cnt > 5)
// 		loc_action = no_resp_action();

// 	return loc_action;
// }

// loc_state_t dist_loc_act(dw_addr_t* send_addr, size_t* send_size, uint8_t* send_buf, loc_action_t loc_action, loc_state_t loc_state)
// {
// 	switch (loc_action)
// 	{

// 	}
// }

// THD_FUNCTION(DIST_LOC, arg)
// {
// 	(void)arg;

// 	init();

// 	uint8_t recvd_message[MAX_MSG_SIZE];
// 	uint8_t send_buf[MAX_MSG_SIZE];

// 	loc_state_t loc_state = LOC_INIT;
// 	dw_recv_info_t recv_info;
// 	dw_addr_t recvd_addr;
// 	dw_addr_t send_addr;
// 	size_t send_size;
// 	uint8_t w4r = 0;
// 	loc_action_t loc_action = LOC_NO_RESP;
// 	sysinterval_t tmo = TIME_US2I(40000);

// 	while (true)
// 	{
// 		// Sense
// 		if (!w4r)
// 			recv_info = dw_recv_tmo(&recvd_addr, recvd_message, MAX_MSG_SIZE, tmo);

// 		recv_info.recvd_size = 0;
// 		recv_info.rx_time = 0;
// 		recv_info.state = DW_NO_RESP;

// 		// Decide(Think)
// 		loc_action = dist_loc_decide(recvd_addr, recv_info, recvd_message);

// 		// Act
// 		loc_state = dist_loc_act(&send_addr, &send_size, send_buf, loc_action, loc_state);

// 		switch (loc_state)
// 		{
// 			case LOC_COMM:
// 				tmo = TIME_US2I(40000);
// 				break;
// 			case LOC_INIT:
// 				tmo = TIME_US2I((rand()&0xF000)+40000);
// 				break;
// 			default:
// 				break;
// 		}
		
// 		// Commit action
// 		w4r = 0;
// 		switch (loc_action)
// 		{
// 			case LOC_SEND:
// 				dw_send_tmo(send_addr, send_buf, send_size, tmo);
// 				break;
// 			case LOC_SEND_W4R:
// 				w4r = 1;
// 				recv_info = dw_send_w4r_tmo(send_addr, send_buf, send_size, 0, &recvd_addr, recvd_message, MAX_MSG_SIZE, tmo);
// 				break;
// 			case LOC_SSTWR:
// 				recv_info = dw_sstwr(send_addr, send_buf, 0, recvd_message, MAX_MSG_SIZE);
// 				break;
// 			// case config:
// 			// 	break;
// 			default:
// 				break;
// 		}
// 	}
// }

// void update_peer_pos(void)
// {
// 	float d = (get_distance(panadr_own.short_addr, peers[0].peer_addr) + get_distance(peers[0].peer_addr, panadr_own.short_addr))/2.0f;;
// 	float d0 = (get_distance(panadr_own.short_addr, peers[1].peer_addr) + get_distance(peers[1].peer_addr, panadr_own.short_addr))/2.0f;
// 	float d1 = (get_distance(peers[0].peer_addr, peers[1].peer_addr) + get_distance(peers[1].peer_addr, peers[0].peer_addr))/2.0f;

// 	if (d > 0.1)
// 		peer_positions[1][0] = d;

// 	if (d0 > 0.1 && d1 > 0.1)
// 	{
// 		peer_positions[2][0] = (d*d+d0*d0-d1*d1)/(2*d);
// 		peer_positions[2][1] = (float)(sqrt((double)(d0*d0 - (peer_positions[2][0])*(peer_positions[2][0]))));
// 	}
// }

// THD_FUNCTION(SYSTEM_STATUS, arg)
// {
// 	(void)arg;

// 	barrier_init(3);

// 	init_d_m();

// 	peer_positions[0][0] = 0;
// 	peer_positions[0][1] = 0;
// 	peer_positions[0][2] = 0;
// 	peer_positions[1][0] = 0;
// 	peer_positions[1][1] = 0;
// 	peer_positions[1][2] = 0;
// 	peer_positions[2][0] = 0;
// 	peer_positions[2][1] = 0;
// 	peer_positions[2][2] = 0;

// 	sdStart(&SD1, &serial_cfg);

// 	barrier();
// 	euclidean_d_m.addrs[0] = panadr_own.short_addr;

// 	while (true)
// 	{
// 		for (uint8_t i = 1; i < NEIGHBOUR_NUM+1; i++)
// 		{
// 			euclidean_d_m.addrs[i] = peers[i-1].peer_addr;
// 			euclidean_d_m.distances[0][i] = peers_info[i-1].calc_distance;
// 		}

// 		update_peer_pos();

// 		for (uint8_t i = 0; i < NEIGHBOUR_NUM; i++)
// 		{
// 			peers_info[i].d_measures = 0;
// 			if (peers[i].ttl >= PEER_CONN_TTL)
// 				disconnect_peer(peers+i);
// 		}

// 		int scanf_test;
// 		float scanf_float;
// 		char scanf_string[20];
// 		chprintf((BaseSequentialStream*)&SD1, "\nInput an integer: ");
// 		chscanf((BaseBufferedStream*)&SD1, "%d", &scanf_test);
// 		chprintf((BaseSequentialStream*)&SD1, "Read: %d\n", scanf_test);

// 		chprintf((BaseSequentialStream*)&SD1, "\nInput a float: ");
// 		chscanf((BaseBufferedStream*)&SD1, "%f", &scanf_float);
// 		chprintf((BaseSequentialStream*)&SD1, "Read: %f\n", scanf_float);

// 		chprintf((BaseSequentialStream*)&SD1, "\nInput a string: ");
// 		chscanf((BaseBufferedStream*)&SD1, " %20s", scanf_string);
// 		chprintf((BaseSequentialStream*)&SD1, "Read: %s\n", scanf_string);

// 		/*
// 		 *	Print peer positions
// 		 */

// 		// chprintf((BaseSequentialStream*)&SD1, "%d,%d\n", (int)peer_positions[0][0], (int)peer_positions[0][1]);
// 		// chprintf((BaseSequentialStream*)&SD1, "%d,%d\n", (int)peer_positions[1][0], (int)peer_positions[1][1]);
// 		// chprintf((BaseSequentialStream*)&SD1, "%d,%d\n", (int)peer_positions[2][0], (int)peer_positions[2][1]);
// 		// chprintf((BaseSequentialStream*)&SD1, "\n");

// 		/*
// 		 *	Print formatted EDM
// 		 */

// 		// chprintf((BaseSequentialStream*)&SD1, "\n\t| ");

// 		// for (uint8_t j = 0; j < NEIGHBOUR_NUM+1; j++)
// 		// {
// 		// 	chprintf((BaseSequentialStream*)&SD1, "%d\t", (int)euclidean_d_m.addrs[j]);
// 		// 	chprintf((BaseSequentialStream*)&SD1, "| ");
// 		// }
// 		// chprintf((BaseSequentialStream*)&SD1, "\n");
// 		// chprintf((BaseSequentialStream*)&SD1, "-------------------------------------\n");

// 		// for (uint8_t i = 0; i < NEIGHBOUR_NUM+1; i++)
// 		// {
// 		// 	chprintf((BaseSequentialStream*)&SD1, "%d\t", (int)euclidean_d_m.addrs[i]);
// 		// 	chprintf((BaseSequentialStream*)&SD1, "| ");

// 		// 	for (uint8_t j = 0; j < NEIGHBOUR_NUM+1; j++)
// 		// 	{
// 		// 		chprintf((BaseSequentialStream*)&SD1, "%d\t", (int)euclidean_d_m.distances[i][j]);
// 		// 		chprintf((BaseSequentialStream*)&SD1, "| ");
// 		// 	}

// 		// 	chprintf((BaseSequentialStream*)&SD1, "\n");
// 		// 	chprintf((BaseSequentialStream*)&SD1, "-------------------------------------\n");
// 		// }

// 		// chprintf((BaseSequentialStream*)&SD1, "\nXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n");
		
// 		/*
// 		 *	Print calibration EDM 
// 		 */

// 		// chprintf((BaseSequentialStream*)&SD1, "X");
// 		// for (uint8_t j = 0; j < NEIGHBOUR_NUM+1; j++)
// 		// 	chprintf((BaseSequentialStream*)&SD1, "%d,", (int)euclidean_d_m.addrs[j]);

// 		// chprintf((BaseSequentialStream*)&SD1, "\n");

// 		// for (uint8_t i = 0; i < NEIGHBOUR_NUM+1; i++)
// 		// {
// 		// 	for (uint8_t j = 0; j < NEIGHBOUR_NUM+1; j++)
// 		// 		chprintf((BaseSequentialStream*)&SD1, "%d,", (int)(euclidean_d_m.distances[i][j]*1000.0));

// 		// 	chprintf((BaseSequentialStream*)&SD1, "\n");
// 		// }

// 		// chprintf((BaseSequentialStream*)&SD1, "\n");

// 		chThdSleepMilliseconds(1000);
// 	}
// }