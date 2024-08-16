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

#include "source_seeking_app.h"

serial_packet_t uart1_send_buff[UART1_Q_LENGTH];

uint8_t uwb_send_buff[64];
uint8_t uwb_recv_buff[64];

dw_addr_t identifier_map[SS_DEVICE_NUMBER] = {1955, 18, 3213};

dw_addr_t field_source = 9999;

virtual_timer_t comm_slot_timer;

virtual_timer_t consensus_timer;

float init_consensus[SS_DEVICE_NUMBER] = {1.0, 3.0, 0.0};

float consensus_value[SS_DEVICE_NUMBER];

ss_pos_t centroid_c;
ss_pos_t pos_v_sum;

ss_pos_t centroid[SS_DEVICE_NUMBER] = {{.x=0, .y=0}, {.x=0, .y=0}, {.x=0, .y=0}};
ss_pos_t asc_dir[SS_DEVICE_NUMBER];
ss_pos_t position[SS_DEVICE_NUMBER] = {{.x=0, .y=0}, {.x=0, .y=0}, {.x=0, .y=0}};
uint8_t positions_outdated[SS_DEVICE_NUMBER];

dw_addr_t recv_addrs[SS_DEVICE_NUMBER];

uint8_t false_row[SS_DEVICE_NUMBER];
uint8_t true_row[SS_DEVICE_NUMBER];

int8_t current_slot = -1;

dw_addr_t self_addr;
size_t self_id;

binary_semaphore_t slot_free;

uint16_t consensus_iter_n = 0;
uint8_t consensus_step = SS_ITER_N;

uint8_t reset_cnt = 0;
uint8_t fail_cnt = 0;
uint8_t fail_state = 0;

uint8_t sync_message1 = 0x45;
uint8_t sync_message2 = 0x47;

void comm_slot_cb(virtual_timer_t* vtp, void* arg)
{
	current_slot++;

	if (current_slot >= SS_DEVICE_NUMBER)
	{
		current_slot = 0;
		update_consensus();
	}

	chSysLockFromISR();
	chBSemSignalI(&slot_free);
	chSysUnlockFromISR();
}

void init_timers(void)
{
	chVTObjectInit(&comm_slot_timer);
}

void send_source_dist(float source_dist)
{
	uart1_send_buff[0].p_class = SS_P_SOURCE_DIST;
	uart1_send_buff[0].p_length = sizeof(float);

	memcpy(uart1_send_buff[0].p_data, &source_dist, uart1_send_buff[0].p_length);
	chMBPostTimeout(&uart1_send_queue, (msg_t)&(uart1_send_buff[0]), TIME_US2I(50));
}

void send_centroid(void)
{
	uart1_send_buff[0].p_class = SS_P_CENTROID;
	uart1_send_buff[0].p_length = sizeof(centroid[self_id]);

	memcpy(uart1_send_buff[0].p_data, &(centroid[self_id]), uart1_send_buff[0].p_length);
	chMBPostTimeout(&uart1_send_queue, (msg_t)&(uart1_send_buff[0]), TIME_US2I(50));
}

void send_asc_dir(void)
{
	uart1_send_buff[0].p_class = SS_P_ASC_DIR;
	uart1_send_buff[0].p_length = sizeof(position[self_id]);

	memcpy(uart1_send_buff[0].p_data, &(position[self_id]), uart1_send_buff[0].p_length);
	chMBPostTimeout(&uart1_send_queue, (msg_t)&(uart1_send_buff[0]), TIME_US2I(50));
}

void send_confirmation(void)
{
	uart1_send_buff[0].p_class = SS_P_CONFIRMATION;
	uart1_send_buff[0].p_length = sizeof(identifier_map);

	memcpy(uart1_send_buff[0].p_data, identifier_map, uart1_send_buff[0].p_length);
	chMBPostTimeout(&uart1_send_queue, (msg_t)&(uart1_send_buff[0]), TIME_US2I(50));
}

void send_debug(void)
{
	uart1_send_buff[0].p_class = SS_P_DEBUG;
	uart1_send_buff[0].p_length = sizeof(consensus_iter_n) + sizeof(consensus_value[self_id]);

	memcpy(uart1_send_buff[0].p_data, &consensus_iter_n, sizeof(consensus_iter_n));
	memcpy(uart1_send_buff[0].p_data+sizeof(consensus_iter_n), consensus_value+self_id, sizeof(consensus_value[self_id]));
	chMBPostTimeout(&uart1_send_queue, (msg_t)&(uart1_send_buff[0]), TIME_US2I(50));
}

void recv_serial(void)
{
	serial_packet_t* recvd_packet;

	chMBFetchTimeout(&uart1_recv_queue, (msg_t*)&recvd_packet, TIME_US2I(50));

	switch(recvd_packet->p_class)
	{
		case SS_P_IDENTITIES:
			get_id_from_ap(recvd_packet->p_data, recvd_packet->p_length);
			chThdSleepMilliseconds(20);
			send_confirmation();
			break;
		case SS_P_CONFIRMATION:
			break;
		case SS_P_NED_POS:
			positions_outdated[self_id] = false;
			get_ned_pos(recvd_packet->p_data, recvd_packet->p_length);
			break;
		case SS_P_CENTROID:
			break;
		case SS_P_ASC_DIR:
			break;
		case SS_P_DEBUG:
			break;
		default:
			break;
	}
}

void update_consensus(void)
{
	float consensus_sum = 0;
	consensus_iter_n++;

	if (consensus_iter_n >= SS_ITER_N)
	{
		consensus_iter_n = 0;
		//reload_consensus();
	}
	else
	{	
		// Average

		// for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
		// 	consensus_sum += consensus_value[self_id]-consensus_value[i];
		
		// consensus_value[self_id] += -SS_K_GAIN*consensus_sum/SS_DEVICE_NUMBER;
	
		update_centroid();
	}
}

void update_centroid(void)
{
	float centr_x_sum = 0;
	float centr_y_sum = 0;
	float p_x_sum = 0;
	float p_y_sum = 0;

	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{
		centr_x_sum += centroid[self_id].x - centroid[i].x;
		centr_y_sum += centroid[self_id].y - centroid[i].y;
	}

	centroid[self_id].x += -SS_K_GAIN*(centr_x_sum - pos_v_sum.x)/SS_DEVICE_NUMBER;
	centroid[self_id].y += -SS_K_GAIN*(centr_y_sum - pos_v_sum.y)/SS_DEVICE_NUMBER;
}

void get_id_from_ap(uint8_t* data, size_t size)
{
	memcpy(identifier_map, data, sizeof(identifier_map));
}

void get_ned_pos(uint8_t* data, size_t size)
{
	memcpy(&(position[self_id]), data, size);
}

void ss_sync(void)
{
	dw_addr_t recv_addr = 0;
	uint8_t recv_message[1] = {0};
	switch(self_addr)
	{
		case 5923:
			chThdSleepMilliseconds(150);
			dw_send_tmo(0xFFFF, &sync_message1, sizeof(sync_message1), DEF_TMO_MS);
			break;

		default:
			while (recv_addr != 5923 || recv_message[0] != sync_message1)
				dw_recv_tmo(&recv_addr, recv_message, sizeof(sync_message1), DEF_TMO_MS);
			break;
	}
}

// void ss_sync(void)
// {
// 	dw_addr_t recv_addr = 0; 
// 	uint8_t recv_m[1] = {0};

// 	while (recv_addr != 7090 || recv_m[0] != sync_message1)
// 		dw_send_w4r_tmo(field_source, &sync_message1, sizeof(sync_message1), 0, &recv_addr, recv_m, sizeof(sync_message1), DEF_TMO_MS);
	
// 	while (recv_addr != 7090 || recv_m[0] != sync_message2)
// 		dw_recv_tmo(&recv_addr, recv_m, sizeof(sync_message2), DEF_TMO_MS);
// }

void run_consensus_average(void)
{
	dw_addr_t recvd_addr = 0;
	dw_recv_info_t dw_recv_info;
	
	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{
		chBSemWait(&slot_free);

		if (consensus_iter_n == 0 && identifier_map[i] == self_addr)
		{
			dw_recv_info = dw_sstwr(field_source, NULL, 0, (uint8_t*)(&(consensus_value[i])), sizeof(consensus_value[i]));
			chprintf((BaseSequentialStream*)&SD1, "SSTWR: %d field value read: %f\n\n", self_id, consensus_value[i]);
		}
		else
		{
			if (identifier_map[i] == self_addr)
			{
				chThdSleepMilliseconds(10);
				dw_send_tmo(0xFFFF, (uint8_t*)(&(consensus_value[i])), sizeof(consensus_value[i]), TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER-SS_RTOS_DELAY_US));
				
				//chprintf((BaseSequentialStream*)&SD1, "Iter: %d %d sent: %f\n\n", consensus_iter_n, self_id, consensus_value[i]);
			}
			else
			{
				float recv_value = -99999.9;
				dw_recv_info = dw_recv_tmo(&recvd_addr, (uint8_t*)(&recv_value), sizeof(recv_value), TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER-SS_RTOS_DELAY_US));
				size_t recvd = dw_recv_info.recvd_size;
				if (recvd == sizeof(recv_value) && recvd_addr == identifier_map[i] && COMM_GRAPH[i][self_id] > 0)
				{
					consensus_value[i] = recv_value;
					//chprintf((BaseSequentialStream*)&SD1, "Iter: %d %d received: %f\n\n", consensus_iter_n, self_id, recv_value);
				}
				else
				{
					// if (recvd == 0)
					// {
					// 	fail_cnt++;
					// 	chprintf((BaseSequentialStream*)&SD1, "Iter: %d timeout\n\n", consensus_iter_n);
					// }
					// else if (recvd_addr != identifier_map[i])
					// {
					// 	fail_cnt++;
					// 	chprintf((BaseSequentialStream*)&SD1, "Iter: %d %d received from %d\n\n", consensus_iter_n, self_id, identifier_map[i]);
					// }
					// else if (COMM_GRAPH[i][self_id] <= 0)
					// {
					// 	fail_cnt++;
					// 	chprintf((BaseSequentialStream*)&SD1, "Iter: %d %d received from %d\n\n", consensus_iter_n, self_id, identifier_map[i]);
					// }

					consensus_value[i] = consensus_value[self_id];
				}
			}
		}
	}
}

uint8_t check_error_consensus(ss_pos_t recv_value)
{
	return recv_value.x > -121212.121212-10e-6 && recv_value.x < -121212.121212+10e-6 && recv_value.y > -121212.121212-10e-6 && recv_value.y < -121212.121212+10e-6;
}

void calculate_centroid(void)
{
	centroid_c.x = 0;
	centroid_c.y = 0;
	pos_v_sum.x = 0;
	pos_v_sum.y = 0;

	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{
		centroid_c.x += position[i].x;
		centroid_c.y += position[i].y;
		pos_v_sum.x += position[self_id].x - position[i].x;
		pos_v_sum.y += position[self_id].y - position[i].y;
		centroid[i].x = 0;
		centroid[i].y = 0;
	}

	centroid_c.x /= SS_DEVICE_NUMBER;
	centroid_c.y /= SS_DEVICE_NUMBER;

	chprintf((BaseSequentialStream*)&SD1, "POS Id: %d C: (%.3f,%.3f)\n\n", self_id, position[self_id].x, position[self_id].y);
	chprintf((BaseSequentialStream*)&SD1, "Real Id: %d C: (%.3f,%.3f)\n\n", self_id, centroid_c.x, centroid_c.y);
}

void run_consensus_centroid(void)
{
	dw_addr_t recvd_addr = 0;
	dw_recv_info_t dw_recv_info;
	uint8_t serial_read_fail_cnt = 0;
	
	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{
		chBSemWait(&slot_free);

		// First iteration
		if (consensus_iter_n == 0)
		{
			// Exchange positions

			if (identifier_map[i] == self_addr)
			{
				position[self_id].x = 0;
				position[self_id].y = 0;
				// Read position from paparazzi
				while (fabs(position[self_id].x) < 1e-4 && fabs(position[self_id].y) < 1e-4 && serial_read_fail_cnt < 10)
				{
					recv_serial();
					serial_read_fail_cnt++;
				}
				chThdSleepMilliseconds(2);
				dw_send_tmo(0xFFFF, (uint8_t*)(&(position[i])), sizeof(position[i]), TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER-SS_RTOS_DELAY_US));
				//chprintf((BaseSequentialStream*)&SD1, "Iter: %d %d sent: (%f,%f)\n\n", consensus_iter_n, self_id, position[i].x, position[i].y);
			}
			else
			{
				ss_pos_t recv_value = {.x = -99999.9, .y = -99999.9};
				dw_recv_info = dw_recv_tmo(&recvd_addr, (uint8_t*)(&recv_value), sizeof(recv_value), TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER-SS_RTOS_DELAY_US));
				size_t recvd = dw_recv_info.recvd_size;
				if (recvd == sizeof(recv_value) && recvd_addr == identifier_map[i])
					position[i] = recv_value;
				else
					position[i] = position[self_id];

				chThdSleepMilliseconds(2);

				//chprintf((BaseSequentialStream*)&SD1, "Iter: %d %d received: (%f,%f)\n\n", consensus_iter_n, self_id, recv_value.x, recv_value.y);
			}
		}
		else
		{
			if (identifier_map[i] == self_addr)
			{
				chThdSleepMilliseconds(5);
				dw_send_tmo(0xFFFF, (uint8_t*)(&(centroid[i])), sizeof(centroid[i]), TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER-SS_RTOS_DELAY_US));
				//chprintf((BaseSequentialStream*)&SD1, "Iter: %d %d sent: (%f,%f)\n\n", consensus_iter_n, self_id, centroid[i].x, centroid[i].y);
			}
			else
			{
				ss_pos_t recv_value = {.x = -99999.9, .y = -99999.9};
				dw_recv_info = dw_recv_tmo(&recvd_addr, (uint8_t*)(&recv_value), sizeof(recv_value), TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER-SS_RTOS_DELAY_US));
				size_t recvd = dw_recv_info.recvd_size;
				if (check_error_consensus(recv_value))
					fail_state = 1;
				if (recvd == sizeof(recv_value) && recvd_addr == identifier_map[i] && COMM_GRAPH[i][self_id] > 0)
				{
					//chprintf((BaseSequentialStream*)&SD1, "Iter: %d %d received: (%f,%f)\n\n", consensus_iter_n, self_id, recv_value.x, recv_value.y);
					centroid[i] = recv_value;
				}
				else
				{
					if (COMM_GRAPH[i][self_id] > 0)
					{
						//chprintf((BaseSequentialStream*)&SD1, "Iter: %d ID: %d failed receive\n\n", consensus_iter_n, self_id);
						fail_cnt++;
						fail_state = 1;
					}
					centroid[i] = centroid[self_id];
					centroid[i].x -= position[self_id].x - position[i].x;
					centroid[i].y -= position[self_id].y - position[i].y;
				}

				chThdSleepMilliseconds(2);
			}
		}
	}

	if (consensus_iter_n == 0)
	{
		// Calculate centroid
		calculate_centroid();	
	}
}

void broad_consensus(ss_pos_t value)
{
	ss_header_t ss_header;

	ss_header.step = consensus_step;
	ss_header.type = SS_M_CON_V;

	memcpy(uwb_send_buff, &ss_header, sizeof(ss_header));
	memcpy(uwb_send_buff+sizeof(ss_header), &value, sizeof(value));

	dw_send_tmo(0xFFFF, uwb_send_buff, sizeof(ss_header)+sizeof(value), DEF_TMO_MS);
	// chprintf((BaseSequentialStream*)&SD1, "Sent Id: %d step %u \n\n", self_id, consensus_step);
}

void broad_consensus_last(ss_pos_t value, dw_addr_t addr)
{
	ss_header_t ss_header;

	ss_header.step = consensus_step-1;
	ss_header.type = SS_M_CON_LV;

	memcpy(uwb_send_buff, &ss_header, sizeof(ss_header));
	memcpy(uwb_send_buff+sizeof(ss_header), &value, sizeof(value));

	dw_send_tmo(addr, uwb_send_buff, sizeof(ss_header)+sizeof(value), DEF_TMO_MS);
	// chprintf((BaseSequentialStream*)&SD1, "Sent Id: %d  step %u addr %u LAST\n\n", self_id, consensus_step-1, addr);
}

uint8_t ss_dev_id(dw_addr_t addr)
{
	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{
		if (identifier_map[i] == addr)
			return i;
	}

	return 255;
}

uint8_t is_consensus_device(dw_addr_t addr, uint8_t id)
{
	uint8_t addr_id = ss_dev_id(addr);

	if (addr_id < SS_DEVICE_NUMBER)
		if (COMM_GRAPH[addr_id][id] > 0)
			return addr_id;

	return 255;
}

void exchange_positions(void)
{
	ss_header_t ss_header;
	ss_header_t* ss_header_recv_p;
	dw_recv_info_t recvd;
	dw_addr_t recv_addr;
	uint8_t recv_id;
	uint8_t skip_exch = false;
	uint8_t other_message_cnt = 0;
	ss_pos_t* pos_recvd_p;
	uint8_t positions_recvd[SS_DEVICE_NUMBER];
	memset(positions_recvd, false, sizeof(positions_recvd));
	memset(positions_outdated, true, sizeof(positions_outdated));

	ss_header.step = 0;
	ss_header.type = SS_M_CON_POS;

	// while (positions_outdated[self_id])
	// 	recv_serial();
	positions_outdated[self_id] = false;
	position[self_id].x = 30+((40-30+1)*rand()/(float)RAND_MAX);
	position[self_id].y = 30+((40-30+1)*rand()/(float)RAND_MAX);
	// position[self_id].x = self_id+1;
	// position[self_id].y = self_id+1;
	chThdSleepMilliseconds(2);

	memcpy(uwb_send_buff, &ss_header, sizeof(ss_header));

	while (memcmp(true_row, positions_recvd, sizeof(positions_recvd)) != 0 && !skip_exch)
	{
		memcpy(uwb_send_buff+sizeof(ss_header), position, sizeof(position));
		dw_send_tmo(0xFFFF, uwb_send_buff, sizeof(ss_header)+sizeof(position), DEF_TMO_MS);
		// chprintf((BaseSequentialStream*)&SD1, "Positions sent ID %u 0 (%.3f,%.3f) 1 (%.3f,%.3f) 2(%.3f,%.3f)\n", self_id, position[0].x, position[0].y, position[1].x, position[1].y, position[2].x, position[2].y);

		for (size_t i = 0; i < SS_DEVICE_NUMBER-1; i++)
		{
			recvd = dw_recv_tmo(&recv_addr, uwb_recv_buff, sizeof(ss_header_t)+sizeof(position), TIME_MS2I(40+((100-40+1)*rand()/(float)RAND_MAX)));
			recv_id = ss_dev_id(recv_addr);
			if (recv_id < SS_DEVICE_NUMBER)
			{
				ss_header_recv_p = (ss_header_t*)uwb_recv_buff;
				if (!positions_recvd[recv_id] && ss_header_recv_p->type == SS_M_CON_POS)
				{
					pos_recvd_p = (ss_pos_t*)(uwb_recv_buff+sizeof(ss_header_t));
					uint8_t pos_cnt = 0;
					// chprintf((BaseSequentialStream*)&SD1, "Positions recv ID %u recvid %u 0 (%.3f,%.3f) 1 (%.3f,%.3f) 2(%.3f,%.3f)\n", self_id, recv_id, pos_recvd_p[0].x, pos_recvd_p[0].y, pos_recvd_p[1].x, pos_recvd_p[1].y, pos_recvd_p[2].x, pos_recvd_p[2].y);
					for (size_t j = 0; j < SS_DEVICE_NUMBER; j++)
					{
						if (fabs(pos_recvd_p[j].x) > 1e-4 && fabs(pos_recvd_p[j].y) > 1e-4)
						{
							pos_cnt++;
							if (positions_outdated[j])
							{
								positions_outdated[j] = 0;
								if (j != self_id)
									position[j] = pos_recvd_p[j];
							}
						}
					}
					if (pos_cnt == SS_DEVICE_NUMBER)
						positions_recvd[recv_id] = true;
					other_message_cnt = 0;
				}
				else
					other_message_cnt++;
			}

			if (memcmp(false_row, positions_outdated, sizeof(positions_outdated)) == 0)
				positions_recvd[self_id] = true;

			uint8_t dev_not_ack_cnt = 0;
			for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
			{
				if (!positions_recvd[i] && i != self_id)
					dev_not_ack_cnt++;
			}

			skip_exch = dev_not_ack_cnt == 1 && positions_recvd[self_id] && other_message_cnt > 10;
		}
	}
}

void run_consensus_new(void)
{
	dw_addr_t recv_addr;
	uint8_t recv_id;
	ss_header_t* ss_header_p;
	static uint8_t comm_row[SS_DEVICE_NUMBER];
	static ss_pos_t last_value = {.x=0, .y=0};
	static dw_addr_t last_addr = 0;
	static uint8_t send_cond = true;
	dw_recv_info_t recvd;

	if (consensus_step == SS_ITER_N)
	{
		reset_cnt++;
		chprintf((BaseSequentialStream*)&SD1, "FINAL Id: %d C: (%.3f,%.3f)\n\n", self_id, position[self_id].x - centroid[self_id].x , position[self_id].y - centroid[self_id].y);
		// send_centroid();

		// NEW ITER
		consensus_step = 0;
		memcpy(comm_row, COMM_GRAPH[self_id], sizeof(comm_row));

		// Get positions or other data
		exchange_positions();
		// chprintf((BaseSequentialStream*)&SD1, "positions ID %u 0 (%.3f,%.3f) 1 (%.3f,%.3f) 2(%.3f,%.3f)\n", self_id, position[0].x, position[0].y, position[1].x, position[1].y, position[2].x, position[2].y);

		calculate_centroid();
		update_centroid();
	}

	if (memcmp(false_row, comm_row, sizeof(comm_row)) == 0) // All required data received
	{
		chprintf((BaseSequentialStream*)&SD1, "Centroid ID %u 0 (%.3f,%.3f) 1 (%.3f,%.3f) 2(%.3f,%.3f)\n", self_id, centroid[0].x, centroid[0].y, centroid[1].x, centroid[1].y, centroid[2].x, centroid[2].y);
		update_centroid();
		chprintf((BaseSequentialStream*)&SD1, "UPDATE Id: %d Step: %u C: (%.3f,%.3f)\n\n", self_id, consensus_step, position[self_id].x - centroid[self_id].x , position[self_id].y - centroid[self_id].y);

		consensus_step++;
		last_addr = 0;
		last_value = centroid[self_id];
		memcpy(comm_row, COMM_GRAPH[self_id], sizeof(comm_row));
		send_cond = true;
	}

	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{	
		//	Send broadcast of value
		if (self_id == i && send_cond)
		{
			if (last_addr == 0)
				broad_consensus(centroid[self_id]);
			else
			{
				broad_consensus_last(last_value, last_addr);
				last_addr = 0;
			}
			send_cond = false;
		}
		else 	// Receive
		{
			if (COMM_GRAPH[i][self_id] > 0)
			{
				recvd = dw_recv_tmo(&recv_addr, uwb_recv_buff, sizeof(ss_header_t)+sizeof(*centroid), TIME_MS2I(60));
				recv_id = is_consensus_device(recv_addr, self_id);

				if (recv_id != 255) // right device
				{
					ss_header_p = (ss_header_t*)uwb_recv_buff;
					// chThdSleepMilliseconds(5);
					// chprintf((BaseSequentialStream*)&SD1, "Id: %d recvid: %u m_step: %u recv_step: %u value: (%.3f,%.3f)\n\n", self_id, recv_id, consensus_step, ss_header_p->step, ((ss_pos_t*)(uwb_recv_buff+sizeof(ss_header_t)))->x , ((ss_pos_t*)(uwb_recv_buff+sizeof(ss_header_t)))->y);
					if (ss_header_p->step == consensus_step && ss_header_p->type != SS_M_CON_POS) 
					{
						centroid[recv_id] = *((ss_pos_t*)(uwb_recv_buff+sizeof(ss_header_t)));					
						if (comm_row[recv_id] > 0)
							comm_row[recv_id] = 0;
						else
							send_cond = true;
					}
					else // right device wrong iter
					{
						if (ss_header_p->type != SS_M_CON_LV)
						{
							// When this happens the last value should be sent instead of the current value until current iter is recvd frm recv_id
							if (ss_header_p->step == consensus_step-1)
								last_addr = recv_addr;
							else if (ss_header_p->step < consensus_step-1 && consensus_step == SS_ITER_N-1)
								consensus_step++;
						}
						send_cond = true;
					}
				}
				else //timeout or err or wrong device											
				{
					if (recvd.recvd_size == 0)
					{
						// chprintf((BaseSequentialStream*)&SD1, "TIMEOUT: %d \n\n", self_id);
						send_cond = true;
					}
					
					centroid[i] = centroid[self_id];
					centroid[i].x -= position[self_id].x - position[i].x;
					centroid[i].y -= position[self_id].y - position[i].y;
				}
			}
			else
			{
				centroid[i] = centroid[self_id];
				centroid[i].x -= position[self_id].x - position[i].x;
				centroid[i].y -= position[self_id].y - position[i].y;	
				
				if (self_id == i+1)
					chThd_rand_wait(50000, 60000);
			} 
		}
	}
}

// void consens_new()
// {
// 	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
// 	{
// 		if (COMM_GRAPH[self_id][i] > 0)
// 		{
// 			while (!received(recv_addr))
// 			{
// 				recvd = dw_recv_tmo(&recv_addr, uwb_recv_buff, sizeof(ss_header_t)+sizeof(*centroid), TIME_MS2I(60));
// 				recv_id = is_consensus_device(recv_addr, self_id);

// 				if (recv_id != 255) // right device
// 				{
// 					ss_header_p = (ss_header_t*)uwb_recv_buff;
// 					// chprintf((BaseSequentialStream*)&SD1, "Id: %d recvid: %u m_step: %u recv_step: %u value: (%.3f,%.3f)\n\n", self_id, recv_id, consensus_step, ss_header_p->step, ((ss_pos_t*)(uwb_recv_buff+sizeof(ss_header_t)))->x , ((ss_pos_t*)(uwb_recv_buff+sizeof(ss_header_t)))->y);
// 					if (ss_header_p->step == consensus_step) 
// 						centroid[recv_id] = *((ss_pos_t*)(uwb_recv_buff+sizeof(ss_header_t)));					
// 					else // right device wrong iter
// 					{
// 						if (ss_header_p->type == SS_M_CON_V)
// 						{
// 							// When this happens the last value should be sent instead of the current value until current iter is recvd frm recv_id
// 							if (ss_header_p->step == consensus_step-1)
// 								last_addr = recv_addr;
// 							else if (ss_header_p->step < consensus_step-1 && consensus_step == SS_ITER_N-1)
// 								consensus_step++;
// 						}
// 						send = true;
// 					}
// 				}
// 				else //timeout or err or unknown device											
// 				{
// 					if (recvd.recvd_size == 0)
// 					{
// 						// chprintf((BaseSequentialStream*)&SD1, "TIMEOUT: %d \n\n", self_id);
// 						tmo_cnt++
// 					}
// 				}

// 				if (tmo_cnt > 5)
// 					send = true;

// 				if (send)
// 				{

// 				}
// 			}

// 			tmo_cnt = 0;
// 		}
// 		else if (COMM_GRAPH[self_id][i] <= 0 && i != self_id)
// 		{
// 			centroid[i] = centroid[self_id];
// 			centroid[i].x -= position[self_id].x - position[i].x;
// 			centroid[i].y -= position[self_id].y - position[i].y;
// 		}
// 	}

// 	consensus_step++;
// 	update_centroid();
// 	recv_addrs

// 	if (consensus_step == SS_ITER_N)
// 	{

// 	}
// }

// void source_device(void)
// {
// 	dw_addr_t recv_addr = 0;
// 	uint8_t recv_sync[1] = {0};
// 	uint8_t correct[SS_DEVICE_NUMBER];
// 	uint8_t all_ready = 0;
// 	uint8_t synced = 0;
// 	systime_t time_cnt = 0; 
// 	systime_t start_time = chVTGetSystemTime();

// 	while (true)
// 	{
// 		dw_recv_tmo(NULL, NULL, 0, TIME_MS2I(100));

// 		time_cnt = chVTGetSystemTime();

// 		if (TIME_I2MS(time_cnt-start_time) >= 1500)
// 		{
// 			synced = 0;
// 			all_ready = 0;
// 			for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
// 				correct[i] = 0;

// 			while (!synced)
// 			{
// 				for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
// 				{
// 					dw_recv_tmo(&recv_addr, recv_sync, sizeof(sync_message1), TIME_MS2I(20));
					
// 					if (recv_addr == identifier_map[i] && recv_sync[0] == sync_message1)
// 					{
// 						chThdSleepMilliseconds(2);
// 						dw_send_tmo(identifier_map[i], &sync_message1, sizeof(sync_message1), DEF_TMO_MS);
// 						correct[i] = 1; 
// 					}

// 					recv_addr = 0;
// 					recv_sync[0] = 0;
// 				}

// 				for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
// 				{
// 					if (correct[i] > 0)
// 						all_ready++;
// 				}

// 				if (all_ready >= SS_DEVICE_NUMBER)
// 				{
// 					chThdSleepMilliseconds(20);
// 					dw_send_tmo(0xFFFF, &sync_message2, sizeof(sync_message2), TIME_MS2I(10));
// 					synced = 1;
// 				}
// 				else
// 					all_ready = 0;
// 			}
// 			start_time = chVTGetSystemTime();
// 		}
// 	}
// }

void reload_consensus(void)
{
	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
		consensus_value[i] = init_consensus[self_id];
}

THD_FUNCTION(SS, arg)
{
	(void) arg;

	chThdSleepMilliseconds(5000);

	self_addr = dw_get_addr();

	// if (self_addr == field_source)
	// 	source_device();

	while (identifier_map[0] == 0)
		recv_serial();

	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{
		if (self_addr == identifier_map[i])
			self_id = i;
	}

	memset(false_row, false, sizeof(false_row));
	memset(true_row, true, sizeof(true_row));

	// chBSemObjectInit(&slot_free, 1);
	// init_timers();

	// ss_sync();

	// chVTSetContinuous(&comm_slot_timer, TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER), comm_slot_cb, NULL);

	while(!chThdShouldTerminateX())
	{
		run_consensus_new();

		if (reset_cnt > 100 || fail_state)
		{
			// chprintf((BaseSequentialStream*)&SD1, "Reset: %d\n\n", self_id);

			reset_cnt = 0;
			fail_state = 0;

			// chVTReset(&comm_slot_timer);
			dw_reset_sys();
			chThdSleepMilliseconds(20);
			// chBSemReset(&slot_free, 1);
			// ss_sync();
			// chVTSetContinuous(&comm_slot_timer, TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER), comm_slot_cb, NULL);
		}
	}
}

THD_FUNCTION(DISTANCE_FIELD, arg)
{
	(void) arg;

	chThdSleepMilliseconds(5000);

	self_addr = dw_get_addr();

	dw_addr_t recv_addr = 0;
	uint8_t recv_data[1];
	uint8_t send_data[1] = {SS_P_FIELD_MEASURE};
	dw_recv_info_t dw_recv_info;

	float distance = 0.0;
	uint32_t err_cnt = 0;
	uint32_t iter_cnt = 0;

	while (identifier_map[0] == 0 && self_addr != field_source)
		recv_serial();

	send_confirmation();
	chBSemObjectInit(&slot_free, 1);

	chThdSleepMilliseconds(50);

	init_timers();

	chVTSetContinuous(&comm_slot_timer, TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER), comm_slot_cb, NULL);

	// systime_t time_now = chVTGetSystemTime();
	// chprintf((BaseSequentialStream*)&SD1, "start time: %u\n", time_now);

	while(!chThdShouldTerminateX())
	{
		distance = 0.0;
		if (self_addr == field_source)
		{
			for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
			{
				chBSemWait(&slot_free);
				// time_now = chVTGetSystemTime();
				// chprintf((BaseSequentialStream*)&SD1, "iter: %u, time: %u\n", iter_cnt, time_now);
				chThdSleepMicroseconds(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER/10-SS_RTOS_DELAY_US);
				dw_send_w4r_tmo(identifier_map[i], send_data, sizeof(send_data), 0, NULL, NULL, 0, TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER-SS_RTOS_DELAY_US));
			}

			iter_cnt++;
			if (iter_cnt > 1000)
			{
				iter_cnt = 0;
				chThdSleepMilliseconds(500);
			}
		}
		else
		{
			dw_recv_tmo(&recv_addr, recv_data, 1, TIME_US2I(CONSENSUS_PERIOD_US+SS_RTOS_DELAY_US));
			if (recv_addr == field_source && recv_data[0] == SS_P_FIELD_MEASURE)
				dw_recv_info = dw_sstwr(field_source, NULL, 0, (uint8_t*)(&distance), sizeof(distance));
			else
				err_cnt++;

			// time_now = chVTGetSystemTime();
			// chprintf((BaseSequentialStream*)&SD1, "Recv time: %u\n", time_now);

			if (err_cnt > 100)
			{
				err_cnt = 0;
				recv_serial();
			}
			
			if (distance < 0.1)
			{
				if (recv_addr == 0)
					distance = -1.0;
				else
					distance = -2.0;
			}
			//chprintf((BaseSequentialStream*)&SD1, "%fm\n", distance);

			send_source_dist(distance);

			recv_addr = 0;
			recv_data[0] = 0;
			
		}
	}
}