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

// DEBUGGING CHANGE POSITION ID MAP
dw_addr_t identifier_map[SS_DEVICE_NUMBER] = {0, 0, 0};

dw_addr_t field_source = 2177;
float field_value = 0;

virtual_timer_t comm_slot_timer;

virtual_timer_t consensus_timer;

float init_consensus[SS_DEVICE_NUMBER] = {1.0, 3.0, 0.0};

float consensus_value[SS_DEVICE_NUMBER];

ss_pos_t centroid_c;
ss_pos_t pos_v_sum;

ss_pos_t centroid[SS_DEVICE_NUMBER] = {{.x=0, .y=0}, {.x=0, .y=0}, {.x=0, .y=0}};
ss_pos_t last_centroid[SS_DEVICE_NUMBER] = {{.x=0, .y=0}, {.x=0, .y=0}, {.x=0, .y=0}};
ss_pos_t asc_dir[SS_DEVICE_NUMBER] = {{.x=0, .y=0}, {.x=0, .y=0}, {.x=0, .y=0}};
ss_pos_t position[SS_DEVICE_NUMBER] = {{.x=0, .y=0}, {.x=0, .y=0}, {.x=0, .y=0}};
uint8_t positions_outdated[SS_DEVICE_NUMBER];

dw_addr_t recv_addrs[SS_DEVICE_NUMBER];

uint8_t false_row[SS_DEVICE_NUMBER];
uint8_t true_row[SS_DEVICE_NUMBER];

int8_t current_slot = -1;

dw_addr_t self_addr;
size_t self_id;

uint16_t consensus_iter_n = 0;
uint8_t consensus_step = SS_ITER_N;

uint8_t reset_cnt = 0;
uint8_t fail_cnt = 0;
uint8_t fail_state = 0;

void send_source_dist(float source_dist)
{
	uart1_send_buff[0].p_class = SS_P_SOURCE_DIST;
	uart1_send_buff[0].p_length = sizeof(float);

	memcpy(uart1_send_buff[0].p_data, &source_dist, uart1_send_buff[0].p_length);
	chMBPostTimeout(&uart1_send_queue, (msg_t)&(uart1_send_buff[0]), TIME_US2I(50));
}

void send_debug(void)
{
	uart1_send_buff[0].p_class = SS_P_DEBUG;
	uart1_send_buff[0].p_length = 0;

	chMBPostTimeout(&uart1_send_queue, (msg_t)&(uart1_send_buff[0]), TIME_US2I(50));
}

void send_centroid(void)
{
	send_debug();
	uart1_send_buff[0].p_class = SS_P_CENTROID;
	uart1_send_buff[0].p_length = sizeof(centroid[self_id]);

	memcpy(uart1_send_buff[0].p_data, &(centroid[self_id]), uart1_send_buff[0].p_length);
	chMBPostTimeout(&uart1_send_queue, (msg_t)&(uart1_send_buff[0]), TIME_US2I(50));
}

void send_asc_dir(void)
{
	send_debug();
	uart1_send_buff[0].p_class = SS_P_ASC_DIR;
	uart1_send_buff[0].p_length = sizeof(asc_dir[self_id]);
	
	// char a[8] = {'a','a','a','a','a','a','a','a'};

	// memcpy(uart1_send_buff[0].p_data, a, uart1_send_buff[0].p_length);
	
	// Send normalized ascension direction
	ss_pos_t norm_asc_dir = {.x = 0, .y = 0};
	float asc_dir_size = sqrt(asc_dir[self_id].x*asc_dir[self_id].x + asc_dir[self_id].y*asc_dir[self_id].y);
	
	if (fabs(asc_dir_size) > 1e-6)
	{
		norm_asc_dir.x = asc_dir[self_id].x/asc_dir_size;
		norm_asc_dir.y = asc_dir[self_id].y/asc_dir_size;
	}

	memcpy(uart1_send_buff[0].p_data, &(norm_asc_dir), uart1_send_buff[0].p_length);
	chMBPostTimeout(&uart1_send_queue, (msg_t)&(uart1_send_buff[0]), TIME_US2I(50));
}

void send_confirmation(void)
{
	uart1_send_buff[0].p_class = SS_P_CONFIRMATION;
	uart1_send_buff[0].p_length = sizeof(identifier_map);

	memcpy(uart1_send_buff[0].p_data, identifier_map, uart1_send_buff[0].p_length);
	chMBPostTimeout(&uart1_send_queue, (msg_t)&(uart1_send_buff[0]), TIME_US2I(50));
}

void get_field_value_sim(uint8_t* data, size_t size)
{
	memcpy(&(field_value), data, size);
}

void recv_serial(void)
{
	serial_packet_t* recvd_packet;

	msg_t ret_msg = chMBFetchTimeout(&uart1_recv_queue, (msg_t*)&recvd_packet, TIME_US2I(100));

	if(ret_msg == MSG_OK)
	{
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
				if (SS_SIM_MODE)
				{
					get_ned_pos(recvd_packet->p_data, sizeof(position[self_id]));
					get_field_value_sim(recvd_packet->p_data + sizeof(position[self_id]), sizeof(field_value));
				}
				else
					get_ned_pos(recvd_packet->p_data, recvd_packet->p_length);
				break;
			case SS_P_CENTROID:
				break;
			// case SS_P_SOURCE_DIST:
			// 	get_field_value_sim(recvd_packet->p_data, recvd_packet->p_length);
				break;
			case SS_P_ASC_DIR:
				break;
			case SS_P_DEBUG:
				break;
			default:
				break;
		}
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

void update_asc_dir(void)
{
	float asc_dir_x_sum = 0;
	float asc_dir_y_sum = 0;

	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{
		asc_dir_x_sum += asc_dir[self_id].x - asc_dir[i].x;
		asc_dir_y_sum += asc_dir[self_id].y - asc_dir[i].y;
	}

	asc_dir[self_id].x += -SS_K_GAIN*(asc_dir_x_sum)/SS_DEVICE_NUMBER;
	asc_dir[self_id].y += -SS_K_GAIN*(asc_dir_y_sum)/SS_DEVICE_NUMBER;
}

void get_id_from_ap(uint8_t* data, size_t size)
{
	memcpy(identifier_map, data, sizeof(identifier_map));
}

void get_ned_pos(uint8_t* data, size_t size)
{
	memcpy(&(position[self_id]), data, size);
}

void prepare_consensus(void)
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
		asc_dir[i].x = 0;
		asc_dir[i].y = 0;
	}

	centroid_c.x /= SS_DEVICE_NUMBER;
	centroid_c.y /= SS_DEVICE_NUMBER;

	// chprintf((BaseSequentialStream*)&SD1, "POS Id: %d C: (%.3f,%.3f)\n\n", self_id, position[self_id].x, position[self_id].y);
	// chprintf((BaseSequentialStream*)&SD1, "Real Id: %d C: (%.3f,%.3f)\n\n", self_id, centroid_c.x, centroid_c.y);
}

void broad_consensus(void)
{
	ss_header_t ss_header;

	ss_header.step = consensus_step;
	ss_header.type = SS_M_CON_V;

	memcpy(uwb_send_buff, &ss_header, sizeof(ss_header));
	memcpy(uwb_send_buff+sizeof(ss_header), &(centroid[self_id]), sizeof(centroid[self_id]));
	memcpy(uwb_send_buff+sizeof(ss_header)+sizeof(centroid[self_id]), &(asc_dir[self_id]), sizeof(asc_dir[self_id]));
	
	dw_send_tmo(0xFFFF, uwb_send_buff, sizeof(ss_header)+sizeof(centroid[self_id])+sizeof(asc_dir[self_id]), DEF_TMO_MS);
	// chprintf((BaseSequentialStream*)&SD1, "SENT Id: %d step: %u  CENT: (%.3f,%.3f) ASC: (%.3f,%.3f)\n\n", self_id, consensus_step, centroid[self_id].x, centroid[self_id].y, asc_dir[self_id].x, asc_dir[self_id].y);
}

void broad_consensus_last(ss_pos_t last_cen, ss_pos_t last_asc_dir, dw_addr_t addr)
{
	ss_header_t ss_header;

	ss_header.step = consensus_step-1;
	ss_header.type = SS_M_CON_LV;

	memcpy(uwb_send_buff, &ss_header, sizeof(ss_header));
	memcpy(uwb_send_buff+sizeof(ss_header), &last_cen, sizeof(last_cen));
	memcpy(uwb_send_buff+sizeof(ss_header)+sizeof(last_cen), &(last_asc_dir), sizeof(last_asc_dir));

	dw_send_tmo(addr, uwb_send_buff, sizeof(ss_header)+sizeof(last_cen)+sizeof(last_asc_dir), DEF_TMO_MS);
	// chprintf((BaseSequentialStream*)&SD1, "SENT LAST Id: %d step: %u  CENT: (%.3f,%.3f) ASC: (%.3f,%.3f)\n\n", self_id, consensus_step-1, last_cen.x, last_cen.y, last_asc_dir.x, last_asc_dir.y);
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

float get_field(void)
{
	dw_recv_info_t dw_recv_info;
	field_value = 0.0;
	uint8_t iter_cnt = 0;

	if(SS_SIM_MODE)
	{
		chThd_rand_wait(40000,100000);
		send_source_dist(field_value);
	}
	else
	{
		while ((field_value < 0.1 || field_value > 300) && iter_cnt < 10)
		{
			chThd_rand_wait(20000,100000); 
			dw_recv_info = dw_sstwr(field_source, NULL, 0, (uint8_t*)(&field_value), sizeof(field_value));
			iter_cnt++;
		}
	}

	return field_value;
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
	uint8_t* outdated_recvd_p;
	uint8_t positions_recvd[SS_DEVICE_NUMBER];
	memset(positions_recvd, false, sizeof(positions_recvd));
	memset(positions_outdated, true, sizeof(positions_outdated));

	ss_header.step = 0;
	ss_header.type = SS_M_CON_POS;

	// DEBUGGING CHANGE POSITION READ

	while (positions_outdated[self_id])
		recv_serial();
	positions_outdated[self_id] = false;
	// position[self_id].x = 30+((40-30+1)*rand()/(float)RAND_MAX);
	// position[self_id].y = 30+((40-30+1)*rand()/(float)RAND_MAX);
	// switch (self_id)
	// {
	// 	case 0:
	// 		position[self_id].x = 1;
	// 		position[self_id].y = 1;
	// 		break;
	// 	case 1:
	// 		position[self_id].x = 1;
	// 		position[self_id].y = 2;
	// 		break;
	// 	case 2:
	// 		position[self_id].x = 2;
	// 		position[self_id].y = 1;
	// 		break;
	// 	default:
	// 		break;
	// }

	chThdSleepMilliseconds(2);

	memcpy(uwb_send_buff, &ss_header, sizeof(ss_header));

	/*
	*	MESSAGE FORMAT
	*	[HEADER][N_positions][N_bools]
	*
	*/

	while (memcmp(true_row, positions_recvd, sizeof(positions_recvd)) != 0 && !skip_exch)
	{
		memcpy(uwb_send_buff+sizeof(ss_header), position, sizeof(position));
		memcpy(uwb_send_buff+sizeof(ss_header)+sizeof(position), positions_outdated, sizeof(positions_outdated));
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
					outdated_recvd_p = (uwb_recv_buff+sizeof(ss_header_t)+sizeof(position));
					uint8_t pos_cnt = 0;
					// chprintf((BaseSequentialStream*)&SD1, "Positions recv ID %u recvid %u 0 (%.3f,%.3f) 1 (%.3f,%.3f) 2(%.3f,%.3f)\n", self_id, recv_id, pos_recvd_p[0].x, pos_recvd_p[0].y, pos_recvd_p[1].x, pos_recvd_p[1].y, pos_recvd_p[2].x, pos_recvd_p[2].y);
					for (size_t j = 0; j < SS_DEVICE_NUMBER; j++)
					{
						if (outdated_recvd_p[j] == false)
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
	static ss_pos_t last_cen = {.x=0, .y=0};
	static ss_pos_t last_asc_dir = {.x=0, .y=0};
	static dw_addr_t last_addr = 0;
	static uint8_t send_cond = true;
	dw_recv_info_t recvd;
	static float field_value = 0.0;
	static uint8_t failed_iter_cnt = 0;

	if (failed_iter_cnt > 100)
	{
		consensus_step = SS_ITER_N;
		failed_iter_cnt = 0;
	}

	if (consensus_step == SS_ITER_N)
	{
		reset_cnt++;
		consensus_iter_n++;
		// chprintf((BaseSequentialStream*)&SD1, "FINAL %d Id: %d C: (%.3f,%.3f)\n\n", consensus_iter_n, self_id, position[self_id].x - centroid[self_id].x , position[self_id].y - centroid[self_id].y);
		ss_pos_t norm_asc_dir = asc_dir[self_id];
		float asc_dir_size = sqrt(asc_dir[self_id].x*asc_dir[self_id].x + asc_dir[self_id].y*asc_dir[self_id].y);
		norm_asc_dir.x = asc_dir[self_id].x/asc_dir_size;
		norm_asc_dir.y = asc_dir[self_id].y/asc_dir_size;
		// chprintf((BaseSequentialStream*)&SD1, "FINAL %d Id: %d A: (%.3f,%.3f)\n\n", consensus_iter_n, self_id, norm_asc_dir.x, norm_asc_dir.y);
		memcpy(last_centroid, centroid, sizeof(centroid));
		send_centroid();
		send_asc_dir();

		// NEW ITER
		consensus_step = 0;
		memcpy(comm_row, COMM_GRAPH[self_id], sizeof(comm_row));

		// Get positions or other data
		field_value = get_field();
		switch (self_id)
		{
			case 0:
				field_value += 19.0;
				break;
			case 1:
				field_value += 3.0;
				break;
			case 2:
				field_value += -23.0;
				break;
			default:
				break;
		}
		// chprintf((BaseSequentialStream*)&SD1, "Distance %.3f\n", self_id, field_value);
		if (!SS_SIM_MODE)
			send_source_dist(field_value);
		exchange_positions();
		// chprintf((BaseSequentialStream*)&SD1, "positions ID %u 0 (%.3f,%.3f) 1 (%.3f,%.3f) 2(%.3f,%.3f)\n", self_id, position[0].x, position[0].y, position[1].x, position[1].y, position[2].x, position[2].y);

		prepare_consensus();
		update_centroid();
		asc_dir[self_id].x = field_value*last_centroid[self_id].x;
		asc_dir[self_id].y = field_value*last_centroid[self_id].y;
		// chprintf((BaseSequentialStream*)&SD1, "First asc dir ID %u 0 (%.3f,%.3f) 1 (%.3f,%.3f) 2(%.3f,%.3f)\n", self_id, asc_dir[0].x, asc_dir[0].y, asc_dir[1].x, asc_dir[1].y, asc_dir[2].x, asc_dir[2].y);
	}


	if (memcmp(false_row, comm_row, sizeof(comm_row)) == 0) // All required data received
	{
		last_cen = centroid[self_id];
		last_asc_dir = asc_dir[self_id];

		// chprintf((BaseSequentialStream*)&SD1, "Centroid ID %u 0 (%.3f,%.3f) 1 (%.3f,%.3f) 2(%.3f,%.3f)\n", self_id, centroid[0].x, centroid[0].y, centroid[1].x, centroid[1].y, centroid[2].x, centroid[2].y);
		// chprintf((BaseSequentialStream*)&SD1, "CENTROID Id: %d Step: %u C: (%.3f,%.3f)\n\n", self_id, consensus_step, position[self_id].x - centroid[self_id].x , position[self_id].y - centroid[self_id].y);
		ss_pos_t norm_asc_dir = asc_dir[self_id];
		// float asc_dir_size = sqrt(asc_dir[self_id].x*asc_dir[self_id].x + asc_dir[self_id].y*asc_dir[self_id].y);
		// norm_asc_dir.x = asc_dir[self_id].x/asc_dir_size;
		// norm_asc_dir.y = asc_dir[self_id].y/asc_dir_size;
		// chprintf((BaseSequentialStream*)&SD1, "ASC_DIR Id: %d Step: %u C: (%.3f,%.3f)\n\n", self_id, consensus_step, norm_asc_dir.x, norm_asc_dir.y);
		// chprintf((BaseSequentialStream*)&SD1, "asc dir ID %u step %u 0 (%.3f,%.3f) 1 (%.3f,%.3f) 2(%.3f,%.3f)\n", self_id, consensus_step, asc_dir[0].x, asc_dir[0].y, asc_dir[1].x, asc_dir[1].y, asc_dir[2].x, asc_dir[2].y);
		update_centroid();
		update_asc_dir();

		consensus_step++;
		last_addr = 0;

		memcpy(comm_row, COMM_GRAPH[self_id], sizeof(comm_row));
		send_cond = true;
		failed_iter_cnt = 0;
	}

	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{	
		//	Send broadcast of value
		if (self_id == i && send_cond)
		{

			if (last_addr == 0)
				broad_consensus();
			else
			{
				broad_consensus_last(last_cen, last_asc_dir, last_addr);
				last_addr = 0;
			}
			send_cond = false;
		}
		else 	// Receive
		{
			if (COMM_GRAPH[i][self_id] > 0)
			{
				recvd = dw_recv_tmo(&recv_addr, uwb_recv_buff, sizeof(ss_header_t)+sizeof(*centroid)+sizeof(*asc_dir), TIME_MS2I(60));
				recv_id = is_consensus_device(recv_addr, self_id);

				if (recv_id != 255) // right device
				{
					ss_header_p = (ss_header_t*)uwb_recv_buff;
					// chThdSleepMilliseconds(5);
					// chprintf((BaseSequential	Stream*)&SD1, "Id: %d recvid: %u m_step: %u recv_step: %u value: (%.3f,%.3f)\n\n", self_id, recv_id, consensus_step, ss_header_p->step, ((ss_pos_t*)(uwb_recv_buff+sizeof(ss_header_t)))->x , ((ss_pos_t*)(uwb_recv_buff+sizeof(ss_header_t)))->y);
					if (ss_header_p->step == consensus_step && ss_header_p->type != SS_M_CON_POS) 
					{
						centroid[recv_id] = *((ss_pos_t*)(uwb_recv_buff+sizeof(ss_header_t)));
						asc_dir[recv_id] = *((ss_pos_t*)(uwb_recv_buff+sizeof(ss_header_t)+sizeof(*centroid)));
						// chprintf((BaseSequentialStream*)&SD1, "Id: %d recvid: %u step: %u  CENT: (%.3f,%.3f) ASC: (%.3f,%.3f)\n\n", self_id, recv_id, consensus_step, centroid[recv_id].x, centroid[recv_id].y, asc_dir[recv_id].x, asc_dir[recv_id].y);
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
							// chprintf((BaseSequentialStream*)&SD1, "Wrong iter\n\n");
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
					asc_dir[i] = asc_dir[self_id];
				}
			}
			else
			{
				centroid[i] = centroid[self_id];
				centroid[i].x -= position[self_id].x - position[i].x;
				centroid[i].y -= position[self_id].y - position[i].y;
				asc_dir[i] = asc_dir[self_id];
				
				if (self_id == i+1)
					chThd_rand_wait(50000, 60000);
			} 
		}
	}

	failed_iter_cnt++;
}

void source_device(void)
{
	while (true)
	{
		chThdSleepMilliseconds(1);
		dw_recv_tmo(NULL, NULL, 0, 200);
	}
}

THD_FUNCTION(SS, arg)
{
	(void) arg;

	chThdSleepMilliseconds(5000);

	self_addr = dw_get_addr();

	if (self_addr == field_source)
		source_device();

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
}		// chprintf((BaseSequentialStream*)&SD1, "UPDATE Id: %d Step: %u C: (%.3f,%.3f)\n\n", self_id, consensus_step, position[self_id].x - centroid[self_id].x , position[self_id].y - centroid[self_id].y);