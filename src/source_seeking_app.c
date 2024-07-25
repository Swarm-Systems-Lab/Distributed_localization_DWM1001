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

dw_addr_t identifier_map[SS_DEVICE_NUMBER] = {2177, 7090, 5923};

dw_addr_t field_source = 1955;

virtual_timer_t comm_slot_timer;

virtual_timer_t consensus_timer;

float init_consensus[SS_DEVICE_NUMBER] = {1.0, 3.0, 0.0};

float consensus_value[SS_DEVICE_NUMBER];

ss_pos_t centroid_c;
ss_pos_t pos_v_sum;

ss_pos_t centroid[SS_DEVICE_NUMBER] = {{.x=0, .y=0}, {.x=0, .y=0}, {.x=0, .y=0}};
ss_pos_t asc_dir[SS_DEVICE_NUMBER];
ss_pos_t position[SS_DEVICE_NUMBER] = {{.x=-1, .y=-1}, {.x=0, .y=2}, {.x=1, .y=-1}};

int8_t current_slot = -1;

dw_addr_t self_addr;
size_t self_id;

binary_semaphore_t slot_free;

uint16_t consensus_iter_n = 0;

uint8_t reset_cnt = 0;

uint8_t fail_cnt = 0;

uint8_t fail_state = 0;

ss_pos_t error_value = {.x=-121212.121212, .y=-121212.121212};

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
			//get_id_from_ap(recvd_packet->p_data, recvd_packet->p_length);
			chThdSleepMilliseconds(20);
			send_confirmation();
			// chThdSleepMilliseconds(1);
			send_confirmation();
			break;
		case SS_P_CONFIRMATION:
			break;
		case SS_P_NED_POS:
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
	
		// if (reset_cnt & 0b1 == 0)
			update_centroid();
		// else
		// 	update_asc_dir();
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
		case 7090:
			chThdSleepMilliseconds(150);
			dw_send_tmo(0xFFFF, &sync_message1, sizeof(sync_message1), DEF_TMO_MS);
			break;

		default:
			while (recv_addr != 7090 || recv_message[0] != sync_message1)
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

void send_error_consensus(void)
{
	dw_send_tmo(0xFFFF, (uint8_t*)(&(error_value)), sizeof(error_value), TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER-SS_RTOS_DELAY_US));
	fail_state = 1;
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
				while (position[self_id].x == 0 && position[self_id].y == 0 && serial_read_fail_cnt < 10)
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
				// if (fail_cnt)
				// 	send_error_consensus();
				// else
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
						chprintf((BaseSequentialStream*)&SD1, "Iter: %d ID: %d failed receive\n\n", consensus_iter_n, self_id);
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

void run_consensus_asc_dir(void)
{
	dw_addr_t recvd_addr = 0;
	dw_recv_info_t dw_recv_info;
	
	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{
		chBSemWait(&slot_free);

		// First iteration
		if (consensus_iter_n == 0)
		{
			// Exchange positions

			if (identifier_map[i] == self_addr)
			{
				// Read position from paparazzi
				recv_serial();
				chThdSleepMilliseconds(5);
				dw_send_tmo(0xFFFF, (uint8_t*)(&(position[i])), sizeof(position[i]), TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER-SS_RTOS_DELAY_US));
				//chprintf((BaseSequentialStream*)&SD1, "Iter: %d %d sent: (%f,%f)\n\n", consensus_iter_n, self_id, position[i].x, position[i].y);
			}
		}
		else
		{
			if (identifier_map[i] == self_addr)
			{
				chThdSleepMilliseconds(5);
				// if (fail_cnt)
				// 	send_error_consensus();
				// else
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
						chprintf((BaseSequentialStream*)&SD1, "Iter: %d ID: %d failed receive\n\n", consensus_iter_n, self_id);
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

void source_device(void)
{
	dw_addr_t recv_addr = 0;
	uint8_t recv_sync[1] = {0};
	uint8_t correct[SS_DEVICE_NUMBER];
	uint8_t all_ready = 0;
	uint8_t synced = 0;
	systime_t time_cnt = 0; 
	systime_t start_time = chVTGetSystemTime();

	while (true)
	{
		dw_recv_tmo(NULL, NULL, 0, TIME_MS2I(100));

		time_cnt = chVTGetSystemTime();

		if (TIME_I2MS(time_cnt-start_time) >= 1500)
		{
			synced = 0;
			all_ready = 0;
			for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
				correct[i] = 0;

			while (!synced)
			{
				for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
				{
					dw_recv_tmo(&recv_addr, recv_sync, sizeof(sync_message1), TIME_MS2I(20));
					
					if (recv_addr == identifier_map[i] && recv_sync[0] == sync_message1)
					{
						chThdSleepMilliseconds(2);
						dw_send_tmo(identifier_map[i], &sync_message1, sizeof(sync_message1), DEF_TMO_MS);
						correct[i] = 1; 
					}

					recv_addr = 0;
					recv_sync[0] = 0;
				}

				for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
				{
					if (correct[i] > 0)
						all_ready++;
				}

				if (all_ready >= SS_DEVICE_NUMBER)
				{
					chThdSleepMilliseconds(20);
					dw_send_tmo(0xFFFF, &sync_message2, sizeof(sync_message2), TIME_MS2I(10));
					synced = 1;
				}
				else
					all_ready = 0;
			}
			start_time = chVTGetSystemTime();
		}
	}
}

void reload_consensus(void)
{
	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
		consensus_value[i] = init_consensus[self_id];
}

THD_FUNCTION(SS, arg)
{
	(void) arg;

	chThdSleepMilliseconds(500);

	self_addr = dw_get_addr();

	// while (identifier_map[0] == 0 && self_addr != field_source)
	// 	recv_serial();

	send_confirmation();
	
	if (self_addr == field_source)
		source_device();

	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{
		if (self_addr == identifier_map[i])
			self_id = i;
	}

	chBSemObjectInit(&slot_free, 1);

	init_timers();

	ss_sync();

	chVTSetContinuous(&comm_slot_timer, TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER), comm_slot_cb, NULL);

	while(!chThdShouldTerminateX())
	{
		// if (reset_cnt & 0b1 == 0)
			run_consensus_centroid();
		// else
		// 	run_consensus_asc_dir();

		if (consensus_iter_n == SS_ITER_N-1)
		{
			// chprintf((BaseSequentialStream*)&SD1, "Id: %d C: (%f,%f)\n\n", self_id, position[self_id].x - centroid[self_id].x , position[self_id].y - centroid[self_id].y);
			// chprintf((BaseSequentialStream*)&SD1, "Id: %d P: (%f,%f) (%f,%f) (%f,%f)\n\n", self_id, position[0].x, position[0].y, position[1].x, position[1].y, position[2].x, position[2].y);
			send_centroid();
			//send_asc_dir();
			reset_cnt++;
		}

		if (reset_cnt > 10 || fail_state)
		{
			// chprintf((BaseSequentialStream*)&SD1, "Reset: %d\n\n", self_id);

			reset_cnt = 0;
			fail_state = 0;

			chVTReset(&comm_slot_timer);
			dw_reset_sys();
			chThdSleepMilliseconds(20);

			chBSemReset(&slot_free, 1);
			ss_sync();
			chVTSetContinuous(&comm_slot_timer, TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER), comm_slot_cb, NULL);
		}
	}
}

THD_FUNCTION(DISTANCE_FIELD, arg)
{
	(void) arg;

	chThdSleepMilliseconds(200);

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