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

dw_addr_t identifier_map[SS_DEVICE_NUMBER] = {1955, 3213, 18};

dw_addr_t field_source = 5923;

ss_pos_t ss_ned_pos;

virtual_timer_t comm_slot_timer;

virtual_timer_t consensus_timer;

float init_consensus[SS_DEVICE_NUMBER] = {1.0, 3.0, 0.0};

float consensus_value[SS_DEVICE_NUMBER];

int8_t current_slot = -1;

dw_addr_t self_addr;
size_t self_id;

binary_semaphore_t slot_free;

uint16_t consensus_iter_n = 0;

uint8_t fail_cnt = 0;

void comm_slot_cb(virtual_timer_t* vtp, void* arg)
{
	current_slot++;

	if (current_slot >= SS_DEVICE_NUMBER)
	{
		current_slot = 0;
		update_consensus(consensus_value);
	}

	chSysLockFromISR();
	chBSemSignalI(&slot_free);
	chSysUnlockFromISR();
}

void init_timers(void)
{
	chVTObjectInit(&comm_slot_timer);
}

void send_centroid2d(uint16_t x, uint16_t y)
{
	uart1_send_buff[0].p_class = SS_P_DEBUG;
	uart1_send_buff[0].p_length = 4;
	float a = 45.78;
	memcpy(uart1_send_buff[0].p_data, &a, uart1_send_buff[0].p_length);
	chMBPostTimeout(&uart1_send_queue, (msg_t)&(uart1_send_buff[0]), TIME_US2I(50));
}

void send_asc_2d(void)
{
	uart1_send_buff[0].p_class = SS_P_NED_POS;
	uart1_send_buff[0].p_length = sizeof(ss_ned_pos);

	memcpy(uart1_send_buff[0].p_data, &ss_ned_pos, uart1_send_buff[0].p_length);
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

void update_consensus(float* values)
{
	float consensus_sum = 0;
	consensus_iter_n++;

	if (consensus_iter_n >= SS_ITER_N)
	{
		consensus_iter_n = 0;
		reload_consensus();
	}
	else
	{
		for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
			consensus_sum += values[self_id]-values[i];
		
		values[self_id] += -SS_K_GAIN*consensus_sum/SS_DEVICE_NUMBER;
	}
}

void get_id_from_ap(uint8_t* data, size_t size)
{
	memcpy(identifier_map, data, sizeof(identifier_map));
}

void get_ned_pos(uint8_t* data, size_t size)
{
	memcpy(&ss_ned_pos, data, sizeof(ss_ned_pos));
}

void ss_sync(void)
{
	dw_addr_t recv_addr = 0; 

	switch(self_addr)
	{
		case 1955:
			chThdSleepMilliseconds(30);
			dw_send_tmo(0xFFFF, NULL, 0, DEF_TMO_MS);
			break;

		default:
			while (recv_addr != 1955)
				dw_recv_tmo(&recv_addr, NULL, 0, DEF_TMO_MS);
			break;
	}
}

void run_consensus(void)
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
	
	if (self_addr == field_source)
	{
		while (true)
			dw_recv_tmo(NULL, NULL, 0, TIME_MS2I(1000));
	}

	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{
		if (self_addr == identifier_map[i])
			self_id = i;
	}

	reload_consensus();

	chBSemObjectInit(&slot_free, 1);

	chThdSleepMilliseconds(50);

	init_timers();

	ss_sync();

	chVTSetContinuous(&comm_slot_timer, TIME_US2I(CONSENSUS_PERIOD_US/SS_DEVICE_NUMBER), comm_slot_cb, NULL);

	while(true)
	{
		run_consensus();
		if (consensus_iter_n == SS_ITER_N-1)
			chprintf((BaseSequentialStream*)&SD1, "Id: %d Consensus value: %f\n\n", self_id, consensus_value[self_id]);
	}
}