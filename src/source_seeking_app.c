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

dw_addr_t identifier_map[SS_DEVICE_NUMBER] = {3213, 1955, 18};

ss_pos_t ss_ned_pos;

virtual_timer_t comm_slot_timer;

virtual_timer_t consensus_timer;

float consensus_value[SS_DEVICE_NUMBER] = {1.0, 0.0, 0.0};

uint8_t current_slot = 0;
uint8_t current_comm = 0;

dw_addr_t self_addr;
size_t self_id;

binary_semaphore_t slot_free;

static const _comm_slot_data_t _comm_slot_data = 
{
	.current_slot_p = &current_slot,
	.slot_free_p = &slot_free
};

void comm_slot_cb(virtual_timer_t* vtp, void* arg)
{
	 _comm_slot_data_t* slot_data = ((_comm_slot_data_t*)arg);

	(*(slot_data->current_slot_p))++;
	if ((*(slot_data->current_slot_p)) > SS_COMM_SLOT_N)
		(*(slot_data->current_slot_p)) = 0;

	chSysLockFromISR();
	chBSemSignalI(slot_data->slot_free_p);
	chSysUnlockFromISR();

	// chSysLockFromISR();
	// chVTSetI(&comm_slot_timer, TIME_US2I((10000000)/(SS_CONSENSUS_FREQUENCY*COMM_SLOT_N)), comm_slot_cb, NULL);
	// chSysUnlockFromISR();
}

void consensus_cb(virtual_timer_t* vtp, void* arg)
{
	update_consensus((float*)arg);

	// chSysLockFromISR();
	// chVTSetI(&consensus_timer, TIME_MS2I((1000/SS_CONSENSUS_FREQUENCY)), comm_slot_cb, NULL);
	// chSysUnlockFromISR();
}

void init_timers(void)
{
	chVTObjectInit(&consensus_timer);
	chVTObjectInit(&comm_slot_timer);

	chVTSetContinuous(&comm_slot_timer, TIME_US2I((1000000)/(SS_CONSENSUS_FREQUENCY*SS_COMM_SLOT_N)), comm_slot_cb, &_comm_slot_data);
	chVTSetContinuous(&consensus_timer, TIME_MS2I((1000/SS_CONSENSUS_FREQUENCY)), consensus_cb, consensus_value);
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

	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
		consensus_sum += values[self_id]-values[i];
	
	values[self_id] += SS_K_GAIN*consensus_sum/SS_DEVICE_NUMBER;
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

	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{
		for (size_t j = 0; j < SS_DEVICE_NUMBER; j++)
		{
			if (COMM_GRAPH[i][j] > 0 && current_slot == current_comm)
			{
				current_comm++;
				chBSemWait(&slot_free);

				if (self_addr == identifier_map[j])
				{
					float recv_value;
					size_t recvd = dw_recv_tmo(&recvd_addr, (uint8_t*)(&recv_value), sizeof(recv_value), TIME_US2I((1000000)/(SS_CONSENSUS_FREQUENCY*SS_COMM_SLOT_N)));
					if (recvd == sizeof(recv_value))
						consensus_value[i] = recv_value;
				}

				if (self_addr == identifier_map[i])
					dw_send_tmo(identifier_map[j], (uint8_t*)(&consensus_value[i]), sizeof(consensus_value[i]), TIME_US2I((1000000/2)/(SS_CONSENSUS_FREQUENCY*SS_COMM_SLOT_N)));

			}
			if (current_slot == 0)
				current_comm = 0;
		}
	}
}

THD_FUNCTION(SS, arg)
{
	(void) arg;

	chThdSleepMilliseconds(50);

	self_addr = dw_get_addr();

	for (size_t i = 0; i < SS_DEVICE_NUMBER; i++)
	{
		if (self_addr == identifier_map[i])
			self_id = i;
	}

	chBSemObjectInit(&slot_free, 1);

	chThdSleepMilliseconds(50);

	ss_sync();

	init_timers();

	while(true)
	{
		run_consensus();
	}
}