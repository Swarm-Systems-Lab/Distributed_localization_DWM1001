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

uint16_t identifier_map[SS_DEVICE_NUMBER];

void send_centroid2d(uint16_t x, uint16_t y)
{
	uart1_send_buff[0].p_class = 0;
	uart1_send_buff[0].p_length = 2;
	memcpy(uart1_send_buff[0].p_data, &x, uart1_send_buff[0].p_length);
	chMBPostTimeout(&uart1_send_queue, (msg_t)&(uart1_send_buff[0]), TIME_US2I(50));
}

void send_asc_2d(void)
{

}

void recv_serial(void)
{
	serial_packet_t* recvd_packet;

	chMBFetchTimeout(&uart1_recv_queue, (msg_t*)&recvd_packet, TIME_US2I(50));

	switch(recvd_packet->p_class)
	{
		case P_IDENTITIES:
			get_id_from_ap(recvd_packet);
			break;
		default:
			break;
	}
}

void get_id_from_ap(serial_packet_t* p)
{
	memcpy(identifier_map, p->p_data, sizeof(identifier_map));
}

THD_FUNCTION(SS, arg)
{
	(void) arg;

	chThdSleepMilliseconds(10);

	while(true)
	{
		recv_serial();
		send_centroid2d(4,2);
		chThdSleepMilliseconds(3);
	}
}