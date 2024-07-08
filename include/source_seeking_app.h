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

#include "dw1000_ch.h"
#include "sd_protocol.h"

#define SS_DEVICE_NUMBER		3

typedef enum ss_packet_types
{
	P_CENTROID		= 0x0,
	P_ASC,
	P_IDENTITIES
} ss_packet_t;

extern serial_packet_t uart1_send_buff[UART1_Q_LENGTH];

extern uint16_t identifier_map[SS_DEVICE_NUMBER];

extern THD_FUNCTION(SS, arg);

void send_centroid2d(uint16_t x, uint16_t y);

void send_asc_2d(void);

void get_id_from_ap(serial_packet_t* p);

void recv_serial(void);
