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

#ifndef SOURCE_SEEKING_APP_H
#define SOURCE_SEEKING_APP_H

#include "sd_protocol.h"
#include "uwb_comm.h"
#include "math.h"

#define SS_SIM_MODE				1
#define SS_DEVICE_NUMBER		3
#define SS_CONSENSUS_FREQUENCY	2
#define SS_COMM_PERIOD_SLICE	0.5
#define SS_K_GAIN				1
#define CONSENSUS_PERIOD_US		(1000000/SS_CONSENSUS_FREQUENCY)
#define SS_RTOS_DELAY_US		100
#define SS_ITER_N				30

typedef enum ss_packet_types
{
	SS_P_DEBUG			= 0x0,
	SS_P_IDENTITIES,
	SS_P_CONFIRMATION,
	SS_P_NED_POS,
	SS_P_SOURCE_DIST,
	SS_P_CENTROID,
	SS_P_ASC_DIR,
	SS_P_FIELD_MEASURE
} ss_packet_t;

typedef enum ss_uwb_msg_types
{
	SS_M_CON_V			= 0x0,
	SS_M_CON_LV			= 0x1,
	SS_M_CON_POS		= 0x2,
	SS_M_DEBUG
} ss_uwb_msg_t;

typedef struct ss_pos
{
	float x;
	float y;
} ss_pos_t;

typedef struct ss_header
{
	ss_uwb_msg_t type;
	uint16_t step;
} ss_header_t;

extern serial_packet_t uart1_send_buff[UART1_Q_LENGTH];

extern dw_addr_t identifier_map[SS_DEVICE_NUMBER];

extern dw_addr_t field_source;

extern float consensus_value[SS_DEVICE_NUMBER];

extern dw_addr_t self_addr;
extern size_t self_id;

extern uint16_t consensus_iter_n;

extern uint8_t consensus_step;

static const uint8_t COMM_GRAPH[SS_DEVICE_NUMBER][SS_DEVICE_NUMBER] =
{
	{0,1,0},
	{1,0,1},
	{0,1,0}
};

extern THD_FUNCTION(SS, arg);

void get_id_from_ap(uint8_t* data, size_t size);

void get_ned_pos(uint8_t* data, size_t size);

void recv_serial(void);

void update_consensus(void);

void update_centroid(void);

void run_consensus_new(void);

void ss_sync(void);

#endif /* SOURCE_SEEKING_APP_H */