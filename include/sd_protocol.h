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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

// TODO support for more than one thread using uart
#define UART1_USR_NUM	1

#define UART1_Q_LENGTH	1
#define UART_BUFF_SIZE	64

typedef enum sd_read_state
{
	SD_SYNC0,
	SD_SYNC1,
	SD_CLASS,
	SD_LENGTH,
	SD_DATA
} sd_read_state_t;

typedef struct serial_packet
{
	uint8_t p_class;
	uint8_t p_length;
	uint8_t p_data[UART_BUFF_SIZE];
} serial_packet_t;

extern mailbox_t uart1_send_queue;
extern msg_t uart1_send_msgs[UART1_Q_LENGTH];

extern mailbox_t uart1_recv_queue;
extern msg_t uart1_recv_msgs[UART1_Q_LENGTH];

extern serial_packet_t uart1_recv_buff[UART1_Q_LENGTH];

extern SerialConfig serial_cfg;

static THD_WORKING_AREA(UART_RECEIVER_THREAD, 512);
extern THD_FUNCTION(UART_RECEIVER, arg);

static THD_WORKING_AREA(UART_SENDER_THREAD, 512);
extern THD_FUNCTION(UART_SENDER, arg);

void serial_init(void);