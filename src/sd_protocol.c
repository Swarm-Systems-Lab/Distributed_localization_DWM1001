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

#include "sd_protocol.h"

SerialConfig serial_cfg = 
{
  .speed = 115200,
  .tx_pad  = UART_TX,
  .rx_pad  = UART_RX,
};

mailbox_t uart1_send_queue;
msg_t uart1_send_msgs[UART1_Q_LENGTH];

mailbox_t uart1_recv_queue;
msg_t uart1_recv_msgs[UART1_Q_LENGTH];

serial_packet_t uart1_recv_buff[UART1_Q_LENGTH];

void serial_init(void)
{
	memset(uart1_send_msgs, 0, sizeof(uart1_send_msgs));
	memset(uart1_recv_msgs, 0, sizeof(uart1_recv_msgs));
	chMBObjectInit(&uart1_send_queue, uart1_send_msgs, UART1_Q_LENGTH);
	chMBObjectInit(&uart1_recv_queue, uart1_recv_msgs, UART1_Q_LENGTH);
	sdStart(&SD1, &serial_cfg);
}

THD_FUNCTION(UART_RECEIVER, arg)
{
	(void)arg;

	serial_init();
	chThdSleepMilliseconds(5);
	
	uint8_t current_byte = 0;
	size_t data_cnt = 0;
	sd_read_state_t state = SD_SYNC0;

	while(true)
	{
		current_byte = sdGet(&SD1);

		switch (state)
		{
			case SD_SYNC0:
				data_cnt = 0;
				if (current_byte == 'D')
					state = SD_SYNC1;
				break;
			case SD_SYNC1:
				if (current_byte == 'W')
					state = SD_CLASS;
				else
					state = SD_SYNC0;
				break;
			case SD_CLASS:
				uart1_recv_buff[0].p_class = current_byte;
				state = SD_LENGTH;
				break;
			case SD_LENGTH:
				uart1_recv_buff[0].p_length = current_byte;
				state = SD_DATA;
				break;
			case SD_DATA:
				if (data_cnt < uart1_recv_buff[0].p_length)
				{
					uart1_recv_buff[0].p_data[data_cnt] = current_byte;
					data_cnt++;
				}
				else
				{
					chMBPostTimeout(&uart1_recv_queue, (msg_t)uart1_recv_buff, TIME_US2I(50));
					state = SD_SYNC0;
				}
				break;
		}
		
	}
}

THD_FUNCTION(UART_SENDER, arg)
{
	(void)arg;

	chThdSleepMilliseconds(15);

	serial_packet_t* uart_pend;
	const char sync_message[2] = {'D', 'W'};

	while(true)
	{
		if (chMBFetchTimeout(&uart1_send_queue, (msg_t*)&uart_pend, TIME_MS2I(100)) == MSG_OK)
		{
			sdWrite(&SD1, (const uint8_t*)sync_message, sizeof(sync_message));
			sdWrite(&SD1, (const uint8_t*)uart_pend, uart_pend->p_length+2);
		}
	}
}