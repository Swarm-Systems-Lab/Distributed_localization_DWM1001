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

void check_serial(void)
{
	serial_packet_t* uart_pend;
	char read[2] = {0,0};
	sdReadTimeout(&SD1, read, 2,  TIME_MS2I(50));

	if (strcmp(read, "DW"))
	{
		while (chMBFetchTimeout(&uart1_send_queue, (msg_t*)&uart_pend, TIME_US2I(50)) == MSG_OK)
		{
			chprintf((BaseSequentialStream*)&SD1, "DW");
      		sdWrite(&SD1, (const uint8_t*)uart_pend, uart_pend->p_length+2);
		}
	} 
	else
	{
		size_t char_n_read = sdReadTimeout(&SD1, (const uint8_t*)uart1_recv_buff, 2,  TIME_US2I(50));
		if (char_n_read == 2)
		{
			sdReadTimeout(&SD1, (const uint8_t*)(uart1_recv_buff->p_data), uart1_recv_buff->p_length,  TIME_US2I(50));
			chMBPostTimeout(&uart1_recv_queue, (msg_t)uart1_recv_buff, TIME_US2I(50));
		}
	}
}

THD_FUNCTION(UART_CONTROLLER, arg)
{
	(void)arg;

	serial_init();
	chThdSleepMilliseconds(5);

	while(true)
	{
		check_serial();
		chThdSleepMilliseconds(5);
	}
}