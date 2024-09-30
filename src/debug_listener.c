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

#include "debug_listener.h"

#define MAX_MESSAGE_SIZE 128

THD_FUNCTION(DEBUG_LISTNR, arg)
{
	(void) arg;

	chThdSleepMilliseconds(5000);

	dw_recv_info_t dw_recv_info_def = {.state = 0, .recvd_size = 0, .rx_time = 0};
	dw_recv_info_t dw_recv_info = dw_recv_info_def;
	dw_addr_t recv_addr = 0;
	uint8_t buffer[MAX_MESSAGE_SIZE];
	memset(buffer, 0, sizeof(buffer));

	while (!chThdShouldTerminateX())
	{
		dw_recv_info = dw_recv_tmo(&recv_addr, buffer, MAX_MESSAGE_SIZE, TIME_MS2I(60));
		
		systime_t ch_now = chVTGetSystemTime();

		if (dw_recv_info.state == DW_RECV_OK && recv_addr != NULL && buffer != NULL)
		{
			chprintf((BaseSequentialStream*)&SD1, "{\nRECV-\n\nState: %u\nSize: %u\nRX-time: %lu\nAddress: %u\nSystem time: %lu\nData:\n[", dw_recv_info.state, dw_recv_info.recvd_size, dw_recv_info.rx_time, ch_now, recv_addr);
			for (uint8_t i = 0; i < MAX_MESSAGE_SIZE; i++)
				chprintf((BaseSequentialStream*)&SD1, "%02X", buffer[i]);
			chprintf((BaseSequentialStream*)&SD1, "]\n}\n");
		}

		recv_addr = 0;
		memset(buffer, 0, sizeof(buffer));
		dw_recv_info = dw_recv_info_def;
	}
}