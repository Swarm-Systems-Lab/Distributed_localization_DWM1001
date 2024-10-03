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

THD_FUNCTION(DEBUG_LISTNR, arg)
{
	(void) arg;

	chThdSleepMilliseconds(5000);

	dw_recv_info_t dw_recv_info_def = {.state = 0, .recvd_size = 0, .rx_time = 0};
	dw_recv_info_t dw_recv_info = dw_recv_info_def;
	dw_addr_t recv_addr = 0;
	uint8_t buffer[DL_MAX_MESSAGE_SIZE];
	char hex_string[DL_MAX_MESSAGE_SIZE*4];
	uint32_t mem_offset = 0;
	memset(buffer, 0, sizeof(buffer));

	while (!chThdShouldTerminateX())
	{
		dw_recv_info = dw_recv_tmo(&recv_addr, buffer, DL_MAX_MESSAGE_SIZE, TIME_MS2I(60));
		
		systime_t ch_now = chVTGetSystemTime();

		if (dw_recv_info.state == DW_RECV_OK && recv_addr != 0)
		{
			dp_print(0, 0, "{\nRECV\nState: %u\nSize: %u\nRX-time: %lu\nAddress: %u\nSystem time: %lu\n", dw_recv_info.state, dw_recv_info.recvd_size, dw_recv_info.rx_time, recv_addr, ch_now);

			if (buffer != NULL)
			{
				dp_print(0, 0, "Data:\n[");
				for (uint16_t i = 0; i < DL_MAX_MESSAGE_SIZE; i++)
				{
					if (((i+1)&0x3F) == 0) // Mod 64
						mem_offset += chsnprintf(hex_string+mem_offset, sizeof(hex_string), "\n");

					mem_offset += chsnprintf(hex_string+mem_offset, sizeof(hex_string), "%02X", buffer[i]);
				}
				dp_print(0, 0, "%s", hex_string);
				mem_offset = 0;
				memset(hex_string, 0, sizeof(hex_string));
			}
			dp_print(0, 0, "]\n}\n\n");
		}

		recv_addr = 0;
		memset(buffer, 0, sizeof(buffer));
		dw_recv_info = dw_recv_info_def;
	}
}