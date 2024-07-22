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

#include "test_app.h"


THD_FUNCTION(TEST_APP, arg)
{
	(void) arg;

	chThdSleepMilliseconds(200);

	dw_addr_t self_addr = dw_get_addr();

	dw_addr_t source = 1955;
	dw_addr_t tag = 3213;

	dw_recv_info_t dw_recv_info;

	float distance = 0.0;
	uint32_t cnt = 0;

	while(true)
	{
		distance = 0.0;
		if (self_addr == source)
			dw_recv_tmo(NULL, NULL, 0, TIME_MS2I(1000));
		else if (self_addr == tag)
			dw_recv_info = dw_sstwr(source, NULL, 0, (uint8_t*)(&distance), sizeof(distance));
		chThdSleepMilliseconds(60);
		chprintf((BaseSequentialStream*)&SD1, "%fm\n", distance);
	}
}