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

#include "LR-WPANs_MAC.h"

void get_MHR(frame_control_t frame_control, uint8_t* MHR, uint8_t seq_num, uint64_t dest_pan_id, uint64_t dest_addr)
{
	memcpy(MHR, &frame_control.mask, 2);
	memcpy(MHR+2, &seq_num, 1);
	//uint16_t pan_id = get_pan_id();
	//memcpy(MHR+3, &pan_id, 2);
	//if (frame_control.pan_id_compress)
}