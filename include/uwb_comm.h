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
#include "LR-WPANs_MAC.h"

#define DEF_TMO_MS			TIME_MS2I(50)

// TODO ifdef needed to support 64 bit addrs
typedef uint16_t dw_addr_t;

// MHR.frame_control.frame_type = FT_DATA;
// MHR.frame_control.sec_en = 0b0;
// MHR.frame_control.frame_pending = 0b0;
// MHR.frame_control.ack_req = 0b0;
// MHR.frame_control.pan_id_compress = 0b1;
// MHR.frame_control.dest_addr_mode = SHORT_16;
// MHR.frame_control.frame_version = 0x1;
// MHR.frame_control.src_addr_mode = SHORT_16;
static const frame_control_t def_frame_ctrl = {.mask=0x9841};

int64_t dw_send_tmo(dw_addr_t addr, uint8_t* send_data, size_t size, sysinterval_t tmo);
size_t dw_recv_tmo(dw_addr_t* addr, uint8_t* recv_data, size_t size, sysinterval_t tmo);