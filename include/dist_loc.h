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
#include "led.h"
#include "chprintf.h"

#include "dw1000_hal.h"
#include "LR-WPANs_MAC.h"

#define THREAD_STACK_SIZE	4096
#define NEIGHBOUR_NUM		1

#define MCPLOCK_E	(EVENT_MASK(1))
#define MESYNCR_E	(EVENT_MASK(2))
#define MAAT_E		(EVENT_MASK(3))
#define MTXFRB_E	(EVENT_MASK(4))
#define MTXPRS_E	(EVENT_MASK(5))
#define MTXPHS_E	(EVENT_MASK(6))
#define MTXFRS_E	(EVENT_MASK(7))
#define MRXPRD_E	(EVENT_MASK(8))
#define MRXFSDD_E	(EVENT_MASK(9))
#define MLDEDONE_E	(EVENT_MASK(10))
#define MRXPHD_E	(EVENT_MASK(11))
#define MRXPHE_E	(EVENT_MASK(12))
#define MRXDFR_E	(EVENT_MASK(13))
#define MRXFCG_E	(EVENT_MASK(14))
#define MRXFCE_E	(EVENT_MASK(15))
#define MRXRFSL_E	(EVENT_MASK(16))
#define MRXRFTO_E	(EVENT_MASK(17))
#define MLDEERR_E	(EVENT_MASK(18))
#define MRXOVRR_E	(EVENT_MASK(20))
#define MRXPTO_E	(EVENT_MASK(21))
#define MGPIOIRQ_E	(EVENT_MASK(22))
#define MSLP2INIT_E	(EVENT_MASK(23))
#define MRFPLLLL_E	(EVENT_MASK(24))
#define MCPLLLL_E	(EVENT_MASK(25))
#define MRXSFDTO_E	(EVENT_MASK(26))
#define MHPDWARN_E	(EVENT_MASK(27))
#define MTXBERR_E	(EVENT_MASK(28))
#define MAFFREJ_E	(EVENT_MASK(29))

extern thread_reference_t irq_evt;

extern SPIConfig spi_cfg;
extern SerialConfig serial_cfg;

typedef enum loc_state
{
	LOC_STANDBY,
	LOC_DISC,
	LOC_INIT,
	LOC_RESP,
	LOC_TXERR,
	LOC_RXERR,
	LOC_ERR
} loc_state_t;

typedef enum disc_state
{
	DISC_INIT,
	DISC_BROAD,
	DISC_WAIT_RX,
	DISC_TX_ERR,
	DISC_RX_ERR,
	DISC_3WH,
	DISC_IDLE
} disc_state_t;

typedef enum message_types_dis_loc
{
	MT_BROADCAST	= 0x1,
	MT_SYN,
	MT_MCONN,
	MT_DISCONN,
	MT_LOC_REQ,
	MT_LOC_RESP,
	MT_OTHER
} message_t;

typedef struct address_list
{
	uint16_t addrs[NEIGHBOUR_NUM];
	int8_t last_p;
} address_list_t;

typedef struct connection_peer
{
	uint16_t peer_addr;
	uint8_t seq_num;
	uint8_t ack_num;
	uint8_t ttl;
} connection_t;

void ISR_wrapper(void * arg);

void _dw_power_on(void);
void _dw_power_off(void);
void spi1_lock(void);
void spi1_unlock(void);
void spi1_set_cs(void);
void spi1_clear_cs(void);
void spi1_send(size_t count, const uint8_t* buf);
void spi1_recv(size_t count, const uint8_t* buf);

static THD_WORKING_AREA(DW_IRQ_THREAD, THREAD_STACK_SIZE);
extern THD_FUNCTION(DW_IRQ_HANDLER, arg);

static THD_WORKING_AREA(DW_CONTROLLER_THREAD, THREAD_STACK_SIZE);
extern THD_FUNCTION(DW_CONTROLLER, arg);

void dw_reset(void);

void set_fast_spi_freq(void);
void set_slow_spi_freq(void);

uint64_t get_hardware_id(void);

void spi_hal_init(void);

// TODO document must not be preempted
// TODO solve magic number for size
void load_lde(void);

uint64_t load_ldotune(void);

double loc_init_fun(uint16_t addr);
void loc_resp_fun(void);
void loc_disc_fun(void);

int8_t insert_addr(uint16_t addr);
int8_t search_addr(uint16_t addr);
void init_neigh(void);

int32_t _send_message(uint16_t addr, uint8_t* message, size_t size, uint8_t w4r, uint32_t w_time, uint32_t dly_time);
int32_t send_message(uint16_t addr, uint8_t* message, size_t size);
int32_t send_message_w4r(uint16_t addr, uint8_t* message, size_t size, uint32_t time, uint16_t* recv_addr, size_t* recv_size);
int32_t send_message_delay(uint16_t addr, uint8_t* message, size_t size, uint32_t time);
void get_message(uint16_t* addr, size_t* size);
int32_t recv_message(uint16_t* addr, size_t* size, uint32_t timeout);
int32_t recv_disc(void);

void get_distance_to(uint16_t addr);

static THD_WORKING_AREA(SYSTEM_STATUS_THREAD, THREAD_STACK_SIZE);
static THD_FUNCTION(SYSTEM_STATUS, arg);

// static THD_WORKING_AREA(LOCATOR_THREAD, THREAD_STACK_SIZE);
// static THD_FUNCTION(LOCATOR, arg);
