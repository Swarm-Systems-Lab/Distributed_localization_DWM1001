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

#define THREAD_STACK_SIZE	2048
#define NEIGHBOUR_NUM		2

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

#define DW_COMM_OK_E	(EVENT_MASK(30))
#define DW_COMM_F_E		(EVENT_MASK(31))

#define MRXERR_E	(MRXPHE_E | MRXFCE_E | MRXRFSL_E | MRXSFDTO_E | MAFFREJ_E | MLDEERR_E)

#define TX_TIMEOUT	TIME_MS2I(10)
#define CH_TIMEOUT	TIME_S2I(10)
#define DW_ERR_THRESH	10

extern thread_reference_t irq_evt;

extern SPIConfig spi_cfg;
extern SerialConfig serial_cfg;

extern thread_t* dw_thread;
extern thread_t* peer_conn_thread;
extern thread_t* peer_disc_thread;
extern thread_t* comm_thread;

typedef enum loc_state
{
	LOC_STANDBY,
	LOC_DW,
	LOC_DISC,
	LOC_CONN,
	LOC_COM,
	LOC_TWR,
	LOC_DIS_CALC,
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

typedef enum dw_ctrl_req
{
	DW_RECV,
	DW_SEND,
	DW_SEND_W4R,
	DW_SEND_DLY,
	DW_TRX_ERR,
	DW_RESET,
	DW_RECV_TMO,
	DW_CTRL_YIELD
} dw_ctrl_req_t;

typedef enum comm_state
{
	COMM_RECV,
	COMM_SEND,
	COMM_ERR,
	COMM_IDLE
} comm_state_t;

typedef enum conn_state
{
	CONN_SYN_RECV,
	CONN_SYN_SEND,
	CONN_SYN_A_SEND,
	CONN_SYN_A_RECV,
	CONN_RECV,
	CONN_SEND,
	CONN_SEND_ACK,
	CONN_MNT,
	CONN_DIS,
	CONN_ERR,
	CONN_IDLE
} conn_state_t;

typedef enum message_types_dis_loc
{
	MT_BROADCAST	= 0x1,
	MT_SYN,
	MT_SYN_ACK,
	MT_ACK,
	MT_MCONN,
	MT_DISCONN,
	MT_LOC_REQ,
	MT_LOC_RESP,
	MT_OTHER
} message_t;

typedef enum thread_msg
{
	COMM_RECV_CMD		= 0x1,
	COMM_SEND_CMD,
	COMM_SEND_W4R_CMD,
	COMM_SEND_DLY_CMD,
	CONN_SYN_RECV_CMD,
	CONN_SYN_SEND_CMD,
	COMM_ERR_RESP,
	COMM_TMO_RESP,
	COMM_END,
	COMM_OTHER,
	BROAD_RECV_CMD,
	SYN_RECV_CMD,
	CONN_RECV_CMD,
	CONN_SEND_CMD
} thread_msg_t;

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
} peer_connection_t;

typedef struct message_meta
{
	thread_msg_t message;
	peer_connection_t* peer;
	uint8_t seq_num;
	message_t type;
	uint8_t size;
	peer_connection_t* expected_peer;
	message_t expected_message;
} message_meta_t;

typedef struct message_req
{
	thread_msg_t t_message;
	uint16_t expected_addr;
	message_t expected_message;
} message_req_t;

typedef struct message_cmd
{
	thread_msg_t t_message;
	peer_connection_t* peer;
	message_t type;
	uint8_t size;
} message_cmd_t;

typedef struct message_cmd_req
{
	thread_msg_t t_message;
	peer_connection_t* peer;
	message_t type;
	uint8_t size;
	message_t expected_message;
} message_cmd_req_t;



typedef struct peer_loc
{
	uint8_t peer_id;
	peer_connection_t conn;
	double pos_x;
	double pos_y;
	double pos_z;
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

static THD_WORKING_AREA(DW_IRQ_THREAD, 256);
extern THD_FUNCTION(DW_IRQ_HANDLER, arg);

static THD_WORKING_AREA(DW_CONTROLLER_THREAD, THREAD_STACK_SIZE);
extern THD_FUNCTION(DW_CONTROLLER, arg);

static THD_WORKING_AREA(PEER_DISCOVERY_THREAD, THREAD_STACK_SIZE);
extern THD_FUNCTION(PEER_DISCOVERY, arg);

static THD_WORKING_AREA(PEER_CONNECTION_THREAD, THREAD_STACK_SIZE);
extern THD_FUNCTION(PEER_CONNECTION, arg);

static THD_WORKING_AREA(COMMS_THREAD, THREAD_STACK_SIZE);
extern THD_FUNCTION(COMMS, arg);

// static THD_WORKING_AREA(TWR_THREAD, THREAD_STACK_SIZE);
// extern THD_FUNCTION(TWR, arg);

// static THD_WORKING_AREA(DIS_LOC_THREAD, THREAD_STACK_SIZE);
// extern THD_FUNCTION(DIS_LOC, arg);

static THD_WORKING_AREA(SYSTEM_STATUS_THREAD, 256);
extern THD_FUNCTION(SYSTEM_STATUS, arg);

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

void init_peers(void);
peer_connection_t* create_new_peer(uint16_t addr);
peer_connection_t* get_peer(uint16_t addr);

int32_t get_message(message_req_t* msg_req);
void prepare_message(message_cmd_t* msg_cmd);

peer_connection_t* search_peer(uint16_t addr);

void get_distance_to(uint16_t addr);

void set_irq_vector(void);

void read_frame(void);

void CPLOCK_handler(void);
void ESYNCR_handler(void);
void AAT_handler(void);
void TXFRB_handler(void);
void TXPRS_handler(void);
void TXPHS_handler(void);
void TXFRS_handler(void);
void RXPRD_handler(void);
void RXFSDD_handler(void);
void LDEDONE_handler(void);
void RXPHD_handler(void);
void RXPHE_handler(void);
void RXDFR_handler(void);
void RXFCG_handler(void);
void RXFCE_handler(void);
void RXRFSL_handler(void);
void RXRFTO_handler(void);
void LDEERR_handler(void);
void RXOVRR_handler(void);
void RXPTO_handler(void);
void GPIOIRQ_handler(void);
void SLP2INIT_handler(void);
void RFPLLLL_handler(void);
void CPLLLL_handler(void);
void RXSFDTO_handler(void);
void HPDWARN_handler(void);
void TXBERR_handler(void);
void AFFREJ_handler(void);