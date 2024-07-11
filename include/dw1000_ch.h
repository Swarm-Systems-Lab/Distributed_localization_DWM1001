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

#include "dw1000_hal.h"

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

#define MRXERR_E	(MRXPHE_E | MRXFCE_E | MRXRFSL_E | MRXSFDTO_E | MAFFREJ_E | MLDEERR_E)

#define TX_TIMEOUT	TIME_MS2I(10)
#define CH_TIMEOUT	TIME_S2I(5)
#define DW_ERR_THRESH	10

#define MSG_BUFFER_SIZE	128 

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

typedef enum dw_rsp_st
{
	DW_NO_RESP			= 0x0,
	DW_RECV_OK,
	DW_RECV_TMOUT,
	DW_RECV_ERR,
	DW_SEND_OK,
	DW_SEND_ERR,
	DW_SEND_W4R_OK,
	DW_SEND_W4R_S_ERR,
	DW_SEND_W4R_R_ERR,
	DW_SEND_W4R_TMO,
	DW_SEND_DLY_OK,
	DW_SEND_DLY_ERR,
	DW_SEND_DLY_F,
	DW_SYS_ERR,
	DW_BAD_CONF
} dw_rsp_st_t;

typedef struct dw1000_cmd
{
	dw_ctrl_req_t dw_ctrl_req;
	uint32_t dly;
	int32_t wait;
	sysinterval_t tmo;
	size_t size;
	uint8_t send_buf[MSG_BUFFER_SIZE];
} dw1000_cmd_t;

typedef struct dw1000_resp
{
	dw_rsp_st_t state;
	size_t recvd_size;
	uint64_t tx_time;
	uint64_t rx_time;
	uint8_t recv_buf[MSG_BUFFER_SIZE];
} dw1000_resp_t;

extern thread_reference_t irq_evt;
extern thread_t* dw_thread;

extern mailbox_t dw_controller;
extern msg_t dw_controller_msg;

extern mailbox_t dw_controller_resp;
extern msg_t dw_controller_resp_msg;

extern SPIConfig spi_cfg;

extern panadr_t panadr_own;
extern tx_antd_t tx_antd;
extern uint16_t rx_ant_d;

extern uint32_t recv_tmo_usec;

extern dw1000_resp_t dw1000_resp;

extern uint8_t recv_tmo_cnt;

static THD_WORKING_AREA(DW_IRQ_THREAD, 256);
extern THD_FUNCTION(DW_IRQ_HANDLER, arg);

static THD_WORKING_AREA(DW_CONTROLLER_THREAD, 2048);
extern THD_FUNCTION(DW_CONTROLLER, arg);

/**
 * @brief wrapper function for the interrupt service routine
 * 
 * @param arg 
 */
void ISR_wrapper(void * arg);

void _dw_power_on(void);
void _dw_power_off(void);
void spi1_lock(void);
void spi1_unlock(void);
void spi1_set_cs(void);
void spi1_clear_cs(void);
void spi1_send(size_t count, const uint8_t* buf);
void spi1_recv(size_t count, const uint8_t* buf);

void dw_reset(void);

void set_fast_spi_freq(void);
void set_slow_spi_freq(void);

uint64_t get_hardware_id(void);

void spi_hal_init(void);

// TODO document must not be preempted
void load_lde(void);

uint64_t load_ldotune(void);

void set_irq_vector(void);

void dw_setup(void);

void read_frame(void);

uint16_t dw_get_addr(void);
uint16_t dw_get_panid(void);

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