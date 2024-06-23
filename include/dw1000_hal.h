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

#define TRX_RST_TM		10
#define CAR_INT_CTE 	((998.4e6/2.0/1024.0/131072.0) * (-1.0e6/6489.6e6) / 1.0e6)
#define DW_TIME_U		(499.2e6*128.0)
#define C_ATM			299702547

extern uint64_t hardware_id;

typedef enum dw_reg_file_perms
{
	RO,
	WO,
	RW,
	SRW,		// Special Read Write, refer to docs
	ROD			// Read only double buffer
} reg_perm;

typedef enum dw_tx_states
{
	TX_IDLE,
	TX_PREAMBLE,
	TX_SFD,
	TX_PHR,		// Transmitting PHY header data
	TX_SDE,		// Transmitting PHR SECDED bits
	TX_DATA		// Transmitting data block (330 symbols)
} tx_state;

typedef enum dw_rx_states
{
	RX_IDLE			= 0x00,
	START_ANALOG	= 0x01,		// Start analog receiver blocks
	RX_RDY			= 0x04,
	PREAMBLE_FND	= 0x05,		// Receiver is waiting to detect preamble
	PRMBL_TIMEOUT	= 0x06,
	SFD_FND			= 0x07,
	CNFG_PHR_RX		= 0x08,		// Configure for PHR reception
	PHR_RX_STRT		= 0x09,
	DATA_RATE_RDY	= 0x0A,		// Ready for data reception	
	DATA_RX_SEQ		= 0x0C,
	CNFG_DATA_RX	= 0x0D,
	PHR_NOT_OK		= 0x0E,
	LAST_SYMBOL		= 0x0F,		// Received last symbol
	WAIT_RSD_DONE	= 0x10,		// Wait for Reed Solomon decoder to finish
	RSD_OK			= 0x11,
	RSD_NOT_OK		= 0x12,
	RECONFIG_110	= 0x13,
	WAIT_110_PHR	= 0x14
} rx_state;

typedef enum dw_pmsc_states
{
	INIT,
	IDLE,
	TX_WAIT,
	RX_WAIT,
	TX,
	RX
} pmsc_state;

typedef enum dw_phr_mode
{
	STANDARD = 0, 
	LONG = 3
} phr_mode_t;

typedef enum dw_otp_map
{
	EUID0 			= 0x00,
	EUID1 			= 0x01,
	AEUID0 			= 0x02,
	AEUID1 			= 0x03,
	LDOTUNE0 		= 0x04,
	LDOTUNE1 		= 0x05,
	PARTID 			= 0x06,
	LOTID 			= 0x07,
	VOLT 			= 0x08,
	TEMP 			= 0x09,
	TXP_CH1_PRF16 	= 0x10,
	TXP_CH1_PRF64	= 0x11,
	TXP_CH2_PRF16	= 0x12,
	TXP_CH2_PRF64	= 0x13,
	TXP_CH3_PRF16 	= 0x14,
	TXP_CH3_PRF64 	= 0x15,
	TXP_CH4_PRF16 	= 0x16,
	TXP_CH4_PRF64 	= 0x17,
	TXP_CH5_PRF16 	= 0x18,
	TXP_CH5_PRF64 	= 0x19,
	TXP_CH7_PRF16 	= 0x1A,
	TXP_CH7_PRF64 	= 0x1B,
	ANTD		 	= 0x1C,
	USER0		 	= 0x1D,
	XTAL_TRIM	 	= 0x1E,
	USER1		 	= 0x1F
} otp_map;

typedef enum dw_tx_bitrate
{
	BR_110KBPS, 
	BR_850KBPS,
	BR_6_8MBPS
} txbr_t;

typedef enum dw_tx_prf
{
	PRF_4MHZ, 
	PRF_16MHZ,
	PRF_64MHZ
} txprf_t;

typedef enum dw_preamble_length
{
	PL_16		= 0x0, 
	PL_64		= 0x1,
	PL_128		= 0x5,
	PL_256		= 0x9,
	PL_512		= 0xD,
	PL_1024		= 0x2,
	PL_1536		= 0x6,
	PL_2048		= 0xA,
	PL_4096		= 0x3
} preamble_length_t;

// typedef enum dw_modes

// TODO priority for interrupts?

typedef struct dw_spi_hal
{
	void (*_dw_spi_lock)(void);
	void (*_dw_spi_unlock)(void);
	void (*_dw_spi_set_cs)(void);
	void (*_dw_spi_clear_cs)(void);
	void (*_dw_spi_send)(size_t, const uint8_t*);
	void (*_dw_spi_recv)(size_t, const uint8_t*);
} spi_hal_t;

typedef struct dw_irq_vector
{
	union
	{
		struct
		{
			void (*_reserved_irq0)(void);
			void (*_dw_CPLOCK_handler)(void);
			void (*_dw_ESYNCR_handler)(void);
			void (*_dw_AAT_handler)(void);
			void (*_dw_TXFRB_handler)(void);
			void (*_dw_TXPRS_handler)(void);
			void (*_dw_TXPHS_handler)(void);
			void (*_dw_TXFRS_handler)(void);
			void (*_dw_RXPRD_handler)(void);
			void (*_dw_RXFSDD_handler)(void);
			void (*_dw_LDEDONE_handler)(void);
			void (*_dw_RXPHD_handler)(void);
			void (*_dw_RXPHE_handler)(void);
			void (*_dw_RXDFR_handler)(void);
			void (*_dw_RXFCG_handler)(void);
			void (*_dw_RXFCE_handler)(void);
			void (*_dw_RXRFSL_handler)(void);
			void (*_dw_RXRFTO_handler)(void);
			void (*_dw_LDEERR_handler)(void);
			void (*_reserved_irq19)(void);
			void (*_dw_RXOVRR_handler)(void);
			void (*_dw_RXPTO_handler)(void);
			void (*_dw_GPIOIRQ_handler)(void);
			void (*_dw_SLP2INIT_handler)(void);
			void (*_dw_RFPLL_LL_handler)(void);
			void (*_dw_CLKPLL_LL_handler)(void);
			void (*_dw_RXSFDTO_handler)(void);
			void (*_dw_HPDWARN_handler)(void);
			void (*_dw_TXBERR_handler)(void);
			void (*_dw_AFFREJ_handler)(void);
			void (*_reserved_irq30)(void);
			void (*_reserved_irq31)(void);
		};
		void (*vector[32])(void);
	};
} irq_vector_t;

typedef struct dw_op_mode
{
	txbr_t data_rate;
	txprf_t prf;
	preamble_length_t preamble_length;
} dw_op_mode_t;

typedef struct dw_spi_header
{
	struct
	{
		uint8_t id			:6;
		uint8_t subindex	:1;
		uint8_t is_write	:1;
		uint8_t l_sub_addr	:7;
		uint8_t ext_addr	:1;
		uint8_t h_sub_addr;	
	} header;
	size_t size;
} spi_header_t;

typedef struct dw_reg_metadata
{
	uint8_t id;
	size_t size;
	reg_perm perm;
} reg_metadata_t;

typedef struct dw_subreg_metadata
{
	reg_metadata_t parent;
	uint16_t offset;
	size_t size;
	reg_perm perm;
} subreg_metadata_t;

struct dw_register_set
{
    reg_metadata_t DEV_ID;       
    reg_metadata_t EUI;          
    reg_metadata_t PAN_ADR;      
    reg_metadata_t SYS_CFG;       
    reg_metadata_t SYS_TIME;     
    reg_metadata_t TX_FCTRL;     
    reg_metadata_t TX_BUFFER;    
    reg_metadata_t DX_TIME;      
    reg_metadata_t RX_FWTO;      
    reg_metadata_t SYS_CTRL;     
    reg_metadata_t SYS_MASK;     
    reg_metadata_t SYS_STATUS;   
    reg_metadata_t RX_FINFO;     
    reg_metadata_t RX_BUFFER;    
    reg_metadata_t RX_FQUAL;     
    reg_metadata_t RX_TTCKI;     
    reg_metadata_t RX_TTCKO;     
    reg_metadata_t RX_TIME;      
    reg_metadata_t TX_TIME;      
    reg_metadata_t TX_ANTD;      
    reg_metadata_t SYS_STATE;    
    reg_metadata_t ACK_RESP_T;   
    reg_metadata_t RX_SNIFF;     
    reg_metadata_t TX_POWER;     
    reg_metadata_t CHAN_CTRL;    
    reg_metadata_t USR_SFD;      
    reg_metadata_t AGC_CTRL;     
    reg_metadata_t EXT_SYNC;     
    reg_metadata_t ACC_MEM;      
    reg_metadata_t GPIO_CTRL;    
    reg_metadata_t DRX_CONF;     
    reg_metadata_t RF_CONF;      
    reg_metadata_t TX_CAL;       
    reg_metadata_t FS_CTRL;      
    reg_metadata_t AON;          
    reg_metadata_t OTP_IF;       
    reg_metadata_t LDE_CTRL;       
    reg_metadata_t DIG_DIAG;     
    reg_metadata_t PMSC;         
}; 

static const struct dw_register_set DW_REG_INFO = 
{
	{0x00, 4, RO},		// DEV_ID
	{0x01, 8, RW},		// EUI
	{0x03, 4, RW},		// PAN_ADR
	{0x04, 4, RW},		// SYS_CFG
	{0x06, 5, RO},		// SYS_TIME
	{0x08, 5, RW},		// TX_FCTRL
	{0x09, 1024, WO},	// TX_BUFFER
	{0x0A, 5, RW},		// DX_TIME
	{0X0C, 2, RW},		// RX_FWTO
	{0X0D, 4, SRW},		// SYS_CTRL
	{0X0E, 4, RW},		// SYS_MASK
	{0X0F, 5, SRW},		// SYS_STATUS
	{0x10, 4, ROD},		// RX_FINFO
	{0X11, 1024, ROD},	// RX_BUFFER
	{0X12, 8, ROD},		// RX_FQUAL
	{0X13, 4, ROD},		// RX_TTCKI
	{0X14, 5, ROD},		// RX_TTCKO
	{0X15, 14, ROD},	// RX_TIME
	{0X17, 10, RO},		// TX_TIME
	{0X18, 2, RW},		// TX_ANTD
	{0X19, 5, RO},		// SYS_STATE
	{0X1A, 4, RW},		// ACK_RESP_T
	{0X1D, 4, RW},		// RX_SNIFF
	{0X1E, 4, RW},		// TX_POWER
	{0X1F, 4, RW},		// CHAN_CTRL
	{0X21, 41, RW},		// USR_SFD
	{0X23, 33, RW},		// AGC_CTRL
	{0X24, 12, RW},		// EXT_SYNC
	{0X25, 4064, RO},	// ACC_MEM
	{0X26, 44, RW},		// GPIO_CTRL
	{0X27, 44, RW},		// DRX_CONF
	{0x28, 58, RW},		// RF_CONF
	{0X2A, 52, RW},		// TX_CAL
	{0X2B, 21, RW},		// FS_CTRL
	{0X2C, 12, RW},		// AON
	{0X2D, 18, RW}, 	// OTP_IF		OTP TODO CHECK DOC AS IT IS 18
	{0X2E, 13, RW},		// LDE_CTRL
	{0X2F, 41, RW},		// DIG_DIAG
	{0X36, 48, RW}		// PMSC
};

struct dw_subregister_set
{	
	// AGC_CTRL
	subreg_metadata_t AGC_CTRL1;
	subreg_metadata_t AGC_TUNE1;
	subreg_metadata_t AGC_TUNE2;
	subreg_metadata_t AGC_TUNE3;
	subreg_metadata_t AGC_STAT1;

	// EXT_SYNC
	subreg_metadata_t EC_CTRL;
	subreg_metadata_t EC_RXTC;
	subreg_metadata_t EC_GOLP;

	// GPIO_CTRL
	subreg_metadata_t GPIO_MODE;
	subreg_metadata_t GPIO_DIR;
	subreg_metadata_t GPIO_DOUT;
	subreg_metadata_t GPIO_IRQE;
	subreg_metadata_t GPIO_ISEN;
	subreg_metadata_t GPIO_IMODE;
	subreg_metadata_t GPIO_IBES;
	subreg_metadata_t GPIO_ICLR;
	subreg_metadata_t GPIO_IDBE;
	subreg_metadata_t GPIO_RAW;

	// DRX_CONF
	subreg_metadata_t DRX_TUNE0b;
	subreg_metadata_t DRX_TUNE1a;
	subreg_metadata_t DRX_TUNE1b;
	subreg_metadata_t DRX_TUNE2;
	subreg_metadata_t DRX_SFDTOC;
	subreg_metadata_t DRX_PRETOC;
	subreg_metadata_t DRX_TUNE4H;
	subreg_metadata_t DRX_CAR_INT;
	subreg_metadata_t RXPACC_NOSAT;
	
	// RF_CONF
	subreg_metadata_t RF_CONF;
	subreg_metadata_t RF_RXCTRLH;
	subreg_metadata_t RF_TXCTRL;
	subreg_metadata_t RF_STATUS;
	subreg_metadata_t LDO_TUNE;

	// TX_CAL
	subreg_metadata_t TC_SARC;
	subreg_metadata_t TC_SARL;
	subreg_metadata_t TC_SARW;
	subreg_metadata_t TC_PG_CTRL;
	subreg_metadata_t TC_PG_STATUS;
	subreg_metadata_t TC_PGDELAY;
	subreg_metadata_t TC_PGTEST;

	// FS_CTRL
	subreg_metadata_t FS_PLLCFG;
	subreg_metadata_t FS_PLLTUNE;
	subreg_metadata_t FS_XTALT;

	// AON
	subreg_metadata_t AON_WCFG;
	subreg_metadata_t AON_CTRL;
	subreg_metadata_t AON_RDAT;
	subreg_metadata_t AON_ADDR;
	subreg_metadata_t AON_CFG0;
	subreg_metadata_t AON_CFG1;

	// OTP_IF
	subreg_metadata_t OTP_WDAT;
	subreg_metadata_t OTP_ADDR;
	subreg_metadata_t OTP_CTRL;
	subreg_metadata_t OTP_STAT;
	subreg_metadata_t OTP_RDAT;
	subreg_metadata_t OTP_SRDAT;
	subreg_metadata_t OTP_SF;

	// LDE_CTRL
	subreg_metadata_t LDE_THRESH;
	subreg_metadata_t LDE_CFG1;
	subreg_metadata_t LDE_PPINDX;
	subreg_metadata_t LDE_PPAMPL;
	subreg_metadata_t LDE_RXANTD;
	subreg_metadata_t LDE_CFG2;
	subreg_metadata_t LDE_REPC;

	// DIG_DIAG
	subreg_metadata_t EVC_CTRL;
	subreg_metadata_t EVC_PHE;
	subreg_metadata_t EVC_RSE;
	subreg_metadata_t EVC_FCG;
	subreg_metadata_t EVC_FCE;
	subreg_metadata_t EVC_FFR;
	subreg_metadata_t EVC_OVR;
	subreg_metadata_t EVC_STO;
	subreg_metadata_t EVC_PTO;
	subreg_metadata_t EVC_FWTO;
	subreg_metadata_t EVC_TXFS;
	subreg_metadata_t EVC_HPW;
	subreg_metadata_t EVC_TPW;
	subreg_metadata_t DIAG_TMC;

	// PMSC
	subreg_metadata_t PMSC_CTRL0;
	subreg_metadata_t PMSC_CTRL1;
	subreg_metadata_t PMSC_SNOZT;
	subreg_metadata_t PMSC_TXFSEQ;
	subreg_metadata_t PMSC_LEDC;
};

static const struct dw_subregister_set DW_SUBREG_INFO = 
{
	{DW_REG_INFO.AGC_CTRL, 0x2, 2, RW},	 		// AGC_CTRL1
	{DW_REG_INFO.AGC_CTRL, 0x4, 2, RW},  		// AGC_TUNE1
	{DW_REG_INFO.AGC_CTRL, 0xC, 4, RW},  		// AGC_TUNE2
	{DW_REG_INFO.AGC_CTRL, 0x12, 2, RW}, 		// AGC_TUNE3
	{DW_REG_INFO.AGC_CTRL, 0x1E, 3, RO}, 		// AGC_STAT1

	{DW_REG_INFO.EXT_SYNC, 0x0, 4, RW},			// EC_CTRL
	{DW_REG_INFO.EXT_SYNC, 0x4, 4, RO}, 		// EC_RXTC
	{DW_REG_INFO.EXT_SYNC, 0x8, 4, RO}, 		// EC_GOLP

	{DW_REG_INFO.GPIO_CTRL, 0x0, 4, RW},		// GPIO_MODE
	{DW_REG_INFO.GPIO_CTRL, 0x8, 4, RW},		// GPIO_DIR
	{DW_REG_INFO.GPIO_CTRL, 0xC, 4, RW},		// GPIO_DOUT
	{DW_REG_INFO.GPIO_CTRL, 0x10, 4, RW},		// GPIO_IRQE
	{DW_REG_INFO.GPIO_CTRL, 0x14, 4, RW},		// GPIO_ISEN
	{DW_REG_INFO.GPIO_CTRL, 0x18, 4, RW},		// GPIO_IMODE
	{DW_REG_INFO.GPIO_CTRL, 0x1C, 4, RW},		// GPIO_IBES
	{DW_REG_INFO.GPIO_CTRL, 0x20, 4, RW},		// GPIO_ICLR
	{DW_REG_INFO.GPIO_CTRL, 0x24, 4, RW},		// GPIO_IDBE
	{DW_REG_INFO.GPIO_CTRL, 0x28, 4, RO},		// GPIO_RAW

	{DW_REG_INFO.DRX_CONF, 0x2, 2, RW},			// DRX_TUNE0b
	{DW_REG_INFO.DRX_CONF, 0x4, 2, RW},			// DRX_TUNE1a
	{DW_REG_INFO.DRX_CONF, 0x6, 2, RW},			// DRX_TUNE1b
	{DW_REG_INFO.DRX_CONF, 0x8, 4, RW},			// DRX_TUNE2
	{DW_REG_INFO.DRX_CONF, 0x20, 2, RW},		// DRX_SFDTOC
	{DW_REG_INFO.DRX_CONF, 0x24, 2, RW},		// DRX_PRETOC
	{DW_REG_INFO.DRX_CONF, 0x26, 2, RW},		// DRX_TUNE4H
	{DW_REG_INFO.DRX_CONF, 0x28, 3, RO},		// DRX_CAR_INT
	{DW_REG_INFO.DRX_CONF, 0x2C, 2, RO},		// RXPACC_NOSAT

	{DW_REG_INFO.RF_CONF, 0x0, 4, RW},			// RF_CONF
	{DW_REG_INFO.RF_CONF, 0xB, 1, RW},			// RF_RXCTRLH
	{DW_REG_INFO.RF_CONF, 0xC, 3, RW},			// RF_TXCTRL
	{DW_REG_INFO.RF_CONF, 0x2C, 4, RO},			// RF_STATUS
	{DW_REG_INFO.RF_CONF, 0x30, 5, RW},			// LDO_TUNE

	{DW_REG_INFO.TX_CAL, 0x0, 2, RW},			// TC_SARC
	{DW_REG_INFO.TX_CAL, 0x3, 3, RO},			// TC_SARL
	{DW_REG_INFO.TX_CAL, 0x6, 2, RO},			// TC_SARW
	{DW_REG_INFO.TX_CAL, 0x8, 1, RW},			// TC_PG_CTRL
	{DW_REG_INFO.TX_CAL, 0x9, 2, RO},			// TC_PG_STATUS
	{DW_REG_INFO.TX_CAL, 0xB, 1, RW},			// TC_PGDELAY
	{DW_REG_INFO.TX_CAL, 0xC, 1, RW},			// TC_PGTEST

	{DW_REG_INFO.FS_CTRL, 0x7, 4, RW},			// FS_PLLCFG
	{DW_REG_INFO.FS_CTRL, 0xB, 1, RW},			// FS_PLLTUNE
	{DW_REG_INFO.FS_CTRL, 0xE, 1, RW},			// FS_XTALT

	{DW_REG_INFO.AON, 0x0, 2, RW},				// AON_WCFG
	{DW_REG_INFO.AON, 0x2, 1, RW},				// AON_CTRL
	{DW_REG_INFO.AON, 0x3, 1, RW},				// AON_RDAT
	{DW_REG_INFO.AON, 0x4, 1, RW},				// AON_ADDR
	{DW_REG_INFO.AON, 0x6, 4, RW},				// AON_CFG0
	{DW_REG_INFO.AON, 0xA, 2, RW},				// AON_CFG1

	{DW_REG_INFO.OTP_IF, 0x0, 4, RW},			// OTP_WDAT
	{DW_REG_INFO.OTP_IF, 0x4, 2, RW},			// OTP_ADDR
	{DW_REG_INFO.OTP_IF, 0x6, 2, RW},			// OTP_CTRL
	{DW_REG_INFO.OTP_IF, 0x8, 2, RW},			// OTP_STAT
	{DW_REG_INFO.OTP_IF, 0xA, 4, RO},			// OTP_RDAT
	{DW_REG_INFO.OTP_IF, 0xE, 4, RW},			// OTP_SRDAT
	{DW_REG_INFO.OTP_IF, 0x12, 1, RW},			// OTP_SF

	{DW_REG_INFO.LDE_CTRL, 0x0000, 2, RO},		// LDE_THRESH	
	{DW_REG_INFO.LDE_CTRL, 0x0806, 1, RW},		// LDE_CFG1	
	{DW_REG_INFO.LDE_CTRL, 0x1000, 2, RO},		// LDE_PPINDX	
	{DW_REG_INFO.LDE_CTRL, 0x1002, 2, RO},		// LDE_PPAMPL	
	{DW_REG_INFO.LDE_CTRL, 0x1804, 2, RW},		// LDE_RXANTD	
	{DW_REG_INFO.LDE_CTRL, 0x1806, 2, RW},		// LDE_CFG2	
	{DW_REG_INFO.LDE_CTRL, 0x2804, 2, RW},		// LDE_REPC	

	{DW_REG_INFO.DIG_DIAG, 0x00, 4, SRW},		// EVC_CTRL
	{DW_REG_INFO.DIG_DIAG, 0x04, 2, RO},		// EVC_PHE
	{DW_REG_INFO.DIG_DIAG, 0x06, 2, RO},		// EVC_RSE
	{DW_REG_INFO.DIG_DIAG, 0x08, 2, RO},		// EVC_FCG
	{DW_REG_INFO.DIG_DIAG, 0x0A, 2, RO},		// EVC_FCE
	{DW_REG_INFO.DIG_DIAG, 0x0C, 2, RO},		// EVC_FFR
	{DW_REG_INFO.DIG_DIAG, 0x0E, 2, RO},		// EVC_OVR
	{DW_REG_INFO.DIG_DIAG, 0x10, 2, RO},		// EVC_STO
	{DW_REG_INFO.DIG_DIAG, 0x12, 2, RO},		// EVC_PTO
	{DW_REG_INFO.DIG_DIAG, 0x14, 2, RO},		// EVC_FWTO
	{DW_REG_INFO.DIG_DIAG, 0x16, 2, RO},		// EVC_TXFS
	{DW_REG_INFO.DIG_DIAG, 0x18, 2, RO},		// EVC_HPW
	{DW_REG_INFO.DIG_DIAG, 0x1A, 2, RO},		// EVC_TPW
	{DW_REG_INFO.DIG_DIAG, 0x24, 2, RW},		// DIAG_TMC

	{DW_REG_INFO.PMSC, 0x00, 4, RW},			// PMSC_CTRL0
	{DW_REG_INFO.PMSC, 0x04, 4, RW},			// PMSC_CTRL1
	{DW_REG_INFO.PMSC, 0x0C, 1, RW},			// PMSC_SNOZT
	{DW_REG_INFO.PMSC, 0x26, 2, RW},			// PMSC_TXFSEQ
	{DW_REG_INFO.PMSC, 0x28, 4, RW}				// PMSC_LEDC
};

extern spi_hal_t _dw_spi_hal_set;

extern irq_vector_t irq_vector;

typedef struct dw_dev_id
{
	union 
	{
		struct
		{
			uint8_t	rev	:4;
			uint8_t ver	:4;
			uint8_t model;
			uint16_t ridtag;
		};
		uint8_t reg[4]; 
	};
} dev_id_t;

typedef uint64_t eui_t;

typedef struct dw_panadr
{
	union
	{
		struct
		{
			uint16_t short_addr;
			uint16_t pan_id;
		};
		uint8_t reg[4];
		uint32_t mask;
	};
	
} panadr_t;

typedef struct dw_sys_cfg
{
	union 
	{
		struct
		{
			uint32_t FFEN		:1;
			uint32_t FFBC		:1;
			uint32_t FFAB		:1;
			uint32_t FFAD		:1;
			uint32_t FFAA		:1;
			uint32_t FFAM		:1;
			uint32_t FFAR		:1;
			uint32_t FFA4		:1;
			uint32_t FFA5		:1;
			uint32_t HIRQ_POL	:1;
			uint32_t SPI_EDGE	:1;
			uint32_t DIS_FCE	:1;
			uint32_t DIS_DRXB	:1;
			uint32_t DIS_PHE	:1;
			uint32_t DIS_RSDE	:1;
			uint32_t FCS_INT2F	:1;
			uint32_t PHR_MODE	:2;
			uint32_t DIS_STXP	:1;
			uint32_t 			:3;
			uint32_t RXM110K	:1;
			uint32_t 			:5;
			uint32_t RXWTOE		:1;
			uint32_t RXAUTR		:1;
			uint32_t AUTOACK	:1;
			uint32_t AACKPEND	:1;
		};
		uint8_t reg[4]; 
		uint32_t mask; 
	};
} sys_cfg_t;

#pragma pack (1)
typedef struct dw_sys_time
{
	union 
	{
		struct
		{
			uint8_t lower8;
			uint32_t time32;
		};
		uint8_t reg[5];
	};
} sys_time_t;

typedef struct dw_tx_fctrl
{
	union 
	{
		struct
		{
			uint32_t TFLEN		:7;
			uint32_t TFLE		:3;
			uint32_t R			:3;
			uint32_t TXBR		:2;
			uint32_t TR			:1;
			uint32_t TXPRF		:2;
			uint32_t TXPSR		:2;
			uint32_t PE			:2;
			uint32_t TXBOFFS	:10;
			uint8_t IFSDELAY;
		};
		struct
		{
			uint32_t 			:18;
			uint32_t TXPL		:4;
			uint32_t 			:10;
			uint8_t RES;
		};
		uint8_t reg[5]; 
		uint32_t mask;
	};
} tx_fctrl_t;

typedef struct dw_tx_buffer
{
	uint8_t reg[1024];
} tx_buffer_t;

typedef sys_time_t dx_time_t;

typedef uint16_t rx_fwto_t;

typedef struct dw_sys_ctrl
{
	union 
	{
		struct
		{
			uint32_t SFCST		:1;
			uint32_t TXSTRT		:1;
			uint32_t TXDLYS		:1;
			uint32_t CANSFCS	:1;
			uint32_t 			:2;
			uint32_t TRXOFF		:1;
			uint32_t WAIT4RESP	:1;
			uint32_t RXENAB		:1;
			uint32_t RXDLYE		:1;
			uint32_t 			:14;
			uint32_t HRBPT		:1;
			uint32_t			:7;
		};
		uint8_t reg[4];
		uint32_t mask; 
	};
} sys_ctrl_t;

typedef struct dw_sys_mask
{
	union 
	{
		struct
		{
			uint32_t 			:1;
			uint32_t MCPLOCK	:1;
			uint32_t MESYNCR	:1;
			uint32_t MAAT		:1;
			uint32_t MTXFRB		:1;
			uint32_t MTXPRS		:1;
			uint32_t MTXPHS		:1;
			uint32_t MTXFRS		:1;
			uint32_t MRXPRD		:1;
			uint32_t MRXFSDD	:1;
			uint32_t MLDEDONE	:1;
			uint32_t MRXPHD		:1;
			uint32_t MRXPHE		:1;
			uint32_t MRXDFR		:1;
			uint32_t MRXFCG		:1;
			uint32_t MRXFCE		:1;
			uint32_t MRXRFSL	:1;
			uint32_t MRXRFTO	:1;
			uint32_t MLDEERR	:1;
			uint32_t 			:1;
			uint32_t MRXOVRR	:1;
			uint32_t MRXPTO		:1;
			uint32_t MGPIOIRQ	:1;
			uint32_t MSLP2INIT	:1;
			uint32_t MRFPLLLL	:1;
			uint32_t MCPLLLL	:1;
			uint32_t MRXSFDTO	:1;
			uint32_t MHPDWARN	:1;
			uint32_t MTXBERR	:1;
			uint32_t MAFFREJ	:1;
			uint32_t 			:2;
		};
		uint8_t reg[4];
		uint32_t mask; 
	};
} sys_mask_t;

typedef struct dw_sys_status
{
	union 
	{
		struct
		{
			uint32_t IRQS		:1;
			uint32_t CPLOCK		:1;
			uint32_t ESYNCR		:1;
			uint32_t AAT		:1;
			uint32_t TXFRB		:1;
			uint32_t TXPRS		:1;
			uint32_t TXPHS		:1;
			uint32_t TXFRS		:1;
			uint32_t RXPRD		:1;
			uint32_t RXFSDD		:1;
			uint32_t LDEDONE	:1;
			uint32_t RXPHD		:1;
			uint32_t RXPHE		:1;
			uint32_t RXDFR		:1;
			uint32_t RXFCG		:1;
			uint32_t RXFCE		:1;
			uint32_t RXRFSL		:1;
			uint32_t RXRFTO		:1;
			uint32_t LDEERR		:1;
			uint32_t 			:1;
			uint32_t RXOVRR		:1;
			uint32_t RXPTO		:1;
			uint32_t GPIOIRQ	:1;
			uint32_t SLP2INIT	:1;
			uint32_t RFPLL_LL	:1;
			uint32_t CLKPLL_LL	:1;
			uint32_t RXSFDTO	:1;
			uint32_t HPDWARN	:1;
			uint32_t TXBERR		:1;
			uint32_t AFFREJ		:1;
			uint32_t HSRBP		:1;
			uint32_t ICRBP		:1;
			uint8_t	RXRSCS		:1;
			uint8_t	RXPREJ		:1;
			uint8_t	TXPUTE		:1;
			uint8_t				:5;
		};
		uint8_t reg[5]; 
		uint32_t mask;
	};
} sys_status_t;


typedef struct dw_rx_finfo
{
	union 
	{
		struct
		{
			uint32_t RXFLEN		:7;
			uint32_t RXFLE		:3;
			uint32_t 			:1;
			uint32_t RXNSPL		:2;
			uint32_t RXBR		:2;
			uint32_t RNG		:1;
			uint32_t RXPRFR		:2;
			uint32_t RXPSR		:2;
			uint32_t RXPACC		:12;
		};
		uint8_t reg[4];
		uint32_t mask; 
	};
} rx_finfo_t;	

typedef struct dw_rx_buffer
{
	uint8_t reg[1024];
} rx_buffer_t;

typedef struct dw_rx_fqual
{
	union 
	{
		struct
		{
			uint16_t STD_NOISE;
			uint16_t FP_AMPL2;
			uint16_t FP_AMPL3;
			uint16_t CIR_PWR;
		};
		uint8_t reg[8]; 
		uint64_t mask;
	};
} rx_fqual_t;

typedef uint32_t rx_ttcki_t;

typedef struct dw_rx_ttcko
{
	union 
	{
		struct
		{
			uint32_t RXTOFS		:19;
			uint32_t 			:5;
			uint32_t RSMPDEL	:8;
			uint8_t RCPHASE		:7;
			uint8_t 			:1;
		};
		uint8_t reg[5]; 
	};
} rx_ttcko_t;

typedef struct dw_rx_time
{
	union 
	{
		struct
		{
			uint8_t RX_STAMP[5];
			uint8_t FP_INDEX[2];
			uint8_t FP_AMPL1[2];
			uint8_t RX_RAWST[5];
		};
		uint8_t reg[14]; 
	};
} rx_time_t;

typedef struct dw_tx_time
{
	union 
	{
		struct
		{
			uint8_t TX_STAMP[5];
			uint8_t TX_RAWST[5];
		};
		uint8_t reg[10]; 
	};
} tx_time_t;

typedef uint16_t tx_antd_t;

typedef struct dw_sys_state
{
	union 
	{
		struct
		{
			uint32_t TX_STATE	:4;
			uint32_t 			:4;
			uint32_t RX_STATE	:5;
			uint32_t 			:3;
			uint32_t PMSC_STATE	:4;
			uint32_t 			:12;
		};
		uint8_t reg[4];
		struct
		{
			tx_state tx_state;
			rx_state rx_state;
			pmsc_state pmsc_state;
		};
		uint32_t mask;
	};
} sys_state_t;

typedef struct dw_ack_resp_t
{
	union 
	{
		struct
		{
			uint32_t W4R_TIM	:20;
			uint32_t 			:4;
			uint32_t ACK_TIM	:8;
		};
		uint8_t reg[4]; 
		uint32_t mask;
	};
} ack_resp_t_t;

typedef struct dw_rx_sniff
{
	union 
	{
		struct
		{
			uint16_t SNIFF_ONT	:4;
			uint16_t 			:4;
			uint16_t SNIFF_OFFT	:8;
		};
		uint8_t reg[2];
		uint16_t mask;
	};
} rx_sniff_t;

typedef struct dw_tx_power
{
	union 
	{
		struct
		{
			uint32_t BOOST_NORM	:8;
			uint32_t BOOSTP500	:8;
			uint32_t BOOSTP250	:8;
			uint32_t BOOSTP125	:8;
		};
		struct
		{
			uint32_t 			:8;
			uint32_t TXPOWPHR	:8;
			uint32_t TXPOWSD	:8;
			uint32_t 			:8;
		};
		uint8_t reg[4];
		uint32_t mask;
	};
} tx_power_t;

typedef struct dw_chan_ctrl
{
	union 
	{
		struct
		{
			uint32_t TX_CHAN	:4;
			uint32_t RX_CHAN	:4;
			uint32_t 			:9;
			uint32_t DWSFD		:1;
			uint32_t RXPRF		:2;
			uint32_t TNSSFD		:1;
			uint32_t RNSSFD		:1;
			uint32_t TX_PCODE	:5;
			uint32_t RX_PCODE	:5;
		};
		uint8_t reg[4];
		uint32_t mask;
	};
} chan_ctrl_t;

// User defined Start of Frame delimiter (PHY)

// typedef struct dw_usr_sfd
// {
// 	/* data */
// } usr_sfd_t;

typedef uint8_t agc_ctrl1_t;
typedef uint16_t agc_tune1_t;
typedef uint32_t agc_tune2_t;
typedef uint16_t agc_tune3_t;

typedef struct dw_agc_stat1
{
	union 
	{
		struct
		{
			uint32_t 			:6;
			uint32_t EDG1		:5;
			uint32_t EDV2		:9;
			uint32_t			:12;
		};
		uint8_t reg[3];
		uint32_t mask;
	};
} agc_stat1_t;

typedef struct dw_agc_tune
{
	agc_tune1_t agc_tune1;
	agc_tune2_t agc_tune2;
	agc_tune3_t agc_tune3;
} agc_tune_t;

// TODO doesnt respect reserved addresses cannot be used directly
typedef struct dw_agc_ctrl_R
{
	agc_ctrl1_t agc_ctrl1;
	agc_tune_t agc_tune;
	agc_stat1_t agc_stat1;
} agc_ctrl_R_t;

typedef struct dw_ec_ctrl
{
	union 
	{
		struct
		{
			uint16_t OSTSM		:1;
			uint16_t OSRSM		:1;
			uint16_t PLLLDT		:1;
			uint16_t WAIT		:8;
			uint16_t OSTRM		:1;
			uint16_t 			:4;
		};
		uint8_t reg[2];
		uint16_t mask;
	};
} ec_ctrl_t;

typedef uint32_t ec_rxtc_t;
typedef uint32_t ec_golp_t;

typedef struct dw_ext_sync
{
	ec_ctrl_t ec_ctrl;
	uint16_t res1;
	ec_rxtc_t ec_rxtc;
	ec_golp_t ec_golp;
} ext_sync_t;

// Accumulator memory

// typedef struct dw_acc_mem
// {
// 	/* data */
// } acc_mem_t;

// GPIO control

// typedef struct dw_gpio_ctrl
// {
// 	/* data */
// } gpio_ctrl_t;

typedef uint16_t drx_tune0b_t;
typedef uint16_t drx_tune1a_t;
typedef uint16_t drx_tune1b_t;
typedef uint32_t drx_tune2_t;
typedef uint16_t drx_sfdtoc_t;
typedef uint16_t drx_pretoc_t;
typedef uint16_t drx_tune4h_t;
typedef uint32_t drx_car_int_t;
typedef uint16_t rxpacc_nosat_t;

// TODO subreg comp not memory accurate
typedef struct dw_drx_conf
{
	drx_tune0b_t drx_tune0b;
	drx_tune1a_t drx_tune1a;
	drx_tune1b_t drx_tune1b;
	drx_tune2_t drx_tune2;
	drx_sfdtoc_t drx_sfdtoc; // TODO Warning do not set to 0
	drx_pretoc_t drx_pretoc;
	drx_tune4h_t drx_tune4h;
} drx_conf_t;

typedef struct dw_drx_conf_info
{
	drx_car_int_t drx_car_int;
	rxpacc_nosat_t rxpacc_nosat;
} drx_conf_info_t;

typedef struct dw_drx_conf_R
{
	drx_conf_t drx_conf;
	drx_conf_info_t drx_conf_info;
} drx_conf_R_t;

typedef struct dw_rf_conf
{
	union 
	{
		struct
		{
			uint32_t 			:8;
			uint32_t TXFEN		:5;
			uint32_t PLLFEN		:3;
			uint32_t LDOFEN		:5;
			uint32_t TXRXSW		:2;
			uint32_t 			:9;
		};
		uint8_t reg[4];
		uint32_t mask;
	};
} rf_conf_t;

typedef uint8_t rf_rxctrlh_t;

typedef struct dw_rf_txctrl
{
	union 
	{
		struct
		{
			uint32_t 			:5;
			uint32_t TXMTUNE	:4;
			uint32_t TXMQ		:3;
			uint32_t RESERVED	:12;
			uint32_t			:8;
		};
		uint8_t reg[3];
		uint32_t mask;
	};
} rf_txctrl_t;

typedef struct dw_rf_status
{
	union 
	{
		struct
		{
			uint8_t CPLLLOCK	:1;
			uint8_t CPLLLOW		:1;
			uint8_t CPLLHIGH	:1;
			uint8_t RFPLLLOCK	:1;
			uint8_t				:4;
		};
		uint8_t reg[1];
		uint8_t mask;
	};
} rf_status_t;

typedef struct dw_ldotune
{
	uint8_t reg[5];
} ldotune_t;

typedef struct dw_rf_conf_S
{
	rf_conf_t rf_conf;
	rf_rxctrlh_t rf_rxctrlh;
	rf_txctrl_t rf_txctrl;
} rf_conf_S_t;

typedef struct dw_rf_conf_R
{
	rf_conf_S_t rf_conf_S;
	rf_status_t rf_status;
	ldotune_t ldotune;
} rf_conf_R_t;

typedef uint8_t tc_sarc_t;

typedef struct dw_tc_sarl
{
	union 
	{
		struct
		{
			uint16_t SAR_LVBAT	:8;
			uint16_t SAR_LTEMP	:8;
		};
		uint8_t reg[2];
		uint16_t mask;
	};
} tc_sarl_t;

typedef struct dw_tc_sarw
{
	union 
	{
		struct
		{
			uint16_t SAR_WVBAT	:8;
			uint16_t SAR_WTEMP	:8;
		};
		uint8_t reg[2];
		uint16_t mask;
	};
} tc_sarw_t;

typedef struct dw_tc_pg_ctrl
{
	union 
	{
		struct
		{
			uint8_t PG_START	:1;
			uint8_t 			:1;
			uint8_t PG_TMEAS	:4;
			uint8_t 			:2;
		};
		uint8_t reg[1];
		uint8_t mask;
	};
} tc_pg_ctrl_t;

typedef uint16_t tc_pg_status_t;
typedef uint8_t tc_pgdelay_t;
typedef uint8_t tc_pgtest_t;

typedef struct dw_tc_sar_read
{
	tc_sarl_t tc_sarl;
	tc_sarw_t tc_sarw;
} tc_sar_read_t;

typedef struct dw_tc_pg_conf
{
	tc_pg_ctrl_t tc_pg_ctrl;
	tc_pgdelay_t tc_pgdelay;
	tc_pgtest_t tc_pgtest;
} tc_pg_conf_t;

typedef struct dw_tx_cal_R
{
	tc_sarc_t tc_sarc;
	tc_sar_read_t tc_sar_read;
	tc_pg_conf_t tc_pg_conf;
	tc_pg_status_t tc_pg_status;
} tx_cal_R_t;

typedef uint32_t fs_pllcfg_t;
typedef uint8_t fs_plltune_t;
typedef uint8_t fs_xtalt_t; // TODO high 3 bits must be 0b011

typedef struct dw_fs_ctrl_R
{
	fs_pllcfg_t fs_pllcfg;
	fs_plltune_t fs_plltune;
	fs_xtalt_t fs_xtalt;
} fs_ctrl_R_t;

typedef struct dw_aon_wcfg
{
	union 
	{
		struct
		{
			uint16_t ONW_RADC	:1;
			uint16_t ONW_RX		:1;
			uint16_t 			:1;
			uint16_t ONW_LEUI	:1;
			uint16_t 			:2;
			uint16_t ONW_LDC	:1;
			uint16_t ONW_L64P	:1;
			uint16_t PRES_SLEEP	:1;
			uint16_t 			:2;
			uint16_t ONW_LLDE	:1;
			uint16_t ONW_LLDO	:1;
			uint16_t 			:3;
		};
		uint8_t reg[2]; 
		uint16_t mask;
	};
} aon_wcfg_t;

typedef struct dw_aon_ctrl
{
	union 
	{
		struct
		{
			uint8_t RESTORE		:1;
			uint8_t SAVE		:1;
			uint8_t UPL_CFG		:1;
			uint8_t DC_READ		:1;
			uint8_t 			:3;
			uint8_t DCA_ENAB	:1;
		};
		uint8_t reg[1];
		uint8_t mask; 
	};
} aon_ctrl_t;

typedef uint8_t aon_rdat_t;
typedef uint8_t aon_addr_t;

typedef struct dw_aon_cfg0
{
	union 
	{
		struct
		{
			uint32_t SLEEP_EN	:1;
			uint32_t WAKE_PIN	:1;
			uint32_t WAKE_SPI	:1;
			uint32_t WAKE_CNT	:1;
			uint32_t LPDIV_EN	:1;
			uint32_t LPCLKDIVA	:11;
			uint32_t SLEEP_TIM	:16;
		};
		uint8_t reg[4];
		uint32_t mask;
	};
} aon_cfg0_t;

typedef struct dw_aon_cfg1
{
	union 
	{
		struct
		{
			uint8_t SLEEP_CE	:1;
			uint8_t SMXX		:1;
			uint8_t LPOSC_C		:1;
			uint8_t 			:5;
		};
		uint8_t reg[1];
		uint8_t mask;
	};
} aon_cfg1_t;

typedef struct dw_aon_conf
{
	aon_wcfg_t aon_wcfg;
	aon_cfg0_t aon_cfg0;
	aon_cfg1_t aon_cfg1;
} aon_conf_t;

#pragma pack (1)
typedef struct dw_aon
{
	union
	{
		struct
		{
			aon_wcfg_t aon_wcfg;
			aon_ctrl_t aon_ctrl;
			aon_rdat_t aon_rdat;
			aon_addr_t aon_addr;
			uint8_t res1;
			aon_cfg0_t aon_cfg0;
			aon_cfg1_t aon_cfg1;
			uint8_t res2;
		};
		uint8_t reg[12];
	};
} aon_t;

typedef uint32_t otp_wdat_t;
typedef uint16_t otp_addr_t; // Address is lower 11 bits 

typedef struct dw_otp_ctrl
{
	union
	{
		struct
		{
			uint16_t OTPRDEN	:1;
			uint16_t OTPREAD	:1;
			uint16_t 			:1;
			uint16_t OTPMRWR	:1;
			uint16_t 			:2;
			uint16_t OTPPROG	:1;
			uint16_t OTPMR		:4;
			uint16_t 			:4;
			uint16_t LDELOAD	:1;
		};
		uint8_t reg[2];
		uint16_t mask;
	};
} otp_ctrl_t;

typedef struct dw_otp_stat
{
	union
	{
		struct
		{
			uint8_t OTPPRGD	:1;
			uint8_t OTPVPOK	:1;
			uint8_t 		:6;
		};
		uint8_t reg[1];
		uint8_t mask;
	};
} otp_stat_t;

typedef uint32_t otp_rdat_t;
typedef uint32_t otp_srdat_t;

typedef struct dw_otp_sf
{
	union
	{
		struct
		{
			uint8_t OPS_KICK	:1;
			uint8_t LDO_KICK	:1;
			uint8_t 			:3;
			uint8_t OPS_SEL		:2;
			uint8_t 			:1;
		};
		uint8_t reg[1];
		uint8_t mask;
	};
} otp_sf_t;

typedef struct dw_otp_if_R
{
	otp_wdat_t otp_wdat;
	otp_addr_t otp_addr;
	otp_ctrl_t otp_ctrl;
	otp_stat_t otp_stat;
	otp_rdat_t otp_rdat;
	otp_srdat_t otp_srdat;
	otp_sf_t otp_sf;
} otp_if_t_R;

typedef uint16_t lde_thresh_t;

typedef struct dw_lde_cfg1
{
	union
	{
		struct 
		{
			uint8_t NTM		:5;
			uint8_t PMULT	:3;
		};
		uint8_t reg[1];
		uint8_t mask;
	};
} lde_cfg1_t;

typedef uint16_t lde_ppindx_t;
typedef uint16_t lde_ppampl_t;
typedef uint16_t lde_rxantd_t;
typedef uint16_t lde_cfg2_t;
typedef uint16_t lde_repc_t;

typedef struct dw_lde_conf
{
	lde_cfg1_t lde_cfg1;
	lde_cfg2_t lde_cfg2;
	lde_rxantd_t lde_rxantd;
	lde_repc_t lde_repc;
} lde_conf_t;

typedef struct dw_lde_info
{
	lde_thresh_t lde_thresh;
	lde_ppindx_t lde_ppindx;
	lde_ppampl_t lde_ppampl;
} lde_info_t;

typedef struct dw_lde_ctrl_R
{
	lde_conf_t lde_conf;
	lde_info_t lde_info;
} lde_ctrl_R_t;

// TODO Document minimum 2 byte write, reserved bytes as 0
typedef struct dw_evc_ctrl
{
	union
	{
		struct 
		{
			uint16_t EVC_EN		:1;
			uint16_t EVC_CLR	:1;
			uint16_t 			:14;
		};
		uint16_t reg[2];
		uint16_t mask;
	};
} evc_ctrl_t;

typedef uint16_t evc_phe_t;
typedef uint16_t evc_rse_t;
typedef uint16_t evc_fcg_t;
typedef uint16_t evc_fce_t;
typedef uint16_t evc_ffr_t;
typedef uint16_t evc_ovr_t;
typedef uint16_t evc_pto_t;
typedef uint16_t evc_fwto_t;
typedef uint16_t evc_txfs_t;
typedef uint16_t evc_hpw_t;
typedef uint16_t evc_tpw_t;

typedef struct dw_diag_tmc
{
	union
	{
		struct 
		{
			uint16_t 			:4;
			uint16_t TX_PSTM	:1;
			uint16_t 			:11;
		};
		uint16_t reg[2];
		uint16_t mask;
	};
} diag_tmc_t;

typedef struct evc_info
{
	evc_phe_t evc_phe;
	evc_rse_t evc_rse;
	evc_fcg_t evc_fcg;
	evc_fce_t evc_fce;
	evc_ffr_t evc_ffr;
	evc_ovr_t evc_ovr;
	evc_pto_t evc_pto;
	evc_fwto_t evc_fwto;
	evc_txfs_t evc_txfs;
	evc_hpw_t evc_hpw;
	evc_tpw_t evc_tpw;
} evc_info_t;

typedef struct dw_dig_diag_R
{
	evc_ctrl_t evc_ctrl;
	evc_info_t evc_info;
	diag_tmc_t diag_tmc;
} dig_diag_R_t;

typedef struct dw_psmc_ctrl0
{
	union
	{
		struct
		{
			uint32_t SYSCLKS	:2;
			uint32_t RXCLKS		:2;
			uint32_t TXCLKS		:2;
			uint32_t FACE		:1;
			uint32_t 			:3;
			uint32_t ADCCE		:1;
			uint32_t 			:4;
			uint32_t AMCE		:1;
			uint32_t GPCE		:1;
			uint32_t GPRN		:1;
			uint32_t GPDCE		:1;
			uint32_t GPDRN		:1;
			uint32_t 			:3;
			uint32_t KHZCLKEN	:1;
			uint32_t PLL2_SEQ_EN:1;
			uint32_t 			:3;
			uint32_t SOFTRESET	:4;
		};
		uint32_t mask;
		uint8_t reg[4];
	};
} pmsc_ctrl0_t;

typedef struct dw_psmc_ctrl1
{
	union
	{
		struct
		{
			uint32_t 			:1;
			uint32_t ARX2INIT	:1;
			uint32_t 			:1;
			uint32_t PKTSEQ		:8;
			uint32_t ATXSLP		:1;
			uint32_t ARXSLP		:1;
			uint32_t SNOZE		:1;
			uint32_t SNOZR		:1;
			uint32_t PLLSYN		:1;
			uint32_t 			:1;
			uint32_t LDERUNE	:1;
			uint32_t 			:8;
			uint32_t KHZCLKDIV	:6;
		};
		uint32_t mask;
		uint8_t reg[4];
	};
} pmsc_ctrl1_t;

typedef uint8_t pmsc_snozt_t;
typedef uint16_t pmsc_txfseq_t;

typedef struct dw_psmc_ledc
{
	union
	{
		struct
		{
			uint32_t BLINK_TIM	:8;
			uint32_t BLNKEN		:1;
			uint32_t 			:7;
			uint32_t BLNKNOW	:4;
			uint32_t 			:12;
		};
		uint8_t reg[4];
		uint32_t mask;
	};
} pmsc_ledc_t;

typedef struct dw_pmsc_R
{
	pmsc_ctrl0_t pmsc_ctrl0;
	pmsc_ctrl1_t pmsc_ctrl1;
	pmsc_snozt_t pmsc_snozt;
	pmsc_txfseq_t pmsc_txfseq;
	pmsc_ledc_t pmsc_ledc;
} pmsc_R_t;

/*
	0	panadr 0-1 shortaddr
	1	panadr 2-3 panid
	2	syscfg 0
	3	syscfg 1
	4	syscfg 2
	5	syscfg 3
	6	tx_fctrl
	7	rx_fwto
	8	sys_mask 0
	9	sys_mask 1
	10	sys_mask 2
	11	sys_mask 3
	12	tx_antd
	13	rx_sniff
	14	tx_power
	15	chan_ctrl
	16	agc_ctrl1
	17	agc_tune1
	18	agc_tune2
	19	agc_tune3
	20	drx_tune0b
	21	drx_tune1a
	22	drx_tune1b
	23	drx_tune2
	24	drx_sfdtoc
	25	drx_pretoc
	26	drx_tune4h
	27	rf_conf
	28	rf_rxctrlh
	29	rf_txctrl
	30	ldotune
	31	tc_sarc
	32	tc_pg_ctrl
	33 	tc_pgdelay
	34	tc_pgtest
	35	fs_pllcfg
	36	fs_plltune
	37	fs_xtalt
	38	aon_wcfg
	39	aon_cfg0
	40	aon_cfg1
	41	lde_cfg1
	42	lde_cfg2
	43	lde_rxantd
	44	lde_repc
*/

// static const sys_cfg_t SYS_CFG_DEF =
// {
// 	.FFEN		= 0
// 	.FFBC		= 0
// 	.FFAB		= 0
// 	.FFAD		= 0
// 	.FFAA		= 0
// 	.FFAM		= 0
// 	.FFAR		= 0
// 	.FFA4		= 0
// 	.FFA5		= 0
// 	.HIRQ_POL	= 1
// 	.SPI_EDGE	= 0
// 	.DIS_FCE	= 0
// 	.DIS_DRXB	= 1
// 	.DIS_PHE	= 0
// 	.DIS_RSDE	= 0
// 	.FCS_INT2F	= 0
// 	.PHR_MODE	= 0
// 	.DIS_STXP	= 0
// 	.RXM110K	= 0
// 	.RXWTOE		= 0
// 	.RXAUTR		= 0
// 	.AUTOACK	= 0
// 	.AACKPEND	= 0
// };

static const sys_cfg_t SYS_CFG_DEF = {.mask=0x00001200};

static const tx_fctrl_t TX_FCTRL_DEF =
{
	.TFLEN		= 0xC,
	.TFLE		= 0,
	.R			= 0,
	.TXBR		= BR_6_8MBPS,
	.TR			= 0,
	.TXPRF		= PRF_16MHZ,
	.TXPL		= PL_128,
	.TXBOFFS	= 0,
	.IFSDELAY	= 0
};

static const chan_ctrl_t CHAN_CTRL_DEF = {.mask=0x00000055};

static const tx_power_t TX_POWER_DEF = {.mask=0x0E080222};

// First index is DIS_STPX, second index is PRF 16 and 64, last index is channel [(1,2),3,4,5,7]
static const tx_power_t TX_POWER_REF[2][2][5] = 
{
	{
		{
			{.mask=0x15355575}, {.mask=0x0F2F4F6F}, {.mask=0x1F1F3F5F}, {.mask=0x0E082848}, {.mask=0x32527292}
		},
		{
			{.mask=0x07274767}, {.mask=0x2B4B6B8B}, {.mask=0x3A5A7A9A}, {.mask=0x25466788}, {.mask=0x5171B1D1}
		}
	},
	{
		{
			{.mask=0x75757575}, {.mask=0x6F6F6F6F}, {.mask=0x5F5F5F5F}, {.mask=0x48484848}, {.mask=0x92929292}
		},
		{
			{.mask=0x67676767}, {.mask=0x8B8B8B8B}, {.mask=0x9A9A9A9A}, {.mask=0x85858585}, {.mask=0xD1D1D1D1}
		}
	}
};

static const agc_tune1_t AGC_TUNE1_16_REF = 0x8870;
static const agc_tune1_t AGC_TUNE1_64_REF = 0x889B;
static const agc_tune2_t AGC_TUNE2_REF = 0X2502A907;
static const agc_tune3_t AGC_TUNE3_REF = 0x0035;

static const agc_tune_t AGC_TUNE_DEF = 
{
	AGC_TUNE1_64_REF,					// agc_tune1
	AGC_TUNE2_REF,						// agc_tune2
	AGC_TUNE3_REF						// agc_tune3
};

// First index is Standard Non-standard SFD, second index is data rate 110kbps, 850kbps, 6.8Mbps
static const drx_tune0b_t DRX_TUNE0B_REF[2][3] =
{
	{
		0xA, 0x1, 0x1
	},
	{
		0x16, 0x6, 0x2
	}
};

static const drx_tune1a_t DRX_TUNE1A_16_REF = 0x87;
static const drx_tune1a_t DRX_TUNE1A_64_REF = 0x8D;

static const drx_tune1b_t DRX_TUNE1B_1024_REF = 0x64;
static const drx_tune1b_t DRX_TUNE1B_128_REF = 0x20;
static const drx_tune1b_t DRX_TUNE1B_64_REF = 0x10;

static const drx_tune4h_t DRX_TUNE4H_64_REF = 0x10;
static const drx_tune4h_t DRX_TUNE4H_128_REF = 0x28;

static const drx_tune2_t DRX_TUNE2_DEF = 0x311E0035;

// First index is PRF 16 and 64, second index is PAC size 8, 16, 32, 64
static const drx_tune2_t DRX_TUNE2_REF[2][4] =
{
	{
		0x311A002D, 0x331A0052, 0x351A009A, 0x371A011D
	},
	{
		0x313B006B, 0x333B00BE, 0x353B015E, 0x373B0296
	}
};

static const drx_sfdtoc_t DRX_SFDTOC_DEF = 4096+64+1;

static const drx_conf_t DRX_CONF_DEF =
{
	DRX_TUNE0B_REF[0][BR_6_8MBPS],		// drx_tune0b
	DRX_TUNE1A_16_REF,					// drx_tune1a
	DRX_TUNE1B_128_REF,					// drx_tuneb
	DRX_TUNE2_DEF,						// drx_tune2
	DRX_SFDTOC_DEF,						// drx_sfdtoc
	0x0,								// drx_pretoc
	DRX_TUNE4H_128_REF					// drx_tune4h
};

static const rf_conf_t RF_CONF_DEF = {.mask=0x0};

static const rf_rxctrlh_t RF_RXCTRL_1_5_REF = 0XD8;
static const rf_rxctrlh_t RF_RXCTRL_4_7_REF = 0XBC;

static const rf_txctrl_t RF_TXCTRL_1_REF = {.mask=0x00005C40};
static const rf_txctrl_t RF_TXCTRL_2_REF = {.mask=0x00045CA0};
static const rf_txctrl_t RF_TXCTRL_3_REF = {.mask=0x00086CC0};
static const rf_txctrl_t RF_TXCTRL_4_REF = {.mask=0x00045C80};
static const rf_txctrl_t RF_TXCTRL_5_REF = {.mask=0x001E3FE3};
static const rf_txctrl_t RF_TXCTRL_7_REF = {.mask=0x001E7DE0};

static const rf_txctrl_t RF_TXCTRL_DEF = {.mask=0x001E3DE0};

static const rf_conf_S_t RF_CONF_S_DEF =
{
	RF_CONF_DEF,						// rf_conf
	RF_RXCTRL_1_5_REF,					// rf_rxctrlh
	RF_TXCTRL_DEF						// rf_txctrl
};

static const ldotune_t LDOTUNE_DEF = {.reg={0x88, 0x88, 0x88, 0x88, 0x88}};

static const tc_pg_ctrl_t TC_PG_CTRL_DEF = {.mask=0x0};

static const tc_pgdelay_t TC_PGDELAY_1_REF = 0xC9;
static const tc_pgdelay_t TC_PGDELAY_2_REF = 0xC2;
static const tc_pgdelay_t TC_PGDELAY_3_REF = 0xC5;
static const tc_pgdelay_t TC_PGDELAY_4_REF = 0x95;
static const tc_pgdelay_t TC_PGDELAY_5_REF = 0xB5;
static const tc_pgdelay_t TC_PGDELAY_7_REF = 0x93;

static const tc_pgtest_t TC_PGTEST_CW_TEST_MODE = 0x13;

static const tc_pg_conf_t TC_PG_CONF_DEF =
{
	TC_PG_CTRL_DEF,						// tc_pg_ctrl
	TC_PGDELAY_3_REF,					// tc_pgdelay
	0x00								// tc_pgtest
};

static const fs_pllcfg_t FS_PLLCFG_1_REF = 0x09000407;
static const fs_pllcfg_t FS_PLLCFG_2_4_REF = 0x08400508;
static const fs_pllcfg_t FS_PLLCFG_3_REF = 0x08401009;
static const fs_pllcfg_t FS_PLLCFG_5_7_REF = 0x0800041D;

static const fs_plltune_t FS_PLLTUNE_1_REF = 0x1E;
static const fs_plltune_t FS_PLLTUNE_2_4_REF = 0x26;
static const fs_plltune_t FS_PLLTUNE_3_REF = 0x56;
static const fs_plltune_t FS_PLLTUNE_5_7_REF = 0xBE;

static const fs_plltune_t FS_PLLTUNE_DEF = 0x46;

static const fs_xtalt_t FS_XTALT_DEF = 0x60;

static const fs_ctrl_R_t FS_CTRL_R_DEF =
{
	FS_PLLCFG_5_7_REF,					// fs_pllcfg
	FS_PLLTUNE_DEF,						// fs_plltune
	FS_XTALT_DEF						// fs_xtalt
};

static const aon_cfg0_t AON_CFG0_DEF = 
{
	.SLEEP_EN	= 0b0,
	.WAKE_PIN	= 0b1,
	.WAKE_SPI	= 0b1,
	.WAKE_CNT	= 0b1,
	.LPDIV_EN	= 0b0,
	.LPCLKDIVA	= 0b00011111111,
	.SLEEP_TIM	= 0b0101000011111111
};

static const aon_wcfg_t AON_WCFG_DEF = {.mask=0x0};

// static const aon_cfg1_t AON_CFG1_DEF = 
// {
// 	.SLEEP_CE		= 1,
// 	.SMXX			= 1,
// 	.LPOSC_C		= 1
// };
static const aon_cfg1_t AON_CFG1_DEF = {.mask=0x7};

static const aon_conf_t AON_CONF_DEF = 
{
	AON_WCFG_DEF,						// aon_wcfg
	AON_CFG0_DEF,						// aon_cfg0
	AON_CFG1_DEF						// aon_cfg1
};

static const lde_cfg2_t LDE_CFG2_DEF = 0x0;
static const lde_cfg2_t LDE_CFG2_16_REF = 0x1607;
static const lde_cfg2_t LDE_CFG2_64_REF = 0x0607;
static const lde_cfg2_t LDE_CFG2_16_NLOS_REF = 0x0003;

static const lde_cfg1_t LDE_CFG1_DEF = {.mask=0x6C};
static const lde_cfg1_t LDE_CFG1_REF = {.mask=0x6D};

// Values only applicable for 850 kbps and 6.8Mbps data rates, index is RX_PCODE config
static const lde_repc_t LDE_REPC_REF[24] =
{
	0x5998, 0x5998, 0x51EA, 0x428E, 0x451E, 0x2E14, 0x8000, 0x51EA, 0x28F4, 0x3332, 0x3AE0, 0x3D70,
	0x3AE0, 0x35C2, 0x2B84, 0x35C2, 0x3332, 0x35C2, 0x35C2, 0x47AE, 0x3AE0, 0x3850, 0x30A2, 0x3850
};

// TODO check definition
static const uint8_t RX_PCODE_DEF = 4;

static const lde_conf_t LDE_CONF_DEF =
{
	LDE_CFG1_DEF,						// lde_cfg1
	LDE_CFG2_DEF,						// lde_cfg2
	0x0,								// lde_rxantd
	LDE_REPC_REF[RX_PCODE_DEF]			// lde_repc
};

typedef struct dw_rx_config
{
	rx_fwto_t dw_rx_fwto;			// 2
	agc_ctrl1_t dw_agc_ctrl1;		// 1
	agc_tune_t dw_agc_tune;			// 8 
	drx_conf_t dw_drx_conf;			// 16
	lde_conf_t dw_lde_conf;			// 7
} rx_config_t;

static const rx_config_t DW_RX_CONFIG_DEF =
{
	0,										// rx_fwto
	1,										// agc_ctrl1
	AGC_TUNE_DEF,
	DRX_CONF_DEF,
	LDE_CONF_DEF
};

typedef struct dw_tx_config
{
	tx_fctrl_t dw_def_tx_fctrl;		// 5 
	tx_antd_t dw_tx_antd;			// 2
	tx_power_t dw_tx_power;			// 4
	tc_sarc_t dw_tc_sarc;			// 1
	tc_pg_conf_t dw_tc_pg_conf;		// 3
} tx_config_t;

static const tx_config_t DW_TX_CONFIG_DEF =
{
	TX_FCTRL_DEF,							// tx_fctrl
	0,										// tx_antd
	TX_POWER_DEF,							// tx_power
	0,										// tc_sarc
	TC_PG_CONF_DEF
};

// size 109
typedef struct dw_config
{
	panadr_t dw_panadr;				// 4
	sys_cfg_t dw_sys_cfg;			// 4
	sys_mask_t dw_sys_mask;			// 4
	// ack_resp_t					// 0
	rx_sniff_t dw_rx_sniff;			// 2
	chan_ctrl_t dw_chan_ctrl;		// 4
	tx_config_t dw_tx_config;
	rx_config_t dw_rx_config;
	rf_conf_S_t dw_rf_conf_S;		// 8
	ldotune_t dw_ldotune;			// 5
	fs_ctrl_R_t dw_fs_ctrl;			// 6
	aon_conf_t dw_aon_conf;			// 6
} dw_config_t;

static const struct dw_config DW_DEFAULT_CONF = 
{
	{.mask=0xFFFFFFFF},						// panadr
	SYS_CFG_DEF,							// sys_cfg
	{.mask=0x0},							// sys_mask
	{.mask=0x0},							// rx_sniff
	CHAN_CTRL_DEF,							// chan_ctrl
	DW_TX_CONFIG_DEF,
	DW_RX_CONFIG_DEF,
	RF_CONF_S_DEF,
	LDOTUNE_DEF,							// ldo_tune
	FS_CTRL_R_DEF,
	AON_CONF_DEF
};

typedef struct dw_rod_info
{
	rx_finfo_t dw_rx_finfo;
	rx_fqual_t dw_rx_fqual;
	rx_ttcki_t dw_rx_ttcki;
	rx_ttcko_t dw_rx_ttcko;
	rx_time_t dw_rx_time;
} dw_rod_info_t;

typedef struct dw_info
{
	sys_status_t dw_sys_status;
	dw_rod_info_t dw_rod_info[2];
	tx_time_t dw_tx_time;
	sys_state_t dw_sys_state;
	agc_stat1_t dw_agc_stat1;
	drx_conf_info_t dw_drx_conf_info;
	rf_status_t dw_rf_status;
	tc_sar_read_t dw_tc_sar_read;
	tc_pg_status_t dw_tc_pg_status;
	lde_info_t dw_lde_info;
} dw_info_t;

void dw_set_spi_lock(void (*spi_lock_func)(void));
void dw_set_spi_unlock(void (*spi_unlock_func)(void));
void dw_set_spi_set_cs(void (*spi_set_cs_func)(void));
void dw_set_spi_clear_cs(void (*spi_clear_cs_func)(void));

void dw_set_spi_send(void (*spi_send_func)(size_t, const uint8_t*));
void dw_set_spi_recv(void (*spi_recv_func)(size_t, const uint8_t*));

/**
 * @brief 
 * 
 * @return int8_t 
 */
int8_t validate_spi_hal(void);

/**
 * @brief 
 * 
 * @param info 
 * @return int8_t 
 */
int8_t validate_metadata(reg_metadata_t info);

/**
 * @brief 
 * 
 * @param info 
 * @param count 
 * @param offset 
 * @return int8_t 
 */
int8_t validate_spi_transaction(reg_metadata_t info, size_t count, uint16_t offset);

/**
 * @brief 
 * 
 * @param info 
 * @param count 
 * @param offset 
 * @return int8_t 
 */
int8_t validate_dw(reg_metadata_t info, size_t count, uint16_t offset);

/**
 * @brief 
 * 
 * @param is_read_op 
 * @param id 
 * @param offset 
 * @return spi_header_t 
 */
spi_header_t _build_header(uint8_t is_read_op, uint8_t id, uint16_t offset);

/**
 * @brief 
 * 
 * @param is_read_op 
 * @param reg_id 
 * @param buf 
 * @param count 
 * @param offset 
 */
void _dw_spi_transaction(uint8_t is_read_op,  uint8_t reg_id, uint8_t* buf, size_t count, uint16_t offset);

/**
 * @brief 
 * 
 * @param info 
 * @param buf 
 * @param count 
 * @param offset 
 * @return int8_t 
 */
int8_t dw_read(reg_metadata_t info, uint8_t* buf, size_t count, uint16_t offset);

/**
 * @brief 
 * 
 * @param info 
 * @param buf 
 * @param count 
 * @param offset 
 * @return int8_t 
 */
int8_t dw_write(reg_metadata_t info, uint8_t* buf, size_t count, uint16_t offset);

/**
 * @brief 
 * 
 * @param set_mask 
 */
void dw_set_irq(sys_mask_t set_mask);

/**
 * @brief 
 * 
 * @param clear_mask 
 */
void dw_clear_irq(sys_mask_t clear_mask);

/**
 * @brief 
 * 
 */
void dw_soft_reset(void);

/**
 * @brief 
 * 
 */
void dw_soft_reset_rx(void);

/**
 * @brief 
 * 
 */
// TODO add to docs chibios event callbacks are in syslock check mode
void _dw_irq_handler(void);

/**
 * @brief 
 * 
 * @param tx_fctrl 
 * @param tx_buf 
 * @param dly_time 
 * @param w4r_time 
 */
void dw_start_tx(tx_fctrl_t tx_fctrl, uint8_t * tx_buf, dx_time_t dly_time, ack_resp_t_t w4r_time);

/**
 * @brief 
 * 
 * @param dly_time 
 */
void dw_start_rx(dx_time_t dly_time);

/**
 * @brief 
 * 
 * @return sys_state_t 
 */
sys_state_t dw_transceiver_off(void);

/**
 * @brief 
 * 
 * @param address 
 */
void dw_command_read_OTP(uint16_t address);

/**
 * @brief 
 * 
 * @return uint64_t 
 */
uint64_t dw_get_tx_time(void);

/**
 * @brief 
 * 
 * @return uint64_t 
 */
uint64_t dw_get_rx_time(void);

/**
 * @brief 
 * 
 * @return int32_t 
 */
int32_t dw_get_car_int(void);

void dw_get_full_config(dw_config_t* full_cfg);

void dw_get_tx_config(tx_config_t* tx_config);

void dw_get_rx_config(rx_config_t* rx_config);

void dw_get_tc_pg_config(tc_pg_conf_t* tc_pg_conf);

void default_config(void);