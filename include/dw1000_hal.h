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

extern uint64_t hardware_id;

typedef enum dw_reg_file_perms
{
	RO,
	WO,
	RW,
	SRW,		// Special Read Write, refer to docs
	ROD		// Read only double buffer
} reg_perm;

typedef enum dw_tx_states
{
	TX_IDLE,
	PREAMBLE,
	SFD,
	PHR,
	SDE,
	DATA
} tx_state;

typedef enum dw_rx_states
{
	RX_IDLE,
	START_ANALOG,
	RESERVED0,
	RESERVED1,
	RX_RDY,
	PREAMBLE_FND,
	PRMBL_TIMEOUT,
	SFD_FND,
	CNFG_PHR_RX,
	PHR_RX_STRT,
	DATA_RATE_RDY,
	RESERVED2,
	DATA_RX_SEQ,
	CNFG_DATA_RX,
	PHR_NOT_OK,
	LAST_SYMBOL,
	WAIT_RSD_DONE,
	RSD_OK,
	RSD_NOT_OK,
	RECONFIG_110,
	WAIT_110_PHR
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
	{0x00, 4, RO},		
	{0x01, 8, RW},		
	{0x03, 4, RW},		
	{0x04, 4, RW},		
	{0x06, 5, RO},		
	{0x08, 5, RW},		
	{0x09, 1024, WO},	
	{0x0A, 5, RW},
	{0X0C, 2, RW},
	{0X0D, 4, SRW},
	{0X0E, 4, RW},
	{0X0F, 5, SRW},
	{0x10, 4, ROD},
	{0X11, 1024, ROD},
	{0X12, 8, ROD},
	{0X13, 4, ROD},
	{0X14, 5, ROD},
	{0X15, 14, ROD},
	{0X17, 10, RO},
	{0X18, 2, RW},
	{0X19, 5, RO},
	{0X1A, 4, RW},
	{0X1D, 4, RW},
	{0X1E, 4, RW},
	{0X1F, 4, RW},
	{0X21, 41, RW},
	{0X23, 33, RW},
	{0X24, 12, RW},
	{0X25, 4064, RO},
	{0X26, 44, RW},
	{0X27, 44, RW},
	{0x28, 58, RW},
	{0X2A, 52, RW},
	{0X2B, 21, RW},
	{0X2C, 12, RW},
	{0X2D, 18, RW}, //OTP TODO CHECK DOC AS IT IS 18
	{0X2E, 13, RW},
	{0X2F, 41, RW},
	{0X36, 48, RW}
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

typedef struct dw_eui
{
	/* data */
} eui_t;

typedef struct dw_panadr
{
	/* data */
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

typedef struct dw_sys_time
{
	/* data */
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
		uint8_t reg[5]; 
	};
} tx_fctrl_t;

typedef struct dw_tx_buffer
{
	uint8_t reg[1024];
} tx_buffer_t;

#pragma pack (1)
typedef struct dw_dx_time
{
	union 
	{
		struct
		{
			uint8_t reserved;
			uint32_t time;
		};
		uint8_t reg[5]; 
	};
} dx_time_t;

// typedef struct dw_rx_fwto
// {
// 	/* data */
// } rx_fwto_t;

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

// typedef struct dw_rx_buffer
// {
// 	/* data */
// } rx_buffer_t;

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
	};
} rx_fqual_t;

typedef struct dw_rx_ttcki
{
	uint8_t reg[4];
} rx_ttcki_t;

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

// typedef struct dw_tx_antd
// {
// 	/* data */
// } tx_antd_t;

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
	};
} ack_resp_t_t;

typedef struct dw_rx_sniff
{
	union 
	{
		struct
		{
			uint32_t SNIFF_ONT	:4;
			uint32_t 			:4;
			uint32_t SNIFF_OFFT	:8;
			uint32_t 			:16;
		};
		uint8_t reg[4]; 
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
	};
} chan_ctrl_t;

// typedef struct dw_usr_sfd
// {
// 	/* data */
// } usr_sfd_t;

// typedef struct dw_agc_ctrl
// {
// 	/* data */
// } agc_ctrl_t;

// typedef struct dw_ext_sync
// {
// 	/* data */
// } ext_sync_t;

// typedef struct dw_acc_mem
// {
// 	/* data */
// } acc_mem_t;

// typedef struct dw_gpio_ctrl
// {
// 	/* data */
// } gpio_ctrl_t;

#pragma pack (1)
typedef struct dw_drx_conf
{
	union 
	{
		struct
		{
			uint16_t drx_res1;
			uint16_t drx_tune0b;
			uint16_t drx_tune1a;
			uint16_t drx_tune1b;
			uint32_t drx_tune2;
			uint8_t drx_res2[20];
			uint16_t drx_sfdtoc; // TODO Warning do not set to 0
			uint16_t drx_res3;
			uint16_t drx_pretoc;
			uint16_t drx_tune4h;
			uint8_t drx_car_int[3];
			uint8_t reserved;
			uint16_t rxpacc_nosat;
		};
		uint8_t reg[46];
	};
} drx_conf_t; 

// typedef struct dw_rf_conf
// {
// 	/* data */
// } rf_conf_t;

// typedef struct dw_tx_cal
// {
// 	/* data */
// } tx_cal_t;

// typedef struct dw_fs_ctrl
// {
// 	/* data */
// } fs_ctrl_t;

typedef struct dw_aon
{
	union 
	{
		struct 
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
				uint8_t aon_wcfg[2]; 
			};
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
				uint8_t aon_ctrl[1]; 
			};
			uint8_t aon_rdat[1]; 
			uint8_t aon_addr[1]; 
			uint8_t aon_res1[1]; 
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
				uint8_t aon_cfg0[4]; 
			};
			union 
			{
				struct
				{
					uint16_t SLEEP_CE	:1;
					uint16_t SMXX		:1;
					uint16_t LPOSC_C	:1;
					uint16_t BOOSTP125	:13;
				};
				uint8_t aon_cfg1[2]; 
			};
		};
		uint8_t reg[12]; 
	};
} aon_t;

typedef struct dw_otp_if
{
	union 
	{
		struct 
		{
			uint8_t otp_wdat[4]; 
			uint8_t otp_addr[2]; // Address is lower 11 bits 
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
				uint8_t otp_ctrl[2]; 
			};
			union 
			{
				struct
				{
					uint16_t OTPPRGD	:1;
					uint16_t OTPVPOK	:1;
					uint16_t 			:14;
				};
				uint8_t otp_stat[2]; 
			};
			uint8_t otp_rdat[4]; 
			uint8_t otp_srdat[4]; 
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
				uint8_t otp_sf[1]; 
			};
		};
		uint8_t reg[19]; // TODO CHECK DOC AS IT IS 18
	};
} otp_if_t;

// typedef struct dw_lde_ctrl
// {
// 	/* data */
// } lde_ctrl_t;

// typedef struct dw_dig_diag
// {
// 	/* data */
// } dig_diag_t;

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

#pragma pack (1)
typedef struct dw_pmsc
{
	union
	{
		struct
		{	
			pmsc_ctrl0_t PMSC_CTRL0;
			pmsc_ctrl1_t PMSC_CTRL1;
			uint32_t PMSC_RES1;
			uint8_t SNOZT;
			uint8_t RESERVED0[3]; // TODO check
			uint8_t PMSC_RES2[22];
			uint16_t PMSC_TXFSEQ;
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
				uint32_t PMSC_LEDC;
			};
			uint32_t RESERVED1;
		};
		uint8_t reg[48];
	};
} pmsc_t;

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

void dw_clear_register(uint8_t* reg, size_t size);

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

void dw_soft_reset(void);

void dw_soft_reset_rx(void);

// TODO add to docs chibios event callbacks are in syslock check mode
void _dw_irq_handler(void);

void dw_start_tx(tx_fctrl_t tx_fctrl, uint8_t * tx_buf, dx_time_t dly_time, ack_resp_t_t w4r_time);

void dw_start_rx(dx_time_t dly_time);

sys_state_t dw_transceiver_off(void);

void dw_command_read_OTP(uint16_t address);

uint64_t dw_get_tx_time(void);

uint64_t dw_get_rx_time(void);

int32_t dw_get_car_int(void);

//TODO add function to include tx time in uwb message format ieee with delayed tx