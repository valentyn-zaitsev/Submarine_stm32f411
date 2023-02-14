
//Library:				NRF24L01 software library for STM32 MCUs

/*
 * nrf24l01.h
 *
 *  Created on: 1 авг. 2019 г.
 *      Author: dima
 */

#ifndef INC_NRF24_H
#define INC_NRF24_H


#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)

/* Memory Map */
#define NRF_CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define NRF_STATUS  0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
#define EN_DPL	    2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define RPD         0x09
#define W_TX_PAYLOAD_NO_ACK  0xB0

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

typedef enum
{
	NRF_OK,
	NRF_INIT_ERROR,

}nrf_state;

typedef enum
{
	RF24_PA_MIN = 0,
	RF24_PA_LOW,
	RF24_PA_HIGH,
	RF24_PA_MAX,
	RF24_PA_ERROR
} rf24_pa_dbm_e;

typedef enum
{
	RF24_1MBPS = 0,
	RF24_2MBPS,
	RF24_250KBPS
} rf24_datarate_e;

typedef enum
{
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8,
	RF24_CRC_16
} rf24_crclength_e;


nrf_state nrf_init(SPI_HandleTypeDef *spi, rf24_pa_dbm_e level, rf24_datarate_e datarate, uint8_t channel, UART_HandleTypeDef *uart);
bool nrf_isChipConnected();
void nrf_startListening(void);
void nrf_stopListening(void);
bool nrf_availableMy(void);
void nrf_read(void* buf, uint8_t len);
bool nrf_write(const void* buf, uint8_t len);
bool nrf_available(uint8_t* pipe_num);
uint8_t nrf_spiTrans(uint8_t cmd);
void nrf_powerDown(void);
void nrf_powerUp(void);
void nrf_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);
bool nrf_isAckPayloadAvailable(void);
uint8_t nrf_whatHappened();
void nrf_startFastWrite(const void* buf, uint8_t len, const bool multicast, bool startTx);
uint8_t nrf_flush_tx(void);
void nrf_closeReadingPipe(uint8_t pipe);
void nrf_setAddressWidth(uint8_t a_width);
void nrf_setRetries(uint8_t delay, uint8_t count);
void nrf_setChannel(uint8_t channel);
uint8_t nrf_getChannel(void);
void nrf_setPayloadSize(uint8_t size);
uint8_t nrf_getPayloadSize(void);
uint8_t nrf_getDynamicPayloadSize(void);
void nrf_enableAckPayload(void);
void nrf_enableDynamicPayloads(void);
void nrf_disableDynamicPayloads(void);
void nrf_enableDynamicAck();
bool nrf_isPVariant(void);
void nrf_setAutoAck(bool enable);
void nrf_setAutoAckPipe(uint8_t pipe, bool enable);
void nrf_setPALevel(rf24_pa_dbm_e level);
uint8_t nrf_getPALevel(void);
bool nrf_setDataRate(rf24_datarate_e speed);
rf24_datarate_e nrf_getDataRate(void);
void nrf_setCRCLength(rf24_crclength_e length);
rf24_crclength_e nrf_getCRCLength(void);
void nrf_disableCRC(void);
void nrf_maskIRQ(bool tx_ok,bool tx_fail,bool rx_ready);
void nrf_openReadingPipe(uint8_t number, uint64_t address);
void nrf_openWritingPipe(uint64_t address);
uint8_t nrf_flush_rx(void);
void nrf_csn(uint8_t mode);
void nrf_ce(uint8_t level);

uint8_t nrf_write_payload(const void* buf, uint8_t len, const uint8_t writeType);
uint8_t nrf_read_payload(void* buf, uint8_t len);
uint8_t nrf_get_status(void);
void nrf_toggle_features(void);


#endif

//#define _BV(x) (1<<(x))
//
///* Memory Map */
//#define REG_CONFIG      0x00
//#define REG_EN_AA       0x01
//#define REG_EN_RXADDR   0x02
//#define REG_SETUP_AW    0x03
//#define REG_SETUP_RETR  0x04
//#define REG_RF_CH       0x05
//#define REG_RF_SETUP    0x06
//#define REG_STATUS      0x07
//#define REG_OBSERVE_TX  0x08
//#define REG_CD          0x09
//#define REG_RX_ADDR_P0  0x0A
//#define REG_RX_ADDR_P1  0x0B
//#define REG_RX_ADDR_P2  0x0C
//#define REG_RX_ADDR_P3  0x0D
//#define REG_RX_ADDR_P4  0x0E
//#define REG_RX_ADDR_P5  0x0F
//#define REG_TX_ADDR     0x10
//#define REG_RX_PW_P0    0x11
//#define REG_RX_PW_P1    0x12
//#define REG_RX_PW_P2    0x13
//#define REG_RX_PW_P3    0x14
//#define REG_RX_PW_P4    0x15
//#define REG_RX_PW_P5    0x16
//#define REG_FIFO_STATUS 0x17
//#define REG_DYNPD	    	0x1C
//#define REG_FEATURE	    0x1D
//
///* Bit Mnemonics */
//#define MASK_RX_DR  6
//#define MASK_TX_DS  5
//#define MASK_MAX_RT 4
//#define BIT_EN_CRC      3
//#define BIT_CRCO        2
//#define BIT_PWR_UP      1
//#define BIT_PRIM_RX     0
//#define BIT_ENAA_P5     5
//#define BIT_ENAA_P4     4
//#define BIT_ENAA_P3     3
//#define BIT_ENAA_P2     2
//#define BIT_ENAA_P1     1
//#define BIT_ENAA_P0     0
//#define BIT_ERX_P5      5
//#define BIT_ERX_P4      4
//#define BIT_ERX_P3      3
//#define BIT_ERX_P2      2
//#define BIT_ERX_P1      1
//#define BIT_ERX_P0      0
//#define BIT_AW          0
//#define BIT_ARD         4
//#define BIT_ARC         0
//#define BIT_PLL_LOCK    4
//#define BIT_RF_DR       3
//#define BIT_RF_PWR      6
//#define BIT_RX_DR       6
//#define BIT_TX_DS       5
//#define BIT_MAX_RT      4
//#define BIT_RX_P_NO     1
//#define BIT_TX_FULL     0
//#define BIT_PLOS_CNT    4
//#define BIT_ARC_CNT     0
//#define BIT_TX_REUSE    6
//#define BIT_FIFO_FULL   5
//#define BIT_TX_EMPTY    4
//#define BIT_RX_FULL     1
//#define BIT_RX_EMPTY    0
//#define BIT_DPL_P5	    5
//#define BIT_DPL_P4	    4
//#define BIT_DPL_P3	    3
//#define BIT_DPL_P2	    2
//#define BIT_DPL_P1	    1
//#define BIT_DPL_P0	    0
//#define BIT_EN_DPL	    2
//#define BIT_EN_ACK_PAY  1
//#define BIT_EN_DYN_ACK  0
//
///* Instruction Mnemonics */
//#define CMD_R_REGISTER    0x00
//#define CMD_W_REGISTER    0x20
//#define CMD_REGISTER_MASK 0x1F
//#define CMD_ACTIVATE      0x50
//#define CMD_R_RX_PL_WID   0x60
//#define CMD_R_RX_PAYLOAD  0x61
//#define CMD_W_TX_PAYLOAD  0xA0
//#define CMD_W_ACK_PAYLOAD 0xA8
//#define CMD_FLUSH_TX      0xE1
//#define CMD_FLUSH_RX      0xE2
//#define CMD_REUSE_TX_PL   0xE3
//#define CMD_NOP           0xFF
//
///* Non-P omissions */
//#define LNA_HCURR   0
//
///* P model memory Map */
//#define REG_RPD         0x09
//
///* P model bit Mnemonics */
//#define RF_DR_LOW   5
//#define RF_DR_HIGH  3
//#define RF_PWR_LOW  1
//#define RF_PWR_HIGH 2
