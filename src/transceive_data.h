/**
 * \brief 	transceive_data Header - SPI + LATCH reception
 * \file	transceive_data.h
 * \author  Rene Reinsch
 * \date		10.11.2013
 * \version  	Rev. 3.1 22.2.2014
 *
 * \details Defines for administration
 *		  	\n -ATMEGA DATASHEET / <avr/*.h>
 * 			\n Function prototypes definitions
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef TRANSCEIVE_DATA_H_
#define TRANSCEIVE_DATA_H_
// SPI
#define SPI_PORT PORTB
#define SPI_PORT_MASK 0x00
#define SPI_DDR DDRB
#define SPI_DDR_MASK (1<<DDB4)
#define SPI_CTRL_REG SPCR
#define SPI_STAT_REG SPSR
#define SPI_CTRL_REG_MASK (1<<SPE)
#define SPI_DATA_REG SPDR
#define SPI_ISR_VECTOR SPI_STC_vect
#define SPI_ENABLE_ISR_MASK (1<<SPIE)
#define SPI_DISABLE_ISR_MASK (~(1<<SPIE))
// GPIO and PIN CHANGE
#define EXT_LAT_DDR DDRB
#define EXT_LAT_PIN PINB1
#define EXT_LAT_PIN_MASK (1<<EXT_LAT_PIN)
#define EXT_LAT_PIN_REG PINB
#define EXT_LAT_PORT_MASK 0
#define EXT_LAT_DDR_MASK 0
#define PIN_CHANGE_EN_MASK_REG PCMSK0 //PORTB=Block0
#define PIN_CHANGE_ISR_EN_REG PCICR
#define PIN_CHANGE_EN_MASK_REG_MASK (1<<PCINT1)
#define PIN_CHANGE_ISR_EN_REG_MASK (1<<PCIE0)
#define PIN_CHANGE_ISR_VECTOR PCINT0_vect
// SPI RX-ADMINISTRATION
#define RX_DATA_VALID 0x01
#define RX_DATA_INVALID 0x00
#define RX_DATA_MAX_COUNT 192 
// EXT_LATCH-ADMINISTRATION
#define EXT_CMD_CLR_RX_BUFFER 0x02
#define EXT_CMD_RESET_BAM 0x04
#define EXT_CMD_SET 0x01
#define EXT_CMD_CLR 0x00
// Prototypes
extern void init_SPI(void);
extern void init_PIN_CHANGE_ISR(void);
extern void check_valid_rx_data(void);
extern void reset_rx_variables(void);

#endif /* TRANSCEIVE_DATA_H_ */
