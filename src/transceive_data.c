/**
 * \brief		SPI + LATCH reception
 * \file		transceive_data.c
 * \author 		Rene Reinsch
 * \date		10.11.2013
 * \version 	Rev. 3.1 22.2.2014
 *
 * \details		\b ext. commands
 * 				\n\b LATCH
 * 				\n ext. LATCH = 0 -> 1 [Pin change from 0 to 1]
 * 				\n 1. SPDR valid, vaild for RX-Counter von 0-191
 *				\n 2. enable the SPI-Interrupt
 *				\n 3. At RX_Counter=192 => saved picture data valid => switch BAM Table
 *				\n ext. LATCH = 1 -> 0 [Pin change from 1 to 0]
 *				\n disable the SPI-Interrupt
 *				\n\b Reset \b RX-Buffer
 *				\n ext. LATCH = 1 & 1 x SPI RX ISR
 *				\n RX-Counter=0 ( Buffer Reset )
 *				\n \b BAM \b cycle \b reset
 *				\n LATCH = 1 & 2 x SPI RX ISR
 *				\n 1. RX-Counter=0 ( Buffer Reset )
 *				\n 2. BAM-Cycle Reset ( external Sync )
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "transceive_data.h"
#include "bam.h"
// volatile ... used also in ISR
static volatile uint8_t rx_buffer; //!< SPI RX-BUFFER to secure data of the SPDR, used in check_valid_rx_data() and ISR(PIN_CHANGE_ISR_VECTOR)
static volatile uint8_t rx_byte_counter; //!< LATCH counter, used in check_valid_rx_data() and ISR(PIN_CHANGE_ISR_VECTOR)
static volatile uint8_t rx_flag; //!< Flag for RX data valid, used in check_valid_rx_data() and ISR(PIN_CHANGE_ISR_VECTOR)
static volatile uint8_t ext_cmd_state_flag; //!< Flag for Reset Buffer/BAM-Cyle, used in ISR(SPI_ISR_VECTOR)

/** \brief Initialize the SPI */
void init_SPI(void){
	SPI_DDR |= SPI_DDR_MASK;
	SPI_PORT |= SPI_PORT_MASK;
	SPI_CTRL_REG = SPI_CTRL_REG_MASK;
	SPI_DATA_REG = 0;
	reset_rx_variables();
}

/** \brief Initialize the pin change Interrupt */
void init_PIN_CHANGE_ISR(void){
	EXT_LAT_DDR &= ~(EXT_LAT_DDR_MASK);
	PIN_CHANGE_EN_MASK_REG |= PIN_CHANGE_EN_MASK_REG_MASK;
	PIN_CHANGE_ISR_EN_REG|=PIN_CHANGE_ISR_EN_REG_MASK;
	ext_cmd_state_flag = EXT_CMD_CLR;
}	

/** \brief reset all variables
 *
 * \details	clear the used administration data such as rx_buffer, clear the SPI
 */
void reset_rx_variables(void){
	uint8_t dum=0;
	rx_buffer=0;
	rx_byte_counter=0;
	rx_flag=RX_DATA_INVALID;
	dum=SPI_STAT_REG;
    dum=SPI_DATA_REG;
}	

/** \brief ISR ( PIN_CHANGE ) - handle ext. LATCH
 *  \param   	PIN_CHANGE_ISR_VECTOR  ISR VECTOR
 *
 * \details	ext. LATCH = 0 -> 1 [Pin change from 0 to 1]
 *	 		\n 1. SPDR valid, vaild for RX-Counter von 0-191
 *			\n 2. enable the SPI-Interrupt
 *			\n 3. At RX_Counter=192 => saved picture data valid => switch BAM Table
 *			\n ext. LATCH = 1 -> 0 [Pin change from 1 to 0]
 *			\n disable the SPI-Interrupt
 *
 * \note	uint8_t dum = SPI_STAT_REG; clears the ISR flag!!!!
 */
ISR(PIN_CHANGE_ISR_VECTOR){	
	if (EXT_LAT_PIN_REG & EXT_LAT_PIN_MASK){
        uint8_t dum = SPI_STAT_REG;
		rx_buffer=SPI_DATA_REG;		
		rx_flag = RX_DATA_VALID;
		SPI_CTRL_REG |= (SPI_ENABLE_ISR_MASK);
	} else {
		SPI_CTRL_REG &= SPI_DISABLE_ISR_MASK;
		ext_cmd_state_flag = EXT_CMD_CLR;
	}
}

/** \brief handle valid rx-data
 *
 * \details In case from rx_byte_counter from 0 to 191 save rx_buffer byte
 * 			to BAM buffer(calc_tbl_mem) ... uses the process_bam_input function
 * 		  	In case rx_byte_counter >= 192 ->switch the source pointer of the BAM
 */
void check_valid_rx_data(void){
	if(rx_flag == RX_DATA_VALID){
		if(rx_byte_counter<RX_DATA_MAX_COUNT){
			process_bam_input(rx_buffer,rx_byte_counter);
			rx_byte_counter++;			
		} else {
			switch_bam_pointer();
			rx_byte_counter=0;
		}	
		rx_flag=RX_DATA_INVALID;
	}
}

/** \brief ISR ( SPI ) - handle Reset Buffer / Reset BAM cyle
 *  \param   SPI_ISR_VECTOR ISR VECTOR
 *
 * \details	\b Reset RX-Buffer
 *			\n ext. LATCH = 1 & 1 x SPI RX ISR
 *			\n RX-Counter=0 ( Buffer Reset )
 *			\n\b BAM cycle reset
 *			\n LATCH = 1 & 2 x SPI RX ISR
 *			\n 1. RX-Counter=0 ( Buffer Reset )
 *			\n 2. **BAM-Cycle Reset** ( external Sync )
 *
 * \note	not used for any BAM picture data
 */
ISR(SPI_ISR_VECTOR){
	reset_rx_variables();
	if(ext_cmd_state_flag == EXT_CMD_CLR_RX_BUFFER){
		reset_BAM();
		start_timer();
	}
	ext_cmd_state_flag = EXT_CMD_CLR_RX_BUFFER;
}
