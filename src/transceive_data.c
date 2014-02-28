/*
 * transceive_data.c
 *
 * Created: 10.11.2013
 * Author: René Reinsch
 * This module does the spi receiving,
 * and handles the ext. latch
 * Rev. 3.1 22.2.2014
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include "transceive_data.h"
#include "bam.h"

static volatile uint8_t rx_buffer;
static volatile uint8_t rx_byte_counter;
static volatile uint8_t rx_flag;
static volatile uint8_t ext_cmd_state_flag;

/* Init spi */
void init_SPI(void){
	SPI_DDR |= SPI_DDR_MASK;
	SPI_PORT |= SPI_PORT_MASK;
	SPI_CTRL_REG = SPI_CTRL_REG_MASK;
	SPI_DATA_REG = 0;
	reset_rx_variables();
}
/* EXT LATCH INIT */
void init_PIN_CHANGE_ISR(void){
	EXT_LAT_DDR &= ~(EXT_LAT_DDR_MASK);
	PIN_CHANGE_EN_MASK_REG |= PIN_CHANGE_EN_MASK_REG_MASK;
	PIN_CHANGE_ISR_EN_REG|=PIN_CHANGE_ISR_EN_REG_MASK;
	ext_cmd_state_flag = EXT_CMD_CLR;
}	

/* RESET_RX_VARIABLES */
void reset_rx_variables(void){
	uint8_t dum=0;
	rx_buffer=0;
	rx_byte_counter=0;
	rx_flag=RX_DATA_INVALID;
	dum=SPI_STAT_REG;
    dum=SPI_DATA_REG;
}	
 
/* ISR EXT LATCH */
/* Ext_Latch from 0 to 1 -> RX_DATA valid and enable SPI ISR*/
/* EXT_LATCH from 1 to 0 -> RX_DATA invalid disable SPI_ISR */
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
/* check_valid rx_data */
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

// ISR while LATCH is set!
// reset rx buffer in the first isr
// next ist BAM cycle reset + buffer reset!
ISR(SPI_ISR_VECTOR){
	reset_rx_variables();
	if(ext_cmd_state_flag == EXT_CMD_CLR_RX_BUFFER){
		reset_BAM();
		start_timer();
	}
	ext_cmd_state_flag = EXT_CMD_CLR_RX_BUFFER;
}
