/**
 * \brief		BAM administration and TLC communication
 * \file		bam.c
 * \author  	Rene Reinsch
 * \date		10.11.2013
 * \version  	Rev. 3.1 22.2.2014
 *
 * \details 	This module does the BAM control and handles the communication with the TLC59281.
 *				\n\b important:	After the 191th byte has been received, a minimum time of 35µS should
 *				\n be waited before the final latch can be send.
 * \note		\b recommended: 50µS pause after every latch
 *
 */

#include "bam.h"
#include "transceive_data.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// BAM TIMER RELOAD MAP L
static volatile const uint8_t bam_timer_map_l[BAM_STEPS]={
	BAM_TMR_RLD_STP_0_L,
	BAM_TMR_RLD_STP_1_L,
	BAM_TMR_RLD_STP_2_L,
	BAM_TMR_RLD_STP_3_L,
	BAM_TMR_RLD_STP_4_L,
	BAM_TMR_RLD_STP_5_L,
	BAM_TMR_RLD_STP_6_L,
	BAM_TMR_RLD_STP_7_L }; //!< Lookuptable - timer16 low byte reload map, used in ISR(TIMER_16_vect)

// BAM TIMER RELOAD MAP H
static volatile const uint8_t bam_timer_map_h[BAM_STEPS]={
	BAM_TMR_RLD_STP_0_H,
	BAM_TMR_RLD_STP_1_H,
	BAM_TMR_RLD_STP_2_H,
	BAM_TMR_RLD_STP_3_H,
	BAM_TMR_RLD_STP_4_H,
	BAM_TMR_RLD_STP_5_H,
	BAM_TMR_RLD_STP_6_H,
	BAM_TMR_RLD_STP_7_H }; //!< Lookuptable - timer16 high byte reload, map used in ISR(TIMER_16_vect)

// BAM STEP TABLE POSITION MAP - for transmit 
static const uint8_t bam_step_map[BAM_STEPS]={
	BAM_TBL_POS_STEP_0,BAM_TBL_POS_STEP_1,BAM_TBL_POS_STEP_2,BAM_TBL_POS_STEP_3,
	BAM_TBL_POS_STEP_4,BAM_TBL_POS_STEP_5,BAM_TBL_POS_STEP_6,BAM_TBL_POS_STEP_7
 };	//!< Lookuptable - timer16 reload map offset, used in ISR(TIMER_16_vect)

// Maps for look up
// BIT MASK = STRING 0 - 5 TLC SPI
static const uint8_t lookup_bit_mask[] = {
	 LED_0_0_R_BIT_POS_MASK,LED_0_0_G_BIT_POS_MASK,LED_0_0_B_BIT_POS_MASK,
	 LED_1_0_R_BIT_POS_MASK,LED_1_0_G_BIT_POS_MASK,LED_1_0_B_BIT_POS_MASK,
	 LED_2_0_R_BIT_POS_MASK,LED_2_0_G_BIT_POS_MASK,LED_2_0_B_BIT_POS_MASK,
	 LED_3_0_R_BIT_POS_MASK,LED_3_0_G_BIT_POS_MASK,LED_3_0_B_BIT_POS_MASK,
	 LED_4_0_R_BIT_POS_MASK,LED_4_0_G_BIT_POS_MASK,LED_4_0_B_BIT_POS_MASK,
	 LED_5_0_R_BIT_POS_MASK,LED_5_0_G_BIT_POS_MASK,LED_5_0_B_BIT_POS_MASK,
	 LED_6_0_R_BIT_POS_MASK,LED_6_0_G_BIT_POS_MASK,LED_6_0_B_BIT_POS_MASK,
	 LED_7_0_R_BIT_POS_MASK,LED_7_0_G_BIT_POS_MASK,LED_7_0_B_BIT_POS_MASK,
	 LED_0_1_R_BIT_POS_MASK,LED_0_1_G_BIT_POS_MASK,LED_0_1_B_BIT_POS_MASK,
	 LED_1_1_R_BIT_POS_MASK,LED_1_1_G_BIT_POS_MASK,LED_1_1_B_BIT_POS_MASK,
	 LED_2_1_R_BIT_POS_MASK,LED_2_1_G_BIT_POS_MASK,LED_2_1_B_BIT_POS_MASK,
	 LED_3_1_R_BIT_POS_MASK,LED_3_1_G_BIT_POS_MASK,LED_3_1_B_BIT_POS_MASK,
	 LED_4_1_R_BIT_POS_MASK,LED_4_1_G_BIT_POS_MASK,LED_4_1_B_BIT_POS_MASK,
	 LED_5_1_R_BIT_POS_MASK,LED_5_1_G_BIT_POS_MASK,LED_5_1_B_BIT_POS_MASK,
	 LED_6_1_R_BIT_POS_MASK,LED_6_1_G_BIT_POS_MASK,LED_6_1_B_BIT_POS_MASK,
	 LED_7_1_R_BIT_POS_MASK,LED_7_1_G_BIT_POS_MASK,LED_7_1_B_BIT_POS_MASK,
	 LED_0_2_R_BIT_POS_MASK,LED_0_2_G_BIT_POS_MASK,LED_0_2_B_BIT_POS_MASK,
	 LED_1_2_R_BIT_POS_MASK,LED_1_2_G_BIT_POS_MASK,LED_1_2_B_BIT_POS_MASK,
	 LED_2_2_R_BIT_POS_MASK,LED_2_2_G_BIT_POS_MASK,LED_2_2_B_BIT_POS_MASK,
	 LED_3_2_R_BIT_POS_MASK,LED_3_2_G_BIT_POS_MASK,LED_3_2_B_BIT_POS_MASK,
	 LED_4_2_R_BIT_POS_MASK,LED_4_2_G_BIT_POS_MASK,LED_4_2_B_BIT_POS_MASK,
	 LED_5_2_R_BIT_POS_MASK,LED_5_2_G_BIT_POS_MASK,LED_5_2_B_BIT_POS_MASK,
	 LED_6_2_R_BIT_POS_MASK,LED_6_2_G_BIT_POS_MASK,LED_6_2_B_BIT_POS_MASK,
	 LED_7_2_R_BIT_POS_MASK,LED_7_2_G_BIT_POS_MASK,LED_7_2_B_BIT_POS_MASK,
	 LED_0_3_R_BIT_POS_MASK,LED_0_3_G_BIT_POS_MASK,LED_0_3_B_BIT_POS_MASK,
	 LED_1_3_R_BIT_POS_MASK,LED_1_3_G_BIT_POS_MASK,LED_1_3_B_BIT_POS_MASK,
	 LED_2_3_R_BIT_POS_MASK,LED_2_3_G_BIT_POS_MASK,LED_2_3_B_BIT_POS_MASK,
	 LED_3_3_R_BIT_POS_MASK,LED_3_3_G_BIT_POS_MASK,LED_3_3_B_BIT_POS_MASK,
	 LED_4_3_R_BIT_POS_MASK,LED_4_3_G_BIT_POS_MASK,LED_4_3_B_BIT_POS_MASK,
	 LED_5_3_R_BIT_POS_MASK,LED_5_3_G_BIT_POS_MASK,LED_5_3_B_BIT_POS_MASK,
	 LED_6_3_R_BIT_POS_MASK,LED_6_3_G_BIT_POS_MASK,LED_6_3_B_BIT_POS_MASK,
	 LED_7_3_R_BIT_POS_MASK,LED_7_3_G_BIT_POS_MASK,LED_7_3_B_BIT_POS_MASK,
	 LED_0_4_R_BIT_POS_MASK,LED_0_4_G_BIT_POS_MASK,LED_0_4_B_BIT_POS_MASK,
	 LED_1_4_R_BIT_POS_MASK,LED_1_4_G_BIT_POS_MASK,LED_1_4_B_BIT_POS_MASK,
	 LED_2_4_R_BIT_POS_MASK,LED_2_4_G_BIT_POS_MASK,LED_2_4_B_BIT_POS_MASK,
	 LED_3_4_R_BIT_POS_MASK,LED_3_4_G_BIT_POS_MASK,LED_3_4_B_BIT_POS_MASK,
	 LED_4_4_R_BIT_POS_MASK,LED_4_4_G_BIT_POS_MASK,LED_4_4_B_BIT_POS_MASK,
	 LED_5_4_R_BIT_POS_MASK,LED_5_4_G_BIT_POS_MASK,LED_5_4_B_BIT_POS_MASK,
	 LED_6_4_R_BIT_POS_MASK,LED_6_4_G_BIT_POS_MASK,LED_6_4_B_BIT_POS_MASK,
	 LED_7_4_R_BIT_POS_MASK,LED_7_4_G_BIT_POS_MASK,LED_7_4_B_BIT_POS_MASK,
	 LED_0_5_R_BIT_POS_MASK,LED_0_5_G_BIT_POS_MASK,LED_0_5_B_BIT_POS_MASK,
	 LED_1_5_R_BIT_POS_MASK,LED_1_5_G_BIT_POS_MASK,LED_1_5_B_BIT_POS_MASK,
	 LED_2_5_R_BIT_POS_MASK,LED_2_5_G_BIT_POS_MASK,LED_2_5_B_BIT_POS_MASK,
	 LED_3_5_R_BIT_POS_MASK,LED_3_5_G_BIT_POS_MASK,LED_3_5_B_BIT_POS_MASK,
	 LED_4_5_R_BIT_POS_MASK,LED_4_5_G_BIT_POS_MASK,LED_4_5_B_BIT_POS_MASK,
	 LED_5_5_R_BIT_POS_MASK,LED_5_5_G_BIT_POS_MASK,LED_5_5_B_BIT_POS_MASK,
	 LED_6_5_R_BIT_POS_MASK,LED_6_5_G_BIT_POS_MASK,LED_6_5_B_BIT_POS_MASK,
	 LED_7_5_R_BIT_POS_MASK,LED_7_5_G_BIT_POS_MASK,LED_7_5_B_BIT_POS_MASK,
	 LED_0_6_R_BIT_POS_MASK,LED_0_6_G_BIT_POS_MASK,LED_0_6_B_BIT_POS_MASK,
	 LED_1_6_R_BIT_POS_MASK,LED_1_6_G_BIT_POS_MASK,LED_1_6_B_BIT_POS_MASK,
	 LED_2_6_R_BIT_POS_MASK,LED_2_6_G_BIT_POS_MASK,LED_2_6_B_BIT_POS_MASK,
	 LED_3_6_R_BIT_POS_MASK,LED_3_6_G_BIT_POS_MASK,LED_3_6_B_BIT_POS_MASK,
	 LED_4_6_R_BIT_POS_MASK,LED_4_6_G_BIT_POS_MASK,LED_4_6_B_BIT_POS_MASK,
	 LED_5_6_R_BIT_POS_MASK,LED_5_6_G_BIT_POS_MASK,LED_5_6_B_BIT_POS_MASK,
	 LED_6_6_R_BIT_POS_MASK,LED_6_6_G_BIT_POS_MASK,LED_6_6_B_BIT_POS_MASK,
	 LED_7_6_R_BIT_POS_MASK,LED_7_6_G_BIT_POS_MASK,LED_7_6_B_BIT_POS_MASK,
	 LED_0_7_R_BIT_POS_MASK,LED_0_7_G_BIT_POS_MASK,LED_0_7_B_BIT_POS_MASK,
	 LED_1_7_R_BIT_POS_MASK,LED_1_7_G_BIT_POS_MASK,LED_1_7_B_BIT_POS_MASK,
	 LED_2_7_R_BIT_POS_MASK,LED_2_7_G_BIT_POS_MASK,LED_2_7_B_BIT_POS_MASK,
	 LED_3_7_R_BIT_POS_MASK,LED_3_7_G_BIT_POS_MASK,LED_3_7_B_BIT_POS_MASK,
	 LED_4_7_R_BIT_POS_MASK,LED_4_7_G_BIT_POS_MASK,LED_4_7_B_BIT_POS_MASK,
	 LED_5_7_R_BIT_POS_MASK,LED_5_7_G_BIT_POS_MASK,LED_5_7_B_BIT_POS_MASK,
	 LED_6_7_R_BIT_POS_MASK,LED_6_7_G_BIT_POS_MASK,LED_6_7_B_BIT_POS_MASK,
	 LED_7_7_R_BIT_POS_MASK,LED_7_7_G_BIT_POS_MASK,LED_7_7_B_BIT_POS_MASK }; //!< Lookuptable - Bit position used in process_bam_input()

// BYTE Pos = TLCOUT BIT NO.
static const uint8_t lookup_byte_pos[] = {
	 LED_0_0_R_BYTE_POS,LED_0_0_G_BYTE_POS,LED_0_0_B_BYTE_POS,
	 LED_1_0_R_BYTE_POS,LED_1_0_G_BYTE_POS,LED_1_0_B_BYTE_POS,
	 LED_2_0_R_BYTE_POS,LED_2_0_G_BYTE_POS,LED_2_0_B_BYTE_POS,
	 LED_3_0_R_BYTE_POS,LED_3_0_G_BYTE_POS,LED_3_0_B_BYTE_POS,
	 LED_4_0_R_BYTE_POS,LED_4_0_G_BYTE_POS,LED_4_0_B_BYTE_POS,
	 LED_5_0_R_BYTE_POS,LED_5_0_G_BYTE_POS,LED_5_0_B_BYTE_POS,
	 LED_6_0_R_BYTE_POS,LED_6_0_G_BYTE_POS,LED_6_0_B_BYTE_POS,
	 LED_7_0_R_BYTE_POS,LED_7_0_G_BYTE_POS,LED_7_0_B_BYTE_POS,
	 LED_0_1_R_BYTE_POS,LED_0_1_G_BYTE_POS,LED_0_1_B_BYTE_POS,
	 LED_1_1_R_BYTE_POS,LED_1_1_G_BYTE_POS,LED_1_1_B_BYTE_POS,
	 LED_2_1_R_BYTE_POS,LED_2_1_G_BYTE_POS,LED_2_1_B_BYTE_POS,
	 LED_3_1_R_BYTE_POS,LED_3_1_G_BYTE_POS,LED_3_1_B_BYTE_POS,
	 LED_4_1_R_BYTE_POS,LED_4_1_G_BYTE_POS,LED_4_1_B_BYTE_POS,
	 LED_5_1_R_BYTE_POS,LED_5_1_G_BYTE_POS,LED_5_1_B_BYTE_POS,
	 LED_6_1_R_BYTE_POS,LED_6_1_G_BYTE_POS,LED_6_1_B_BYTE_POS,
	 LED_7_1_R_BYTE_POS,LED_7_1_G_BYTE_POS,LED_7_1_B_BYTE_POS,
	 LED_0_2_R_BYTE_POS,LED_0_2_G_BYTE_POS,LED_0_2_B_BYTE_POS,
	 LED_1_2_R_BYTE_POS,LED_1_2_G_BYTE_POS,LED_1_2_B_BYTE_POS,
	 LED_2_2_R_BYTE_POS,LED_2_2_G_BYTE_POS,LED_2_2_B_BYTE_POS,
	 LED_3_2_R_BYTE_POS,LED_3_2_G_BYTE_POS,LED_3_2_B_BYTE_POS,
	 LED_4_2_R_BYTE_POS,LED_4_2_G_BYTE_POS,LED_4_2_B_BYTE_POS,
	 LED_5_2_R_BYTE_POS,LED_5_2_G_BYTE_POS,LED_5_2_B_BYTE_POS,
	 LED_6_2_R_BYTE_POS,LED_6_2_G_BYTE_POS,LED_6_2_B_BYTE_POS,
	 LED_7_2_R_BYTE_POS,LED_7_2_G_BYTE_POS,LED_7_2_B_BYTE_POS,
	 LED_0_3_R_BYTE_POS,LED_0_3_G_BYTE_POS,LED_0_3_B_BYTE_POS,
	 LED_1_3_R_BYTE_POS,LED_1_3_G_BYTE_POS,LED_1_3_B_BYTE_POS,
	 LED_2_3_R_BYTE_POS,LED_2_3_G_BYTE_POS,LED_2_3_B_BYTE_POS,
	 LED_3_3_R_BYTE_POS,LED_3_3_G_BYTE_POS,LED_3_3_B_BYTE_POS,
	 LED_4_3_R_BYTE_POS,LED_4_3_G_BYTE_POS,LED_4_3_B_BYTE_POS,
	 LED_5_3_R_BYTE_POS,LED_5_3_G_BYTE_POS,LED_5_3_B_BYTE_POS,
	 LED_6_3_R_BYTE_POS,LED_6_3_G_BYTE_POS,LED_6_3_B_BYTE_POS,
	 LED_7_3_R_BYTE_POS,LED_7_3_G_BYTE_POS,LED_7_3_B_BYTE_POS,
	 LED_0_4_R_BYTE_POS,LED_0_4_G_BYTE_POS,LED_0_4_B_BYTE_POS,
	 LED_1_4_R_BYTE_POS,LED_1_4_G_BYTE_POS,LED_1_4_B_BYTE_POS,
	 LED_2_4_R_BYTE_POS,LED_2_4_G_BYTE_POS,LED_2_4_B_BYTE_POS,
	 LED_3_4_R_BYTE_POS,LED_3_4_G_BYTE_POS,LED_3_4_B_BYTE_POS,
	 LED_4_4_R_BYTE_POS,LED_4_4_G_BYTE_POS,LED_4_4_B_BYTE_POS,
	 LED_5_4_R_BYTE_POS,LED_5_4_G_BYTE_POS,LED_5_4_B_BYTE_POS,
	 LED_6_4_R_BYTE_POS,LED_6_4_G_BYTE_POS,LED_6_4_B_BYTE_POS,
	 LED_7_4_R_BYTE_POS,LED_7_4_G_BYTE_POS,LED_7_4_B_BYTE_POS,
	 LED_0_5_R_BYTE_POS,LED_0_5_G_BYTE_POS,LED_0_5_B_BYTE_POS,
	 LED_1_5_R_BYTE_POS,LED_1_5_G_BYTE_POS,LED_1_5_B_BYTE_POS,
	 LED_2_5_R_BYTE_POS,LED_2_5_G_BYTE_POS,LED_2_5_B_BYTE_POS,
	 LED_3_5_R_BYTE_POS,LED_3_5_G_BYTE_POS,LED_3_5_B_BYTE_POS,
	 LED_4_5_R_BYTE_POS,LED_4_5_G_BYTE_POS,LED_4_5_B_BYTE_POS,
	 LED_5_5_R_BYTE_POS,LED_5_5_G_BYTE_POS,LED_5_5_B_BYTE_POS,
	 LED_6_5_R_BYTE_POS,LED_6_5_G_BYTE_POS,LED_6_5_B_BYTE_POS,
	 LED_7_5_R_BYTE_POS,LED_7_5_G_BYTE_POS,LED_7_5_B_BYTE_POS,
	 LED_0_6_R_BYTE_POS,LED_0_6_G_BYTE_POS,LED_0_6_B_BYTE_POS,
	 LED_1_6_R_BYTE_POS,LED_1_6_G_BYTE_POS,LED_1_6_B_BYTE_POS,
	 LED_2_6_R_BYTE_POS,LED_2_6_G_BYTE_POS,LED_2_6_B_BYTE_POS,
	 LED_3_6_R_BYTE_POS,LED_3_6_G_BYTE_POS,LED_3_6_B_BYTE_POS,
	 LED_4_6_R_BYTE_POS,LED_4_6_G_BYTE_POS,LED_4_6_B_BYTE_POS,
	 LED_5_6_R_BYTE_POS,LED_5_6_G_BYTE_POS,LED_5_6_B_BYTE_POS,
	 LED_6_6_R_BYTE_POS,LED_6_6_G_BYTE_POS,LED_6_6_B_BYTE_POS,
	 LED_7_6_R_BYTE_POS,LED_7_6_G_BYTE_POS,LED_7_6_B_BYTE_POS,
	 LED_0_7_R_BYTE_POS,LED_0_7_G_BYTE_POS,LED_0_7_B_BYTE_POS,
	 LED_1_7_R_BYTE_POS,LED_1_7_G_BYTE_POS,LED_1_7_B_BYTE_POS,
	 LED_2_7_R_BYTE_POS,LED_2_7_G_BYTE_POS,LED_2_7_B_BYTE_POS,
	 LED_3_7_R_BYTE_POS,LED_3_7_G_BYTE_POS,LED_3_7_B_BYTE_POS,
	 LED_4_7_R_BYTE_POS,LED_4_7_G_BYTE_POS,LED_4_7_B_BYTE_POS,
	 LED_5_7_R_BYTE_POS,LED_5_7_G_BYTE_POS,LED_5_7_B_BYTE_POS,
	 LED_6_7_R_BYTE_POS,LED_6_7_G_BYTE_POS,LED_6_7_B_BYTE_POS,
	 LED_7_7_R_BYTE_POS,LED_7_7_G_BYTE_POS,LED_7_7_B_BYTE_POS }; //!< Lookuptable - Byte position used in process_bam_input()

// BAM STEP COUNTER
static volatile uint8_t bam_step; //!< bam step counter, used in ISR(TIMER_16_vect)

// BAM TABLE MEMORY - BAM sorted or for process use
static volatile uint8_t volatile bam_tbl_mem_1[BAM_MEM_SIZE]; //!< data source 32*8 Byte, used in transmit_BAM_step() or transmit_BAM_step()
static volatile uint8_t volatile bam_tbl_mem_2[BAM_MEM_SIZE]; //!< data source 32*8 Byte, used in transmit_BAM_step() or transmit_BAM_step()
static volatile uint8_t *volatile bam_tbl_mem;	//!< source pointer used in transmit_BAM_step(), points to bam_tbl_mem_1 or bam_tbl_mem_2
static volatile uint8_t *volatile bam_tbl_proc; //!< source pointer used in process_bam_input(), points to bam_tbl_mem_1 or bam_tbl_mem_2

// PROTOTYPES
static void init_TLC(void);


/** \brief Initialize GPIO's, timer, variables initialize the TLC's */
void init_BAM(void){
	uint16_t i=0;
	// init BLANK
	BLANK_PORT_DDR|=BLANK_PORT_DDR_MASK;
	BLANK_PORT|=BLANK_PORT_MASK;
	// init SCK PORT
	SCK_PORT_DDR|=SCK_PORT_DDR_MASK;
	SCK_PORT|=SCK_PORT_MASK;
	// init DATA PORT
	DATA_PORT_DDR|=DATA_PORT_DDR_MASK;
	DATA_PORT|=DATA_PORT_MASK;
	// init LAT 
	LAT_PORT_DDR|=LAT_PORT_DDR_MASK;
	LAT_PORT|=LAT_PORT_MASK;
	// init BLANK
	BLANK_PORT_DDR|=BLANK_PORT_DDR_MASK;
	BLANK_PORT|=BLANK_PORT_MASK;
	// init timer1 16-bit
	TIMER_16_CTRL_A=TIMER_16_CTRL_A_MASK;	
	TIMER_16_CTRL_C=TIMER_16_CTRL_C_MASK;
	TIMER_16_IMR = TIMER_16_IMR_MASK;
	// init variables
	for(i=0;i<BAM_MEM_SIZE;i++){
		bam_tbl_mem_1[i] = 0;
		bam_tbl_mem_2[i] = 0;
	}
	bam_tbl_mem=bam_tbl_mem_1;
	bam_tbl_proc=bam_tbl_mem_2;
	bam_step = 0;
	init_TLC();	
}

/** \brief Initialize the TLC's, clear the the TLC buffer */
static void init_TLC(void){
	uint8_t bit_counter=0;
	DATA_PORT = 0;
	SCK_PORT =  0;
	for(bit_counter=0;bit_counter<BAM_STRING_SIZE;bit_counter++){
		DATA_PORT = 0;
		SCK_PORT = SCK_PORT_MASK;
		_delay_us(10);
		SCK_PORT = ~SCK_PORT_MASK;
		_delay_us(10);
	}
	LAT_PORT = LAT_SET;
	_delay_us(100);
	LAT_PORT = LAT_RESET;
	BLANK_PORT &= ~BLANK_PORT_MASK;
}

/** \brief transmit the current BAM-Step to the TLCs
 *
 * \details	Bitbanging on the SoftSPI-GPIO's
 *  		Send's the current bam_tbl_mem (a block of 32 Bytes) to the TLC's
 *  		by toggling the Clock Port at about 2MHz and putting a stored byte
 *  		on the Output-GPIOport. A block of 32 Bytes is per any BAM step...
 *
 *	\note 	This must be interrupt free ! - Surround with cli()...sei() or put it in the timer isr
 *          \n Only 6 bits form a byte in the bam_tbl_mem  are used, because only 6 SOFTSPI's exist
 */
void transmit_BAM_step(void){
		uint8_t volatile *bam_tbl_ptr;
		// clear LAtch 
		LAT_PORT = LAT_RESET;
		// load ptr - bam step*32 + current bam_table , a lut is used...
		bam_tbl_ptr= &bam_tbl_mem[bam_step_map[bam_step]];
		// transmit next 32 Bytes
		// load DATA-byte then toggle SCK-PORT		
		SCK_PORT =  0; // 0
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);		
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;		
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 1		
		DATA_PORT = *bam_tbl_ptr;				
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 2
	    DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
	    SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
	    _delay_us(SOFT_SPI_H_TIME);
	    SCK_PORT = 0; //3
	    DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
	    SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
	    _delay_us(SOFT_SPI_H_TIME);
	    SCK_PORT = 0; // 4
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;		
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 5
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; //6
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 7
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 8
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 9
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT =  0; // 10
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 11
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 12
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 13
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 14
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; //15
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 16
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 17
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 18
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 19
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT =  0; // 20
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 21
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 22
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 23
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 24
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 25
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 26
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 27
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 28
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 29
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 30
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		bam_tbl_ptr++;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0; // 31
		DATA_PORT = *bam_tbl_ptr;
		_delay_us(SOFT_SPI_L_TIME);
		SCK_PORT = SCK_PORT_MASK;
		_delay_us(SOFT_SPI_H_TIME);
		SCK_PORT = 0;	
}

/** \brief ISR ( TIMER_16) - handle BAM cyle
 * \param  	TIMER_16_vect ISR VECTOR
 *
 * \details	load the new timer reload value from bam_timer_map
 *     		prepare the TLC for the next step, this includes
 *     		the transmit_BAM_step()...
 *
 * \note	transmit_BAM_step() needs couple of 10µS
 */
ISR(TIMER_16_vect){
	uint8_t bam_step_local = bam_step; // a local variable is faster!!!
	// reload timer with new value
	TIMER_16_CTRL_B = TIMER_16_STOP_TIMER;
	TIMER_16_CNTR_H = bam_timer_map_h[bam_step_local];
   	TIMER_16_CNTR_L = bam_timer_map_l[bam_step_local];
	bam_step_local++;
	if(bam_step_local>=BAM_STEPS){
		bam_step_local=0;				
	}
	bam_step=bam_step_local;
	// latch data
	LAT_PORT = LAT_PORT_MASK;		
	// start timer
	TIMER_16_CTRL_B = TIMER_16_START_TIMER;		
	// prepare next step
	transmit_BAM_step(); 	
}

/** \brief process the src byte into the BAM mem
 * \param  	uint8_t src 	- data to store
 * \param	uint8_t offset 	- position in the picture
 *
 * \details Processes every bit from input src data and put it in the right order
 *  		of the BAM Table (32 Byte per Cycle [TLC OUT 0-31])
 *  		And then put it in the right bit output byte (0 to 5 (SOFT SPI OUTPUT))
 *  		Also a remapping is used, this is done by the defines and the two lookuptable
 *
 * \note	This function needs a couple of 10µS
 */
void process_bam_input(uint8_t src, uint8_t offset){
	uint8_t byte_pos = lookup_byte_pos[offset]; // Byte Pos 0-31
	uint8_t bit_mask = lookup_bit_mask[offset]; // String Position 0-5
	uint8_t n_bit_mask = ~bit_mask;
	uint8_t volatile *bam_tbl_ptr_local=&bam_tbl_proc[byte_pos];
	// start with the first bit
	if(src & BIT0_MASK  ){ // 0b00000001
		*bam_tbl_ptr_local |= bit_mask;
	} else {
		*bam_tbl_ptr_local &= n_bit_mask;
	}
	// after the bit and byte offset is known , only a 32 add is needed,
	// because the table are allocated as a big block...
	bam_tbl_ptr_local+=BAM_STRING_SIZE;
	if(src & BIT1_MASK  ){ // 0b00000010
		*bam_tbl_ptr_local |= bit_mask;
	} else {
		*bam_tbl_ptr_local &= n_bit_mask;
	}
	bam_tbl_ptr_local+=BAM_STRING_SIZE;
	if(src & BIT2_MASK  ){ // 0b00000100
		*bam_tbl_ptr_local |= bit_mask;
	} else {
		*bam_tbl_ptr_local &= n_bit_mask;
	}
	bam_tbl_ptr_local+=BAM_STRING_SIZE;
	if(src & BIT3_MASK  ){ // 0b00001000
		*bam_tbl_ptr_local |= bit_mask;
	} else {
		*bam_tbl_ptr_local &= n_bit_mask;
	}
	bam_tbl_ptr_local+=BAM_STRING_SIZE;
	if(src & BIT4_MASK  ){ // 0b00010000
		*bam_tbl_ptr_local |= bit_mask;
	} else {
		*bam_tbl_ptr_local &= n_bit_mask;
	}
	bam_tbl_ptr_local+=BAM_STRING_SIZE;
	if(src & BIT5_MASK  ){ // 0b00100000
		*bam_tbl_ptr_local |= bit_mask;
	} else {
		*bam_tbl_ptr_local &= n_bit_mask;
	}	
	bam_tbl_ptr_local+=BAM_STRING_SIZE;
	if(src & BIT6_MASK  ){ // 0b01000000
		*bam_tbl_ptr_local |= bit_mask;
	} else {
		*bam_tbl_ptr_local &= n_bit_mask;
	}
	bam_tbl_ptr_local+=BAM_STRING_SIZE;
	if(src & BIT7_MASK  ){ // 0b10000000
		*bam_tbl_ptr_local |= bit_mask;
	} else {
		*bam_tbl_ptr_local &= n_bit_mask;
	}
} 

/** \brief switch the BAM/CALC-SRC-Pointer
 *
 * \details switch the bam_tbl_calc to bam_tbl_mem and vise versa
 */
void switch_bam_pointer(void){
	if(bam_tbl_mem == bam_tbl_mem_1){
		bam_tbl_mem = bam_tbl_mem_2;
		bam_tbl_proc = bam_tbl_mem_1;
	} else{
		bam_tbl_mem = bam_tbl_mem_1;
		bam_tbl_proc = bam_tbl_mem_2;
	}	
}

/** \brief Start BAM
 *
 * \details Starts the timer16
 */
void start_timer(void){
	TIMER_16_CTRL_B = TIMER_16_START_TIMER;	
}	

/** \brief Reset BAM
 *
 * \details Reset and stop the BAM
 */
void reset_BAM(void){
	TIMER_16_CTRL_B = TIMER_16_STOP_TIMER;
	bam_step=0;
	TIMER_16_CNTR_H = bam_timer_map_h[bam_step];
	TIMER_16_CNTR_L = bam_timer_map_l[bam_step];	
}
