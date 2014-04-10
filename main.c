/** \brief		RGB LED - Matrix-Panel 8*8 "Kachel"
 * \file		main.c
 * \author  	Rene Reinsch
 * \date		10.11.2013
 * \version 	Rev. 3.1 22.2.2014
 *
 * \details	initilizes the system
 * 			\n runs the BAM with a clean picture
 * 		   	\n waiting for new picture data
 *
 * \todo	temperature control -> ADC + handle
 *     		\n watchdog
 *     		\n hardware revision 2 adjustment
 *      	\n-PWM Blank, fan GPIO, Blank polarity
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "bam.h"
#include "transceive_data.h"

/** \brief 	main
 *
 * \details    Initialize the system
 *  			\n run BAM
 *  			\n waiting for new picture data
 *
 * \todo      	temperature control -> ADC + handle
 *     			\n watchdog
 *     			\n hardware revision 2 adjustment
 *      		\n-PWM Blank, fan GPIO, Blank polarity
 */
int main(void)
{
	// wdt reset!!!
	init_SPI();
	init_PIN_CHANGE_ISR();
	init_BAM();
	sei();
	start_timer();
    while(1)
    {
		check_valid_rx_data();
		// temperature control ()!!!!
		// watchdog!!!!
	}
}
