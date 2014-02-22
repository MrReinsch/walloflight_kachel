/*
 * main.c
 *
 * Created: 10.11.2013 16:47:20
 * Author: René Reinsch
 * MAIN
 * Rev. 3.1 22.2.2014
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "bam.h"
#include "transceive_data.h"


int main(void)
{
	init_SPI();
	init_PIN_CHANGE_ISR();
	init_BAM();
	sei();
	start_timer();
    while(1)
    {
		check_valid_rx_data();
		// Temperaturüberwachung()!!!!
	}
}
