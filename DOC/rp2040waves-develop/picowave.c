#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>
#include <hardware/pwm.h>
#include <hardware/dma.h>
#include <hardware/pio.h>
#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "picowave.h"
#include "picowave.pio.h"

static picowave_t pw = {pio1,0,0,0}; 

void picowave_pin_init(){

	//TODO
}

void serial_init(void){

	//TODO
}


// initialize wavform timer and dma engine 

void waveform_init(int freq){
	
	//grab timer 
	pw.dma_timer_id = dma_claim_unused_timer(true); // panics if none available.

	//TODO
	//dma_timer_set_fraction (uint timer, uint16_t numerator, uint16_t denominator)
	
	//grab dma channel 
	pw.dma_chan_id = dma_claim_unused_channel(true);		
	dma_channel_config cfg = dma_channel_get_default_config(pw.dma_chan_id);
	// sync dma reads to timer 
	channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    channel_config_set_dreq(&cfg, dma_get_timer_dreq(pw.dma_timer_id));
    //TODO dma_channel_configure(cap_dma_chan, &cfg, NULL, &cap_pio->rxf[cap_sm], 0, false);

}

void picowave_pio_init(void)
{
    pw.sm = pio_claim_unused_sm(pw.pio, true);
    uint offset = pio_add_program(pw.pio, &picowave_program);
    pio_sm_config c = picowave_program_get_default_config(offset);
    pio_sm_set_consecutive_pindirs(pw.pio, pw.sm, PIN_DOUT0, NUM_DOUT, false);
    sm_config_set_out_pins(&c, PIN_DOUT0,NUM_DOUT);
    sm_config_set_clkdiv(&c, 1);
    sm_config_set_in_shift(&c, false, true, 16);
    pio_sm_clear_fifos(pw.pio, pw.sm);
    pio_sm_init(pw.pio, pw.sm, offset, &c);
    pio_sm_set_enabled(pw.pio, pw.sm, true);
}



int main(void) 
{

	set_sys_clock_khz(PWM_CLOCK/1000, true);
    stdio_init_all();
    serial_init();
    picowave_pin_init();
    picowave_pio_init();
    //TODO: make this configurable
    waveform_init(60);

}