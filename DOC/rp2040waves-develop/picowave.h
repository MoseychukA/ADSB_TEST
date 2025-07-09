/* 
 definitions for wave form generation 
 */

//system def
#define PWM_CLOCK       120000000

//Pin def. 12 bit defined for now. 
#define PIN_DOUT0 0
#define NUM_DOUT 12 
#define PIN_CLKOUT 22 

//struct 
typedef struct picowave_s {
	PIO pio;
	uint8_t sm;
	uint8_t dma_chan_id;
	uint8_t dma_timer_id;
} picowave_t;



/*
  function definitions 
 */

void picowave_pin_init();

void picowave_pio_init();