// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ------------- //
// manchester_tx //
// ------------- //

#define manchester_tx_wrap_target 0
#define manchester_tx_wrap 5

#define manchester_tx_offset_start 4u

static const uint16_t manchester_tx_program_instructions[] = {
            //     .wrap_target
    0xb542, //  0: nop                    side 0 [5] 
    0x1b04, //  1: jmp    4               side 1 [3] 
    0xbd42, //  2: nop                    side 1 [5] 
    0xb342, //  3: nop                    side 0 [3] 
    0x6021, //  4: out    x, 1                       
    0x0022, //  5: jmp    !x, 2                      
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program manchester_tx_program = {
    .instructions = manchester_tx_program_instructions,
    .length = 6,
    .origin = -1,
};

static inline pio_sm_config manchester_tx_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + manchester_tx_wrap_target, offset + manchester_tx_wrap);
    sm_config_set_sideset(&c, 2, true, false);
    return c;
}

static inline void manchester_tx_program_init(PIO pio, uint sm, uint offset, uint pin, float div) {
    pio_sm_set_pins_with_mask(pio, sm, 0, 1u << pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_gpio_init(pio, pin);
    pio_sm_config c = manchester_tx_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset + manchester_tx_offset_start, &c);
    pio_sm_set_enabled(pio, sm, true);
}

#endif

// ------------- //
// manchester_rx //
// ------------- //

#define manchester_rx_wrap_target 3
#define manchester_rx_wrap 5

static const uint16_t manchester_rx_program_instructions[] = {
    0x2020, //  0: wait   0 pin, 0                   
    0x4841, //  1: in     y, 1                   [8] 
    0x00c0, //  2: jmp    pin, 0                     
            //     .wrap_target
    0x20a0, //  3: wait   1 pin, 0                   
    0x4821, //  4: in     x, 1                   [8] 
    0x00c0, //  5: jmp    pin, 0                     
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program manchester_rx_program = {
    .instructions = manchester_rx_program_instructions,
    .length = 6,
    .origin = -1,
};

static inline pio_sm_config manchester_rx_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + manchester_rx_wrap_target, offset + manchester_rx_wrap);
    return c;
}

static inline void manchester_rx_program_init(PIO pio, uint sm, uint offset, uint pin, float div) {
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, false);
    pio_gpio_init(pio, pin);
    pio_sm_config c = manchester_rx_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin); // for WAIT
    sm_config_set_jmp_pin(&c, pin); // for JMP
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c);
    // X and Y are set to 0 and 1, to conveniently emit these to ISR/FIFO.
    pio_sm_exec(pio, sm, pio_encode_set(pio_x, 1));
    pio_sm_exec(pio, sm, pio_encode_set(pio_y, 0));
    // Assume line is idle low, and first transmitted bit is 0. Put SM in a
    // wait state before enabling. RX will begin once the first 0 symbol is
    // detected.
    pio_sm_exec(pio, sm, pio_encode_wait_pin(1, 0) | pio_encode_delay(2));
    pio_sm_set_enabled(pio, sm, true);
}

#endif
