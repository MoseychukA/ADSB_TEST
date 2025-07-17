#pragma once

#include "hardware/pio.h"

// ----------- //
// irq_wrapper //
// ----------- //

#define irq_wrapper_wrap_target 0
#define irq_wrapper_wrap 1

static const uint16_t irq_wrapper_program_instructions[] = {
            //     .wrap_target
    0x20c6, //     0: wait   1 irq, 6                   
    0xc004, //      1: irq    nowait 4                   
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program irq_wrapper_program = {
    .instructions = irq_wrapper_program_instructions,
    .length = 2,
    .origin = -1,
};

static inline pio_sm_config irq_wrapper_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + irq_wrapper_wrap_target, offset + irq_wrapper_wrap);
    return c;
}

// Helper function (for use in C program) to initialize this PIO program
void irq_wrapper_program_init(
    PIO pio,
    uint sm,
    uint offset,
    float div
) {
    // Sets up state machine and wrap target. This function is automatically
    // generated in preamble_detector.pio.h.
    pio_sm_config c = irq_wrapper_program_get_default_config(offset);
    // Set the clock divider for the state machine
    sm_config_set_clkdiv(&c, div);
    // Load configuration and jump to start of the program
    pio_sm_init(pio, sm, offset, &c);
}

#endif

// ----------------- //
// preamble_detector //
// ----------------- //

#define preamble_detector_wrap_target 0
#define preamble_detector_wrap 15

#define preamble_detector_offset_waiting_for_first_edge 0u
#define preamble_detector_offset_follow_irq 14u

static const uint16_t preamble_detector_program_instructions[] = {
            //     .wrap_target
    0xc006, //  0: irq    nowait 6                   
    0xa0e6, //  1: mov    osr, isr                   
    0x6072, //  2: out    null, 18                   
    0xa023, //  3: mov    x, null                    
    0x20a0, //  4: wait   1 pin, 0                   
    0xc015, //  5: irq    nowait 5 rel               
    0x00f0, //  6: jmp    !osre, 16                  
    0xe201, //  7: set    pins, 1                [2] 
    0x2ba0, //  8: wait   1 pin, 0               [11]
    0xe230, //  9: set    x, 16                  [2] 
    0x02c9, // 10: jmp    pin, 9                 [2] 
    0x024a, // 11: jmp    x--, 10                [2] 
    0xe000, // 12: set    pins, 0                    
    0xc030, // 13: irq    wait 0 rel                 
    0x20d4, // 14: wait   1 irq, 4 rel               
    0x2020, // 15: wait   0 pin, 0                   
            //     .wrap
    0xe023, // 16: set    x, 3                       
    0x6041, // 17: out    y, 1                       
    0x0078, // 18: jmp    !y, 24                     
    0xe523, // 19: set    x, 3                   [5] 
    0x00d6, // 20: jmp    pin, 22                    
    0x000e, // 21: jmp    14                         
    0x0054, // 22: jmp    x--, 20                    
    0x0506, // 23: jmp    6                      [5] 
    0xe225, // 24: set    x, 5                   [2] 
    0x00dc, // 25: jmp    pin, 28                    
    0x015a, // 26: jmp    x--, 26                [1] 
    0x0406, // 27: jmp    6                      [4] 
    0x0059, // 28: jmp    x--, 25                    
    0x000e, // 29: jmp    14                         
};

#if !PICO_NO_HARDWARE
static const struct pio_program preamble_detector_program = {
    .instructions = preamble_detector_program_instructions,
    .length = 30,
    .origin = -1,
};

static inline pio_sm_config preamble_detector_program_get_default_config(uint offset) 
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + preamble_detector_wrap_target, offset + preamble_detector_wrap);
    return c;
}

// Вспомогательная функция (для использования в программе на языке C) для инициализации этой программы PIO
void preamble_detector_program_init(
    PIO pio, 
    uint sm, 
    uint offset, 
    uint pulses_pin, 
    uint demod_pin, 
    float div, 
    bool waiting
) {
    // Устанавливает конечный автомат и обертывает цель. Эта функция автоматически
    // генерируется в preamble_detector.pio.h.
    pio_sm_config c = preamble_detector_program_get_default_config(offset);
    // Разрешить PIO считывать GPIO impulses_pin (как вход)
    pio_sm_set_consecutive_pindirs(pio, sm, pulses_pin, 1, false); // установить один вывод, начиная с pulses_pin, на IN
    // Разрешить PIO управлять GPIO demod_pin (как вывод для отладки)
    pio_gpio_init(pio, demod_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, demod_pin, 1, true); // устанавливаем один вывод, начиная с demod_pin, на OUT
    // Подключите вывод impulse_pin к выводу IN (управление с помощью инструкции 'in')
    sm_config_set_in_pins(&c, pulses_pin); // for WAIT
    sm_config_set_jmp_pin(&c, pulses_pin); // for JMP
   // Подключите demod_pin к SET и SIDESET
    sm_config_set_set_pins(&c, demod_pin, 1); // set one pin as SET output
    sm_config_set_sideset_pins(&c, demod_pin); 
    sm_config_set_out_shift(&c, false, false, 32); // OSR сдвигается влево, порог автозапуска установлен на 32 (используется для сравнения OSRE)
    sm_config_set_in_shift(&c, false, false, 32);  // ISR сдвигается влево, автопуш выключен
   // Устанавливаем делитель часов для конечного автомата
    sm_config_set_clkdiv(&c, div);
    // Загрузить конфигурацию и перейти к началу программы
    pio_sm_init(pio, sm, offset + (waiting ? preamble_detector_offset_follow_irq : 0), &c);
}

#endif

// ------------------- //
// message_demodulator //
// ------------------- //

#define message_demodulator_wrap_target 26
#define message_demodulator_wrap 30

#define message_demodulator_offset_initial_entry 16u
#define message_demodulator_offset_high_power_initial_entry 18u

static const uint16_t message_demodulator_program_instructions[] = {
    0xe722, //  0: set    x, 2                   [7] 
    0xa7eb, //  1: mov    osr, !null             [7] 
    0x607e, //  2: out    null, 30                   
    0xa542, //  3: nop                           [5] 
    0x18c6, //  4: jmp    pin, 6          side 1     
    0x0007, //  5: jmp    7                          
    0x6061, //  6: out    null, 1                    
    0x1044, //  7: jmp    x--, 4          side 0     
    0xa020, //  8: mov    x, pins                    
    0x20a1, //  9: wait   1 pin, 1                   
    0x00ed, // 10: jmp    !osre, 13                  
    0xe120, // 11: set    x, 0                   [1] 
    0x000e, // 12: jmp    14                         
    0xe021, // 13: set    x, 1                       
    0x007e, // 14: jmp    !y, 30                     
    0x0019, // 15: jmp    25                         
    0x36a1, // 16: wait   1 pin, 1        side 0 [6] 
    0x0617, // 17: jmp    23                     [6] 
    0x36aa, // 18: wait   1 pin, 10       side 0 [6] 
    0x0617, // 19: jmp    23                     [6] 
    0x2020, // 20: wait   0 pin, 0                   
    0xb82b, // 21: mov    x, !null        side 1     
    0x5021, // 22: in     x, 1            side 0     
    0xf841, // 23: set    y, 1            side 1     
    0x1000, // 24: jmp    0               side 0     
    0x0034, // 25: jmp    !x, 20                     
            //     .wrap_target
    0x20a0, // 26: wait   1 pin, 0                   
    0x5961, // 27: in     null, 1         side 1 [1] 
    0xf040, // 28: set    y, 0            side 0     
    0x0000, // 29: jmp    0                          
    0x0134, // 30: jmp    !x, 20                 [1] 
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program message_demodulator_program = {
    .instructions = message_demodulator_program_instructions,
    .length = 31,
    .origin = -1,
};

static inline pio_sm_config message_demodulator_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + message_demodulator_wrap_target, offset + message_demodulator_wrap);
    sm_config_set_sideset(&c, 2, true, false);
    return c;
}

static inline void message_demodulator_program_init(
    PIO pio, 
    uint sm, 
    uint offset, 
    uint pulses_pin, 
    uint demod_pin,
    uint recovered_clk_pin, 
    float div
)
{
    if (demod_pin == pulses_pin+1) 
    {
        // Стандартный детектор преамбулы.
        pio_sm_set_consecutive_pindirs(pio, sm, pulses_pin, 2, false); // in_pin_base is DEMOD input, in_pin_base + 1 is pulses
    } 
    else 
    {
        // Детектор преамбулы высокой мощности. Имеет другой индекс GPIO для вывода демодулятора.
        // pio_sm_set_pindirs_with_mask(pio, sm, 0b0, (0b1 << demod_pin) | (0b1 << pulses_pin)); // Установить выводы демодулятора и импульсов как входы (0).
    }
    // Разрешить PIO управлять recovery_clk_pin (как вывод для отладки).
    pio_gpio_init(pio, recovered_clk_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, recovered_clk_pin, 1, true);      // устанавливаем один вывод, начиная с out_pin, на OUT
    pio_sm_config c = message_demodulator_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pulses_pin);                                    // Контакты WAIT: используйте pulses_pin в качестве базы GPIO.
    sm_config_set_jmp_pin(&c, pulses_pin);                                    // JMP на основе выходного сигнала компаратора
    sm_config_set_out_shift(&c, false, false, 32);                            // OSR сдвигается влево, автовыбор отключен, порог автовыбора 32 бита
    sm_config_set_in_shift(&c, false, true, 32);                              // ISR сдвигается влево, автопроталкивание включено, порог автопроталкивания 32 бита
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    sm_config_set_clkdiv(&c, div);
    // Подключите out_pin к SET и SIDESET
    sm_config_set_set_pins(&c, recovered_clk_pin, 1);                         // устанавливаем один вывод как выход SET
    sm_config_set_sideset_pins(&c, recovered_clk_pin); 
    pio_sm_init(pio, sm, offset, &c);
}

#endif
