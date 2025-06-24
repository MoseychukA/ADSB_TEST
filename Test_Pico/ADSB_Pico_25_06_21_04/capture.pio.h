//; PIO0
//; SM0: Детектор хорошо сформированной преамбулы № 1
//; SM1: Детектор хорошо сформированной преамбулы № 2
//; SM2: Детектор высокой мощности преамбулы
//; SM3: Оболочка IRQ для детекторов хорошо сформированной преамбулы (оболочки между SM0 и SM1)
//
//; PIO1
//; SM0: Демодулятор для детектора хорошо сформированной преамбулы № 1
//; SM1: Демодулятор для детектора хорошо сформированной преамбулы № 2
//; SM2: Демодулятор для детектора высокой мощности преамбулы
//
//.program irq_wrapper
//
//; Эта программа работает на частоте 48 МГц. Каждый тактовый цикл составляет 1/48us.
//; Единственная цель этой программы — перенести IRQ обратно в начало массива конечных автоматов.
//
//.wrap_target
//wait 1 irq 6 ; Ожидает IRQ 6, затем очищает его. Устанавливает этот номер IRQ равным 4+num_state_machines.
//irq set 4 ; Устанавливает IRQ 4 для срабатывания детектора преамбулы на SM 0 (циклически).
//.wrap

//% c-sdk {

// Вспомогательная функция (для использования в программе на языке C) для инициализации этой программы PIO
void irq_wrapper_program_init(PIO pio, uint sm, uint offset, float div)
{
    // Устанавливает конечный автомат и обертывает цель. Эта функция автоматически
    // генерируется в preamble_detector.pio.h.
   //!! pio_sm_config c = irq_wrapper_program_get_default_config(offset);
    
 // Устанавливаем делитель часов для конечного автомата
   //!! sm_config_set_clkdiv(&c, div);

    // Load configuration and jump to start of the program
   //!! pio_sm_init(pio, sm, offset, &c);
}
//%}

//.program preamble_detector
//
//; Эта программа работает на частоте 48 МГц. Каждый тактовый цикл составляет 1/48us.
 
//.define pulses_pin_index 0
//.define demod_pin_index 1
 
//; Построить шаблон преамбулы и загрузить его в ISR: 0b101000010100000.
//; Обратите внимание, что один нулевой бит был удален из преамбулы, чтобы обеспечить больше циклов для включения демодулятора.
//; установить контакты 0 ; ОТЛАДКА
 
//; НАЧАТЬ ДВОЙНОЙ ИМПУЛЬСНЫЙ МАТЧ
//.wrap_target
//public waiting_for_first_edge:
//    irq set 6                       ; -4 | Spam resetting the high power preamble detector.
//    mov osr isr                     ; -3 | OSR = 0b00000000000000000010100001010000
//    out null 18                     ; -2 | OSR = 0b10100001010000
//    mov x null                      ; -1 | Clear out x from last sampling adventure.
//;Меняйте сверло каждые 24 цикла.
//;   6 cycles: Get bit.
//;   12 cycles: Assert bit.
//;   6 cycles: nop.
//    wait 1 pin, pulses_pin_index    ; 0 | ждать, пока in_pin перейдет в режим HI
//    irq set 5 rel ; 1 | установить IRQ 5+SM, чтобы указать другому детектору преамбулы, что он может начать поиск сейчас
//check_next_bit:
//    jmp !osre preamble_not_done_yet ; 2 
//; .wrap_target
//preamble_matched:
//    set pins 1 [2] ;3:5 | установить демодуляционный вывод для указания начала тела сообщения
//    wait 1 pin, pulses_pin_index [11] ;6:17 | дождитесь середины сообщения, чтобы увидеть конец
//waiting_for_end_of_message:
//    set x, 16 [2] ;18:20 | [2.0us] количество последовательных выборок бездействия, необходимых для обозначения конца пакета
//idle_countdown:
//    jmp pin waiting_for_end_of_message [2] ; 21+2n:24+2n | начать заново обратный отсчет простоя, если получен не пустой бит
//    jmp x-- idle_countdown [2] ; 25+2n:28+2n | все еще бездействует, продолжайте отсчет, если таймер не вышел
//message_finished:
//    set pins, 0 ; 29+2n | установить вывод демодулятора на 0, чтобы указать, что сообщение завершено
//    ; irq set 0 [1] ;30+2n:31+2n | установить DEMOD IRQ, чтобы указать, что тело сообщения завершено
//    irq wait 0 rel ; 30+2n | Установить IRQ 0+SM и дождаться его очистки (не ищите новые сообщения, пока текущее не будет обработано).
//public follow_irq:
//    wait 1 irq 4 rel ;Дождитесь IRQ 4+SM, затем очистите его.
//    wait 0 pin, pulses_pin_index ;Подождите, пока линия не перейдет в режим LO, чтобы не дублировать уже работающий детектор.
//
//.wrap
//
//preamble_not_done_yet:
//    set x 3                         ; 3: (filter_num_samples-1) Assert HI for 12 cycles ((x+1) * (3 cycles per wrap))
//    out y, 1                        ; 4: Find next bit polarity from OSR.
//    jmp !y filter_pin_lo            ; 5:
//
//; Subroutines
//
//assert_pin_hi:
//    set x 3 [5] ; 6-11: Assert HI for 12 cycles ((x+1) * (2 cycles per wrap))
//aph_wrap:
//    jmp pin aph_pin_is_hi
//    jmp follow_irq ; Pin is LO, assert failed.
//aph_pin_is_hi:
//    jmp x-- aph_wrap
//    jmp check_next_bit [5]    ; 20-25: cycle idle at end
//
//; filter_pin_hi:
//;     set y 3  ; 6: (filter_correct_samples-1) Minimum number of correct samples needed to keep looking.
//; fph_wrap_target:
//;     jmp pin fph_pin_hi  ; 7+3n
//; fph_pin_lo:
//;     jmp fph_wrap    ; 8+3n
//; fph_pin_hi:
//;     jmp y-- fph_wrap    ; 8+3n
//; fph_exit_success:
//;     jmp x-- fph_exit_success [2] ; Complete remaining loops.
//;     jmp check_next_bit [3] ; 22-25
//; fph_wrap:
//;     jmp x-- fph_wrap_target ; 9+3n
//;     jmp waiting_for_first_edge  ; Correlation failed.
//    
//filter_pin_lo:
// ; Щедрая реализация, которая принимает один образец LO в любом месте окна выборки как успех.
// ; Это объясняет асимметричную природу схемы среза данных, поскольку она заряжается во время преамбулы
// ; получить образец LO из входящего сообщения гораздо сложнее, чем образец HI.
//    set x 5 [2] ; 6-8: Number of samples = x+1.
//fpl_wrap_target:
//    jmp pin fpl_pin_hi ; 9+2n
//fpl_pin_lo:
//    jmp x-- fpl_pin_lo [1] ; 10+2n,11+2n: Complete remaining loops
//    jmp check_next_bit [4] ; 20-25
//fpl_pin_hi:
//    jmp x-- fpl_wrap_target ; 8+2n
//    jmp follow_irq ; Never found a LO sample, correlation failed.
//
//    ; Single sample implementaiton that samples once in the middle of the interval.
//;     nop [6] ; 6-12
//;     jmp pin fpl_fail ; 13
//; fpl_success:
//;     jmp check_next_bit [11] ; 14-25
//; fpl_fail:
//;     jmp check_next_bit [11] ; 14-25
//
//    ; Correlation implementation that enforces no more than y+1 samples must be incorrect (HI).
//;     set y 1  ; 6: (filter_num_samples - filter_correct_samples - 1) Minimum number of INCORRECT samples needed to abort.
//; fpl_wrap_target:
//;     jmp pin fpl_pin_hi ; 7+3n
//; fpl_pin_lo:
//;     jmp fpl_wrap ; 8+3n
//; fpl_pin_hi:
//;     jmp y-- fpl_wrap ; 8+3n
//;     jmp waiting_for_first_edge
//; fpl_wrap:
//;     jmp x-- fpl_wrap_target ; 9+3n
//;     ; jmp !y waiting_for_first_edge ; 22: Reset if correlation failed.
//;     jmp check_next_bit [3] ; 22-25
//
//
//
//% c-sdk {

// Helper function (for use in C program) to initialize this PIO program
void preamble_detector_program_init(PIO pio, uint sm, uint offset, uint pulses_pin, uint demod_pin, float div, bool waiting) 
{

    //// Sets up state machine and wrap target. This function is automatically
    //// generated in preamble_detector.pio.h.
    //!!pio_sm_config c = preamble_detector_program_get_default_config(offset);

    //// Allow PIO to read GPIO pulses_pin (as input)
    //pio_sm_set_consecutive_pindirs(pio, sm, pulses_pin, 1, false); // set one pin starting at pulses_pin to IN

    //// Allow PIO to control GPIO demod_pin (as output for debugging)
    //pio_gpio_init(pio, demod_pin);
    //pio_sm_set_consecutive_pindirs(pio, sm, demod_pin, 1, true); // set one pin starting at demod_pin to OUT

    //// Connect pulses_pin to IN pin (control with 'in' instruction)
    //sm_config_set_in_pins(&c, pulses_pin); // for WAIT
    //sm_config_set_jmp_pin(&c, pulses_pin); // for JMP

    //// Connect demod_pin to SET and SIDESET
    //sm_config_set_set_pins(&c, demod_pin, 1); // set one pin as SET output
    //sm_config_set_sideset_pins(&c, demod_pin); 

    //sm_config_set_out_shift(&c, false, false, 32); // OSR shifts left, autopull threshold set to 32 (used for OSRE compare)
    //sm_config_set_in_shift(&c, false, false, 32); // ISR shifts left, autopush turned off
    //
    //// Set the clock divider for the state machine
    //sm_config_set_clkdiv(&c, div);

    //// Load configuration and jump to start of the program
    //pio_sm_init(pio, sm, offset + (waiting ? preamble_detector_offset_follow_irq : 0), &c);
}

//%}

//.program message_demodulator
//; Demod pin for knowing when a valid message demod interval has begun.
//.define demod_in_pin_index 1
//; Demod pin for high power preamble detector. Using GPIO 29, pulses pin on GPIO 19 is base.
//.define high_power_demod_in_pin_index 10
//; Pulses input pin for reading current power level.
//.define pulses_pin_index 0
//; Allow side-setting the out_pin for debugging. 1x output pin can be optionally side-set.
//.side_set 1 opt
//; Assumes line is idle low, first bit is 0
//; One bit is 16 cycles
//; a '0' is encoded as 01
//; a '1' is encoded as 10
//;
//; Both the IN base and the JMP pin mapping must be pointed at the GPIO used for RX.
//; Autopush must be enabled.
//; Before enabling the SM, it should be placed in a 'wait 1, pin` state, so that
//; it will not start sampling until the initial line idle state ends.
//
//
//; SAMPLE SUBROUTINE
//; Samples 3x and uses a majority vote to determine whether it's HI or LO.
//; @param[in] y Goes back to end_of_0 if y = 0, goes back to end_of_1 otherwise.
//; @retval x x=1 if >=2/3 samples were LO, x=1 otherwise.
//sample:
//    set x 2 [7]         ; -15:-7 | (x+1) = number of samples to take (added delay 7)
//    mov osr !null [7]   ; -7:0 | OSR = 0b1111111111111111111111 (added delay 7)
//    out null 30         ; 1 | OSR = 0b11 (3 bits required to classify sampling period as HI)
//    nop [5]             ; 2:7 | This delay controls max allowable clock skew per bit.
//; One sample takes 3 cycles.
//sample_wrap_target:
//    jmp pin sample_hi side 1 ; 8+3n
//sample_lo:
//    jmp sample_wrap ; 9+3n
//sample_hi:
//    out null 1 ; 9+3n
//sample_wrap:
//    jmp x-- sample_wrap_target side 0 ; 10+3n
//    mov x, pins ; 17 | read pulses and demod bit into x
//    wait 1 pin demod_in_pin_index ; 18 | bail into idle state if demod bit is 0
//    jmp !osre sample_net_lo ; 19 | used to have no delay
//
//sample_net_hi:
//    set x 0 [1] ; 20-21
//    jmp sample_return ; 21
//sample_net_lo:
//    set x 1 ; 20 |
//    
//sample_return:
//    ; nop [2] ; 20-21 | Previously jumped to complete_demod here but it's not necessary until we interleave multiple SMs.
//    
//    jmp !y end_of_0 ; 22 | End of 0
//    jmp end_of_1  ; 23 | End of 1
//
//public initial_entry:
//    wait 1 pin demod_in_pin_index [6] side 0   ; -16 | Wait to enter DEMOD interval.
//    ; Preamble ends LO, so imitate the end of a 1 with its HI->LO transition.
//    jmp initial_entry_cut_in [6]            ; -15:-14 | wait adjusted here
//
//public high_power_initial_entry:
//    wait 1 pin high_power_demod_in_pin_index [6] side 0
//    jmp initial_entry_cut_in [6]
//
//start_of_1:
//    wait 0 pin pulses_pin_index ; 0 | Wait for the 1->0 transition - at this point we are 0.5 into the bit.
//falling_edge_1:
//    mov x !null side 1          ; 1
//    in x, 1 side 0              ; 2 | Emit a 1.
//initial_entry_cut_in:
//    set y 1 side 1              ; 3
//    jmp sample side 0           ; 4
//end_of_1:
//    jmp !x start_of_1           ; 24 | If signal is 1 again, it's another 1 bit, otherwise it's a 0.
//
//.wrap_target
//start_of_0:                     ; We are 0.25 bits into a 0 - signal is LO
//    wait 1 pin pulses_pin_index ; 0 | Wait for the 0->1 transition - at this point we are 0.5 into the bit.
//rising_edge_0:
//    in null, 1 [1] side 1       ; 1:2 | Emit a 0, sleep 3/4 of a bit
//
//    set y 0 side 0              ; 3
//    jmp sample                  ; 4
//end_of_0:
//    jmp !x start_of_1 [1]       ; 23:24 | If signal is 0 again, it's another 0 bit otherwise it's a 1
//.wrap
//
//% c-sdk {
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
        // Standard preamble detector.
        pio_sm_set_consecutive_pindirs(pio, sm, pulses_pin, 2, false); // in_pin_base is DEMOD input, in_pin_base + 1 is pulses
    } else {
        // High power preamble detector. Has a different GPIO index for the demod pin.
        // pio_sm_set_pindirs_with_mask(pio, sm, 0b0, (0b1 << demod_pin) | (0b1 << pulses_pin)); // Set demod and pulses pins as inputs (0).
    }

    // Allow PIO to control recovered_clk_pin (as output for debugging).
    pio_gpio_init(pio, recovered_clk_pin);
    pio_sm_set_consecutive_pindirs(pio, sm, recovered_clk_pin, 1, true); // set one pin starting at out_pin to OUT

    //pio_sm_config c = message_demodulator_program_get_default_config(offset);
    //sm_config_set_in_pins(&c, pulses_pin); // WAIT pins: Utilize pulses_pin as GPIO base.
    //sm_config_set_jmp_pin(&c, pulses_pin); // JMP based on the comparator output
    //sm_config_set_out_shift(&c, false, false, 32); // OSR shifts left, autopull turned off, autopull threshold 32 bits
    //sm_config_set_in_shift(&c, false, true, 32); // ISR shifts left, autopush turned on, autopush threshold 32 bits
    //sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    //sm_config_set_clkdiv(&c, div);

    //// Connect out_pin to SET and SIDESET
    //sm_config_set_set_pins(&c, recovered_clk_pin, 1); // set one pin as SET output
    //sm_config_set_sideset_pins(&c, recovered_clk_pin); 

    //pio_sm_init(pio, sm, offset, &c);
}
//%}