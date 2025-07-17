#pragma once

#include "hardware/pio.h"

class BSP 
{
   public:
    static const uint16_t kMaxNumDemodStateMachines = 4;
    uint16_t r1090_pulses_pins[kMaxNumDemodStateMachines] = {19, 22, 19};
    uint16_t r1090_demod_pins[kMaxNumDemodStateMachines] = {20, 23, 29};
    uint16_t r1090_recovered_clk_pins[kMaxNumDemodStateMachines] = {21, 24, 26}; // Установить RECOVERED_CLK на фальшивый вывод для детектора преамбулы высокой мощности. Будет переопределено
 
};

extern BSP bsp;