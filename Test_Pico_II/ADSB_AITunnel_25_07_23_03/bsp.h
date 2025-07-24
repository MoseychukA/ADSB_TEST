#pragma once

#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "stdint.h"

class BSP 
{
   public:
    static const uint16_t kMaxNumDemodStateMachines = 4;



    uint16_t gnss_uart_tx_pin = 0;
    uint16_t gnss_uart_rx_pin = 1;
    uint16_t gnss_pps_pin = 26;
    uint16_t gnss_enable_pin = UINT16_MAX;  // Set to UINT16_MAX to indicate not connected.

    uint16_t comms_uart_tx_pin = 4;
    uint16_t comms_uart_rx_pin = 5;

    PIO preamble_detector_pio = pio0;
    uint preamble_detector_demod_pin_irq = IO_IRQ_BANK0;
    PIO message_demodulator_pio = pio1;
    uint preamble_detector_demod_complete_irq = PIO0_IRQ_0;

    uint16_t r1090_led_pin = 15;
    uint16_t r1090_num_demod_state_machines = 3;  //
    uint16_t r1090_high_power_demod_state_machine_index = 2;
    uint16_t r1090_pulses_pins[kMaxNumDemodStateMachines] = {19, 22, 19};
    uint16_t r1090_demod_pins[kMaxNumDemodStateMachines] = {20, 23, 29};
    uint16_t r1090_recovered_clk_pins[kMaxNumDemodStateMachines] = {21, 24, 26}; // Установить RECOVERED_CLK на фальшивый вывод для детектора преамбулы высокой мощности. Будет переопределено
                                                                                 // higher priority (lower index) SM.
    uint16_t r1090_tl_pwm_pin = 9;             // Pin for Trigger Level PWM output.
    uint16_t r1090_tl_adc_pin = 27;            // Pin for reading filtered Trigger Level.
    uint16_t r1090_tl_adc_input = 1;           // ADC input for reading filtered Trigger Level.
    uint16_t r1090_rssi_adc_pin = 28;          // Pin for reading RSSI.
    uint16_t r1090_rssi_adc_input = 2;         // ADC input for reading RSSI.
    uint16_t r1090_bias_tee_enable_pin = 18;
    i2c_inst_t* onboard_i2c = i2c1;            // I2C peripheral used to talk to EEPROM (if supported).
    uint16_t onboard_i2c_sda_pin = 2;          // SDA pin for I2C.
    uint16_t onboard_i2c_scl_pin = 3;          // SCL pin for I2C.
    uint32_t onboard_i2c_clk_freq_hz = 400e3;  // 400kHz
    bool onboard_i2c_requires_init = false;    // В случае, если I2c используется совместно с чем-то другим, что уже инициализирует его.
};

extern BSP bsp;