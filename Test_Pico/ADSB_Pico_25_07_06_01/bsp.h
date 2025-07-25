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
    //uint16_t gnss_pps_pin = 26;
    uint16_t gnss_enable_pin = UINT16_MAX;  // Set to UINT16_MAX to indicate not connected.

    uint16_t comms_uart_tx_pin = 4;
    uint16_t comms_uart_rx_pin = 5;

    //spi_inst_t* copro_spi_handle = spi1;
    //uint32_t copro_spi_clk_freq_hz = 20e6;  // 20 MHz (originally 40MHz, but turned down to work with flex adapter PCB).
    //uint16_t copro_spi_clk_pin = 10;
    //uint16_t copro_spi_mosi_pin = 11;
    //uint16_t copro_spi_miso_pin = 12;
    //gpio_drive_strength copro_spi_drive_strength = GPIO_DRIVE_STRENGTH_12MA;
    //bool copro_spi_pullup = false;
    //bool copro_spi_pulldown = true;

    //bool has_esp32 = true;
    //uint16_t esp32_enable_pin = 14;
    //uint16_t esp32_spi_cs_pin = 9;
    //uint16_t esp32_spi_handshake_pin = 13;
    //uart_inst_t* esp32_uart_handle = uart0;
    //uint16_t esp32_uart_tx_pin = 16;
    //uint16_t esp32_uart_rx_pin = 17;

    PIO preamble_detector_pio = pio0;
    uint preamble_detector_demod_pin_irq = IO_IRQ_BANK0;
    PIO message_demodulator_pio = pio1;
    uint preamble_detector_demod_complete_irq = PIO0_IRQ_0;

   // uint16_t r1090_led_pin = 15;
    uint16_t r1090_num_demod_state_machines = 3;
    uint16_t r1090_high_power_demod_state_machine_index = 2;
    uint16_t r1090_pulses_pins[kMaxNumDemodStateMachines] = {19, 22, 19};
    uint16_t r1090_demod_pins[kMaxNumDemodStateMachines] = {20, 23, 29};
    uint16_t r1090_recovered_clk_pins[kMaxNumDemodStateMachines] = {21, 24, 26}; // ���������� RECOVERED_CLK �� ��������� ����� ��� ��������� ��������� ������� ��������. ����� ��������������
                                                                                 // higher priority (lower index) SM.
    uint16_t r1090_tl_pwm_pin = 9;             // Pin for Trigger Level PWM output.
    uint16_t r1090_tl_adc_input = 1;           // ADC input for reading filtered Trigger Level.
    uint16_t r1090_tl_adc_pin = 27;            // Pin for reading filtered Trigger Level.
    uint16_t r1090_rssi_adc_input = 2;         // ADC input for reading RSSI.
    uint16_t r1090_rssi_adc_pin = 28;          // Pin for reading RSSI.param gpio The GPIO number to use. Allowable GPIO numbers are 26 to 29 inclusive on RP2040 
    uint16_t r1090_bias_tee_enable_pin = 18;
    i2c_inst_t* onboard_i2c = i2c1;            // I2C peripheral used to talk to EEPROM (if supported).
    uint16_t onboard_i2c_sda_pin = 2;          // SDA pin for I2C.
    uint16_t onboard_i2c_scl_pin = 3;          // SCL pin for I2C.
    uint32_t onboard_i2c_clk_freq_hz = 400e3;  // 400kHz
    bool onboard_i2c_requires_init = false;    // � ������, ���� I2c ������������ ��������� � ���-�� ������, ��� ��� �������������� ���.
};

extern BSP bsp;