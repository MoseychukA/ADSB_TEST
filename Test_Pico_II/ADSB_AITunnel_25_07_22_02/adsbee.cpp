#include "adsbee.h"

#include "Arduino.h"
#include <hardware/structs/systick.h>

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/exception.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/rand.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "stdio.h"  // for printing
#include "capture_pio.h"
#include "hardware/irq.h"
//#include "packet_decoder.h"
#include "pico/binary_info.h"

#include <string.h>  // for strcat
//#include "comms.h"   // For debug prints. 

#define MLAT_SYSTEM_CLOCK_RATIO 48 / 125
// Scales 125MHz system clock into a 48MHz counter.
static const uint32_t kMLATWrapCounterIncrement = (1 << 24) * MLAT_SYSTEM_CLOCK_RATIO;

uint16_t r1090_num_demod_state_machines = 3;
uint16_t r1090_high_power_demod_state_machine_index = 2;

constexpr float kPreambleDetectorFreq = 48e6;    // Running at 48MHz (24 clock cycles per half bit).
constexpr float kMessageDemodulatorFreq = 48e6;  // Run at 48 MHz to demodulate bits at 1Mbps.

constexpr float kInt16MaxRecip = 1.0f / INT16_MAX;

ADSBee* isr_access = nullptr;

/** ������ �������� ������� ��� ���������� ������� **/
void on_systick_exception() { isr_access->OnSysTickWrap(); }

void on_demod_pin_change(uint gpio, uint32_t event_mask)
{
    switch (event_mask)
    {
    case GPIO_IRQ_EDGE_RISE:
        isr_access->OnDemodBegin(gpio);
        break;
    case GPIO_IRQ_EDGE_FALL:
        break;
    case GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL:
        break;
    }
    gpio_acknowledge_irq(gpio, event_mask);
}

void on_demod_complete() { isr_access->OnDemodComplete(); }

/** End pass-through functions for public access **/

ADSBee::ADSBee(ADSBeeConfig config_in)
{
    config_ = config_in;

    for (uint16_t sm_index = 0; sm_index < /*bsp.*/r1090_num_demod_state_machines; sm_index++)
    {
        preamble_detector_sm_[sm_index] = pio_claim_unused_sm(config_.preamble_detector_pio, true);
        message_demodulator_sm_[sm_index] = pio_claim_unused_sm(config_.message_demodulator_pio, true);
    }
    irq_wrapper_sm_ = pio_claim_unused_sm(config_.preamble_detector_pio, true);

    preamble_detector_offset_ = pio_add_program(config_.preamble_detector_pio, &preamble_detector_program);
    irq_wrapper_offset_ = pio_add_program(config_.preamble_detector_pio, &irq_wrapper_program);
    message_demodulator_offset_ = pio_add_program(config_.message_demodulator_pio, &message_demodulator_program);

    // �������� ��������� IRQ � ���������� ������� �������� ��� ISR on_demod_complete.
    isr_access = this;

    // ���������� �������� ����� � ������, ������� ����� �������������� ��� ��������� �������� ����� ���.
    tl_pwm_slice_ = pwm_gpio_to_slice_num(config_.tl_pwm_pin);  // ����� ����� ���, ������� ��������� ��������� GPIO.
    tl_pwm_chan_ = pwm_gpio_to_channel(config_.tl_pwm_pin);     // ����� ���, ������� ��������� ��������� GPIO. 
}

bool ADSBee::Init()
{
    gpio_init(config_.r1090_led_pin);
    gpio_set_dir(config_.r1090_led_pin, GPIO_OUT);
    gpio_put(config_.r1090_led_pin, 0);

    //// Initialize the TL bias PWM output.// �������������� ����� �������� TL ���.
    //gpio_set_function(config_.tl_pwm_pin, GPIO_FUNC_PWM);  //pin = 25;
    //pwm_set_wrap(tl_pwm_slice_, kTLMaxPWMCount);           // \param wrap �������� ��� ��������� �������� (5000)

    //SetTLMilliVolts(SettingsManager::Settings::kDefaultTLMV); // ��������� ��������� ���
    //pwm_set_enabled(tl_pwm_slice_, true);                     //

    //// �������������� ���� �������� ������ �������� ���.
    //adc_init();
    //adc_gpio_init(config_.tl_adc_pin);
    //adc_gpio_init(config_.rssi_adc_pin);

    //===================================================================================

    // �������� ������ MLAT � ������� 24-������� ������� SysTick, ������������� � �������� ������� ���������� 125 ���.
    // ������� ���������� � ��������� SysTick
    systick_hw->csr = 0b110;  // �������� = ������� ����������, TickInt = ��������, ������� = ���������.
    // ������� ������������ �������� SysTick
    systick_hw->rvr = 0xFFFFFF;  // ����������� ������ 24-������ �������� �������� �������� �������.
    // 0xFFFFFF = 16777215 �������� @ 125 ��� = �������� 0,134 �������.

    // ������� ������� OnSysTickWrap ������ ���, ����� ������ SysTick ��������� 0.
   //  exception_set_exclusive_handler(SYSTICK_EXCEPTION, on_systick_exception);  //�������� ��� �����
    // 
    systick_hw->csr |= 0b1;  // �������� �������.

    /** PREAMBLE DETECTOR PIO **/
    // Calculate the PIO clock divider. // ���������� �������� �������� ������� PIO.
    float preamble_detector_div = (float)clock_get_hz(clk_sys) / kPreambleDetectorFreq;
    irq_wrapper_program_init(config_.preamble_detector_pio, /*bsp.*/r1090_num_demod_state_machines, irq_wrapper_offset_, preamble_detector_div);


    for (uint16_t sm_index = 0; sm_index < /*bsp.*/r1090_num_demod_state_machines; sm_index++)
    {
        // ��������� �������� ������� ����� ������� ������ � ��� ������, ���� �� ������ � ����������� ������ ��������� �������������� �������� ����������.
        bool make_sm_wait = sm_index > 0 && sm_index < /*bsp.*/r1090_high_power_demod_state_machine_index;
        // �������������� ��������� � ������� ��������������� ������� ����� .pio
        preamble_detector_program_init(config_.preamble_detector_pio,                     // Use PIO block 0.
            preamble_detector_sm_[sm_index],                   // State machines 0-2
            preamble_detector_offset_ /* + starting_offset*/,  // Program startin offset.
            config_.pulses_pins[sm_index],                     // Pulses pin (input).
            config_.demod_pins[sm_index],                      // Demod pin (output).
            preamble_detector_div,                             // Clock divisor (for 48MHz).
            make_sm_wait  // Whether state machine should wait for an IRQ to begin.
        );

        // ��������� ���������� GPIO (��� ���������� ������ ��������� �����������).
        gpio_set_irq_enabled_with_callback(config_.demod_pins[sm_index], GPIO_IRQ_EDGE_RISE /* | GPIO_IRQ_EDGE_FALL */,
            true, on_demod_pin_change);

        // ���������� ������������������ ��������� � ISR: ISR: 0b101000010100000(0)
        // ��������� 0 ������ �� ������������������ ���������, ����� ���� ������������ ������ ������� ��� �������.
        // mov isr null ; �������� ISR.
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_mov(pio_isr, pio_null));
        // ��������� ������ ������� ��������� ������� ������, ���� �������� ������� ������������ ��� ����������� ������� �������� ���������.
        if (sm_index == /*bsp.*/r1090_high_power_demod_state_machine_index)
        {
            // ��������� ������� ��������.
            // set x 0b111  ; ISR = 0b00000000000000000000000000000000
            pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_set(pio_x, 0b111));
            // in x 3       ; ISR = 0b00000000000000000000000000000111
            pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_in(pio_x, 3));
            // set x 0b101  ; ISR = 0b00000000000000000000000000000000
            pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_set(pio_x, 0b101));
        }
        else
        {
            // ������ ������������ ���������.
            // set x 0b101  ; ISR = 0b00000000000000000000000000000000
            pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_set(pio_x, 0b101));
            // in x 3       ; ISR = 0b00000000000000000000000000000101
            pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_in(pio_x, 3));
        }
        // in null 4    ; ISR = 0b00000000000000000000000001?10000
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_in(pio_null, 4));
        // in x 3       ; ISR = 0b00000000000000000000001?10000101
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_in(pio_x, 3));
        // in null 4    ; ISR = 0b0000000000000000001?100001010000
       // ����������: ��� ������ ���������� ������, �� ��� ����� �������������� ����� ��� ������� ������������.
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_in(pio_null, 4));
        // mov x null   ; Clear scratch x.
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_mov(pio_x, pio_null));
        // ����������� ��� ���������� ��� �������� ������������ ������������ ��������� (���������� ISR � RX FIFO).
        // pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_push(false, true));
    }

    // �������� ���������� DEMOD �� PIO1_IRQ_0.
    pio_set_irq0_source_enabled(config_.preamble_detector_pio, pis_interrupt0, true);  // PIO0 state machine 0
    pio_set_irq0_source_enabled(config_.preamble_detector_pio, pis_interrupt1, true);  // PIO0 state machine 1
    pio_set_irq0_source_enabled(config_.preamble_detector_pio, pis_interrupt2, true);  // PIO0 state machine 2

  // ��������� PIO0 IRQ0.
    irq_set_exclusive_handler(config_.preamble_detector_demod_complete_irq, on_demod_complete);
    irq_set_enabled(config_.preamble_detector_demod_complete_irq, true);

    /** MESSAGE DEMODULATOR PIO **/
    float message_demodulator_div = (float)clock_get_hz(clk_sys) / kMessageDemodulatorFreq;
    for (uint16_t sm_index = 0; sm_index < /*bsp.*/r1090_num_demod_state_machines; sm_index++)
    {
        message_demodulator_program_init(config_.message_demodulator_pio, message_demodulator_sm_[sm_index],
            message_demodulator_offset_, config_.pulses_pins[sm_index],
            config_.demod_pins[sm_index], config_.recovered_clk_pins[sm_index],
            message_demodulator_div);
    }

    // ���������� ��������� ���������� GPIO ����, ��� � ���������� DEMOD, ����� ��������� ��������� RSSI.
    // irq_set_priority(config_.preamble_detector_demod_complete_irq, 1);
    irq_set_priority(config_.preamble_detector_demod_pin_irq, 0);

    // ������������� ����� ������� ���������� ���������� �������.
    last_aircraft_dictionary_update_timestamp_ms_ = millis();//get_time_since_boot_ms();

    // �������� �������� ��������.
    pio_sm_set_enabled(config_.preamble_detector_pio, irq_wrapper_sm_, true);
    // ������� ����� �������� SM ������������, ��� ��� ���� �������� ��������� ����������� IRQ, �� ����������� ��
    // �������, �� �������� � ����� (� �����, ���, ��������, ������� ��������� ��� ���).
    for (uint16_t sm_index = 0; sm_index < /*bsp.*/r1090_num_demod_state_machines; sm_index++)
    {
        // pio_sm_set_enabled(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], true);
        pio_sm_set_enabled(config_.message_demodulator_pio, message_demodulator_sm_[sm_index], true);
    }
    // �������� ����������� ������� ������ �������������� ���������� ��������.
    // ����������: �� ���������� ��������, ����� ��������� ������ ��������� ��������� ������� ��������, ��������� ��� ���������� IRQ, �� ������� ���������
    // �������� ��������� ������� ��������. ��� ��������� ���� �����, ��� �������� ��������� ������� �������� ����������
    // ��� �� ��� PIO, ������� ��������� ����������� ������� ��� ���������� ������ �������������� ���������.
    for (uint16_t sm_index = 0; sm_index < /*bsp.*/r1090_high_power_demod_state_machine_index; sm_index++)
    {
        pio_sm_set_enabled(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], true);
    }
    // �������� �������� ��������� ������� ��������.
    pio_sm_set_enabled(config_.preamble_detector_pio, preamble_detector_sm_[/*bsp.*/r1090_high_power_demod_state_machine_index], true);

    //  DisableWatchdog();
    return true;
}

void ADSBee::FlashStatusLED(uint32_t led_on_ms)
{
    SetStatusLED(true);
    led_on_timestamp_ms_ = millis();
}

uint64_t ADSBee::GetMLAT48MHzCounts(uint16_t num_bits)
{
    // ���������� ������� ��������� � ������� ��������� �������� SysTick � ������������� �� 48 ���.
    // ����������: 24-������ �������� SysTick ���������� �� UINT_24_MAX, ����� ��� ������� �����, � �� ����.
    return (mlat_counter_wraps_ + ((0xFFFFFF - systick_hw->cvr) * MLAT_SYSTEM_CLOCK_RATIO)) & (UINT64_MAX >> (64 - num_bits));
}

uint64_t ADSBee::GetMLAT12MHzCounts(uint16_t num_bits)
{
    // ������������� ������� ������� � ����� ������� ����������� 48 ���.
    return GetMLAT48MHzCounts(50) >> 2;  // Divide 48MHz counter by 4, widen the mask by 2 bits to compensate.
}

int ADSBee::GetNoiseFloordBm() { return AD8313MilliVoltsTodBm(noise_floor_mv_); }

uint16_t ADSBee::GetTLLearningTemperatureMV() { return tl_learning_temperature_mv_; }

void ADSBee::OnDemodBegin(uint gpio)
{
    // �������� ������� MLAT � ������, ����� ��������� ������� ����� ����������.
    uint64_t mlat_48mhz_64bit_counts = GetMLAT48MHzCounts();
    uint16_t sm_index;
    for (sm_index = 0; sm_index < /*bsp.*/r1090_num_demod_state_machines; sm_index++)
    {
        if (config_.demod_pins[sm_index] == gpio)
        {
            break;
        }
    }
    if (sm_index >= /*bsp.*/r1090_num_demod_state_machines)
        return;  // ������������; �� ���� ������� ��������� ����������� ��� ���������� SM.
   // ������ ����������� ����������! ��������� ������� MLAT.
    rx_packet_[sm_index].mlat_48mhz_64bit_counts = mlat_48mhz_64bit_counts;
}

void ADSBee::OnDemodComplete()
{
   // Serial2.printf("ADSBee::OnDemodComplete. \r\n");
    for (uint16_t sm_index = 0; sm_index < /*bsp.*/r1090_num_demod_state_machines; sm_index++)
    {
        if (!pio_interrupt_get(config_.preamble_detector_pio, sm_index))
        {
            continue;
        }
        pio_sm_set_enabled(config_.message_demodulator_pio, message_demodulator_sm_[sm_index], false);
        // ��������� ������� RSSI �������� ������.
        rx_packet_[sm_index].sigs_dbm = ReadSignalStrengthdBm();
        rx_packet_[sm_index].sigq_db = rx_packet_[sm_index].sigs_dbm - GetNoiseFloordBm();
        rx_packet_[sm_index].source = sm_index;  // ���������� ���� �������� ������� ��� �������� ������.
        //!!Serial2.printf("ADSBee::source. \t %d \r\n", rx_packet_[sm_index].source);

        if (!pio_sm_is_rx_fifo_full(config_.message_demodulator_pio, message_demodulator_sm_[sm_index]))
        {
            // �������� ����� �������� ����������� 32-������ ����� � �������� FIFO.
            pio_sm_exec_wait_blocking(config_.message_demodulator_pio, message_demodulator_sm_[sm_index], pio_encode_push(false, true));
        }

        // �������� ����� ������� ������������.
        memset((void*)rx_packet_[sm_index].buffer, 0x0, Raw1090Packet::kMaxPacketLenWords32);

        // ������� ��� ����� �� RX FIFO.
        volatile uint16_t packet_num_words = pio_sm_get_rx_fifo_level(config_.message_demodulator_pio, message_demodulator_sm_[sm_index]);
       //!! Serial2.printf("ADSBee::OnDemodComplete. %d \r\n", packet_num_words);
        if (packet_num_words > Raw1090Packet::kMaxPacketLenWords32)
        {
            // ������������ ����� ������; �������� ��� � ���������� ����� � ��������� ���.
            // ��������� ��� ������ ������ ��� �������! ������ �� ���������� �������� � ���� ���������� USB.
           // Serial2.printf("ADSBee::OnDemodComplete. Received a packet with %d 32-bit words, expected maximum of %d.", packet_num_words, Raw1090Packet::kExtendedSquitterPacketNumWords32);
            pio_sm_clear_fifos(config_.message_demodulator_pio, (uint32_t)message_demodulator_sm_);
            packet_num_words = Raw1090Packet::kMaxPacketLenWords32;
        }

        // �����������, ��� �� ���������� ���-�� ��������������.
       // aircraft_dictionary.Record1090Demod();

        // ������� Raw1090Packet � �������� ��� � �������.
        for (uint16_t i = 0; i < packet_num_words; i++)
        {
            rx_packet_[sm_index].buffer[i] = pio_sm_get(config_.message_demodulator_pio, message_demodulator_sm_[sm_index]);
            if (i == packet_num_words - 1)
            {
               //  rx_packet_[sm_index].buffer[i] >>= 1;  // �������� ������ �������� ��� �� ���������� ����� � ������.
                // ����������� � ����������� �� ������ ���� �������� ����� �� ������ ����� � �����.
                switch (packet_num_words)
                {
                case Raw1090Packet::kSquitterPacketNumWords32:
                   // aircraft_dictionary.Record1090RawSquitterFrame();
                    rx_packet_[sm_index].buffer[i] = (rx_packet_[sm_index].buffer[i] & 0xFFFFFF) << 8;
                    rx_packet_[sm_index].buffer_len_bits = Raw1090Packet::kSquitterPacketLenBits;
                    raw_1090_packet_queue.Push(rx_packet_[sm_index]);//??
                   //!! Serial2.printf("ADSBee::Push(rx_packet. \t%08lX%06lX  56\r\n", rx_packet_[sm_index].buffer[0], rx_packet_[sm_index].buffer[0], rx_packet_[sm_index].buffer[1] >> (2 * kBitsPerNibble)); //

                   // decoder.raw_1090_packet_in_queue.Push(rx_packet_[sm_index]);
                    break;
                case Raw1090Packet::kExtendedSquitterPacketNumWords32:
                   // aircraft_dictionary.Record1090RawExtendedSquitterFrame();
                    rx_packet_[sm_index].buffer[i] = (rx_packet_[sm_index].buffer[i] & 0xFFFF) << 16; 
                    rx_packet_[sm_index].buffer_len_bits = Raw1090Packet::kExtendedSquitterPacketLenBits;
                    raw_1090_packet_queue.Push(rx_packet_[sm_index]);//??
                   // Serial2.printf("ADSBee::Push(rx_packet. %d \r\n", rx_packet_[sm_index]); //%08lX%08lX%08lX%04lX
                   // Serial2.printf("ADSBee::Push(rx_packet. \t%08lX%08lX%08lX%04lX 112\r\n", rx_packet_[sm_index].buffer[0], rx_packet_[sm_index].buffer[0], rx_packet_[sm_index].buffer[1], rx_packet_[sm_index].buffer[2], rx_packet_[sm_index].buffer[3] >> (4 * kBitsPerNibble)); //%08lX%08lX%08lX%04lX
                   // decoder.raw_1090_packet_in_queue.Push(rx_packet_[sm_index]);
                    break;
                default:
                    // Don't push partial packets.
                    // Printing to tinyUSB from within an interrupt causes crashes! Don't do it.
                    //!! CONSOLE_WARNING("ADSBee::OnDemodComplete", "Unhandled case while creating Raw1090Packet, received packet with %d 32-bit words.", packet_num_words);
                    break;
                }
            }
        }

        // Clear the FIFO by pushing partial word from ISR, not bothering to block if FIFO is full (it shouldn't be).
        pio_sm_exec_wait_blocking(config_.message_demodulator_pio, message_demodulator_sm_[sm_index],
            pio_encode_push(false, false));
        while (!pio_sm_is_rx_fifo_empty(config_.message_demodulator_pio, message_demodulator_sm_[sm_index]))
        {
            pio_sm_get(config_.message_demodulator_pio, message_demodulator_sm_[sm_index]);
        }

        // �������� �������� ������� ������������ ��� �������� ���������� ��������� �������������, ����� �������� ���.
        pio_sm_restart(config_.message_demodulator_pio, message_demodulator_sm_[sm_index]);  // Reset FIFOs, ISRs, etc.
        // ������������ ����������� ����� ������ ��������� �����, ����� ������ ��� ����, ��� ������ ��� DEMOD
        // �������� ����������. ��� ����� �������� ������ ��� ���������� �������� ���������, ����������� �������� ������������ ����������� ��
        // ������ ������� �������� GPIO.
        uint demodulator_program_start =
            sm_index == /*bsp.*/r1090_high_power_demod_state_machine_index
            ? message_demodulator_offset_ + message_demodulator_offset_high_power_initial_entry
            : message_demodulator_offset_ + message_demodulator_offset_initial_entry;
        pio_sm_exec_wait_blocking(config_.message_demodulator_pio, message_demodulator_sm_[sm_index],
            pio_encode_jmp(demodulator_program_start));  // Jump to beginning of program.
        pio_sm_set_enabled(config_.message_demodulator_pio, message_demodulator_sm_[sm_index], true);

        // ������� �������� ��������� �� ��������� ��������.
        if (sm_index == /*bsp.*/r1090_high_power_demod_state_machine_index)
        {
            // ������ ��������� ������� �������� �������� �������������� � �� ��������� � �������� ���������� ������ ������� SM. ���
            // ������ ���������� ����� �� ������������ ���������� ����� ��������� ��������� ��������� �������������� ���������, ��
            // ���������� ����� ����� �������� �� � �������� ������� ������� � ��������� �� ������������� ����������� ����� ������� ��������, ���� ��
            // ������ ��������.
            pio_sm_exec_wait_blocking(
                config_.preamble_detector_pio, preamble_detector_sm_[sm_index],
                pio_encode_jmp(preamble_detector_offset_ + preamble_detector_offset_waiting_for_first_edge));
        }

        pio_interrupt_clear(config_.preamble_detector_pio, sm_index);
    }
}




void ADSBee::OnSysTickWrap() { mlat_counter_wraps_ += kMLATWrapCounterIncrement; }

int ADSBee::ReadSignalStrengthMilliVolts()
{
    adc_select_input(config_.rssi_adc_input);
    int rssi_adc_counts = adc_read();
    return rssi_adc_counts * 3300 / 4095;
}

int ADSBee::ReadSignalStrengthdBm() { return AD8313MilliVoltsTodBm(ReadSignalStrengthMilliVolts()); }

int ADSBee::ReadTLMilliVolts()
{
    // Read back the low level TL bias output voltage.
    adc_select_input(config_.tl_adc_input);
    tl_adc_counts_ = adc_read();
    return ADCCountsToMilliVolts(tl_adc_counts_);
}

bool ADSBee::SetTLMilliVolts(int tl_mv)
{
    if (tl_mv > kTLMaxMV || tl_mv < kTLMinMV)
    {
        Serial2.printf("ADSBee::SetTLMilliVolts", "Unable to set tl_mv_ to %d, outside of permissible range %d-%d.\r\n", 
            tl_mv, kTLMinMV, kTLMaxMV);
        return false;
    }
    tl_mv_ = tl_mv;
    tl_pwm_count_ = tl_mv_ * kTLMaxPWMCount / kVDDMV; // tl_mv_ * 5000/3300

    return true;
}

void ADSBee::StartTLLearning(uint16_t tl_learning_num_cycles, uint16_t tl_learning_start_temperature_mv,
    uint16_t tl_min_mv, uint16_t tl_max_mv)
{
    tl_learning_temperature_mv_ = tl_learning_start_temperature_mv;
    tl_learning_temperature_step_mv_ = tl_learning_start_temperature_mv / tl_learning_num_cycles;
    tl_learning_cycle_start_timestamp_ms_ = millis();
}
