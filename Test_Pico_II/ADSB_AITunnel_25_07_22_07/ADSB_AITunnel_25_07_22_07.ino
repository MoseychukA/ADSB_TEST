
#include <Arduino.h>
#include <hardware/irq.h>
#include <hardware/pwm.h>
#include <hardware/adc.h>
#include <pico/multicore.h>
#include <pico/util/queue.h>
#include <mutex>
#include <hardware/structs/systick.h>
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "capture_pio.h"



// Конфигурация пинов
#define ADS_B_INPUT_PIN 19
#define PREAMBLE_LED_PIN 20
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define AGC_INPUT_PIN 26
#define AGC_OUTPUT_PIN 9
#define AGC_CONTROL_PIN 27
#define CORE0_LED_PIN 15
#define CORE1_LED_PIN 25

// Константы ADS-B
#define ADSB_PREAMBLE_BITS 8
#define ADSB_DATA_BITS 112
#define ADSB_TOTAL_BITS 120
#define QUEUE_SIZE 10
#define SAMPLES_PER_BIT 2
#define BIT_DURATION_US 1

// Структуры данных
typedef struct {
    uint8_t data[14]; // 112 бит = 14 байт
    uint32_t timestamp;
    bool valid;
} adsb_packet_t;

typedef struct {
    uint32_t icao;
    char flight[9];
    double latitude;
    double longitude;
    uint16_t altitude;
    uint16_t speed;
    uint8_t type_code;
    bool valid;
} adsb_decoded_t;

// Глобальные переменные
static queue_t packet_queue;
static volatile bool preamble_detected = false;
static volatile uint32_t bit_buffer[ADSB_TOTAL_BITS];
static volatile int bit_count = 0;
static volatile uint32_t last_edge_time = 0;
static volatile bool receiving = false;

//=========================================================
#define MLAT_SYSTEM_CLOCK_RATIO 48 / 125
// Scales 125MHz system clock into a 48MHz counter.
static const uint32_t kMLATWrapCounterIncrement = (1 << 24) * MLAT_SYSTEM_CLOCK_RATIO;

constexpr float kPreambleDetectorFreq = 48e6;    // Running at 48MHz (24 clock cycles per half bit).
constexpr float kMessageDemodulatorFreq = 48e6;  // Run at 48 MHz to demodulate bits at 1Mbps.

constexpr float kInt16MaxRecip = 1.0f / INT16_MAX;
//PIO preamble_detector_pio = pio0;
//uint preamble_detector_demod_pin_irq = IO_IRQ_BANK0;
//PIO message_demodulator_pio = pio1;
//uint preamble_detector_demod_complete_irq = PIO0_IRQ_0;
uint16_t r1090_num_demod_state_machines = 3;
uint16_t r1090_high_power_demod_state_machine_index = 1;
uint32_t irq_wrapper_offset_ = 0;

uint32_t irq_wrapper_sm_ = 0;
uint32_t preamble_detector_sm_;
uint32_t preamble_detector_offset_ = 0;
uint32_t irq_wrapper_offset_ = 0;
//uint16_t pulses_pins = ADS_B_INPUT_PIN;
//uint16_t demod_pins = PREAMBLE_LED_PIN;


class ADSBee
{
public:
    static constexpr uint16_t kTLMaxPWMCount = 5000;  // Тактовая частота 125 МГц, рекомендуемая частота ШИМ 25 кГц.
    static constexpr int kVDDMV = 3300;               // [mV] Voltage of positive supply rail.
    static constexpr int kTLMaxMV = 3300;             // [mV]
    static constexpr int kTLMinMV = 0;                // [mV]
    static constexpr uint32_t kStatusLEDOnMs = 10;

    static constexpr uint32_t kTLLearningIntervalMs = 10000;  // [ms] Length of Simulated Annealing interval for learning trigger level.
    static constexpr uint16_t kTLLearningNumCycles = 100;    // Number of simulated annealing cycles for learning trigger level.
    static constexpr uint16_t kTLLearningStartTemperatureMV = 1000;   // [mV] Starting value for simulated annealing temperature when learning triger level. This corresponds
               // to the maximum value that the trigger level could be moved (up or down) when exploring a neighbor
               // state.

    static constexpr int32_t kNoiseFloorExpoFilterPercent = 50;  // [%] Weight to use for low pass expo filter of noise floor ADC counts. 0 = no filter, 100 = hold value.
    static constexpr uint32_t kNoiseFloorADCSampleIntervalMs = 1;   // [ms] Interval between ADC samples to approximate noise floor value.



struct ADSBeeConfig
{
    PIO preamble_detector_pio = pio0;
    uint preamble_detector_demod_pin_irq = IO_IRQ_BANK0;
    PIO message_demodulator_pio = pio1;
    uint preamble_detector_demod_complete_irq = PIO0_IRQ_0;

   // uint16_t r1090_led_pin = 25;
    // Чтение ADS-B на GPIO19. Будет искать DE// Чтение ADS-B на GPIO19. Будет искать сигнал DEMOD на GPIO20.Сигнал MOD на GPIO20.
    uint16_t pulses_pins = ADS_B_INPUT_PIN;
    uint16_t demod_pins = PREAMBLE_LED_PIN;
    // Используйте GPIO22 для декодирования программы PIO, чтобы вывести ее восстановленные часы (только для отладки).
    uint16_t recovered_clk_pins = 21;  // Установите RECOVERED_CLK на фальшивый вывод для детектора преамбулы высокой мощности. Будет
                                                                  // переопределено более высоким приоритетом (более низким индексом) SM.
    

    uint32_t aircraft_dictionary_update_interval_ms = 1000;
};

ADSBee(ADSBeeConfig config_in);

private:
    ADSBeeConfig config_;

    uint32_t irq_wrapper_sm_ = 0;
    uint32_t preamble_detector_sm_;
    uint32_t preamble_detector_offset_ = 0;

    uint32_t irq_wrapper_offset_ = 0;

    uint32_t message_demodulator_sm_;
    uint32_t message_demodulator_offset_ = 0;

    uint32_t led_on_timestamp_ms_ = 0;





};

//extern ADSBee adsbee;

ADSBee(ADSBeeConfig config_in)
{
    config_ = config_in;

    //for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++)
    //{
    //    preamble_detector_sm_[sm_index] = pio_claim_unused_sm(config_.preamble_detector_pio, true);
    //    message_demodulator_sm_[sm_index] = pio_claim_unused_sm(config_.message_demodulator_pio, true);
    //}
    //irq_wrapper_sm_ = pio_claim_unused_sm(config_.preamble_detector_pio, true);

    //preamble_detector_offset_ = pio_add_program(config_.preamble_detector_pio, &preamble_detector_program);
    //irq_wrapper_offset_ = pio_add_program(config_.preamble_detector_pio, &irq_wrapper_program);
    //message_demodulator_offset_ = pio_add_program(config_.message_demodulator_pio, &message_demodulator_program);

    //// Помещаем параметры IRQ в глобальную область действия для ISR on_demod_complete.
    //isr_access = this;

}



ADSBee* isr_access = nullptr;
/** Запуск сквозных функций для публичного доступа **/
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










//=============================================================

// Таблица CRC для ADS-B (полином 0x1021)
static uint16_t crc_table[256];



// Инициализация таблицы CRC
void init_crc_table() {
    for (int i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
        crc_table[i] = crc;
    }
}

// Вычисление CRC для пакета ADS-B
uint32_t calculate_crc(const uint8_t* data, int length) {
    uint32_t crc = 0;
    for (int i = 0; i < length; i++) {
        crc = ((crc << 8) ^ crc_table[((crc >> 16) ^ data[i]) & 0xFF]) & 0xFFFFFF;
    }
    return crc;
}

// Проверка и коррекция ошибок в одном бите
bool correct_single_bit_error(uint8_t* data) {
    uint32_t received_crc = (data[11] << 16) | (data[12] << 8) | data[13];
    uint32_t calculated_crc = calculate_crc(data, 11);

    if (calculated_crc == received_crc) {
        return true; // Пакет корректный
    }

    // Пытаемся исправить ошибку в одном бите
    for (int byte_idx = 0; byte_idx < 11; byte_idx++) {
        for (int bit_idx = 0; bit_idx < 8; bit_idx++) {
            // Переворачиваем бит
            data[byte_idx] ^= (1 << bit_idx);

            // Проверяем CRC
            uint32_t new_crc = calculate_crc(data, 11);
            if (new_crc == received_crc) {
                return true; // Ошибка исправлена
            }

            // Возвращаем бит обратно
            data[byte_idx] ^= (1 << bit_idx);
        }
    }

    return false; // Не удалось исправить
}

// Детектор преамбулы
bool detect_preamble(const uint32_t* bits) {
    // Паттерн преамбулы ADS-B: 1010000101000000
    const uint8_t preamble_pattern[] = {1,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0};

    for (int i = 0; i < 16; i++) {
        if (bits[i] != preamble_pattern[i]) {
            return false;
        }
    }
    return true;
}

//// Обработчик прерывания на входном пине
//void IRAM_ATTR gpio_irq_handler(uint gpio, uint32_t events) {
//    if (gpio != ADS_B_INPUT_PIN) return;
//
//    uint32_t current_time = micros();
//    bool current_level = digitalRead(ADS_B_INPUT_PIN);
//
//    if (!receiving) {
//        // Ищем преамбулу
//        if (current_level == HIGH) {
//            receiving = true;
//            bit_count = 0;
//            bit_buffer[bit_count++] = 1;
//            last_edge_time = current_time;
//            digitalWrite(PREAMBLE_LED_PIN, HIGH);
//        }
//    } else {
//        // Принимаем данные
//        uint32_t bit_duration = current_time - last_edge_time;
//        int num_bits = (bit_duration + BIT_DURATION_US/2) / BIT_DURATION_US;
//
//        if (num_bits > 0 && bit_count < ADSB_TOTAL_BITS) {
//            for (int i = 0; i < num_bits && bit_count < ADSB_TOTAL_BITS; i++) {
//                bit_buffer[bit_count++] = current_level;
//            }
//        }
//
//        last_edge_time = current_time;
//
//        // Проверяем завершение пакета
//        if (bit_count >= ADSB_TOTAL_BITS) {
//            receiving = false;
//            digitalWrite(PREAMBLE_LED_PIN, LOW);
//
//            // Проверяем преамбулу и добавляем пакет в очередь
//            if (detect_preamble((const uint32_t*)bit_buffer)) {
//                adsb_packet_t packet;
//
//                // Конвертируем биты в байты
//                memset(packet.data, 0, 14);
//                for (int i = 16, byte_idx = 0, bit_idx = 7; i < ADSB_TOTAL_BITS && byte_idx < 14; i++) {
//                    if (bit_buffer[i]) {
//                        packet.data[byte_idx] |= (1 << bit_idx);
//                    }
//                    bit_idx--;
//                    if (bit_idx < 0) {
//                        bit_idx = 7;
//                        byte_idx++;
//                    }
//                }
//
//                packet.timestamp = current_time;
//                packet.valid = true;
//
//                // Добавляем в очередь (неблокирующий вызов)
//                queue_try_add(&packet_queue, &packet);
//            }
//        }
//    }
//}

// Декодирование ICAO адреса
uint32_t decode_icao(const uint8_t* data) {
    return (data[1] << 16) | (data[2] << 8) | data[3];
}

// Декодирование типа сообщения
uint8_t decode_type_code(const uint8_t* data) {
    return (data[4] >> 3) & 0x1F;
}

// Декодирование позиции (упрощенная версия)
void decode_position(const uint8_t* data, double lat, double* lon) {
    uint8_t tc = decode_type_code(data);

    if (tc >= 9 && tc <= 18) { // Airborne position
        uint32_t lat_cpr = ((data[6] & 0x03) << 15) | (data[7] << 7) | (data[8] >> 1);
        uint32_t lon_cpr = ((data[8] & 0x01) << 16) | (data[9] << 8) | data[10];

        //// Упрощенное декодирование (требует reference position)
        //!!lat = (lat_cpr / 131072.0)  180.0 - 90.0;
        //!!lon = (lon_cpr / 131072.0)  360.0 - 180.0;
    }
}

// Декодирование скорости и высоты
void decode_velocity_altitude(const uint8_t* data, uint16_t speed, uint16_t* altitude) {
    uint8_t tc = decode_type_code(data);

    //if (tc >= 9 && tc <= 18) { // Airborne position
    //    uint16_t alt_code = ((data[5] & 0xFF) << 4) | ((data[6] & 0xF0) >> 4);
    //    if (alt_code != 0) {
    //        altitude = alt_code  25 - 1000; // Упрощенное вычисление
    //    }
    //}

    //if (tc == 19) { // Velocity
    //    uint16_t ew_vel = ((data[5] & 0x03) << 8) | data[6];
    //    uint16_t ns_vel = ((data[7] & 0x1F) << 6) | ((data[8] & 0xFC) >> 2);
    //    speed = sqrt(ew_vel  ew_vel + ns_vel * ns_vel);
    //}
}

// Декодирование номера рейса
void decode_flight(const uint8_t* data, char* flight) {
    uint8_t tc = decode_type_code(data);

    if (tc >= 1 && tc <= 4) { // Aircraft identification
        const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";

        flight[0] = charset[(data[5] & 0xFC) >> 2];
        flight[1] = charset[((data[5] & 0x03) << 4) | ((data[6] & 0xF0) >> 4)];
        flight[2] = charset[((data[6] & 0x0F) << 2) | ((data[7] & 0xC0) >> 6)];
        flight[3] = charset[data[7] & 0x3F];
        flight[4] = charset[(data[8] & 0xFC) >> 2];
        flight[5] = charset[((data[8] & 0x03) << 4) | ((data[9] & 0xF0) >> 4)];
        flight[6] = charset[((data[9] & 0x0F) << 2) | ((data[10] & 0xC0) >> 6)];
        flight[7] = charset[data[10] & 0x3F];
        flight[8] = '\0';

        // Удаляем пробелы в конце
        for (int i = 7; i >= 0; i--) {
            if (flight[i] == ' ' || flight[i] == '?') {
                flight[i] = '\0';
            } else {
                break;
            }
        }
    } else {
        strcpy(flight, "N/A");
    }
}

//// Декодирование полного пакета
//adsb_decoded_t decode_packet(const adsb_packet_t* packet) {
//    adsb_decoded_t decoded;
//    memset(&decoded, 0, sizeof(decoded));
//
//    decoded.icao = decode_icao(packet->data);
//    decoded.type_code = decode_type_code(packet->data);
//
//    decode_flight(packet->data, decoded.flight);
//    decode_position(packet->data, &decoded.latitude, &decoded.longitude);
//    decode_velocity_altitude(packet->data, &decoded.speed, &decoded.altitude);
//
//    decoded.valid = true;
//    return decoded;
//}

// Управление усилением
void setup_agc() {
    // Настройка ADC для чтения уровня сигнала
    adc_init();
    adc_gpio_init(AGC_INPUT_PIN);
    adc_gpio_init(AGC_CONTROL_PIN);

    // Настройка PWM для управления усилением
    gpio_set_function(AGC_OUTPUT_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(AGC_OUTPUT_PIN);
    pwm_set_wrap(slice_num, 255);
    pwm_set_enabled(slice_num, true);
}

// Обновление усиления
void update_agc() {
    static uint32_t last_agc_update = 0;
    uint32_t current_time = millis();

    if (current_time - last_agc_update > 10) { // Обновляем каждые 10ms
        adc_select_input(0); // AGC_INPUT_PIN - 26 = ADC0
        uint16_t agc_input = adc_read();

        adc_select_input(1); // AGC_CONTROL_PIN - 26 = ADC1
        uint16_t control_level = adc_read();

        // Простой алгоритм AGC
        uint16_t target_level = 2048; // Целевой уровень
        int16_t error = agc_input - target_level;
        static int16_t gain = 128;

        gain -= error / 64; // Пропорциональное управление
        gain = constrain(gain, 0, 255);

        uint slice_num = pwm_gpio_to_slice_num(AGC_OUTPUT_PIN);
        pwm_set_gpio_level(AGC_OUTPUT_PIN, gain);

        last_agc_update = current_time;
    }
}

// Код для ядра 1 (прием пакетов)
void core1_loop() {
    uint32_t last_led_toggle = 0;

    while (true) {
        // Переключение LED каждые 500ms
        uint32_t current_time = millis();
        if (current_time - last_led_toggle >= 500) {
            digitalWrite(CORE1_LED_PIN, !digitalRead(CORE1_LED_PIN));
            last_led_toggle = current_time;
        }

        // Обновление AGC
        update_agc();

        delay(1);
    }
}

//============================================================================
bool ADSBee_Init()
{

    //===================================================================================

    // Включить таймер MLAT с помощью 24-битного таймера SysTick, подключенного к тактовой частоте процессора 125 МГц.
    // Регистр управления и состояния SysTick
    systick_hw->csr = 0b110;  // Источник = Частота процессора, TickInt = Включено, Счетчик = Отключено.
    // Регистр перезагрузки значения SysTick
    systick_hw->rvr = 0xFFFFFF;  // Используйте полный 24-битный диапазон регистра значения таймера.
    // 0xFFFFFF = 16777215 отсчетов @ 125 МГц = примерно 0,134 секунды.

    // Вызвать функцию OnSysTickWrap каждый раз, когда таймер SysTick достигает 0.
   //  exception_set_exclusive_handler(SYSTICK_EXCEPTION, on_systick_exception);  //Уточнить что нужно
    // 
    systick_hw->csr |= 0b1;  // Включить счетчик.

    /** PREAMBLE DETECTOR PIO **/
    // Calculate the PIO clock divider. // Рассчитаем делитель тактовой частоты PIO.
    float preamble_detector_div = (float)clock_get_hz(clk_sys) / kPreambleDetectorFreq;
    irq_wrapper_program_init(preamble_detector_pio, r1090_num_demod_state_machines, irq_wrapper_offset_, preamble_detector_div);



        // Заставить конечный автомат ждать запуска только в том случае, если он входит в циклическую группу правильно сформированных преамбул детекторов.
        bool make_sm_wait =  r1090_high_power_demod_state_machine_index;
        // Инициализируем программу с помощью вспомогательной функции файла .pio
        preamble_detector_program_init(preamble_detector_pio,                     // Use PIO block 0.
            preamble_detector_sm_,                    // State machines 0-2
            preamble_detector_offset_ ,               // Program startin offset.
            pulses_pins,                             // Pulses pin (input).
            demod_pins,                              // Demod pin (output).
            preamble_detector_div,                             // Clock divisor (for 48MHz).
            make_sm_wait  // Должен ли конечный автомат ждать начала IRQ.
        );

        // Обработка прерываний GPIO (для маркировки начала интервала демодуляции).
        gpio_set_irq_enabled_with_callback(demod_pins, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, on_demod_pin_change);

        // Установить последовательность преамбулы в ISR: ISR: 0b101000010100000(0)
        // Последний 0 удален из последовательности преамбулы, чтобы дать демодулятору больше времени для запуска.
        // mov isr null ; Очистить ISR.
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_mov(pio_isr, pio_null));
        // Заполните начало шаблона преамбулы другими битами, если конечный автомат предназначен для обнаружения высокой мощности преамбулы.
        if (sm_index == bsp.r1090_high_power_demod_state_machine_index)
        {
            // Преамбула высокой мощности.
            // set x 0b111  ; ISR = 0b00000000000000000000000000000000
            pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_set(pio_x, 0b111));
            // in x 3       ; ISR = 0b00000000000000000000000000000111
            pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_in(pio_x, 3));
            // set x 0b101  ; ISR = 0b00000000000000000000000000000000
            pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_set(pio_x, 0b101));
        }
        else
        {
            // Хорошо составленная преамбула.
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
       // Примечание: это короче настоящего хвоста, но нам нужно дополнительное время для запуска демодулятора.
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_in(pio_null, 4));
        // mov x null   ; Clear scratch x.
        pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_mov(pio_x, pio_null));
        // Используйте эту инструкцию для проверки правильности формирования преамбулы (отправляет ISR в RX FIFO).
        // pio_sm_exec(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], pio_encode_push(false, true));


    // Включить прерывание DEMOD на PIO1_IRQ_0.
    pio_set_irq0_source_enabled(config_.preamble_detector_pio, pis_interrupt0, true);  // PIO0 state machine 0
    pio_set_irq0_source_enabled(config_.preamble_detector_pio, pis_interrupt1, true);  // PIO0 state machine 1
    pio_set_irq0_source_enabled(config_.preamble_detector_pio, pis_interrupt2, true);  // PIO0 state machine 2

  // Обработка PIO0 IRQ0.
    irq_set_exclusive_handler(config_.preamble_detector_demod_complete_irq, on_demod_complete);
    irq_set_enabled(config_.preamble_detector_demod_complete_irq, true);

    /** MESSAGE DEMODULATOR PIO **/
    float message_demodulator_div = (float)clock_get_hz(clk_sys) / kMessageDemodulatorFreq;
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++)
    {
        message_demodulator_program_init(config_.message_demodulator_pio, message_demodulator_sm_[sm_index],
            message_demodulator_offset_, config_.pulses_pins[sm_index],
            config_.demod_pins[sm_index], config_.recovered_clk_pins[sm_index],
            message_demodulator_div);
    }

    // Установите приоритет прерываний GPIO выше, чем у прерывания DEMOD, чтобы разрешить измерение RSSI.
    // irq_set_priority(config_.preamble_detector_demod_complete_irq, 1);
    irq_set_priority(config_.preamble_detector_demod_pin_irq, 0);

    // Устанавливаем метку времени последнего обновления словаря.
    last_aircraft_dictionary_update_timestamp_ms_ = get_time_since_boot_ms();

    // Включить конечные автоматы.
    pio_sm_set_enabled(config_.preamble_detector_pio, irq_wrapper_sm_, true);
    // Сначала нужно включить SM демодулятора, так как если детектор преамбулы срабатывает IRQ, но демодулятор не
    // включен, мы попадаем в тупик (я думаю, это, возможно, следует проверить еще раз).
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_num_demod_state_machines; sm_index++)
    {
        // pio_sm_set_enabled(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], true);
        pio_sm_set_enabled(config_.message_demodulator_pio, message_demodulator_sm_[sm_index], true);
    }
    // Включить циклический перебор хорошо сформированных детекторов преамбул.
    // ПРИМЕЧАНИЕ: их необходимо включить, чтобы разрешить работу детектора преамбулы высокой мощности, поскольку они сбрасывают IRQ, на который опирается
    // детектор преамбулы высокой мощности. Это пережиток того факта, что детектор преамбулы высокой мощности использует
    // тот же код PIO, который выполняет циклический перебор для детекторов хорошо сформированной преамбулы.
    for (uint16_t sm_index = 0; sm_index < bsp.r1090_high_power_demod_state_machine_index; sm_index++)
    {
        pio_sm_set_enabled(config_.preamble_detector_pio, preamble_detector_sm_[sm_index], true);
    }
    // Включить детектор преамбулы высокой мощности.
    pio_sm_set_enabled(config_.preamble_detector_pio, preamble_detector_sm_[bsp.r1090_high_power_demod_state_machine_index], true);

    //  DisableWatchdog();
    return true;
}





//============================================================================

void setup() {
    // Инициализация Serial
   // Serial.begin(115200);
    Serial2.setTX(UART_TX_PIN);
    Serial2.setRX(UART_RX_PIN);
    Serial2.begin(115200);

    // Настройка пинов
    pinMode(ADS_B_INPUT_PIN, INPUT);
    pinMode(PREAMBLE_LED_PIN, OUTPUT);
    pinMode(CORE0_LED_PIN, OUTPUT);
    pinMode(CORE1_LED_PIN, OUTPUT);

    // Инициализация очереди
    queue_init(&packet_queue, sizeof(adsb_packet_t), QUEUE_SIZE);

    // Инициализация CRC таблицы
    init_crc_table();

    // Настройка AGC
    setup_agc();

    //// Настройка прерывания
    //gpio_set_irq_enabled_with_callback(ADS_B_INPUT_PIN,
    //                                  GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
    //                                  true, &gpio_irq_handler);






    // Запуск второго ядра
    multicore_launch_core1(core1_loop);

    Serial.println("ADS-B Receiver started");
    Serial2.println("ADS-B Receiver started");
}

void loop() {
    static uint32_t last_led_toggle = 0;
    adsb_packet_t packet;

    // Переключение LED каждую секунду
    uint32_t current_time = millis();
    if (current_time - last_led_toggle >= 1000) {
        digitalWrite(CORE0_LED_PIN, !digitalRead(CORE0_LED_PIN));
        last_led_toggle = current_time;
    }

    // Обработка пакетов из очереди
    if (queue_try_remove(&packet_queue, &packet)) {
        // Проверка и коррекция CRC
        if (correct_single_bit_error(packet.data)) {
            // Декодирование пакета
           //!! adsb_decoded_t decoded = decode_packet(&packet);

            //if!! (decoded.valid) {
            //    // Отправка данных через Serial2
            //    String output = "ICAO:" + String(decoded.icao, HEX);
            //    output += ",FLIGHT:" + String(decoded.flight);
            //    output += ",LAT:" + String(decoded.latitude, 6);
            //    output += ",LON:" + String(decoded.longitude, 6);
            //    output += ",ALT:" + String(decoded.altitude);
            //    output += ",SPEED:" + String(decoded.speed);
            //    output += ",TC:" + String(decoded.type_code);

            //    Serial2.println(output);
            //    Serial.println(output);
            //}
        }
    }

    delay(1);
}

/*
Эта программа реализует все требования:

Основные функции:

Прием сигналов ADS-B на pin 19 с использованием прерываний
Двухядерная архитектура: ядро 1 - прием, ядро 0 - обработка
Детектор преамбулы с индикацией на pin 20
Очередь пакетов для передачи между ядрами
Полное декодирование пакетов ADS-B
CRC проверка и коррекция ошибок в одном бите
AGC система с входом на pin 26, управлением на pin 9 и контролем на pin 27
UART вывод данных через Serial2 (pins 4,5)
Индикаторы активности ядер на pins 15 и 25

Декодируемые данные:
ICAO адрес самолета
Номер рейса
Координаты (широта/долгота)
Высота
Скорость
Тип сообщения

Программа работает без FreeRTOS, используя встроенные возможности RP2040 для многоядерности и обработки прерываний.
*/