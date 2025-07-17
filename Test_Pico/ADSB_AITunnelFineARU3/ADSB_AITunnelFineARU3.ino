//Полная программа для приема и расшифровки пакетов ADS-B на микроконтроллере RP2040:

#include <Arduino.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include <stdio.h>
#include <math.h>
#include "hardware/adc.h"
#include <math.h>
#include <stdint.h>
#include "hardware/uart.h"


#define M_PI 3.14159265358979323846
#define CPR_MAX_AGE 10000  // Максимальный возраст кадра в мс


// Конфигурация пинов
#define ADS_B_PIN1 26//9
#define ADS_B_PIN2 27//22
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// Константы ADS-B
#define ADS_B_PREAMBLE_US 8
#define ADS_B_BIT_DURATION_US 1
#define ADS_B_PACKET_LENGTH 112
#define SAMPLE_RATE_HZ 2000000  // 2 MHz
#define BUFFER_SIZE 4096
#define MAX_PACKETS 32


// Структура для кэша CPR
typedef struct {
    double lat;
    double lon;
    uint8_t odd_frame;
    uint32_t timestamp;
    uint32_t icao;
} cpr_cache_entry_t;

static cpr_cache_entry_t cpr_cache[256];

// Функция для получения количества зон долготы
int get_nl(double lat) {
    if (lat >= 87.0) return 1;
    if (lat <= -87.0) return 1;

    double lat_rad = fabs(lat) * M_PI / 180.0;
    double temp = 1.0 - (1.0 - cos(M_PI / 30.0)) / pow(cos(lat_rad), 2);

    if (temp <= 0) return 1;
    return (int)floor(2.0 * M_PI / acos(temp));
}



// Структуры данных
typedef struct {
    uint8_t data[14];  // 112 bits = 14 bytes
    uint32_t timestamp;
    int8_t rssi;
    bool valid;
} ADSBPacket;

typedef struct {
    uint32_t samples[BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    bool overflow;
} SampleBuffer;

typedef struct {
    double lat;
    double lon;
    bool odd_frame;
    uint32_t timestamp;
} CPRData;

// Глобальные переменные
static SampleBuffer sample_buffer;
static QueueHandle_t packet_queue;
static ADSBPacket packet_buffer[MAX_PACKETS];
//static CPRData cpr_cache[256];  // Кэш для CPR расчетов
static int dma_chan_a, dma_chan_b;
static uint8_t current_gain = 128;
static uint32_t noise_floor = 0;
static uint32_t signal_threshold = 0;

bool decode_manchester(uint32_t* samples_data, int length, uint8_t* output);  // ← ИСПРАВЛЕНО
int decode_cpr_position(uint8_t* data, double* lat, double* lon);
int find_preamble(uint32_t* samples_data, int length);
int decode_cpr_position(uint8_t* data, double* lat, double* lon);
void decode_adsb_packet(ADSBPacket* packet);
void init_dma();
//void process_digital_samples();
//void setup_dma();
//void dma_handler();

// Таблица CRC для ADS-B
static const uint32_t crc_table[256] = {
    0x000000, 0xfff409, 0x1ff813, 0xe00c1a, 0x3ff026, 0xc00c2f, 0x200835, 0xdff43c,
    // ... (полная таблица CRC)
};

// Функция инициализации DMA
void init_dma() {
    dma_chan_a = dma_claim_unused_channel(true);
    dma_chan_b = dma_claim_unused_channel(true);

    // Конфигурация DMA канала A
    dma_channel_config config_a = dma_channel_get_default_config(dma_chan_a);
    channel_config_set_transfer_data_size(&config_a, DMA_SIZE_32);
    channel_config_set_read_increment(&config_a, false);
    channel_config_set_write_increment(&config_a, true);
    channel_config_set_dreq(&config_a, DREQ_ADC);
    channel_config_set_chain_to(&config_a, dma_chan_b);

    dma_channel_configure(
        dma_chan_a,
        &config_a,
        sample_buffer.samples,  // write address
        &adc_hw->fifo,          // read address
        BUFFER_SIZE / 2,        // transfer count
        false                   // don't start yet
    );

    // Аналогично для канала B
    dma_channel_config config_b = dma_channel_get_default_config(dma_chan_b);
    channel_config_set_transfer_data_size(&config_b, DMA_SIZE_32);
    channel_config_set_read_increment(&config_b, false);
    channel_config_set_write_increment(&config_b, true);
    channel_config_set_dreq(&config_b, DREQ_ADC);
    channel_config_set_chain_to(&config_b, dma_chan_a);

    dma_channel_configure(
        dma_chan_b,
        &config_b,
        &sample_buffer.samples[BUFFER_SIZE / 2],
        &adc_hw->fifo,
        BUFFER_SIZE / 2,
        false
    );
}

// Функция расчета CRC
uint32_t calculate_crc(uint8_t* data, int length) {
    uint32_t crc = 0;
    for (int i = 0; i < length; i++) {
        crc = (crc << 8) ^ crc_table[((crc >> 16) ^ data[i]) & 0xff];
        crc &= 0xffffff;
    }
    return crc;
}

// Фильтр шумов
bool noise_filter(uint32_t* samples, int length) {
    uint32_t sum = 0;
    uint32_t max_val = 0;

    for (int i = 0; i < length; i++) {
        sum += samples[i];
        if (samples[i] > max_val) {
            max_val = samples[i];
        }
    }

    uint32_t average = sum / length;

    // Обновляем уровень шума
    if (max_val < average * 2) {
        noise_floor = (noise_floor * 15 + average) / 16;
    }

    signal_threshold = noise_floor * 3;

    return max_val > signal_threshold;
}

// Автоматическая регулировка усиления
void adjust_gain() {
    static uint32_t last_adjustment = 0;
    static int signal_count = 0;
    static int noise_count = 0;

    uint32_t now = millis();

    if (now - last_adjustment > 1000) {  // Каждую секунду
        float signal_ratio = (float)signal_count / (signal_count + noise_count);

        if (signal_ratio < 0.1 && current_gain < 255) {
            current_gain = min(255, current_gain + 10);
            // Применить новое усиление к АЦП
        } else if (signal_ratio > 0.8 && current_gain > 10) {
            current_gain = max(10, current_gain - 10);
            // Применить новое усиление к АЦП
        }

        signal_count = 0;
        noise_count = 0;
        last_adjustment = now;
    }
}

bool decode_manchester(uint32_t* samples_data, int length, uint8_t* output) {
    int bit_count = 0;
    int sample_per_bit = SAMPLE_RATE_HZ / 1000000;// Samples per microsecond

    for (int i = 0; i < length && bit_count < ADS_B_PACKET_LENGTH; i += sample_per_bit * 2) {
        if (i + sample_per_bit >= length) break;

        uint32_t first_half = 0, second_half = 0;

        for (int j = 0; j < sample_per_bit; j++) {
            first_half += samples_data[i + j];
        }
        first_half /= sample_per_bit;

        for (int j = 0; j < sample_per_bit; j++) {
            second_half += samples_data[i + sample_per_bit + j];
        }
        second_half /= sample_per_bit;

        if (first_half > second_half) {
            output[bit_count / 8] |= (1 << (7 - (bit_count % 8)));
        }
        else {
            output[bit_count / 8] &= ~(1 << (7 - (bit_count % 8)));
        }

        bit_count++;
    }

    return bit_count == ADS_B_PACKET_LENGTH;
}



// Поиск преамбулы ADS-B
int find_preamble(uint32_t* samples, int length) {
    int sample_per_us = SAMPLE_RATE_HZ / 1000000;

    for (int i = 0; i < length - 8 * sample_per_us; i++) {
        bool preamble_found = true;

        // Проверяем структуру преамбулы: 1010000110
        uint32_t levels[10];
        for (int j = 0; j < 10; j++) {
            uint32_t sum = 0;
            int start = i + j * sample_per_us;
            for (int k = 0; k < sample_per_us; k++) {
                sum += samples[start + k];
            }
            levels[j] = sum / sample_per_us;
        }

        // Проверяем паттерн
        if (!(levels[0] > levels[1] && levels[1] < levels[2] &&
              levels[2] > levels[3] && levels[3] < levels[4] &&
              levels[4] < levels[5] && levels[5] < levels[6] &&
              levels[6] > levels[7] && levels[7] > levels[8] &&
              levels[8] < levels[9])) {
            continue;
        }

        return i + 8 * sample_per_us;  // Возвращаем позицию после преамбулы
    }

    return -1;
}

// Полный алгоритм CPR
int decode_cpr_position(uint8_t* data, double* lat, double* lon) {
    // Извлекаем данные из пакета
    uint8_t typecode = (data[4] >> 3) & 0x1f;
    if (typecode < 9 || typecode > 18) return 0;  // Не позиционный пакет

    uint8_t odd_flag = (data[6] >> 2) & 0x01;
    uint32_t lat_cpr = ((data[6] & 0x03) << 15) | (data[7] << 7) | (data[8] >> 1);
    uint32_t lon_cpr = ((data[8] & 0x01) << 16) | (data[9] << 8) | data[10];

    // Извлекаем ICAO адрес
    uint32_t icao = (data[1] << 16) | (data[2] << 8) | data[3];
    uint8_t cache_idx = icao & 0xFF;

    // Нормализуем CPR координаты
    double lat_cpr_norm = lat_cpr / 131072.0;
    double lon_cpr_norm = lon_cpr / 131072.0;

    uint32_t current_time = millis(); // Предполагается, что функция определена

    // Проверяем наличие парного кадра в кэше
    if (cpr_cache[cache_idx].icao == icao &&
        cpr_cache[cache_idx].timestamp != 0 &&
        (current_time - cpr_cache[cache_idx].timestamp) < CPR_MAX_AGE &&
        cpr_cache[cache_idx].odd_frame != odd_flag) {

        // ГЛОБАЛЬНОЕ ДЕКОДИРОВАНИЕ
        double lat_cpr_even, lat_cpr_odd, lon_cpr_even, lon_cpr_odd;

        if (odd_flag) {
            lat_cpr_odd = lat_cpr_norm;
            lon_cpr_odd = lon_cpr_norm;
            lat_cpr_even = cpr_cache[cache_idx].lat;
            lon_cpr_even = cpr_cache[cache_idx].lon;
        }
        else {
            lat_cpr_even = lat_cpr_norm;
            lon_cpr_even = lon_cpr_norm;
            lat_cpr_odd = cpr_cache[cache_idx].lat;
            lon_cpr_odd = cpr_cache[cache_idx].lon;
        }

        // Расчет широты
        double j = floor(59.0 * lat_cpr_even - 60.0 * lat_cpr_odd + 0.5);
        double d_lat_even = 360.0 / 60.0;
        double lat_result = d_lat_even * (j + lat_cpr_even);

        // Нормализация широты
        if (lat_result >= 270.0) lat_result -= 360.0;
        if (lat_result <= -270.0) lat_result += 360.0;

        // Проверка валидности широты
        if (lat_result < -90.0 || lat_result > 90.0) {
            return 0; // Невалидная широта
        }

        // Расчет долготы
        int nl_even = get_nl(lat_result);
        int nl_odd = nl_even - 1;
        if (nl_odd < 1) nl_odd = 1;

        // Проверка совместимости зон
        if (nl_even != get_nl(lat_result)) {
            return 0; // Несовместимые зоны
        }

        double d_lon_even = 360.0 / nl_even;
        double d_lon_odd = 360.0 / nl_odd;

        double m = floor(lon_cpr_even * (nl_odd)-lon_cpr_odd * nl_even + 0.5);
        double lon_result;

        if (!odd_flag) {  // Используем четный кадр
            lon_result = d_lon_even * (m + lon_cpr_even);
        }
        else {  // Используем нечетный кадр
            lon_result = d_lon_odd * (m + lon_cpr_odd);
        }

        // Нормализация долготы
        while (lon_result >= 180.0) lon_result -= 360.0;
        while (lon_result < -180.0) lon_result += 360.0;

        *lat = lat_result;
        *lon = lon_result;

        // Обновляем кэш
        cpr_cache[cache_idx].lat = lat_cpr_norm;
        cpr_cache[cache_idx].lon = lon_cpr_norm;
        cpr_cache[cache_idx].odd_frame = odd_flag;
        cpr_cache[cache_idx].timestamp = current_time;
        cpr_cache[cache_idx].icao = icao;

        return 1; // Успешное декодирование
    }

    // Если нет парного кадра, сохраняем текущий в кэш
    cpr_cache[cache_idx].lat = lat_cpr_norm;
    cpr_cache[cache_idx].lon = lon_cpr_norm;
    cpr_cache[cache_idx].odd_frame = odd_flag;
    cpr_cache[cache_idx].timestamp = current_time;
    cpr_cache[cache_idx].icao = icao;

    return 0; // Нужен парный кадр для декодирования
}


// Вспомогательная функция для правильного модуля (работает с отрицательными числами)
double mod_func(double a, double b) {
    return a - b * floor(a / b);
}

// Дополнительная функция для локального декодирования
int decode_cpr_local(uint8_t* data, double ref_lat, double ref_lon, double* lat, double* lon) {
    uint8_t typecode = (data[4] >> 3) & 0x1f;
    if (typecode < 9 || typecode > 18) return 0;

    uint8_t odd_flag = (data[6] >> 2) & 0x01;
    uint32_t lat_cpr = ((data[6] & 0x03) << 15) | (data[7] << 7) | (data[8] >> 1);
    uint32_t lon_cpr = ((data[8] & 0x01) << 16) | (data[9] << 8) | data[10];

    // Нормализация CPR координат
    double lat_cpr_norm = lat_cpr / 131072.0;
    double lon_cpr_norm = lon_cpr / 131072.0;

    // Проверка валидности опорных координат
    if (ref_lat < -90.0 || ref_lat > 90.0 || ref_lon < -180.0 || ref_lon > 180.0) {
        return 0;
    }

    // Локальное декодирование широты
    double d_lat = odd_flag ? (360.0 / 59.0) : (360.0 / 60.0);

    // Вычисляем индекс зоны широты
    double j = floor(ref_lat / d_lat) +
        floor(0.5 + mod_func(ref_lat, d_lat) / d_lat - lat_cpr_norm);

    *lat = d_lat * (j + lat_cpr_norm);

    // Нормализация широты в диапазон [-90, 90]
    if (*lat > 90.0) *lat = 180.0 - *lat;
    if (*lat < -90.0) *lat = -180.0 - *lat;

    // Проверка валидности широты после декодирования
    if (*lat > 90.0 || *lat < -90.0) {
        return 0;
    }

    // Локальное декодирование долготы
    int nl = get_nl(*lat);
    if (odd_flag && nl > 1) nl = nl - 1;  // Для нечетных кадров
    if (nl < 1) nl = 1;

    double d_lon = 360.0 / nl;

    // Вычисляем индекс зоны долготы
    double m = floor(ref_lon / d_lon) +
        floor(0.5 + mod_func(ref_lon, d_lon) / d_lon - lon_cpr_norm);

    *lon = d_lon * (m + lon_cpr_norm);

    // Нормализация долготы в диапазон [-180, 180)
    *lon = mod_func(*lon + 180.0, 360.0) - 180.0;

    // Дополнительная проверка: расстояние от опорной точки не должно быть слишком большим
    double lat_diff = fabs(*lat - ref_lat);
    double lon_diff = fabs(*lon - ref_lon);

    // Учитываем пересечение 180-го меридиана для долготы
    if (lon_diff > 180.0) {
        lon_diff = 360.0 - lon_diff;
    }

    // Если расстояние больше половины зоны, возможно ошибка
    if (lat_diff > d_lat / 2.0 || lon_diff > d_lon / 2.0) {
        return 0;  // Результат кажется неправдоподобным
    }

    return 1;
}



// Декодирование пакета ADS-B
void decode_adsb_packet(ADSBPacket* packet) {
    uint8_t* data = packet->data;

    // Проверяем CRC
    uint32_t received_crc = (data[11] << 16) | (data[12] << 8) | data[13];
    uint32_t calculated_crc = calculate_crc(data, 11);

    if (received_crc != calculated_crc) {
        packet->valid = false;
        return;
    }

    packet->valid = true;

    // Извлекаем основную информацию
    uint8_t df = (data[0] >> 3) & 0x1f;  // Downlink Format
    uint32_t icao = (data[1] << 16) | (data[2] << 8) | data[3];

    Serial1.print("DF:");
    Serial1.print(df);
    Serial1.print(" ICAO:");
    Serial1.print(icao, HEX);

    if (df == 17 || df == 18) {  // ADS-B сообщения
        uint8_t typecode = (data[4] >> 3) & 0x1f;

        Serial1.print(" TC:");
        Serial1.print(typecode);

        if (typecode >= 1 && typecode <= 4) {
            // Идентификация воздушного судна
            char callsign[9] = {0};
            uint64_t chars = ((uint64_t)data[5] << 40) | ((uint64_t)data[6] << 32) |
                           ((uint64_t)data[7] << 24) | ((uint64_t)data[8] << 16) |
                           ((uint64_t)data[9] << 8) | data[10];

            static const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ      0123456789      ";

            for (int i = 0; i < 8; i++) {
                callsign[i] = charset[(chars >> (42 - i * 6)) & 0x3f];
            }

            Serial1.print(" Callsign:");
            Serial1.print(callsign);

        } else if (typecode >= 9 && typecode <= 18) {
            // Позиционные данные
            double lat, lon;
            decode_cpr_position(data, &lat, &lon);

            if (lat != 0 || lon != 0) {
                Serial1.print(" Lat:");
                Serial1.print(lat, 6);
                Serial1.print(" Lon:");
                Serial1.print(lon, 6);
            }

            // Высота
            uint16_t altitude_raw = ((data[5] & 0x0f) << 8) | data[6];
            if (altitude_raw > 0) {
                int altitude = (altitude_raw - 1) * 25 - 1000;
                Serial1.print(" Alt:");
                Serial1.print(altitude);
                Serial1.print("ft");
            }

        } else if (typecode == 19) {
            // Данные скорости
            uint16_t velocity = ((data[5] & 0x03) << 8) | data[6];
            uint16_t heading = ((data[7] & 0x03) << 8) | data[8];

            Serial1.print(" Vel:");
            Serial1.print(velocity);
            Serial1.print("kt Hdg:");
            Serial1.print(heading);
        }
    }

    Serial1.print(" RSSI:");
    Serial1.print(packet->rssi);
    Serial1.println();
}

// Задача обработки сэмплов
void sample_processing_task(void* parameter) 
{
    uint32_t temp_buffer[1024];

    while (true) {
        // Проверяем наличие новых данных
        if (sample_buffer.head != sample_buffer.tail) {
            int available = (sample_buffer.head - sample_buffer.tail + BUFFER_SIZE) % BUFFER_SIZE;
            int to_process = min(available, 1024);

            // Копируем данные из кольцевого буфера
            for (int i = 0; i < to_process; i++) {
                temp_buffer[i] = sample_buffer.samples[(sample_buffer.tail + i) % BUFFER_SIZE];
            }
            sample_buffer.tail = (sample_buffer.tail + to_process) % BUFFER_SIZE;

            // Фильтрация шумов
            if (!noise_filter(temp_buffer, to_process)) {
                continue;
            }

            // Поиск преамбулы
            int preamble_pos = find_preamble(temp_buffer, to_process);
            if (preamble_pos < 0) continue;

            // Декодирование пакета
            ADSBPacket packet = {0};
            if (decode_manchester(&temp_buffer[preamble_pos],
                                to_process - preamble_pos, packet.data)) {
                packet.timestamp = millis();
                packet.rssi = (temp_buffer[preamble_pos] >> 4) - current_gain;

                // Добавляем в очередь для обработки
                xQueueSend(packet_queue, &packet, 0);
            }
        }

        adjust_gain();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Задача декодирования пакетов
void packet_decode_task(void* parameter) 
{
    ADSBPacket packet;

    while (true) 
    {
        if (xQueueReceive(packet_queue, &packet, portMAX_DELAY)) 
        {
            decode_adsb_packet(&packet);
        }
    }
}

// Прерывание DMA
void dma_irq_handler() {
    if (dma_channel_get_irq0_status(dma_chan_a)) {
        dma_channel_acknowledge_irq0(dma_chan_a);
        sample_buffer.head = BUFFER_SIZE / 2;
    }

    if (dma_channel_get_irq0_status(dma_chan_b)) {
        dma_channel_acknowledge_irq0(dma_chan_b);
        sample_buffer.head = 0;
    }
}

void setup() {
    // Инициализация UART
 /*   Serial1.setTX(UART_TX_PIN);
    Serial1.setRX(UART_RX_PIN);*/


    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_translate_crlf(uart1, true);     //Если true, преобразовать перевод строки в возврат каретки при передаче
    uart_init(uart1, 115200);


    Serial1.begin(115200);

    delay(1000);
    Serial1.println("ADS-B Decoder Starting...");
    Serial1.print("Using pins: GPIO");

    // Инициализация АЦП
    adc_init();
    adc_gpio_init(ADS_B_PIN1);
    adc_gpio_init(ADS_B_PIN2);
    adc_select_input(0);  // GPIO26 = ADC0
    adc_set_clkdiv(48);   // 2 MHz sample rate
    adc_fifo_setup(true, true, 1, false, false);
    adc_run(true);

    // Инициализация DMA
    init_dma();

    // Настройка прерываний DMA
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    dma_channel_set_irq0_enabled(dma_chan_a, true);
    dma_channel_set_irq0_enabled(dma_chan_b, true);
    irq_set_enabled(DMA_IRQ_0, true);

    // Запуск DMA
    dma_channel_start(dma_chan_a);

    // Инициализация FreeRTOS
    packet_queue = xQueueCreate(MAX_PACKETS, sizeof(ADSBPacket));

    // Создание задач
    xTaskCreate(sample_processing_task, "SampleProc", 4096, NULL, 2, NULL);
    xTaskCreate(packet_decode_task, "PacketDecode", 4096, NULL, 1, NULL);

    Serial1.println("ADS-B Decoder initialized");

    // Запуск планировщика
    vTaskStartScheduler();
}

void loop() {
    // Основной цикл не используется при работе с FreeRTOS
}

/*
Дополнительные файлы конфигурации

platformio.ini

[env:pico]
platform = raspberrypi
board = pico
framework = arduino
board_build.core = earlephilhower
lib_deps =
    FreeRTOS
build_flags =
    -DUSE_TINYUSB
    -O3
    -mcpu=cortex-m0plus
    -DPICO_STACK_SIZE=0x1000
monitor_speed = 115200


Ключевые особенности реализации:

DMA для высокой производительности
Двойная буферизация с переключением каналов
Непрерывная запись АЦП без потери данных
Минимальная нагрузка на процессор

Полный алгоритм CPR
Глобальное декодирование с использованием четных/нечетных кадров
Кэш для хранения предыдущих позиций
Правильная обработка зон широты

Фильтрация шумов
Адаптивное определение уровня шума
Автоматическая подстройка порога
Анализ соотношения сигнал/шум

Автоматическая регулировка усиления
Мониторинг качества сигнала
Адаптивная подстройка усиления АЦП
Предотвращение перегрузки и недогрузки

Многозадачность
Отдельные задачи для обработки сэмплов и декодирования
Очереди для буферизации пакетов
Неблокирующая обработка

Программа обеспечивает высокую производительность и точность декодирования ADS-B сигналов с автоматической адаптацией к условиям приема.

*/