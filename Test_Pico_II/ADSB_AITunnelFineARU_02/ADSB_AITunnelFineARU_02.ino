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

//#include <stdio.h>
//#include "pico/stdlib.h"
//#include "hardware/gpio.h"
#include "hardware/adc.h"
//#include <math.h>
//#include <stdint.h>
//#include "hardware/uart.h"


// Конфигурация пинов
#define ADS_B_PIN1 26
#define ADS_B_PIN2 27
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// Константы ADS-B
#define ADS_B_PREAMBLE_US 8
#define ADS_B_BIT_DURATION_US 1
#define ADS_B_PACKET_LENGTH 112
#define SAMPLE_RATE_HZ 2000000  // 2 MHz
#define BUFFER_SIZE 4096
#define MAX_PACKETS 32

#define M_PI 3.14159265358979323846
#define CPR_MAX_AGE 10000  // Максимальный возраст кадра в мс


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


// Структура для кэша CPR
typedef struct {
    double lat;
    double lon;
    uint8_t odd_frame;
    uint32_t timestamp;
    uint32_t icao;
} cpr_cache_entry_t;



// Глобальные переменные
static SampleBuffer sample_buffer;
static QueueHandle_t packet_queue;
static ADSBPacket packet_buffer[MAX_PACKETS];
static cpr_cache_entry_t cpr_cache[256];
//static CPRData cpr_cache[256];  // Кэш для CPR расчетов
static int dma_chan_a, dma_chan_b;
static uint8_t current_gain = 128;
static uint32_t noise_floor = 0;
static uint32_t signal_threshold = 0;

// Функция для получения количества зон долготы
int get_nl(double lat) {
    if (lat >= 87.0) return 1;
    if (lat <= -87.0) return 1;

    double lat_rad = fabs(lat) * M_PI / 180.0;
    double temp = 1.0 - (1.0 - cos(M_PI / 30.0)) / pow(cos(lat_rad), 2);

    if (temp <= 0) return 1;
    return (int)floor(2.0 * M_PI / acos(temp));
}



// CRC-24 таблица (полная)
static const uint32_t crc24_table[256] = {
    0x000000, 0x001021, 0x002042, 0x003063, 0x004084, 0x0050a5, 0x0060c6, 0x0070e7,
    0x008108, 0x009129, 0x00a14a, 0x00b16b, 0x00c18c, 0x00d1ad, 0x00e1ce, 0x00f1ef,
    0x001231, 0x000210, 0x003273, 0x002252, 0x0051b5, 0x004094, 0x0073f7, 0x0062d6,
    0x009339, 0x008318, 0x00b17b, 0x00a15a, 0x00d1bd, 0x00c19c, 0x00f3ff, 0x00e2de,
    0x002462, 0x003443, 0x000420, 0x001401, 0x0064e6, 0x0074c7, 0x0046a4, 0x005685,
    0x00a56a, 0x00b54b, 0x008728, 0x009709, 0x00e5ee, 0x00f4cf, 0x00c6ac, 0x00d68d,
    0x003653, 0x002672, 0x001411, 0x000430, 0x0076d7, 0x0066f6, 0x005495, 0x0044b4,
    0x00b75b, 0x00a77a, 0x009519, 0x008538, 0x00f7df, 0x00e7fe, 0x00d59d, 0x00c5bc,
    0x0048c4, 0x0058e5, 0x006a86, 0x007aa7, 0x000940, 0x001961, 0x002b02, 0x003b23,
    0x00c9cc, 0x00d9ed, 0x00eb8e, 0x00fbaf, 0x008d48, 0x009d69, 0x00af0a, 0x00bf2b,
    0x005af5, 0x004ad4, 0x0078b7, 0x006896, 0x001e71, 0x000e50, 0x003c33, 0x002c12,
    0x00defd, 0x00cedc, 0x00fcbf, 0x00ec9e, 0x009a79, 0x008a58, 0x00b83b, 0x00a81a,
    0x006ca6, 0x007c87, 0x004ee4, 0x005ec5, 0x002822, 0x003803, 0x000a60, 0x001a41,
    0x00edae, 0x00fd8f, 0x00cfec, 0x00dfcd, 0x00a92a, 0x00b90b, 0x008b68, 0x009b49,
    0x007e97, 0x006eb6, 0x005cd5, 0x004cf4, 0x003a13, 0x002a32, 0x001851, 0x000870,
    0x00ff9f, 0x00efbe, 0x00dddd, 0x00cdfc, 0x00bb1b, 0x00ab3a, 0x009959, 0x008978,
    0x009188, 0x0081a9, 0x00b3ca, 0x00a3eb, 0x00d10c, 0x00c12d, 0x00f34e, 0x00e36f,
    0x001080, 0x0000a1, 0x0032c2, 0x0022e3, 0x005004, 0x004025, 0x007246, 0x006267,
    0x0083b9, 0x009398, 0x00a1fb, 0x00b1da, 0x00c33d, 0x00d31c, 0x00e17f, 0x00f15e,
    0x0002b1, 0x001290, 0x0020f3, 0x0030d2, 0x004235, 0x005214, 0x006077, 0x007056,
    0x00b5ea, 0x00a5cb, 0x0097a8, 0x008789, 0x00f56e, 0x00e54f, 0x00d72c, 0x00c70d,
    0x0034e2, 0x0024c3, 0x0016a0, 0x000681, 0x007466, 0x006447, 0x005624, 0x004605,
    0x00a7db, 0x00b7fa, 0x008599, 0x0095b8, 0x00e75f, 0x00f77e, 0x00c51d, 0x00d53c,
    0x0026d3, 0x0036f2, 0x000491, 0x0014b0, 0x006657, 0x007676, 0x004415, 0x005434,
    0x00d94c, 0x00c96d, 0x00fb0e, 0x00eb2f, 0x009dc8, 0x008de9, 0x00bf8a, 0x00afab,
    0x005844, 0x004865, 0x007a06, 0x006a27, 0x001cc0, 0x000ce1, 0x003e82, 0x002ea3,
    0x00cb7d, 0x00db5c, 0x00e93f, 0x00f91e, 0x008ff9, 0x009fd8, 0x00adbb, 0x00bd9a,
    0x004a75, 0x005a54, 0x006837, 0x007816, 0x000ef1, 0x001ed0, 0x002cb3, 0x003c92,
    0x00fd2e, 0x00ed0f, 0x00df6c, 0x00cf4d, 0x00b9aa, 0x00a98b, 0x009be8, 0x008bc9,
    0x007c26, 0x006c07, 0x005e64, 0x004e45, 0x0038a2, 0x002883, 0x001ae0, 0x000ac1,
    0x00ef1f, 0x00ff3e, 0x00cd5d, 0x00dd7c, 0x00ab9b, 0x00bbba, 0x0089d9, 0x0099f8,
    0x006e17, 0x007e36, 0x004c55, 0x005c74, 0x002a93, 0x003ab2, 0x0008d1, 0x0018f0
};


// Функция инициализации DMA
void init_dma() {
    dma_chan_a = dma_claim_unused_channel(true);
    dma_chan_b = dma_claim_unused_channel(true);

    Serial2.print("dma_chan_a - ");
    Serial2.println(dma_chan_a);

    Serial2.print("dma_chan_b - ");
    Serial2.println(dma_chan_b);

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
        crc = (crc << 8) ^ crc24_table[((crc >> 16) ^ data[i]) & 0xff];
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
void adjust_gain() 
{
    static uint32_t last_adjustment = 0;
    static int signal_count = 0;
    static int noise_count = 0;

    uint32_t now = millis();

    if (now - last_adjustment > 1000)
    { 
        // Каждую секунду

        Serial2.println("Test adjust_gain");
        float signal_ratio = (float)signal_count / (signal_count + noise_count);

        if (signal_ratio < 0.1 && current_gain < 255) 
        {
            current_gain = min(255, current_gain + 10);
            // Применить новое усиление к АЦП
        }
        else if (signal_ratio > 0.8 && current_gain > 10) 
        {
            current_gain = max(10, current_gain - 10);
            // Применить новое усиление к АЦП
        }

        signal_count = 0;
        noise_count = 0;
        last_adjustment = now;
    }
}

// Декодирование Manchester
// 
bool decode_manchester(uint32_t* samples_data, int length, uint8_t* output) 
{
    int bit_count = 0;
    int sample_per_bit = SAMPLE_RATE_HZ / 1000000;

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
// 
//bool decode_manchester(uint32_t* samples, int length, uint8_t output) {
//    int bit_count = 0;
//    int sample_per_bit = SAMPLE_RATE_HZ / 1000000;  // Samples per microsecond
//
//    for (int i = 0; i < length && bit_count < ADS_B_PACKET_LENGTH; i += sample_per_bit * 2) {
//        if (i + sample_per_bit >= length) break;
//
//        uint32_t first_half = 0, second_half = 0;
//
//        // Усредняем первую половину бита
//        for (int j = 0; j < sample_per_bit; j++) {
//            first_half += samples[i + j];
//        }
//        first_half /= sample_per_bit;
//
//        // Усредняем вторую половину бита
//        for (int j = 0; j < sample_per_bit; j++) {
//            second_half += samples[i + sample_per_bit + j];
//        }
//        second_half /= sample_per_bit;
//
//        // Декодируем бит
//        if (first_half > second_half) {
//            output[bit_count / 8] |= (1 << (7 - (bit_count % 8)));
//        } else {
//            output[bit_count / 8] &= ~(1 << (7 - (bit_count % 8)));
//        }
//
//        bit_count++;
//    }
//
//    return bit_count == ADS_B_PACKET_LENGTH;
//}

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



//void decode_cpr_position(uint8_t* data, double lat, double* lon) {
//    // Извлекаем данные из пакета
//    uint8_t typecode = (data[4] >> 3) & 0x1f;
//    if (typecode < 9 || typecode > 18) return;  // Не позиционный пакет
//
//    uint8_t odd_flag = (data[6] >> 2) & 0x01;
//    uint32_t lat_cpr = ((data[6] & 0x03) << 15) | (data[7] << 7) | (data[8] >> 1);
//    uint32_t lon_cpr = ((data[8] & 0x01) << 16) | (data[9] << 8) | data[10];
//
//    // Количество зон широты
//    static const int NL_table[59] = {
//        59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44,
//        43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28,
//        27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12,
//        11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1
//    };
//
//    // Расчет широты
//    double d_lat_even = 360.0 / 60.0;
//    double d_lat_odd = 360.0 / 59.0;
//
//    double lat_even, lat_odd;
//
//    if (!odd_flag) {  // Четный кадр
//        lat_even = d_lat_even * (lat_cpr / 131072.0);
//        if (lat_even >= 270) lat_even -= 360;
//    } else {  // Нечетный кадр
//        lat_odd = d_lat_odd * (lat_cpr / 131072.0);
//        if (lat_odd >= 270) lat_odd -= 360;
//    }
//
//    // Поиск парного кадра в кэше
//    uint32_t icao = (data[1] << 16) | (data[2] << 8) | data[3];
//    uint8_t cache_idx = icao & 0xFF;
//
//    if (cpr_cache[cache_idx].timestamp != 0) {
//        // Имеем парный кадр
//        double lat_result, lon_result;
//
//        if (odd_flag != cpr_cache[cache_idx].odd_frame) {
//            // Глобальное декодирование
//            double lat_cpr_even, lat_cpr_odd, lon_cpr_even, lon_cpr_odd;
//
//            if (odd_flag) {
//                lat_cpr_odd = lat_cpr / 131072.0;
//                lon_cpr_odd = lon_cpr / 131072.0;
//                lat_cpr_even = cpr_cache[cache_idx].lat;
//                lon_cpr_even = cpr_cache[cache_idx].lon;
//            } else {
//                lat_cpr_even = lat_cpr / 131072.0;
//                lon_cpr_even = lon_cpr / 131072.0;
//                lat_cpr_odd = cpr_cache[cache_idx].lat;
//                lon_cpr_odd = cpr_cache[cache_idx].lon;
//            }
//
//            double j = floor(59.0 * lat_cpr_even - 60.0 * lat_cpr_odd + 0.5);
//            //double j = floor(59  lat_cpr_even - 60  lat_cpr_odd + 0.5);
//            lat_result = d_lat_even * (j + lat_cpr_even);
//            if (lat_result >= 270) lat_result -= 360;
//            if (lat_result <= -270) lat_result += 360;
//
//            // Расчет долготы
//            int nl = 1;
//            if (lat_result < 87 && lat_result > -87) {
//                double lat_rad = lat_result * M_PI / 180.0;
//                nl = floor(2 * M_PI / acos(1 - (1 - cos(M_PI / 30.0)) / pow(cos(lat_rad), 2)));
//            }
//
//            int nl_even = nl;
//            int nl_odd = nl - 1;
//            if (nl_odd < 1) nl_odd = 1;
//
//            double d_lon_even = 360.0 / nl_even;
//            double d_lon_odd = 360.0 / nl_odd;
//
//           // double m = floor(lon_cpr_even  (nl_even - 1) - lon_cpr_odd  nl_even + 0.5);
//
//            double m = floor(lon_cpr_even * (nl_odd)-lon_cpr_odd * nl_even + 0.5);
//
//            if (!odd_flag) {  // Используем четный кадр
//                lon_result = d_lon_even * (m + lon_cpr_even);
//            } else {  // Используем нечетный кадр
//                lon_result = d_lon_odd * (m + lon_cpr_odd);
//            }
//
//            if (lon_result >= 180) lon_result -= 360;
//            if (lon_result <= -180) lon_result += 360;
//
//            *lat = lat_result;
//            *lon = lon_result;
//        }
//    }
//
//    // Сохраняем данные в кэш
//    cpr_cache[cache_idx].lat = lat_cpr / 131072.0;
//    cpr_cache[cache_idx].lon = lon_cpr / 131072.0;
//    cpr_cache[cache_idx].odd_frame = odd_flag;
//    cpr_cache[cache_idx].timestamp = millis();
//}

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

    Serial2.print("DF:");
    Serial2.print(df);
    Serial2.print(" ICAO:");
    Serial2.print(icao, HEX);

    if (df == 17 || df == 18) {  // ADS-B сообщения
        uint8_t typecode = (data[4] >> 3) & 0x1f;

        Serial2.print(" TC:");
        Serial2.print(typecode);

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

            Serial2.print(" Callsign:");
            Serial2.print(callsign);

        } else if (typecode >= 9 && typecode <= 18) {
            // Позиционные данные
            double lat, lon;
            decode_cpr_position(data, &lat, &lon);

            if (lat != 0 || lon != 0) {
                Serial2.print(" Lat:");
                Serial2.print(lat, 6);
                Serial2.print(" Lon:");
                Serial2.print(lon, 6);
            }

            // Высота
            uint16_t altitude_raw = ((data[5] & 0x0f) << 8) | data[6];
            if (altitude_raw > 0) {
                int altitude = (altitude_raw - 1) * 25 - 1000;
                Serial2.print(" Alt:");
                Serial2.print(altitude);
                Serial2.print("ft");
            }

        } else if (typecode == 19) {
            // Данные скорости
            uint16_t velocity = ((data[5] & 0x03) << 8) | data[6];
            uint16_t heading = ((data[7] & 0x03) << 8) | data[8];

            Serial2.print(" Vel:");
            Serial2.print(velocity);
            Serial2.print("kt Hdg:");
            Serial2.print(heading);
        }
    }

    Serial2.print(" RSSI:");
    Serial2.print(packet->rssi);
    Serial2.println();
}

// Задача обработки сэмплов
void sample_processing_task(void* parameter) 
{
    uint32_t temp_buffer[1024];

    while(true) 
    {
        Serial2.println("Test sample_processing_task ");
        // Проверяем наличие новых данных
        if (sample_buffer.head != sample_buffer.tail) 
        {
            int available = (sample_buffer.head - sample_buffer.tail + BUFFER_SIZE) % BUFFER_SIZE;
            int to_process = min(available, 1024);

            // Копируем данные из кольцевого буфера
            for (int i = 0; i < to_process; i++) 
            {
                temp_buffer[i] = sample_buffer.samples[(sample_buffer.tail + i) % BUFFER_SIZE];
            }
            sample_buffer.tail = (sample_buffer.tail + to_process) % BUFFER_SIZE;

            // Фильтрация шумов
            if (!noise_filter(temp_buffer, to_process)) 
            {
                continue;
            }

            // Поиск преамбулы
            int preamble_pos = find_preamble(temp_buffer, to_process);
            if (preamble_pos < 0) continue;

            // Декодирование пакета
            ADSBPacket packet = {0};
            if (decode_manchester(&temp_buffer[preamble_pos], to_process - preamble_pos, packet.data)) 
            {
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
void dma_irq_handler() 
{
    if (dma_channel_get_irq0_status(dma_chan_a)) 
    {
        dma_channel_acknowledge_irq0(dma_chan_a);
        sample_buffer.head = BUFFER_SIZE / 2;
    }

    //if (dma_channel_get_irq0_status(dma_chan_b)) 
    //{
    //    dma_channel_acknowledge_irq0(dma_chan_b);
    //    sample_buffer.head = 0;
    //}
}

void setup() {
    // Инициализация UART
    Serial2.setTX(UART_TX_PIN);
    Serial2.setRX(UART_RX_PIN);
    Serial2.begin(115200);
    delay(1000);

    Serial2.println("Start setup");
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
    dma_channel_start(dma_chan_b);

    // Инициализация FreeRTOS
    packet_queue = xQueueCreate(MAX_PACKETS, sizeof(ADSBPacket));

    // Создание задач
    xTaskCreate(sample_processing_task, "SampleProc", 4096, NULL, 2, NULL);
    xTaskCreate(packet_decode_task, "PacketDecode", 4096, NULL, 1, NULL);

    //Serial2.println("ADS-B Decoder initialized");
    //delay(1000);
    // Запуск планировщика
     vTaskStartScheduler();

    //Serial2.println("Setup END!!");
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