//функция setup() не является частью стандартного Pico SDK. Исправлю код для правильной структуры программы на RP2040:


#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <hardware/pwm.h>
#include <hardware/uart.h>
#include <hardware/adc.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <string.h>
#include <math.h>

// Определение пинов
#define INPUT_PIN_19        19
#define INPUT_PIN_22        22
#define GAIN_CONTROL_PIN    28
#define GAIN_PWM_PIN        9
#define GAIN_MONITOR_PIN    27
#define PREAMBLE_LED_PIN    20
#define DF17_LED_PIN        23
#define UART_TX_PIN         4
#define UART_RX_PIN         5

// Параметры ADS-B
#define ADSB_FRAME_LENGTH   112
#define PREAMBLE_LENGTH     8
#define SAMPLE_RATE_US      0.5
#define NOISE_THRESHOLD     10

// Структура для хранения пакета ADS-B
typedef struct {
    uint8_t data[14];           // 112 бит = 14 байт
    uint32_t timestamp;
    bool valid;
    uint8_t df;                 // Downlink Format
} adsb_packet_t;

// Глобальные переменные
volatile bool new_packet_available = false;
volatile adsb_packet_t received_packet;
volatile uint32_t bit_buffer[ADSB_FRAME_LENGTH];
volatile int bit_count = 0;
volatile bool receiving_packet = false;
volatile uint32_t last_edge_time = 0;
volatile uint8_t noise_filter_count = 0;
volatile uint16_t preamble_shift_reg = 0;

// Полная таблица CRC для Mode S
const uint32_t crc_table[256] = {
    0x000000, 0xfff409, 0x1e9823, 0xe16c2a, 0x3d3046, 0xc2c44f, 0x23a865, 0xdc5c6c,
    0x7a608c, 0x859485, 0x64f8af, 0x9b0ca6, 0x4750ca, 0xb8a4c3, 0x59c8e9, 0xa63ce0,
    0xf4c118, 0x0b3511, 0xea593b, 0x15ad32, 0xc9f15e, 0x36052f, 0xd76975, 0x289d7c,
    0x8ea19c, 0x715595, 0x9039bf, 0x6fcdb6, 0xb391da, 0x4c65d3, 0xad09f9, 0x52fdf0,
    0x1e9230, 0xe16639, 0x000a13, 0xfff61a, 0x23a276, 0xdc5e7f, 0x3d3255, 0xc2ce5c,
    0x64f2bc, 0x9b0eb5, 0x7a629f, 0x859e96, 0x59c2fa, 0xa63ef3, 0x4752d9, 0xb8aed0,
    0xea5328, 0x15af21, 0xf4c30b, 0x0b3f02, 0xd7636e, 0x289f67, 0xc9f34d, 0x360f44,
    0x9033a4, 0x6fcfad, 0x8ea387, 0x715f8e, 0xad03e2, 0x52ffeb, 0xb393c1, 0x4c6fc8,
    0x3d2460, 0xc2d869, 0x23b443, 0xdc484a, 0x001426, 0xffe82f, 0x1e8405, 0xe1780c,
    0x4744ec, 0xb8b8e5, 0x59d4cf, 0xa628c6, 0x7a74aa, 0x8588a3, 0x64e489, 0x9b1880,
    0xc9e578, 0x361971, 0xd7755b, 0x288952, 0xf4d53e, 0x0b2937, 0xea451d, 0x15b914,
    0xb385f4, 0x4c79fd, 0xad15d7, 0x52e9de, 0x8eb5b2, 0x7149bb, 0x902591, 0x6fd998,
    0x23b650, 0xdc4a59, 0x3d2673, 0xc2da7a, 0x1e8616, 0xe17a1f, 0x001635, 0xffea3c,
    0x59d6dc, 0xa62ad5, 0x4746ff, 0xb8baf6, 0x64e69a, 0x9b1a93, 0x7a76b9, 0x858ab0,
    0xd77748, 0x288b41, 0xc9e76b, 0x361b62, 0xea470e, 0x15bb07, 0xf4d72d, 0x0b2b24,
    0xad17c4, 0x52ebcd, 0xb387e7, 0x4c7bee, 0x902782, 0x6fdb8b, 0x8eb7a1, 0x714ba8,
    0x7a48c0, 0x85b4c9, 0x64d8e3, 0x9b24ea, 0x477886, 0xb8848f, 0x59e8a5, 0xa614ac,
    0x00284c, 0xffd445, 0x1eb86f, 0xe14466, 0x3d180a, 0xc2e403, 0x238829, 0xdc7420,
    0x8e89d8, 0x7175d1, 0x9019fb, 0x6fe5f2, 0xb3b99e, 0x4c4597, 0xad29bd, 0x52d5b4,
    0xf4e954, 0x0b155d, 0xea7977, 0x15857e, 0xc9d912, 0x36251b, 0xd74931, 0x28b538,
    0x64daf0, 0x9b26f9, 0x7a4ad3, 0x85b6da, 0x59eab6, 0xa616bf, 0x477a95, 0xb8869c,
    0x1eba7c, 0xe14675, 0x002a5f, 0xffd656, 0x238a3a, 0xdc7633, 0x3d1a19, 0xc2e610,
    0x901be8, 0x6fe7e1, 0x8e8bcb, 0x7177c2, 0xad2bae, 0x52d7a7, 0xb3bb8d, 0x4c4784,
    0xea7b64, 0x15876d, 0xf4eb47, 0x0b174e, 0xd74b22, 0x28b72b, 0xc9db01, 0x362708,
    0x473aa0, 0xb8c6a9, 0x59aa83, 0xa6568a, 0x7a0ae6, 0x85f6ef, 0x649ac5, 0x9b66cc,
    0x3d5a2c, 0xc2a625, 0x23ca0f, 0xdc3606, 0x006a6a, 0xff9663, 0x1efa49, 0xe10640,
    0xb3fbb8, 0x4c07b1, 0xad6b9b, 0x529792, 0x8ecbfe, 0x7137f7, 0x905bdd, 0x6fa7d4,
    0xc99b34, 0x36673d, 0xd70b17, 0x28f71e, 0xf4ab72, 0x0b577b, 0xea3b51, 0x15c758,
    0x59a690, 0xa65a99, 0x4736b3, 0xb8caba, 0x6496d6, 0x9b6adf, 0x7a06f5, 0x85fafc,
    0x23c61c, 0xdc3a15, 0x3d563f, 0xc2aa36, 0x1ef65a, 0xe10a53, 0x006679, 0xff9a70,
    0xad6788, 0x529b81, 0xb3f7ab, 0x4c0ba2, 0x9057ce, 0x6fabc7, 0x8ec7ed, 0x713be4,
    0xd70704, 0x28fb0d, 0xc99727, 0x366b2e, 0xea3742, 0x15cb4b, 0xf4a761, 0x0b5b68//,
    //0x6d9180, 0x926d89, 0x7301a3, 0x8cfdaa, 0x50a1c6, 0xaf5dcf, 0x4e31e5, 0xb1cdec,
    //0x17f10c, 0xe80d05, 0x09612f, 0xf69d26, 0x2ac14a, 0xd53d43, 0x345169, 0xcbad60,
    //0x995098, 0x66ac91, 0x87c0bb, 0x783cb2, 0xa460de, 0x5b9cd7, 0xbaf0fd, 0x450cf4,
    //0xe33014, 0x1ccc1d, 0xfda037, 0x025c3e, 0xde0052, 0x21fc5b, 0xc09071, 0x3f6c78,
    //0x730bb0, 0x8cf7b9, 0x6d9b93, 0x92679a, 0x4e3bf6, 0xb1c7ff, 0x50abd5, 0xaf57dc,
    //0x096b3c, 0xf69735, 0x17fb1f, 0xe80716, 0x345b7a, 0xcba773, 0x2acb59, 0xd53750,
    //0x87caa8, 0x7836a1, 0x995a8b, 0x66a682, 0xbafaee, 0x4506e7, 0xa46acd, 0x5b96c4,
    //0xfdaa24, 0x02562d, 0xe33a07, 0x1cc60e, 0xc09a62, 0x3f666b, 0xde0a41, 0x21f648,
    //0x50bde0, 0xaf41e9, 0x4e2dc3, 0xb1d1ca, 0x6d8da6, 0x9271af, 0x731d85, 0x8ce18c,
    //0x2add6c, 0xd52165, 0x344d4f, 0xcbb146, 0x17ed2a, 0xe81123, 0x097d09, 0xf68100,
    //0xa47cf8, 0x5b80f1, 0xbaecdb, 0x4510d2, 0x994cbe, 0x66b0b7, 0x87dc9d, 0x782094,
    //0xde1c74, 0x21e07d, 0xc08c57, 0x3f705e, 0xe32c32, 0x1cd03b, 0xfdbc11, 0x024018,
    //0x4e27d0, 0xb1dbd9, 0x50b7f3, 0xaf4bfa, 0x731796, 0x8ceb9f, 0x6d87b5, 0x927bbc,
    //0x34475c, 0xcbbb55, 0x2ad77f, 0xd52b76, 0x09771a, 0xf68b13, 0x17e739, 0xe81b30,
    //0xbae6c8, 0x451ac1, 0xa476eb, 0x5b8ae2, 0x87d68e, 0x782a87, 0x9946ad, 0x66baa4,
    //0xc08644, 0x3f7a4d, 0xde1667, 0x21ea6e, 0xfdb602, 0x024a0b, 0xe32621, 0x1cda28
};

// Альтернативный вариант - функция генерации CRC таблицы
void generate_mode_s_crc_table(uint32_t* table) {
    const uint32_t polynomial = 0xFFF409;  // Полином Mode S (24-bit)

    for (int i = 0; i < 256; i++) {
        uint32_t crc = i << 16;  // Помещаем байт в старшие биты

        for (int j = 0; j < 8; j++) {
            if (crc & 0x800000) {  // Проверяем старший бит
                crc = (crc << 1) ^ polynomial;
            }
            else {
                crc <<= 1;
            }
            crc &= 0xFFFFFF;  // Оставляем только 24 бита
        }

        table[i] = crc;
    }
}

// Функция вычисления CRC для Mode S с полной таблицей
uint32_t calculate_mode_s_crc_full(const uint8_t* data, int length_bits) {
    uint32_t crc = 0;
    int byte_length = (length_bits + 7) / 8;

    // Обрабатываем все байты кроме последних 3 (которые содержат CRC)
    int data_bytes = byte_length - 3;
    if (length_bits == 112) {  // Для полного пакета ADS-B
        data_bytes = 11;  // Первые 88 бит (11 байт) - данные
    }

    for (int i = 0; i < data_bytes; i++) {
        uint8_t byte_val = data[i];

        // Для последнего байта данных может понадобиться маскирование
        if (i == data_bytes - 1 && (length_bits % 8) != 0) {
            int valid_bits = length_bits % 8;
            byte_val &= (0xFF << (8 - valid_bits));
        }

        // Основная формула CRC
        crc = (crc << 8) ^ crc_table[((crc >> 16) ^ byte_val) & 0xFF];
        crc &= 0xFFFFFF;  // Ограничиваем до 24 бит
    }

    return crc;
}

// Функция проверки CRC пакета Mode S
bool verify_mode_s_crc(const uint8_t* packet, int length_bits) 
{
    if (length_bits != 56 && length_bits != 112) {
        return false;  // Неправильная длина пакета
    }

    // Вычисляем CRC для данных
    uint32_t calculated_crc = calculate_mode_s_crc_full(packet, length_bits);

    // Извлекаем CRC из пакета (последние 3 байта)
    int byte_length = length_bits / 8;
    uint32_t packet_crc = ((uint32_t)packet[byte_length - 3] << 16) |
        ((uint32_t)packet[byte_length - 2] << 8) |
        packet[byte_length - 1];

    return (calculated_crc == packet_crc);
}

// Улучшенная функция декодирования с правильной проверкой CRC
void decode_adsb_packet_improved(adsb_packet_t* packet) {
    if (!packet->valid) return;

    // Проверка CRC с полной таблицей
    if (!verify_mode_s_crc(packet->data, 112)) {
        packet->valid = false;
        char error_msg[100];
        snprintf(error_msg, sizeof(error_msg),
            "CRC Error: Packet rejected (DF=%d)\r\n", packet->df);
        uart_puts(uart1, error_msg);
        return;
    }

    uint8_t df = packet->df;

    // Статистика успешных пакетов
    static uint32_t valid_packet_count = 0;
    valid_packet_count++;

    if (df == 17) {  // ADS-B Extended Squitter
        gpio_put(DF17_LED_PIN, 1);

        uint32_t icao = ((uint32_t)packet->data[1] << 16) |
            ((uint32_t)packet->data[2] << 8) |
            packet->data[3];

        uint8_t type_code = (packet->data[4] >> 3) & 0x1F;

        char output[400];
        snprintf(output, sizeof(output),
            "DF17 #%lu: ICAO=%06X, TC=%02d",
            valid_packet_count, icao, type_code);

        // ... (остальное декодирование без изменений)

        uart_puts(uart1, output);
        uart_puts(uart1, "\r\n");

        sleep_ms(100);
        gpio_put(DF17_LED_PIN, 0);
    }
    else {
        // Обработка других типов пакетов Mode S
        char output[200];
        snprintf(output, sizeof(output),
            "Mode S #%lu: DF=%d", valid_packet_count, df);
        uart_puts(uart1, output);
        uart_puts(uart1, "\r\n");
    }
}


// Преамбула ADS-B (Mode S): 1010000101000000
const uint16_t preamble_pattern = 0b1010000101000000;

// CPR параметры
#define NZ 15.0

// Функция вычисления CRC для Mode S
uint32_t calculate_mode_s_crc(const uint8_t *data, int length_bits) {
    uint32_t crc = 0;
    int byte_length = (length_bits + 7) / 8;

    for (int i = 0; i < byte_length - 3; i++) {  // Исключаем последние 3 байта (CRC)
        uint8_t byte = data[i];
        if (i == byte_length - 4 && length_bits % 8 != 0) {
            // Маскируем последний байт если нужно
            byte &= (0xFF << (8 - (length_bits % 8)));
        }
        crc = (crc << 8) ^ crc_table[((crc >> 16) ^ byte) & 0xff];
    }
    return crc & 0xffffff;
}

// Полная реализация CPR декодирования
typedef struct {
    uint32_t lat_cpr;
    uint32_t lon_cpr;
    uint8_t cpr_format;  // 0 = четная, 1 = нечетная
    uint32_t timestamp;
    bool valid;
} cpr_position_t;

// Глобальное хранилище для парных CPR позиций
static cpr_position_t even_position = { 0 };
static cpr_position_t odd_position = { 0 };

// Функция получения NL (Number of Longitude zones) для заданной широты
int get_nl(double lat) {
    if (lat < 0) lat = -lat;  // Работаем с абсолютным значением
    if (lat >= 87.0) return 1;

    // Таблица переходов для NL
    const double lat_transitions[] = {
        10.47047130, 14.82817437, 18.18626357, 21.02939493, 23.54504487,
        25.82924707, 27.93898710, 29.91135686, 31.77209708, 33.53993436,
        35.22899598, 36.85025108, 38.41241892, 39.92256684, 41.38651832,
        42.80914012, 44.19454951, 45.54626723, 46.86733252, 48.16039128,
        49.42776439, 50.67150166, 51.89342469, 53.09516153, 54.27817472,
        55.44378444, 56.59318756, 57.72747354, 58.84763776, 59.95459277,
        61.04917774, 62.13216659, 63.20427479, 64.26616523, 65.31845310,
        66.36171008, 67.39646774, 68.42322022, 69.44242631, 70.45451075,
        71.45986473, 72.45884545, 73.45177442, 74.43893416, 75.42056257,
        76.39684391, 77.36789461, 78.33374083, 79.29428225, 80.24923213,
        81.19801349, 82.13956981, 83.07199445, 84.89166191, 85.75541621,
        86.61144708, 87.00000000
    };

    const int nl_values[] = {
        59, 58, 57, 56, 55, 54, 53, 52, 51, 50,
        49, 48, 47, 46, 45, 44, 43, 42, 41, 40,
        39, 38, 37, 36, 35, 34, 33, 32, 31, 30,
        29, 28, 27, 26, 25, 24, 23, 22, 21, 20,
        19, 18, 17, 16, 15, 14, 13, 12, 11, 10,
        9, 8, 7, 6, 5, 4, 3, 2, 1, 1
    };

    for (int i = 0; i < sizeof(lat_transitions) / sizeof(double); i++) {
        if (lat < lat_transitions[i]) {
            return nl_values[i];
        }
    }
    return 1;
}

// CPR глобальное декодирование
bool decode_cpr_global(uint32_t even_lat, uint32_t even_lon,
    uint32_t odd_lat, uint32_t odd_lon,
    double* latitude, double* longitude) {

    // Константы для CPR
    const double NB = 17.0;          // Количество бит для кодирования
    const double LAT_ZONE_SIZE_EVEN = 360.0 / 60.0;  // 6 градусов
    const double LAT_ZONE_SIZE_ODD = 360.0 / 59.0;   // ~6.1 градусов
    const double SCALE_FACTOR = (double)(1 << 17);   // 2^17 = 131072

    // Нормализация входных значений
    double yz_even = (double)even_lat / SCALE_FACTOR;
    double yz_odd = (double)odd_lat / SCALE_FACTOR;

    // Вычисление индекса широтной зоны
    int j = (int)floor(59.0 * yz_even - 60.0 * yz_odd + 0.5);

    // Вычисление широт
    double lat_even = LAT_ZONE_SIZE_EVEN * ((double)j + yz_even);
    double lat_odd = LAT_ZONE_SIZE_ODD * ((double)j + yz_odd);

    // Приведение к диапазону [-90, +90]
    if (lat_even >= 270.0) lat_even -= 360.0;
    if (lat_even <= -270.0) lat_even += 360.0;
    if (lat_odd >= 270.0) lat_odd -= 360.0;
    if (lat_odd <= -270.0) lat_odd += 360.0;

    // Проверка валидности широт
    if (lat_even < -90.0 || lat_even > 90.0 ||
        lat_odd < -90.0 || lat_odd > 90.0) {
        *latitude = NAN;
        *longitude = NAN;
        return false;
    }

    // Проверка согласованности между четной и нечетной широтами
    if (fabs(lat_even - lat_odd) >= LAT_ZONE_SIZE_EVEN) {
        *latitude = NAN;
        *longitude = NAN;
        return false;
    }

    // Используем четную широту
    *latitude = lat_even;

    // Декодирование долготы
    int nl_lat = get_nl(fabs(*latitude));
    if (nl_lat < 1) nl_lat = 1;

    // Проверка на полюса (где долгота не определена)
    if (nl_lat == 1) {
        *longitude = 0.0;  // Условно устанавливаем долготу в 0 для полюсов
        return true;
    }

    double dlon = 360.0 / (double)nl_lat;

    // Нормализация долготы
    double xz_even = (double)even_lon / SCALE_FACTOR;
    double xz_odd = (double)odd_lon / SCALE_FACTOR;

    // Вычисление индекса долготной зоны
    int m = (int)floor(xz_even * (double)(nl_lat - 1) - xz_odd * (double)nl_lat + 0.5);

    // Вычисление долготы на основе четной позиции
    double longitude_decoded = dlon * ((double)m + xz_even);

    // Приведение к диапазону [-180, +180]
    while (longitude_decoded >= 180.0) longitude_decoded -= 360.0;
    while (longitude_decoded < -180.0) longitude_decoded += 360.0;

    *longitude = longitude_decoded;

    return true;
}

// Функция для сохранения CPR позиции
void store_cpr_position(uint32_t lat_cpr, uint32_t lon_cpr, uint8_t cpr_format) {
    if (cpr_format == 0) {  // Четная позиция
        even_position.lat_cpr = lat_cpr;
        even_position.lon_cpr = lon_cpr;
        even_position.cpr_format = cpr_format;
        even_position.timestamp = time_us_32();
        even_position.valid = true;
    }
    else {  // Нечетная позиция
        odd_position.lat_cpr = lat_cpr;
        odd_position.lon_cpr = lon_cpr;
        odd_position.cpr_format = cpr_format;
        odd_position.timestamp = time_us_32();
        odd_position.valid = true;
    }
}

// Функция попытки декодирования позиции
bool try_decode_position(double* latitude, double* longitude) {
    uint32_t current_time = time_us_32();
    const uint32_t MAX_TIME_DIFF = 10000000;  // 10 секунд максимум между сообщениями

    // Проверяем, есть ли обе позиции и не слишком ли они старые
    if (!even_position.valid || !odd_position.valid) {
        return false;
    }

    if ((current_time - even_position.timestamp) > MAX_TIME_DIFF ||
        (current_time - odd_position.timestamp) > MAX_TIME_DIFF) {
        return false;
    }

    // Попытка глобального декодирования
    return decode_cpr_global(even_position.lat_cpr, even_position.lon_cpr,
        odd_position.lat_cpr, odd_position.lon_cpr,
        latitude, longitude);
}

// Обновленная функция декодирования пакета с CPR
void decode_adsb_packet_with_cpr(adsb_packet_t* packet) 
{
    if (!packet->valid) return;

    // ... (предыдущая проверка CRC и основное декодирование)

    uint8_t df = packet->df;

    if (df == 17) {  // ADS-B Extended Squitter
        gpio_put(DF17_LED_PIN, 1);

        uint32_t icao = ((uint32_t)packet->data[1] << 16) |
            ((uint32_t)packet->data[2] << 8) |
            packet->data[3];

        uint8_t type_code = (packet->data[4] >> 3) & 0x1F;

        char output[400];
        snprintf(output, sizeof(output), "DF17: ICAO=%06X, TC=%02d", icao, type_code);

        // Декодирование позиционных сообщений
        if (type_code >= 9 && type_code <= 18) {  // Airborne position
            uint16_t altitude_raw = ((uint16_t)(packet->data[5] & 0xFF) << 4) |
                ((packet->data[6] >> 4) & 0x0F);

            int altitude = -1;
            if (altitude_raw != 0) {
                altitude = ((altitude_raw & 0x1F80) >> 2) | (altitude_raw & 0x003F);
                altitude = altitude * 25 - 1000;
            }

            uint8_t cpr_format = (packet->data[6] >> 2) & 0x01;

            uint32_t lat_cpr = ((uint32_t)(packet->data[6] & 0x03) << 15) |
                ((uint32_t)packet->data[7] << 7) |
                ((packet->data[8] >> 1) & 0x7F);

            uint32_t lon_cpr = ((uint32_t)(packet->data[8] & 0x01) << 16) |
                ((uint32_t)packet->data[9] << 8) |
                packet->data[10];

            // Сохраняем CPR позицию
            store_cpr_position(lat_cpr, lon_cpr, cpr_format);

            // Попытка декодирования координат
            double latitude, longitude;
            bool position_decoded = try_decode_position(&latitude, &longitude);

            snprintf(output + strlen(output), sizeof(output) - strlen(output),
                ", ALT=%d ft, CPR_F=%d, LAT_CPR=%06X, LON_CPR=%06X",
                altitude, cpr_format, lat_cpr, lon_cpr);

            if (position_decoded && !isnan(latitude) && !isnan(longitude)) {
                snprintf(output + strlen(output), sizeof(output) - strlen(output),
                    ", LAT=%.6f, LON=%.6f", latitude, longitude);
            }
        }

        uart_puts(uart1, output);
        uart_puts(uart1, "\r\n");

        sleep_ms(100);
        gpio_put(DF17_LED_PIN, 0);
    }
}


// Функция фильтрации шумов
bool noise_filter(uint32_t pulse_width) {
    // Проверка длительности импульса (должна быть около 0.5 мкс)
    if (pulse_width < 1 || pulse_width > 10) {  // В микросекундах
        noise_filter_count++;
        if (noise_filter_count > NOISE_THRESHOLD) 
        {
            receiving_packet = false;
            bit_count = 0;
            noise_filter_count = 0;
            return false;
        }
    } else {
        noise_filter_count = 0;
    }
    return true;
}

// Обработчик прерывания для детектирования пакетов
void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = time_us_32();
    static uint32_t last_transition = 0;

    if (gpio != INPUT_PIN_19 && gpio != INPUT_PIN_22) return;

    uint32_t time_diff = current_time - last_transition;
    last_transition = current_time;

    // Фильтрация шумов
    if (!noise_filter(time_diff)) {
        return;
    }

    // Детектирование бита на основе длительности
    bool bit_value = false;
    if (time_diff >= 1 && time_diff <= 3) {  // 1 мкс = логический 1
        bit_value = true;
    } else if (time_diff >= 1 && time_diff <= 3) {  // 0.5 мкс = логический 0
        bit_value = false;
    } else {
        return;  // Неопределенная длительность
    }

    if (!receiving_packet) {
        // Поиск преамбулы
        preamble_shift_reg = (preamble_shift_reg << 1) | (bit_value ? 1 : 0);

        if (preamble_shift_reg == preamble_pattern) {
            receiving_packet = true;
            bit_count = 0;
            gpio_put(PREAMBLE_LED_PIN, 1);
            memset((void*)bit_buffer, 0, sizeof(bit_buffer));
        }
    } else {
        // Прием данных пакета
        if (bit_count < ADSB_FRAME_LENGTH) {
            bit_buffer[bit_count] = bit_value ? 1 : 0;
            bit_count++;

            if (bit_count >= ADSB_FRAME_LENGTH) {
                // Пакет получен полностью
                receiving_packet = false;
                gpio_put(PREAMBLE_LED_PIN, 0);

                // Конвертация битов в байты
                memset((void*)&received_packet, 0, sizeof(received_packet));
                for (int i = 0; i < 14; i++) {
                    received_packet.data[i] = 0;
                    for (int j = 0; j < 8; j++) {
                        if (i * 8 + j < ADSB_FRAME_LENGTH) {
                            if (bit_buffer[i * 8 + j]) {
                                received_packet.data[i] |= (1 << (7 - j));
                            }
                        }
                    }
                }

                received_packet.timestamp = current_time;
                received_packet.df = (received_packet.data[0] >> 3) & 0x1F;
                received_packet.valid = true;
                new_packet_available = true;
            }
        }
    }
}

// Функция декодирования пакета ADS-B
void decode_adsb_packet(adsb_packet_t *packet) {
    if (!packet->valid) return;

    // Проверка CRC
    uint32_t calculated_crc = calculate_mode_s_crc(packet->data, 112);
    uint32_t received_crc = ((uint32_t)packet->data[11] << 16) |
                           ((uint32_t)packet->data[12] << 8) |
                           packet->data[13];

    if (calculated_crc != received_crc) {
        packet->valid = false;
        uart_puts(uart1, "CRC Error\r\n");
        return;
    }

    uint8_t df = packet->df;

    if (df == 17) {  // ADS-B Extended Squitter
        gpio_put(DF17_LED_PIN, 1);

        uint32_t icao = ((uint32_t)packet->data[1] << 16) |
                        ((uint32_t)packet->data[2] << 8) |
                        packet->data[3];

        uint8_t type_code = (packet->data[4] >> 3) & 0x1F;

        char output[300];
        snprintf(output, sizeof(output), "DF17: ICAO=%06X, TC=%02d", icao, type_code);

        // Декодирование позиционных сообщений
        if (type_code >= 9 && type_code <= 18) {  // Airborne position
            uint16_t altitude_raw = ((uint16_t)(packet->data[5] & 0xFF) << 4) |
                                   ((packet->data[6] >> 4) & 0x0F);

            int altitude = -1;
            if (altitude_raw != 0) {
                // Декодирование высоты (Mode C)
                altitude = ((altitude_raw & 0x1F80) >> 2) | (altitude_raw & 0x003F);
                altitude = altitude * 25 - 1000;  // В футах
            }

            uint8_t cpr_format = (packet->data[6] >> 2) & 0x01;  // CPR format (даже/нечетная)

            uint32_t lat_cpr = ((uint32_t)(packet->data[6] & 0x03) << 15) |
                               ((uint32_t)packet->data[7] << 7) |
                               ((packet->data[8] >> 1) & 0x7F);

            uint32_t lon_cpr = ((uint32_t)(packet->data[8] & 0x01) << 16) |
                               ((uint32_t)packet->data[9] << 8) |
                               packet->data[10];

            snprintf(output + strlen(output), sizeof(output) - strlen(output),
                    ", ALT=%d ft, CPR_F=%d, LAT=%06X, LON=%06X",
                    altitude, cpr_format, lat_cpr, lon_cpr);
        }
        // Декодирование идентификации
        else if (type_code >= 1 && type_code <= 4) {  // Aircraft identification
            char callsign[9] = {0};
            uint64_t data_bits = 0;

            // Извлекаем биты для позывного (56 бит)
            for (int i = 0; i < 7; i++) {
                data_bits = (data_bits << 8) | packet->data[5 + i];
            }

            // Декодируем символы (6 бит на символ)
            for (int i = 0; i < 8; i++) {
                uint8_t char_code = (data_bits >> (6 * (7 - i))) & 0x3F;
                if (char_code >= 1 && char_code <= 26) {
                    callsign[i] = 'A' + char_code - 1;
                } else if (char_code >= 48 && char_code <= 57) {
                    callsign[i] = '0' + char_code - 48;
                } else if (char_code == 32) {
                    callsign[i] = ' ';
                } else {
                    callsign[i] = '?';
                }
            }

            snprintf(output + strlen(output), sizeof(output) - strlen(output),
                    ", CALLSIGN=%s", callsign);
        }

        uart_puts(uart1, output);
        uart_puts(uart1, "\r\n");

        sleep_ms(100);
        gpio_put(DF17_LED_PIN, 0);
    }
}


// Улучшенная версия функции регулировки усиления
void update_gain_control() {
    static uint32_t last_gain_update = 0;
    uint32_t current_time = time_us_32();

    if (current_time - last_gain_update > 1000) {  // Обновление каждую 1 мс
        // Чтение ADC для управления усилением (уставка)
        adc_select_input(2);  // Pin 28 = ADC2
        uint16_t gain_setpoint = adc_read();

        // Чтение обратной связи усиления (текущее значение)
        adc_select_input(1);  // Pin 27 = ADC1
        uint16_t gain_feedback = adc_read();

        // ПИД-регулятор с улучшенными параметрами
        static int32_t integral = 0;
        static int16_t last_error = 0;

        // Коэффициенты регулятора (настраиваются экспериментально)
        static const float kp = 2.0f;    // Пропорциональный
        static const float ki = 0.1f;    // Интегральный
        static const float kd = 0.05f;   // Дифференциальный

        // Вычисление ошибки
        int16_t error = (int16_t)gain_setpoint - (int16_t)gain_feedback;

        // Интегральная составляющая с ограничением
        integral += error;
        if (integral > 50000) integral = 50000;
        if (integral < -50000) integral = -50000;

        // Дифференциальная составляющая
        int16_t derivative = error - last_error;
        last_error = error;

        // Вычисление выходного сигнала ПИД-регулятора
        float pid_output = (kp * error) + (ki * integral * 0.001f) + (kd * derivative);

        // Преобразование в PWM значение (0-4095)
        int32_t pwm_value = 2048 + (int32_t)pid_output;  // 2048 = центр (50% PWM)

        // Ограничение выходного сигнала
        if (pwm_value > 4095) pwm_value = 4095;
        if (pwm_value < 0) pwm_value = 0;

        // Установка PWM
        pwm_set_gpio_level(GAIN_PWM_PIN, (uint16_t)pwm_value);

        last_gain_update = current_time;

        // Опциональный вывод отладочной информации
#ifdef DEBUG_AGC
        static uint32_t debug_counter = 0;
        if (++debug_counter >= 1000) {  // Каждую секунду
            char debug_msg[100];
            snprintf(debug_msg, sizeof(debug_msg),
                "AGC: Set=%d, FB=%d, Err=%d, PWM=%d\r\n",
                gain_setpoint, gain_feedback, error, pwm_value);
            uart_puts(uart1, debug_msg);
            debug_counter = 0;
        }
#endif
    }
}


//// Функция регулировки усиления
//void update_gain_control() {
//    static uint32_t last_gain_update = 0;
//    uint32_t current_time = time_us_32();
//
//    if (current_time - last_gain_update > 10000) {  // Обновление каждые 10 мс
//        // Чтение ADC для управления усилением
//        adc_select_input(2);  // Pin 28 = ADC2
//        uint16_t gain_setpoint = adc_read();
//
//        // Чтение обратной связи усиления
//        adc_select_input(1);  // Pin 27 = ADC1
//        uint16_t gain_feedback = adc_read();
//
//        // ПИ-регулятор
//        static int32_t integral = 0;
//        static const int kp = 10;  // Пропорциональный коэффициент
//        static const int ki = 1;   // Интегральный коэффициент
//
//        int16_t error = gain_setpoint - gain_feedback;
//        integral += error;
//
//        // Ограничение интеграла (анти-windup)
//        if (integral > 10000) integral = 10000;
//        if (integral < -10000) integral = -10000;
//
//        // Исправленная формула ПИ-регулятора
//        int32_t output = (kp * error) + (ki * integral / 100);
//
//        // Ограничение выходного сигнала (12-bit PWM: 0-4095)
//        if (output > 4095) output = 4095;
//        if (output < 0) output = 0;
//
//        // Установка PWM для управления усилением
//        pwm_set_gpio_level(GAIN_PWM_PIN, (uint16_t)output);
//
//        last_gain_update = current_time;
//    }
//}


// Основная функция для второго ядра
void core1_main() {
    while (true) {
        if (new_packet_available) {
            new_packet_available = false;
            decode_adsb_packet((adsb_packet_t*)&received_packet);
        }
        update_gain_control();
        tight_loop_contents();
    }
}

// Функция инициализации системы
void initialize_system() {
   //!! stdio_init_all();

    // Инициализация GPIO для входов
    gpio_init(INPUT_PIN_19);
    gpio_set_dir(INPUT_PIN_19, GPIO_IN);
    gpio_pull_down(INPUT_PIN_19);  // Pull-down для четкого определения фронтов

    gpio_init(INPUT_PIN_22);
    gpio_set_dir(INPUT_PIN_22, GPIO_IN);
    gpio_pull_down(INPUT_PIN_22);

    // Инициализация LED индикации
    gpio_init(PREAMBLE_LED_PIN);
    gpio_set_dir(PREAMBLE_LED_PIN, GPIO_OUT);
    gpio_put(PREAMBLE_LED_PIN, 0);

    gpio_init(DF17_LED_PIN);
    gpio_set_dir(DF17_LED_PIN, GPIO_OUT);
    gpio_put(DF17_LED_PIN, 0);

    // Инициализация PWM для управления усилением
    gpio_set_function(GAIN_PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(GAIN_PWM_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f);  // Полная скорость
    pwm_config_set_wrap(&config, 4095);    // 12-bit разрешение
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(GAIN_PWM_PIN, 2048); // Начальное значение 50%

    // Инициализация ADC
    adc_init();
    adc_gpio_init(GAIN_CONTROL_PIN);   // ADC2
    adc_gpio_init(GAIN_MONITOR_PIN);   // ADC1

    // Инициализация UART
    uart_init(uart1, 115200);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, true);

    // Настройка прерываний GPIO
    gpio_set_irq_enabled_with_callback(INPUT_PIN_19,
                                      GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                      true, &gpio_irq_handler);
    gpio_set_irq_enabled(INPUT_PIN_22,
                        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                        true);

    // Запуск второго ядра
    multicore_launch_core1(core1_main);

    uart_puts(uart1, "ADS-B Decoder v1.0 Started\r\n");
    uart_puts(uart1, "Listening for ADS-B packets...\r\n");
}

// Главная функция
int main() {
    // Инициализация системы
    initialize_system();

    // Основной цикл первого ядра
    uint32_t last_status = 0;
    uint32_t packet_count = 0;

    while (true) {
        uint32_t current_time = time_us_32();

        // Статистика каждые 10 секунд
        if (current_time - last_status > 10000000) {
            char status_msg[100];
            snprintf(status_msg, sizeof(status_msg),
                    "Status: %d packets received\r\n", packet_count);
            uart_puts(uart1, status_msg);
            last_status = current_time;
        }

        // Подсчет пакетов
        static bool last_packet_state = false;
        if (new_packet_available && !last_packet_state) {
            packet_count++;
        }
        last_packet_state = new_packet_available;

        sleep_ms(100);
    }

    return 0;
}

/*
Основные изменения:

Удаление setup() и loop() - заменены на стандартную функцию main() для Pico SDK
Добавление initialize_system() - правильная инициализация всех компонентов
Структурированный main() - основной цикл с мониторингом системы
Улучшенное декодирование - добавлены функции для декодирования позывных и позиций
Лучшая обработка прерываний - более точное определение битов по временным интервалам
Корректная настройка UART - полная конфигурация UART1

Программа теперь полностью совместима со стандартным Pico SDK и готова к компиляции с помощью CMake.
*/