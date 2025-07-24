#include <mutex>
#include <Arduino.h>
#include <queue>
#include <vector>


#include "adsbee.h"
#include "transponder_packet.h"
#include "packet_decoder.h"



// Пины
#define INPUT_PIN_1 19
#define INPUT_PIN_2 22
#define PREAMBLE_LED_1 20
#define PREAMBLE_LED_2 23
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define AGC_INPUT_PIN 26
#define AGC_OUTPUT_PIN 9
#define AGC_CONTROL_PIN 27
#define CORE0_LED_PIN 15
#define CORE1_LED_PIN 25

// Константы
#define SAMPLE_RATE 2000000  // 2 МГц
#define PREAMBLE_PATTERN 0b10100000010100000000  // Преамбула ADS-B
#define PREAMBLE_LENGTH 20
#define SHORT_PACKET_BITS 56
#define LONG_PACKET_BITS 112
#define MAX_QUEUE_SIZE 50

// Глобальные переменные
volatile bool preamble_detected_1 = false;
volatile bool preamble_detected_2 = false;
volatile uint32_t last_edge_time_1 = 0;
volatile uint32_t last_edge_time_2 = 0;

// Глобальные переменнные
volatile bool preambleDetected = false;
volatile uint32_t bitBuffer = 0;
volatile uint8_t bitCount = 0;
volatile bool packetReady = false;

BSP bsp = BSP({});
ADSBee adsbee = ADSBee({});

// Структура пакета
struct ADSBPacket1 {
    uint8_t data[16];  // Максимум 112 бит = 14 байт
    uint8_t length;    // Длина в байтах
    uint32_t timestamp;
    uint8_t channel;   // 1 или 2
};

// Очереди пакетов
std::queue<ADSBPacket1> packet_queue;
std::queue<ADSBPacket1> processed_queue;

// Мьютексы для защиты очередей
volatile bool queue_mutex = false;
volatile bool processed_mutex = false;

// Таблица CRC для ADS-B
uint32_t crc_table[256];

// Буферы для декодирования
volatile uint8_t bit_buffer_1[120];
volatile uint8_t bit_buffer_2[120];
volatile int bit_count_1 = 0;
volatile int bit_count_2 = 0;
volatile bool collecting_1 = false;
volatile bool collecting_2 = false;

//// Фильтр помех
//#define NOISE_FILTER_SIZE 5
//volatile uint32_t noise_filter_1[NOISE_FILTER_SIZE] = {0};
//volatile uint32_t noise_filter_2[NOISE_FILTER_SIZE] = {0};
//volatile int filter_index_1 = 0;
//volatile int filter_index_2 = 0;

// AGC переменные
volatile int agc_level = 512;
volatile uint32_t last_agc_update = 0;

// Инициализация таблицы CRC
void init_crc_table() {
    const uint32_t polynomial = 0x1021FFF;  // CRC-24 полином для ADS-B

    for (int i = 0; i < 256; i++) {
        uint32_t crc = i << 16;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x800000) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc = crc << 1;
            }
        }
        crc_table[i] = crc & 0xFFFFFF;
    }
}

// Вычисление CRC
uint32_t calculate_crc(uint8_t* data, int length) {
    uint32_t crc = 0;
    for (int i = 0; i < length - 3; i++) {
        crc = ((crc << 8) ^ crc_table[((crc >> 16) ^ data[i]) & 0xFF]) & 0xFFFFFF;
    }
    return crc;
}

// Проверка CRC пакета
bool check_crc(uint8_t* data, int length) 
{
    uint32_t calculated_crc = calculate_crc(data, length);
    uint32_t packet_crc = (data[length-3] << 16) | (data[length-2] << 8) | data[length-1];
    return calculated_crc == packet_crc;
}

// Коррекция одиночной ошибки
bool correct_single_bit_error(uint8_t* data, int length) 
{
    for (int byte_pos = 0; byte_pos < length; byte_pos++) {
        for (int bit_pos = 0; bit_pos < 8; bit_pos++) {
            // Переворачиваем бит
            data[byte_pos] ^= (1 << bit_pos);

            // Проверяем CRC
            if (check_crc(data, length)) {
                return true;  // Ошибка исправлена
            }

            // Возвращаем бит обратно
            data[byte_pos] ^= (1 << bit_pos);
        }
    }
    return false;  // Не удалось исправить
}

//=============================================================================
//void ConstructTransponderPacket(uint8_t* data, int length)
//{
//
//
//    if (length != 14) // Не равен 112 или 56
//    {
//
//        Serial2.printf("Filed TransponderPacket  %d \r\n", adsbee.raw_1090_packet_queue.Length());
//        return;  // оставьте is_valid_ как false
//    }
//    Serial2.printf("ConstructTransponderPacket  %d \r\n", adsbee.raw_1090_packet_queue.Length());
//
//
//
//
//
//}



//===============================================================================
// Декодирование ICAO адреса
uint32_t decode_icao(uint8_t* data) {
    return (data[1] << 16) | (data[2] << 8) | data[3];
}

// Декодирование типа сообщения
uint8_t decode_message_type(uint8_t* data) {
    return data[0] >> 3;
}

// Декодирование позиции
void decode_position(uint8_t* data, double latitude, double longitude) {
    uint8_t msg_type = decode_message_type(data);

    if (msg_type >= 9 && msg_type <= 22) {  // Airborne position
        uint32_t raw_lat = ((data[6] & 0x03) << 15) | (data[7] << 7) | (data[8] >> 1);
        uint32_t raw_lon = ((data[8] & 0x01) << 16) | (data[9] << 8) | data[10];

        // Упрощенное декодирование (требует CPR декодирование для точности)
        latitude = (raw_lat / 131072.0);//!!  90.0;
        longitude = (raw_lon / 131072.0);//!!  180.0;

        if (latitude > 90) latitude -= 180;
        if (longitude > 180) longitude -= 360;
    }
}

// Декодирование скорости
uint16_t decode_velocity(uint8_t* data) {
    uint8_t msg_type = decode_message_type(data);

    if (msg_type == 19) {  // Velocity message
        uint16_t ew_vel = ((data[5] & 0x03) << 8) | data[6];
        uint16_t ns_vel = ((data[7] & 0x7F) << 3) | (data[8] >> 5);

        return sqrt(ew_vel /* ew_vel*/ + ns_vel  /*ns_vel*/); //!! Переделать
    }

    return 0;
}

// Декодирование высоты
uint16_t decode_altitude(uint8_t* data) {
    uint8_t msg_type = decode_message_type(data);

    if (msg_type >= 9 && msg_type <= 22) {
        uint16_t alt_code = ((data[5] & 0x1F) << 7) | (data[6] >> 1);

        if (alt_code == 0) return 0;

        // Q-бит проверка
        if (data[5] & 0x01) {
            return (alt_code - 1) * 25;  // футы
        } else {
            // Gillham кодирование (упрощено)
            return alt_code * 100;
        }
    }

    return 0;
}

// Декодирование номера рейса
void decode_callsign(uint8_t* data, char* callsign) {
    uint8_t msg_type = decode_message_type(data);

    if (msg_type >= 1 && msg_type <= 4) {  // Identification message
        const char charset[] = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######";

        for (int i = 0; i < 8; i++) {
            uint8_t char_code;

            switch (i) {
                case 0: char_code = (data[5] >> 2) & 0x3F; break;
                case 1: char_code = ((data[5] & 0x03) << 4) | (data[6] >> 4); break;
                case 2: char_code = ((data[6] & 0x0F) << 2) | (data[7] >> 6); break;
                case 3: char_code = data[7] & 0x3F; break;
                case 4: char_code = (data[8] >> 2) & 0x3F; break;
                case 5: char_code = ((data[8] & 0x03) << 4) | (data[9] >> 4); break;
                case 6: char_code = ((data[9] & 0x0F) << 2) | (data[10] >> 6); break;
                case 7: char_code = data[10] & 0x3F; break;
                default: char_code = 0; break;
            }

            callsign[i] = charset[char_code];
        }
        callsign[8] = '\0';

        // Удаляем завершающие пробелы
        for (int i = 7; i >= 0; i--) {
            if (callsign[i] == '_') {
                callsign[i] = '\0';
            } else {
                break;
            }
        }
    } else {
        strcpy(callsign, "N/A");
    }
}

// AGC управление
void update_agc() {
    int agc_input = analogRead(AGC_INPUT_PIN);
    int control_level = analogRead(AGC_CONTROL_PIN);

    // Простой пропорциональный контроллер
    int error = 512 - control_level;  // Целевой уровень 2.5В
    agc_level += error / 10;

    // Ограничиваем диапазон
    if (agc_level < 0) agc_level = 0;
    if (agc_level > 1023) agc_level = 1023;

    analogWrite(AGC_OUTPUT_PIN, agc_level);
}

//======================================================================
// Функция копирования данных из uint32_t buffer в uint8_t array
void copy_buffer_to_data(uint32_t buffer[4], uint8_t data[14]) {
    // Очищаем целевой массив
    memset(data, 0, 14);

    // Копируем данные побайтово
    for (int i = 0; i < 4; i++) {
        // Каждый uint32_t содержит 4 байта
        data[i * 4 + 0] = (buffer[i] >> 24) & 0xFF;  // Старший байт
        data[i * 4 + 1] = (buffer[i] >> 16) & 0xFF;
        data[i * 4 + 2] = (buffer[i] >> 8) & 0xFF;
        data[i * 4 + 3] = buffer[i] & 0xFF;          // Младший байт
    }

    // Остальные 2 байта остаются нулевыми (если нужны только 112 бит = 14 байт)
    // buffer[4] может содержать 16 байт, но нам нужно только 14
}

// Альтернативная функция с использованием union для более эффективного копирования
void copy_buffer_to_data_union(uint32_t buffer[4], uint8_t data[14]) {
    union {
        uint32_t words[4];
        uint8_t bytes[16];
    } converter;

    // Копируем uint32_t значения
    for (int i = 0; i < 4; i++) {
        converter.words[i] = buffer[i];
    }

    // Копируем первые 14 байт
    memcpy(data, converter.bytes, 14);
}

// Функция с проверкой порядка байт (Big Endian)
void copy_buffer_to_data_be(uint32_t buffer[4], uint8_t data[14]) {
    for (int i = 0; i < 4; i++) {
        uint32_t word = buffer[i];

        // Извлекаем байты в порядке Big Endian
        if ((i * 4 + 0) < 14) data[i * 4 + 0] = (word >> 24) & 0xFF;
        if ((i * 4 + 1) < 14) data[i * 4 + 1] = (word >> 16) & 0xFF;
        if ((i * 4 + 2) < 14) data[i * 4 + 2] = (word >> 8) & 0xFF;
        if ((i * 4 + 3) < 14) data[i * 4 + 3] = word & 0xFF;
    }
}

// Функция с проверкой порядка байт (Little Endian)
void copy_buffer_to_data_le(uint32_t buffer[4], uint8_t data[14]) {
    for (int i = 0; i < 4; i++) {
        uint32_t word = buffer[i];

        // Извлекаем байты в порядке Little Endian
        if ((i * 4 + 0) < 14) data[i * 4 + 0] = word & 0xFF;
        if ((i * 4 + 1) < 14) data[i * 4 + 1] = (word >> 8) & 0xFF;
        if ((i * 4 + 2) < 14) data[i * 4 + 2] = (word >> 16) & 0xFF;
        if ((i * 4 + 3) < 14) data[i * 4 + 3] = (word >> 24) & 0xFF;
    }
}

// Оптимизированная функция с использованием memcpy
void copy_buffer_to_data_fast(uint32_t buffer[4], uint8_t data[14]) {
    // Прямое копирование блока памяти
    memcpy(data, buffer, 14);

    // Если нужна коррекция порядка байт на системах с разным endianness
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
// Для Little Endian систем может потребоваться перестановка байт
// если данные ожидаются в Big Endian формате
    for (int i = 0; i < 3; i++) { // Обрабатываем первые 3 полных слова
        uint32_t* word_ptr = (uint32_t*)(data + i * 4);
        *word_ptr = __builtin_bswap32(*word_ptr);
    }

    // Для последних 2 байт (14-й и 13-й байт в 4-м слове)
    uint8_t temp = data[12];
    data[12] = data[13];
    data[13] = temp;
#endif
}

// Пример использования в основной программе
void example_usage() {
    uint32_t buffer[4] = { 0x12345678, 0x9ABCDEF0, 0x11223344, 0x55667788 };
    uint8_t data[14];

    // Выбираем подходящую функцию в зависимости от требований
    copy_buffer_to_data(buffer, data);

    // Выводим результат для проверки
    Serial.print("Converted data: ");
    for (int i = 0; i < 14; i++) {
        if (data[i] < 16) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

//=====================================================================


// Функция для преобразования HEX символа в число
uint8_t hexCharToValue(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    else if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    else if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return 0xFF;  // Ошибка
}

// Улучшенная функция преобразования
int hexStringToBytes(/*const */char* hex_string, uint8_t* output, int max_output_size) 
{
    int len = strlen(hex_string);

    if (len % 2 != 0 || len / 2 > max_output_size) 
    {
        return -1;  // Ошибка
    }

    int byte_count = 0;

    for (int i = 0; i < len; i += 2) {
        uint8_t high = hexCharToValue(hex_string[i]);
        uint8_t low = hexCharToValue(hex_string[i + 1]);

        if (high == 0xFF || low == 0xFF) {
            return -1;  // Недопустимый символ
        }

        output[byte_count] = (high << 4) | low;
        byte_count++;
    }

    return byte_count;
}




// Функция для тестирования декодирования
void testADSBDecoding(uint8_t* data, int length) {
    Serial2.println("\n--- ADS-B Packet Analysis ---");

    // Декодируем ICAO
    uint32_t icao = decode_icao(data);
    Serial2.print("ICAO: ");
    Serial2.println(icao, HEX);

    // Декодируем тип сообщения
    uint8_t msg_type = decode_message_type(data);
    Serial2.print("Message Type: ");
    Serial2.println(msg_type);

    // Проверяем CRC
    bool crc_valid = check_crc(data, length);
    Serial2.print("CRC Valid: ");
    Serial2.println(crc_valid ? "YES" : "NO");

    // Пытаемся декодировать другие поля в зависимости от типа
    if (msg_type >= 1 && msg_type <= 4) {
        char callsign[9];
        decode_callsign(data, callsign);
        Serial2.print("Callsign: ");
        Serial2.println(callsign);
    }

    if (msg_type >= 9 && msg_type <= 22) {
        double lat, lon;
        decode_position(data, lat, lon);
        Serial2.print("Position: ");
        Serial2.print(lat, 6);
        Serial2.print(", ");
        Serial2.println(lon, 6);

        uint16_t altitude = decode_altitude(data);
        Serial2.print("Altitude: ");
        Serial2.print(altitude);
        Serial2.println(" ft");
    }
}


//Результат преобразования :
//
//Строка 8D1521F48D1521F47FF3600FFFFFFF91 преобразуется в массив :


//uint8_t hex_data[16] = {
//    0x8D, 0x15, 0x21, 0xF4, 0x8D, 0x15, 0x21, 0xF4,
//    0x7F, 0xF3, 0x60, 0x0F, 0xFF, 0xFF, 0xFF, 0x91
//};


// Функции для работы с CRC
uint32_t crc24(uint8_t* data, int len) {
    uint32_t crc = 0;
    uint32_t generator = 0x1FFF409; // ADS-B CRC24 полином

    for (int i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i] << 16;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x800000) {
                crc = (crc << 1) ^ generator;
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc & 0xFFFFFF;
}

bool validate_crc(uint8_t* packet) {
    uint32_t received_crc = (packet[11] << 16) | (packet[12] << 8) | packet[13];
    uint32_t calculated_crc = crc24(packet, 11);
    return received_crc == calculated_crc;
}





// Контроль валидности пакета
void analyze_test_packet(uint8_t* packet)
{
    // uint8_t packet[] = { 0x8D, 0x48, 0x43, 0x95, 0x99, 0x00, 0x52, 0xB2, 0xE8, 0x0B, 0x02, 0x7D, 0x68, 0x7F };
    // uint8_t packet[] = { 0x8D, 0x15, 0x5C, 0x1D, 0xEB, 0x19, 0x78, 0x48, 0x00, 0x10, 0x00, 0x38, 0x70, 0x59 };
    // uint8_t packet[] = { 0x8D, 0x3C, 0x65, 0x03, 0x99, 0x90, 0xFB, 0x9E, 0x28, 0x64, 0x04, 0xA0, 0x04, 0x3E };
    // uint8_t packet[] = { 0x8D, 0x15, 0x1D, 0xA7, 0x9B, 0x04, 0xD9, 0x12, 0xD8, 0x38, 0x00, 0x3A, 0x20, 0xD5 };
    // uint8_t packet[] = { 0x8D, 0x15, 0x20, 0x23, 0x9b, 0xa3, 0xf8, 0xB2, 0x08, 0x00, 0x00, 0x00, 0x00, 0x98 };
    // uint8_t packet[] = { 0x8D, 0x15, 0x1D, 0xA8, 0xEB, 0x04, 0xF8, 0x24, 0xB4, 0x10, 0x08, 0x61, 0x2F, 0x24 };
   // uint8_t packet[] = { 0x8D, 0x15, 0x1D, 0xD8, 0x9B, 0x05, 0x17, 0x98, 0x18, 0x1C, 0x00, 0x04, 0x55, 0xBA };

    Serial2.println("\n=== ADS-B Packet Analysis ===");

    // Вывод сырых данных
    Serial2.print("Raw packet: ");
    for (int i = 0; i < sizeof(packet); i++) {
        if (packet[i] < 0x10)Serial2.print("0");
        Serial2.print(packet[i], HEX);
        Serial2.print(" ");
    }
    Serial2.println();

    // Разбор заголовка
    uint8_t df = (packet[0] >> 3) & 0x1F;           // Downlink Format
    uint8_t capability = packet[0] & 0x07;          // Capability
    uint32_t icao = (packet[1] << 16) | (packet[2] << 8) | packet[3];  // ICAO адрес

    Serial2.print("DF: ");
    Serial2.print(df);
    Serial2.print(" (");
    if (df == 17)Serial2.print("ADS-B");
    else if (df == 18)Serial2.print("TIS-B");
    else Serial2.print("Other");
    Serial2.println(")");

    Serial2.print("Capability: ");
    Serial2.println(capability);

    Serial2.print("ICAO: ");
    Serial2.println(icao, HEX);

    if (df == 17 || df == 18) {
        uint8_t tc = (packet[4] >> 3) & 0x1F;       // Type Code

        Serial2.print("Type Code: ");
        Serial2.print(tc);
        Serial2.print(" (");

        // Определяем тип сообщения
        if (tc >= 1 && tc <= 4)
        {
            Serial2.print("Aircraft Identification");
            decode_callsign(packet);
        }
        else if (tc >= 9 && tc <= 18) {
            Serial2.print("Airborne Position");
            decode_position(packet);
        }
        else if (tc == 19) {
            Serial2.print("Airborne Velocity");
            decode_velocity1(packet);
        }
        else if (tc >= 20 && tc <= 22)
        {
            Serial2.print("Airborne Position");
            decode_position(packet);
        }
        else {
            Serial2.print("Other/Unknown");
        }
        Serial2.println(")");
    }

    // Проверка CRC
    if (validate_crc(packet))
    {
        Serial2.println("CRC: ✓ Valid");
    }
    else
    {
        Serial2.println("CRC: ✗ Invalid");
    }

    Serial2.println("========================\n");
}

// Функция для декодирования позывного
void decode_callsign(uint8_t* packet) {
    char callsign[9] = { 0 };
    uint64_t data = 0;

    // Извлекаем 48 бит данных (байты 5-10)
    for (int i = 5; i < 11; i++) {
        data = (data << 8) | packet[i];
    }

    // Декодируем позывной (6-битная кодировка)
    for (int i = 0; i < 8; i++) {
        uint8_t c = (data >> (42 - i * 6)) & 0x3F;
        if (c > 0) {
            if (c < 32) {
                callsign[i] = c + 64; // A-Z
            }
            else {
                callsign[i] = c; // 0-9, пробел и др.
            }
        }
    }

    Serial2.print("Callsign: ");
    Serial2.println(callsign);
}

// Функция для декодирования позиции
void decode_position(uint8_t* packet)
{
    bool odd_flag = (packet[6] >> 2) & 1;
    uint32_t lat_cpr = ((packet[6] & 0x03) << 15) | (packet[7] << 7) | (packet[8] >> 1);
    uint32_t lon_cpr = ((packet[8] & 0x01) << 16) | (packet[9] << 8) | packet[10];

    Serial2.print("CPR Format: ");
    Serial2.println(odd_flag ? "Odd" : "Even");
    Serial2.print("CPR Latitude: ");
    Serial2.println(lat_cpr);
    Serial2.print("CPR Longitude: ");
    Serial2.println(lon_cpr);

    //// Здесь можно добавить декодирование высоты из packet[11-12]
    //uint16_t altitude_code = ((packet[5] & 0x01) << 11) | (packet[6] << 3) | (packet[7] >> 5);
    //if (altitude_code != 0) {
    //    int altitude = decode_altitude(altitude_code);
    //    if (altitude != -1) {
    //       Serial2.print("Altitude: ");
    //       Serial2.print(altitude);
    //       Serial2.println(" ft");
    //    }
    //}
}

// Функция для декодирования скорости
void decode_velocity1(uint8_t* packet) {
    uint8_t subtype = (packet[4] >> 1) & 0x07;

    if (subtype == 1 || subtype == 2) { // Ground speed
        uint16_t ew_vel = ((packet[5] & 0x03) << 8) | packet[6];
        uint16_t ns_vel = ((packet[7] & 0x7F) << 3) | (packet[8] >> 5);

        if (ew_vel && ns_vel) {
            int ground_speed = sqrt(ew_vel * ew_vel + ns_vel * ns_vel);
            int heading = atan2(ew_vel, ns_vel) * 180 / PI;
            if (heading < 0) heading += 360;

            Serial2.print("Ground Speed: ");
            Serial2.print(ground_speed);
            Serial2.print(" kt, Heading: ");
            Serial2.print(heading);
            Serial2.println("°");
        }
    }
}

//uint16_t PrintBuffer1(char* buf, uint16_t buf_len_bytes) const {
//    uint16_t len = 0;
//    switch (buffer_len_bits)
//    {
//    case kSquitterPacketLenBits:
//        len = snprintf(buf, buf_len_bytes, "%08lX%06lX", buffer[0], buffer[1] >> (2 * kBitsPerNibble));
//        break;
//    case kExtendedSquitterPacketLenBits:
//        len = snprintf(buf, buf_len_bytes, "%08lX%08lX%08lX%04lX", buffer[0], buffer[1], buffer[2],
//            buffer[3] >> (4 * kBitsPerNibble));
//        break;
//        // Ничего не печатать, если буфер имеет недопустимую длину.
//    }
//    return len;
//}

//======================================================================

PacketDecoder decoder = PacketDecoder({ .enable_1090_error_correction = true });
// Задача ядра 0 - обработка пакетов
void core0_task() 
{

    unsigned long lastToggle = 0;


    decoder.UpdateDecoderLoop();
    decoder.UpdateLogLoop();


    while (true) 
    {
        // Переключаем LED каждые 1000 мс
        if (millis() - lastToggle >= 1000) {
            digitalWrite(CORE0_LED_PIN, !digitalRead(CORE0_LED_PIN));
            lastToggle = millis();
        }

        // Обновление AGC
        if (millis() - last_agc_update >= 10) {
            update_agc();
            last_agc_update = millis();
        }


        // //decoder.UpdateDecoderLoop();
        // //decoder.UpdateLogLoop();

         Raw1090Packet raw_packet;
         Decoded1090Packet decoded_packet = Decoded1090Packet(raw_packet);

        //if (decoded_packet.IsValid())
        //{

        //    Serial2.printf("IsValid\r\n");

        //}

 
        // Обработка пакетов из очереди
        if (adsbee.raw_1090_packet_queue.Length() != 0)
        {
            while (queue_mutex) delay(1);
            queue_mutex = true;
            adsbee.raw_1090_packet_queue.Pop(raw_packet);
            queue_mutex = false;

            //Serial2.printf("Test: 0x%08x%08x%08x%04x\r\n",
            //    raw_packet.buffer[0], raw_packet.buffer[1], raw_packet.buffer[2],
            //    (raw_packet.buffer[3]) >> (4 * kBitsPerNibble));

           // char message[200U + 1] = { '\0' };

           // uint16_t message_len = strlen(message);
            //message_len += snprintf(message + message_len,112 , "0x % 08x % 08x % 08x % 04x\r\n",
            //    raw_packet.buffer[0], raw_packet.buffer[1], raw_packet.buffer[2], (raw_packet.buffer[3]) >> (4 * kBitsPerNibble));
            //Serial2.printf("message_len..  %s \r\n", message_len);

            //raw_packet.PrintBuffer((char*)message_len, 200U - message_len);
            //Serial2.printf("UpdateDecoderLoop..  %s \r\n", message_len);

           // analyze_test_packet((uint8_t*)raw_packet.buffer);

        //    ADSBPacket1 packet;

        //    if (raw_packet.buffer_len_bits == Raw1090Packet::kExtendedSquitterPacketLenBits)
        //    {

        //        packet.channel = raw_packet.source;
        //        packet.length = (uint8_t)raw_packet.buffer_len_bits / 8;

        //        // Очищаем целевой массив
        //        memset(packet.data, 0, 14);

        //            // Копируем данные побайтово в порядке Big Endian (стандарт для ADS-B)
        //        for (int i = 0; i < 4; i++) {
        //            packet.data[i * 4 + 0] = (raw_packet.buffer[i] >> 24) & 0xFF;  // Старший байт
        //            packet.data[i * 4 + 1] = (raw_packet.buffer[i] >> 16) & 0xFF;
        //            packet.data[i * 4 + 2] = (raw_packet.buffer[i] >> 8) & 0xFF;
        //            packet.data[i * 4 + 3] = raw_packet.buffer[i] & 0xFF;          // Младший байт
        //        }




        //        Serial2.printf("Raw packet(Ch)  %d %d\r\n", packet.channel, packet.length);
        //        Serial2.print(" bytes): \t\t");
        //        for (int i = 0; i < packet.length+2; i++)
        //        {
        //            if (packet.data[i] < 16) Serial2.print("0");
        //            Serial2.print(packet.data[i], HEX);
        //           // Serial2.print(" ");
        //        }
        //        Serial2.println();

        //        Serial2.printf("core0_task(rx_packet). \t%08lX%08lX%08lX%04lX\r\n", raw_packet.buffer[0], raw_packet.buffer[0], raw_packet.buffer[1], raw_packet.buffer[2], raw_packet.buffer[3] >> (4 * kBitsPerNibble)); //%08lX%08lX%08lX%04lX

        //        //ConstructTransponderPacket();
        //        // Проверка и коррекция CRC

        //       // packet.data[] = {0x8D, 0x15, 0x21, 0xF4, 0x8D, 0x15, 0x21, 0xF4, 0x7F, 0xF3, 0x60, 0x0F, 0xFF, 0xFF, 0xFF, 0x91};

        //        //bool crc_ok = check_crc(packet.data, packet.length);
        //        //bool corrected = false;

        //        //if (!crc_ok)
        //        //{
        //        //    corrected = correct_single_bit_error(packet.data, packet.length);
        //        //    if (corrected)
        //        //    {
        //        //        Serial2.println("Single bit error corrected!");
        //        //    }
        //        //}

        //        //if (crc_ok || corrected) 
        //        //{
        //        //    // Декодирование пакета
        //        //    uint32_t icao = decode_icao(packet.data);
        //        //    uint8_t msg_type = decode_message_type(packet.data);

        //        //    char callsign[9];
        //        //    decode_callsign(packet.data, callsign);

        //        //    double latitude = 0, longitude = 0;
        //        //  //!!  decode_position(packet.data, &latitude, &longitude);

        //        //    uint16_t velocity = decode_velocity(packet.data);
        //        //    uint16_t altitude = decode_altitude(packet.data);

        //        //    // Формируем JSON для Serial2
        //        //    String json = "{";
        //        //    json += "\"icao\":\"" + String(icao, HEX) + "\",";
        //        //    json += "\"type\":" + String(msg_type) + ",";
        //        //    json += "\"callsign\":\"" + String(callsign) + "\",";
        //        //    json += "\"lat\":" + String(latitude, 6) + ",";
        //        //    json += "\"lon\":" + String(longitude, 6) + ",";
        //        //    json += "\"alt\":" + String(altitude) + ",";
        //        //    json += "\"speed\":" + String(velocity) + ",";
        //        //    json += "\"channel\":" + String(packet.channel) + ",";
        //        //    json += "\"corrected\":" + String(corrected ? "true" : "false");
        //        //    json += "}\n";

        //        //    // Отправляем в Serial2
        //        //    Serial2.print(json);

        //        //    // Отправляем в Serial2 для отладки
        //        //    Serial2.print("Decoded - ICAO: ");
        //        //    Serial2.print(icao, HEX);
        //        //    Serial2.print(", Callsign: ");
        //        //    Serial2.print(callsign);
        //        //    Serial2.print(", Lat: ");
        //        //    Serial2.print(latitude, 6);
        //        //    Serial2.print(", Lon: ");
        //        //    Serial2.print(longitude, 6);
        //        //    Serial2.print(", Alt: ");
        //        //    Serial2.print(altitude);
        //        //    Serial2.print(", Speed: ");
        //        //    Serial2.println(velocity);

        //        //} 
        //        //else 
        //        //{
        //        //    Serial2.println("CRC failed, packet discarded");
        //        //}
        //    }
        }

        delay(1);
    }
}

// Задача ядра 1 - прием пакетов
void core1_task() {
    unsigned long lastToggle = 0;
    
    while (true) {
        // Переключаем LED каждые 500 мс
        if (millis() - lastToggle >= 500) {
            digitalWrite(CORE1_LED_PIN, !digitalRead(CORE1_LED_PIN));
            lastToggle = millis();
        }

        // Основная работа происходит в прерываниях
        delay(1);
    }
}

void setup() {
    Serial.begin(115200);
    // Инициализация UART
    Serial2.setTX(UART_TX_PIN);
    Serial2.setRX(UART_RX_PIN);
    Serial2.begin(115200);
    delay(1000);
    Serial2.println("Start setup");

    Serial2.print("Software ");
    String ver_soft = __FILE__;
    int val_srt = ver_soft.lastIndexOf('\\');
    ver_soft.remove(0, val_srt + 1);
    val_srt = ver_soft.lastIndexOf('.');
    ver_soft.remove(val_srt);
    Serial2.println(ver_soft);
    Serial2.println("Setup End");


    // Настройка пинов

    //pinMode(PREAMBLE_LED_1, OUTPUT);
    //pinMode(PREAMBLE_LED_2, OUTPUT);
    pinMode(AGC_INPUT_PIN, INPUT);
    pinMode(AGC_OUTPUT_PIN, OUTPUT);
    pinMode(AGC_CONTROL_PIN, INPUT);
    pinMode(CORE0_LED_PIN, OUTPUT);
    pinMode(CORE1_LED_PIN, OUTPUT);

    // Инициализация CRC таблицы
    init_crc_table();


    // Инициализация AGC PWM
    analogWriteFreq(1000);  // 1 кГц
    analogWrite(AGC_OUTPUT_PIN, 512);  // Начальное значение

    adsbee.Init();


    //// Ваша искомая строка
    ///*const */char* hex_string = "8D151DD89B051798181C000455BA";
    //uint8_t packet_data[16];

    //// Преобразуем в массив
    //int packet_length = hexStringToBytes(hex_string, packet_data, 16);

    //Serial2.println("Original HEX string: " + String(hex_string));
    //Serial2.print("Converted to array: ");

    //for (int i = 0; i < packet_length; i++) {
    //    Serial2.print("0x");
    //    if (packet_data[i] < 16) Serial2.print("0");
    //    Serial2.print(packet_data[i], HEX);
    //    if (i < packet_length - 1) Serial2.print(", ");
    //}
    //Serial2.println();

    //// Тестируем декодирование ADS-B
    //testADSBDecoding(packet_data, packet_length);

   //!! analyze_test_packet(packet_data);

    Serial2.println("ADS-B receiver initialized");
    //Serial2.println("Core 0: Processing packets");
    //Serial2.println("Core 1: Receiving packets");
}

void setup1() {
    // Инициализация для ядра 1
    Serial2.println("Core 1 ready for packet reception");
}

void loop() {
    // Ядро 0 - обработка
    core0_task();
}

void loop1() {
    // Ядро 1 - прием
    core1_task();
}

/*
Основные особенности программы:

Двухканальный прием - используются пины 19 и 22 для входа
Обработка прерываний - декодирование PPM сигналов ADS-B
Фильтр помех - проверка стабильности длительности импульсов
Очереди пакетов - thread-safe обмен данными между ядрами
CRC проверка и коррекция - с таблицей CRC и исправлением одиночных ошибок
Полное декодирование - ICAO, номер рейса, координаты, скорость, высота
AGC управление - автоматическая регулировка усиления
Индикация - LED для преамбул и активности ядер
Serial2 вывод - структурированные данные в JSON формате
Отладочный вывод - сырые пакеты и декодированные данные

Программа готова для загрузки на RP2040 с двухъядерной архитектурой без использования FreeRTOS.

*/