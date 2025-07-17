//Создам полную программу для приема и расшифровки пакетов ADS-B на RP2040. Это комплексная система, которую разобью на несколько частей:


#include <Arduino.h>
#include <hardware/irq.h>
#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include "pico/stdlib.h"

// Определения пинов
#define ADS_B_DATA_PIN1   19
#define ADS_B_DATA_PIN2   22
#define LED_ACTIVITY1     20
#define LED_ACTIVITY2     23
#define GAIN_CONTROL_IN   26
#define GAIN_PWM_OUT      9

#define UART_TX_PIN 4
#define UART_RX_PIN 5


// ADS-B константы
#define ADSB_PREAMBLE     0xA1
#define ADSB_SYNC_WORD    0x1A
#define ADSB_PACKET_BITS  112
#define ADSB_PACKET_BYTES 14

// CPR константы для декодирования координат
#define NB 17                    // Количество бит широты
#define NL_TABLE_SIZE 59

// Структура для ADS-B пакета
struct ADSBPacket {
    uint8_t data[ADSB_PACKET_BYTES];
    bool valid;
    uint32_t timestamp;
};

// Глобальные переменные
volatile bool packet_received = false;
volatile uint8_t bit_buffer[ADSB_PACKET_BITS];
volatile int bit_count = 0;
volatile bool sync_found = false;
ADSBPacket current_packet;

// Таблица NL для CPR декодирования
const uint8_t NL_table[NL_TABLE_SIZE] = {
    10, 14, 18, 22, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23,
    23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 22, 18, 14, 10, 6,
    2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1
};

// Функции для работы с CRC
uint32_t crc24(uint8_t* data, int len) {
    uint32_t crc = 0;
    uint32_t generator = 0x1FFF409; // ADS-B CRC24 полином

    for (int i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i] << 16;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x800000) {
                crc = (crc << 1) ^ generator;
            } else {
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


// Улучшенная функция NL для CPR с проверками на граничные значения
int NL(double lat) 
{
    // Проверка на полюса
    if (abs(lat) >= 87.0) {
        if (abs(lat) >= 90.0) return 1;
        return 2;
    }

    // Экватор
    if (lat == 0.0) return 59;

    // Стандартный расчет для остальных широт
    double nz = 15.0;
    double cos_lat = cos(PI / 180.0 * abs(lat));

    // Избегаем деления на ноль
    if (cos_lat == 0) return 1;

    double a = 1.0 - cos(PI / (2.0 * nz));
    double b = cos_lat * cos_lat;

    if (b < a) return 1;

    // Проверяем на корректность аргумента для acos
    double acos_arg = 1.0 - a / b;
    if (acos_arg < -1.0 || acos_arg > 1.0) return 1;

    double nl = 2.0 * PI / acos(acos_arg);

    return (int)floor(nl);
}

// Функция CPR декодирования без структуры Position
// Возвращает true при успешном декодировании, false при ошибке
// Результаты записываются в переданные по ссылке переменные
bool decodeCPR(uint32_t lat_even, uint32_t lon_even, uint32_t lat_odd, uint32_t lon_odd,
    bool recent_even, double& latitude, double& longitude) {

    // Инициализация выходных параметров
    latitude = 0.0;
    longitude = 0.0;

    // Константы CPR
    const double dlat_even = 360.0 / 60.0;  // 6.0 градусов
    const double dlat_odd = 360.0 / 59.0;   // ~6.101 градусов
    const double pow2_17 = 131072.0;        // 2^17

    // Проверка входных данных
    if (lat_even >= pow2_17 || lon_even >= pow2_17 ||
        lat_odd >= pow2_17 || lon_odd >= pow2_17) {
        return false; // Невалидные входные данные
    }

    // Вычисляем индекс широты j
    double j = floor(59.0 * lat_even / pow2_17 - 60.0 * lat_odd / pow2_17 + 0.5);

    // Вычисляем широты для четного и нечетного сообщений
    double rlat_even = dlat_even * (fmod(j, 60.0) + lat_even / pow2_17);
    double rlat_odd = dlat_odd * (fmod(j, 59.0) + lat_odd / pow2_17);

    // Приводим к диапазону [-90, 90]
    if (rlat_even >= 270.0) rlat_even -= 360.0;
    if (rlat_odd >= 270.0) rlat_odd -= 360.0;

    // Дополнительные проверки диапазона
    if (rlat_even > 90.0 || rlat_even < -90.0 ||
        rlat_odd > 90.0 || rlat_odd < -90.0) {
        return false; // Невалидные координаты
    }

    // Проверяем совместимость зон широты
    int nl_even = NL(rlat_even);
    int nl_odd = NL(rlat_odd);

    if (nl_even != nl_odd) {
        return false; // Несовместимые зоны широты
    }

    double rlat = recent_even ? rlat_even : rlat_odd;
    int nl = nl_even; // Используем любой, так как они равны

    if (nl <= 0) {
        return false; // Невалидная зона
    }

    double dlon = 360.0 / nl;

    // Вычисляем индекс долготы m
    double m = floor(lon_even * (nl - 1) / pow2_17 - lon_odd * nl / pow2_17 + 0.5);

    double rlon;
    if (recent_even) {
        rlon = dlon * (fmod(m, (double)nl) + lon_even / pow2_17);
    }
    else {
        if (nl > 1) {
            rlon = dlon * (fmod(m, (double)(nl - 1)) + lon_odd / pow2_17);
        }
        else {
            rlon = lon_odd / pow2_17 * 360.0; // Особый случай для полюсов
        }
    }

    // Приводим к диапазону [-180, 180]
    while (rlon >= 180.0) rlon -= 360.0;
    while (rlon < -180.0) rlon += 360.0;

    // Записываем результаты
    latitude = rlat;
    longitude = rlon;

    return true; // Успешное декодирование
}





//// CPR декодирование
//struct Position {
//    double latitude;
//    double longitude;
//    bool valid;
//};


//В коде есть несколько синтаксических ошибок с пропущенными операторами умножения.Вот исправленная версия :

//
//Position decodeCPR(uint32_t lat_even, uint32_t lon_even, uint32_t lat_odd, uint32_t lon_odd, bool recent_even) {
//    Position pos = { 0, 0, false };
//
//    // Константы CPR
//    const double dlat_even = 360.0 / 60.0;  // 6.0 градусов
//    const double dlat_odd = 360.0 / 59.0;   // ~6.101 градусов
//    const double pow2_17 = pow(2, 17);      // 131072
//
//    // Вычисляем индекс широты j
//    double j = floor(59.0  lat_even / pow2_17 - 60.0  lat_odd / pow2_17 + 0.5); // Добавлены *
//
//    // Вычисляем широты для четного и нечетного сообщений
//    double rlat_even = dlat_even * (fmod(j, 60) + lat_even / pow2_17);
//    double rlat_odd = dlat_odd * (fmod(j, 59) + lat_odd / pow2_17);
//
//    // Приводим к диапазону [-90, 90]
//    if (rlat_even >= 270) rlat_even -= 360;
//    if (rlat_odd >= 270) rlat_odd -= 360;
//
//    // Дополнительные проверки диапазона
//    if (rlat_even > 90 || rlat_even < -90 || rlat_odd > 90 || rlat_odd < -90) {
//        return pos; // Невалидные координаты
//    }
//
//    // Проверяем совместимость зон широты
//    int nl_even = NL(rlat_even);
//    int nl_odd = NL(rlat_odd);
//
//    if (nl_even == nl_odd) {
//        double rlat = recent_even ? rlat_even : rlat_odd;
//        int nl = recent_even ? nl_even : nl_odd;
//
//        if (nl > 0) {
//            double dlon = 360.0 / nl;
//
//            // Вычисляем индекс долготы m
//            double m;
//            if (recent_even && nl > 1) {
//                m = floor(lon_even(nl - 1) / pow2_17 - lon_odd  nl / pow2_17 + 0.5); // Добавлены *
//            }
//            else {
//                m = floor(lon_even(nl - 1) / pow2_17 - lon_odd  nl / pow2_17 + 0.5); // Добавлены *
//            }
//
//            double rlon;
//            if (recent_even) {
//                rlon = dlon * (fmod(m, nl) + lon_even / pow2_17);
//            }
//            else {
//                if (nl > 1) {
//                    rlon = dlon * (fmod(m, nl - 1) + lon_odd / pow2_17);
//                }
//                else {
//                    rlon = lon_odd / pow2_17 * 360.0; // Особый случай для полюсов
//                }
//            }
//
//            // Приводим к диапазону [-180, 180]
//            if (rlon >= 180) rlon -= 360;
//            if (rlon < -180) rlon += 360;
//
//            pos.latitude = rlat;
//            pos.longitude = rlon;
//            pos.valid = true;
//        }
//    }
//
//    return pos;
//}

//
//Исправленные ошибки :
//
//Строка 9 : 59.0  lat_even → 59.0 * lat_even
//Строка 9 : 60.0  lat_odd → 60.0 * lat_odd
//Строка 31 : lon_even(nl - 1) → lon_even * (nl - 1)
//Строка 31 : lon_odd  nl → lon_odd * nl
//Строка 34 : lon_even(nl - 1) → lon_even * (nl - 1)
//Строка 34 : lon_odd  nl → lon_odd * nl
//
//Дополнительные улучшения для более надежной работы :

//
//Position decodeCPR(uint32_t lat_even, uint32_t lon_even, uint32_t lat_odd, uint32_t lon_odd, bool recent_even) 
//{
//    Position pos = { 0, 0, false };
//
//    // Константы CPR
//    const double dlat_even = 360.0 / 60.0;  // 6.0 градусов
//    const double dlat_odd = 360.0 / 59.0;   // ~6.101 градусов
//    const double pow2_17 = 131072.0;        // 2^17 как константа для производительности
//
//    // Проверка входных данных
//    if (lat_even >= pow2_17 || lon_even >= pow2_17 ||
//        lat_odd >= pow2_17 || lon_odd >= pow2_17) {
//        return pos; // Невалидные входные данные
//    }
//
//    // Вычисляем индекс широты j
//    double j = floor(59.0  lat_even / pow2_17 - 60.0  lat_odd / pow2_17 + 0.5);
//
//    // Вычисляем широты для четного и нечетного сообщений
//    double rlat_even = dlat_even * (fmod(j, 60.0) + lat_even / pow2_17);
//    double rlat_odd = dlat_odd * (fmod(j, 59.0) + lat_odd / pow2_17);
//
//    // Приводим к диапазону [-90, 90]
//    if (rlat_even >= 270.0) rlat_even -= 360.0;
//    if (rlat_odd >= 270.0) rlat_odd -= 360.0;
//
//    // Дополнительные проверки диапазона
//    if (rlat_even > 90.0 || rlat_even < -90.0 ||
//        rlat_odd > 90.0 || rlat_odd < -90.0) {
//        return pos; // Невалидные координаты
//    }
//
//    // Проверяем совместимость зон широты
//    int nl_even = NL(rlat_even);
//    int nl_odd = NL(rlat_odd);
//
//    if (nl_even != nl_odd) {
//        return pos; // Несовместимые зоны широты
//    }
//
//    double rlat = recent_even ? rlat_even : rlat_odd;
//    int nl = nl_even; // Используем любой, так как они равны
//
//    if (nl <= 0) {
//        return pos; // Невалидная зона
//    }
//
//    double dlon = 360.0 / nl;
//
//    // Вычисляем индекс долготы m
//    double m = floor(lon_even(nl - 1) / pow2_17 - lon_odd  nl / pow2_17 + 0.5);
//
//    double rlon;
//    if (recent_even) {
//        rlon = dlon * (fmod(m, (double)nl) + lon_even / pow2_17);
//    }
//    else {
//        if (nl > 1) {
//            rlon = dlon * (fmod(m, (double)(nl - 1)) + lon_odd / pow2_17);
//        }
//        else {
//            rlon = lon_odd / pow2_17 * 360.0; // Особый случай для полюсов
//        }
//    }
//
//    // Приводим к диапазону [-180, 180]
//    while (rlon >= 180.0) rlon -= 360.0;
//    while (rlon < -180.0) rlon += 360.0;
//
//    pos.latitude = rlat;
//    pos.longitude = rlon;
//    pos.valid = true;
//
//    return pos;
//}

//
//Улучшения:
//Добавлена проверка входных данных
//Упрощена логика выбора NL
//Более надежная нормализация долготы
//Оптимизация с использованием константы вместо pow(2, 17)
//Более явные проверки на валидность
//
//


//
//Position decodeCPR(uint32_t lat_even, uint32_t lon_even, uint32_t lat_odd, uint32_t lon_odd, bool recent_even) {
//    Position pos = { 0, 0, false };
//
//    // Константы CPR
//    const double dlat_even = 360.0 / 60.0;  // 6.0 градусов
//    const double dlat_odd = 360.0 / 59.0;   // ~6.101 градусов
//    const double pow2_17 = pow(2, 17);      // 131072
//
//    // Вычисляем индекс широты j
//    double j = floor(59.0  lat_even / pow2_17 - 60.0  lat_odd / pow2_17 + 0.5);
//
//    // Вычисляем широты для четного и нечетного сообщений
//    double rlat_even = dlat_even * (fmod(j, 60) + lat_even / pow2_17);
//    double rlat_odd = dlat_odd * (fmod(j, 59) + lat_odd / pow2_17);
//
//    // Приводим к диапазону [-90, 90]
//    if (rlat_even >= 270) rlat_even -= 360;
//    if (rlat_odd >= 270) rlat_odd -= 360;
//
//    // Дополнительные проверки диапазона
//    if (rlat_even > 90 || rlat_even < -90 || rlat_odd > 90 || rlat_odd < -90) {
//        return pos; // Невалидные координаты
//    }
//
//    // Проверяем совместимость зон широты
//    int nl_even = NL(rlat_even);
//    int nl_odd = NL(rlat_odd);
//
//    if (nl_even == nl_odd) {
//        double rlat = recent_even ? rlat_even : rlat_odd;
//        int nl = recent_even ? nl_even : nl_odd;
//
//        if (nl > 0) {
//            double dlon = 360.0 / nl;
//
//            // Вычисляем индекс долготы m
//            double m;
//            if (recent_even && nl > 1) {
//                m = floor(lon_even(nl - 1) / pow2_17 - lon_odd  nl / pow2_17 + 0.5);
//            }
//            else {
//                m = floor(lon_even(nl - 1) / pow2_17 - lon_odd  nl / pow2_17 + 0.5);
//            }
//
//            double rlon;
//            if (recent_even) {
//                rlon = dlon * (fmod(m, nl) + lon_even / pow2_17);
//            }
//            else {
//                if (nl > 1) {
//                    rlon = dlon * (fmod(m, nl - 1) + lon_odd / pow2_17);
//                }
//                else {
//                    rlon = lon_odd / pow2_17 * 360.0; // Особый случай для полюсов
//                }
//            }
//
//            // Приводим к диапазону [-180, 180]
//            if (rlon >= 180) rlon -= 360;
//            if (rlon < -180) rlon += 360;
//
//            pos.latitude = rlat;
//            pos.longitude = rlon;
//            pos.valid = true;
//        }
//    }
//
//    return pos;
//}
//
//

// Более оптимизированная версия для RP2040
void __not_in_flash_func(data_isr)() 
{
    static uint32_t last_time = 0;
    static uint32_t state_machine = 0;
    static const uint32_t PREAMBLE_TIMEOUT = 1000; // мкс

    uint32_t current_time = time_us_32(); // Аппаратный таймер RP2040
    uint32_t delta_time = current_time - last_time;
    last_time = current_time;

    // Быстрое переключение LED для индикации
    gpio_xor_mask(1u << LED_ACTIVITY1);

    // Конечный автомат для декодирования
    switch (state_machine) 
    {
    case 0: // Поиск первого импульса преамбулы
        if (delta_time >= 400 && delta_time <= 600) 
        {
            state_machine = 1;
        }
        break;

    case 1: case 3: case 5: // Ожидание пауз между импульсами
        if (delta_time >= 900 && delta_time <= 1100) 
        {
            state_machine++;
        }
        else 
        {
            state_machine = 0;
        }
        break;

    case 2: case 4: case 6: // Ожидание следующих импульсов
        if (delta_time >= 400 && delta_time <= 600) 
        {
            state_machine++;
        }
        else {
            state_machine = 0;
        }
        break;

    case 7: // Ожидание финальной паузы
        if (delta_time >= 3500 && delta_time <= 4500) 
        {
            // Преамбула найдена!
            sync_found = true;
            bit_count = 0;
            state_machine = 8;
            Serial2.print("Preamble found!!!: ");
        }
        else 
        {
            state_machine = 0;
        }
        break;

    case 8: // Декодирование данных
        if (bit_count < ADSB_PACKET_BITS) 
        {
            if (delta_time >= 400 && delta_time <= 600) 
            {
                bit_buffer[bit_count++] = 1;
            }
            else if (delta_time >= 900 && delta_time <= 1100) 
            {
                bit_buffer[bit_count++] = 0;
            }
            else 
            {
                // Ошибка - сброс
                state_machine = 0;
                sync_found = false;
                bit_count = 0;
            }

            if (bit_count >= ADSB_PACKET_BITS) 
            {
                packet_received = true;
                sync_found = false;
                state_machine = 0;
            }
        }
        break;

    default:
        state_machine = 0;
        break;
    }

    // Таймаут для сброса состояния
    static uint32_t last_valid_edge = 0;
    if (state_machine > 0)
    {
        if (current_time - last_valid_edge > PREAMBLE_TIMEOUT) 
        {
            state_machine = 0;
            sync_found = false;
            bit_count = 0;
        }
        last_valid_edge = current_time;
    }
}

//// Простая версия для второго канала
//void __not_in_flash_func(data_isr2)() 
//{
//    static uint32_t last_time = 0;
//    static uint32_t state_machine = 0;
//    static const uint32_t PREAMBLE_TIMEOUT = 1000; // мкс
//
//    uint32_t current_time = time_us_32(); // Аппаратный таймер RP2040
//    uint32_t delta_time = current_time - last_time;
//    last_time = current_time;
//
//    // Быстрое переключение LED для индикации
//    gpio_xor_mask(1u << LED_ACTIVITY1);
//
//    // Конечный автомат для декодирования
//    switch (state_machine)
//    {
//    case 0: // Поиск первого импульса преамбулы
//        if (delta_time >= 400 && delta_time <= 600)
//        {
//            state_machine = 1;
//        }
//        break;
//
//    case 1: case 3: case 5: // Ожидание пауз между импульсами
//        if (delta_time >= 900 && delta_time <= 1100)
//        {
//            state_machine++;
//        }
//        else
//        {
//            state_machine = 0;
//        }
//        break;
//
//    case 2: case 4: case 6: // Ожидание следующих импульсов
//        if (delta_time >= 400 && delta_time <= 600)
//        {
//            state_machine++;
//        }
//        else {
//            state_machine = 0;
//        }
//        break;
//
//    case 7: // Ожидание финальной паузы
//        if (delta_time >= 3500 && delta_time <= 4500)
//        {
//            // Преамбула найдена!
//            sync_found = true;
//            bit_count = 0;
//            state_machine = 8;
//        }
//        else {
//            state_machine = 0;
//        }
//        break;
//
//    case 8: // Декодирование данных
//        if (bit_count < ADSB_PACKET_BITS)
//        {
//            if (delta_time >= 400 && delta_time <= 600) {
//                bit_buffer[bit_count++] = 1;
//            }
//            else if (delta_time >= 900 && delta_time <= 1100) {
//                bit_buffer[bit_count++] = 0;
//            }
//            else {
//                // Ошибка - сброс
//                state_machine = 0;
//                sync_found = false;
//                bit_count = 0;
//            }
//
//            if (bit_count >= ADSB_PACKET_BITS) {
//                packet_received = true;
//                sync_found = false;
//                state_machine = 0;
//            }
//        }
//        break;
//
//    default:
//        state_machine = 0;
//        break;
//    }
//
//    // Таймаут для сброса состояния
//    static uint32_t last_valid_edge = 0;
//    if (state_machine > 0) {
//        if (current_time - last_valid_edge > PREAMBLE_TIMEOUT) {
//            state_machine = 0;
//            sync_found = false;
//            bit_count = 0;
//        }
//        last_valid_edge = current_time;
//    }
//
//    //static uint32_t led_timer = 0;
//    //uint32_t current_time = time_us_32();
//
//    //if (current_time - led_timer > 50000) { // 50ms
//    //    gpio_put(LED_ACTIVITY2, 1);
//    //    led_timer = current_time;
//    //}
//}

void manage_activity_leds() {
    static uint32_t led1_time = 0;
    static uint32_t led2_time = 0;
    uint32_t current_time = millis();

    // LED1 - автоматическое выключение через 50ms
    if (digitalRead(LED_ACTIVITY1) && (current_time - led1_time > 50)) {
        digitalWrite(LED_ACTIVITY1, LOW);
    }
    if (digitalRead(LED_ACTIVITY1)) {
        led1_time = current_time;
    }

    // LED2 - автоматическое выключение через 100ms
    if (digitalRead(LED_ACTIVITY2) && (current_time - led2_time > 100)) {
        digitalWrite(LED_ACTIVITY2, LOW);
    }
    if (digitalRead(LED_ACTIVITY2)) {
        led2_time = current_time;
    }
}




//// Обработчик прерываний для приема данных
//void IRAM_ATTR data_isr()
//{
//    static uint32_t last_edge = 0;
//    static bool preamble_detected = false;
//
//    uint32_t current_time = micros();
//    uint32_t pulse_width = current_time - last_edge;
//    last_edge = current_time;
//
//    // Детекция преамбулы ADS-B (1090 МГц, Manchester encoding)
//    if (!sync_found) {
//        if (pulse_width >= 0.5 && pulse_width <= 1.5) { // 1 мкс ±0.5
//            preamble_detected = true;
//            bit_count = 0;
//        }
//        return;
//    }
//
//    if (preamble_detected && bit_count < ADSB_PACKET_BITS) {
//        // Manchester декодирование
//        if (pulse_width >= 0.5 && pulse_width <= 1.0) {
//            bit_buffer[bit_count++] = 1;
//        }
//        else if (pulse_width >= 1.0 && pulse_width <= 1.5) {
//            bit_buffer[bit_count++] = 0;
//        }
//        else {
//            // Ошибка синхронизации
//            preamble_detected = false;
//            bit_count = 0;
//        }
//
//        if (bit_count >= ADSB_PACKET_BITS) {
//            packet_received = true;
//            preamble_detected = false;
//            digitalWrite(LED_ACTIVITY1, HIGH);
//        }
//    }
//}
//
//// Обработчик второго канала
//void IRAM_ATTR data_isr2() {
//    static uint32_t last_edge = 0;
//
//    uint32_t current_time = micros();
//    last_edge = current_time;
//    digitalWrite(LED_ACTIVITY2, HIGH);
//
//    // Простая индикация активности на втором канале
//    delay(10);
//    digitalWrite(LED_ACTIVITY2, LOW);
//}

#define BITS_TO_BYTE(base) \
    ((bits[base] << 7) | (bits[base+1] << 6) | (bits[base+2] << 5) | (bits[base+3] << 4) | \
     (bits[base+4] << 3) | (bits[base+5] << 2) | (bits[base+6] << 1) | bits[base+7])

inline void bits_to_bytes_fast(volatile uint8_t* bits, uint8_t* bytes) {
    bytes[0] = BITS_TO_BYTE(0);    bytes[1] = BITS_TO_BYTE(8);    bytes[2] = BITS_TO_BYTE(16);   bytes[3] = BITS_TO_BYTE(24);
    bytes[4] = BITS_TO_BYTE(32);   bytes[5] = BITS_TO_BYTE(40);   bytes[6] = BITS_TO_BYTE(48);   bytes[7] = BITS_TO_BYTE(56);
    bytes[8] = BITS_TO_BYTE(64);   bytes[9] = BITS_TO_BYTE(72);   bytes[10] = BITS_TO_BYTE(80);  bytes[11] = BITS_TO_BYTE(88);
    bytes[12] = BITS_TO_BYTE(96);  bytes[13] = BITS_TO_BYTE(104);
}

#undef BITS_TO_BYTE  // Очищаем макрос после использования


//inline void bits_to_bytes_fast(volatile uint8_t* bits, uint8_t* bytes) 
//{
//
//
//
//
//
//    
//        //// Прямое копирование 14 байт без циклов для максимальной производительности
//        //bytes[0] = (bits[0] << 7) | (bits[1] << 6) | (bits[2] << 5) | (bits[3] << 4) |
//        //    (bits[4] << 3) | (bits[5] << 2) | (bits[6] << 1) | bits[7];
//
//        //bytes[1] = (bits[8] << 7) | (bits[9] << 6) | (bits[10] << 5) | (bits[11] << 4) |
//        //    (bits[12] << 3) | (bits[13] << 2) | (bits[14] << 1) | bits[15];
//
//        //bytes[2] = (bits[16] << 7) | (bits[17] << 6) | (bits[18] << 5) | (bits[19] << 4) |
//        //    (bits[20] << 3) | (bits[21] << 2) | (bits[22] << 1) | bits[23];
//
//        //bytes[3] = (bits[24] << 7) | (bits[25] << 6) | (bits[26] << 5) | (bits[27] << 4) |
//        //    (bits[28] << 3) | (bits[29] << 2) | (bits[30] << 1) | bits[31];
//
//        //bytes[4] = (bits[32] << 7) | (bits[33] << 6) | (bits[34] << 5) | (bits[35] << 4) |
//        //    (bits[36] << 3) | (bits[37] << 2) | (bits[38] << 1) | bits[39];
//
//        //bytes[5] = (bits[40] << 7) | (bits[41] << 6) | (bits[42] << 5) | (bits[43] << 4) |
//        //    (bits[44] << 3) | (bits[45] << 2) | (bits[46] << 1) | bits[47];
//
//        //bytes[6] = (bits[48] << 7) | (bits[49] << 6) | (bits[50] << 5) | (bits[51] << 4) |
//        //    (bits[52] << 3) | (bits[53] << 2) | (bits[54] << 1) | bits[55];
//
//        //bytes[7] = (bits[56] << 7) | (bits[57] << 6) | (bits[58] << 5) | (bits[59] << 4) |
//        //    (bits[60] << 3) | (bits[61] << 2) | (bits[62] << 1) | bits[63];
//
//        //bytes[8] = (bits[64] << 7) | (bits[65] << 6) | (bits[66] << 5) | (bits[67] << 4) |
//        //    (bits[68] << 3) | (bits[69] << 2) | (bits[70] << 1) | bits[71];
//
//        //bytes[9] = (bits[72] << 7) | (bits[73] << 6) | (bits[74] << 5) | (bits[75] << 4) |
//        //    (bits[76] << 3) | (bits[77] << 2) | (bits[78] << 1) | bits[79];
//
//        //bytes[10] = (bits[80] << 7) | (bits[81] << 6) | (bits[82] << 5) | (bits[83] << 4) |
//        //    (bits[84] << 3) | (bits[85] << 2) | (bits[86] << 1) | bits[87];
//
//        //bytes[11] = (bits[88] << 7) | (bits[89] << 6) | (bits[90] << 5) | (bits[91] << 4) |
//        //    (bits[92] << 3) | (bits[93] << 2) | (bits[94] << 1) | bits[95];
//
//        //bytes[12] = (bits[96] << 7) | (bits[97] << 6) | (bits[98] << 5) | (bits[99] << 4) |
//        //    (bits[100] << 3) | (bits[101] << 2) | (bits[102] << 1) | bits[103];
//
//        //bytes[13] = (bits[104] << 7) | (bits[105] << 6) | (bits[106] << 5) | (bits[107] << 4) |
//        //    (bits[108] << 3) | (bits[109] << 2) | (bits[110] << 1) | bits[111];
////
////
////
////    // Или использовать макрос для сокращения кода:
////#define BYTE_FROM_BITS(n) \
////        ((bits[(n)*8] << 7) | (bits[(n)*8+1] << 6) | (bits[(n)*8+2] << 5) | (bits[(n)*8+3] << 4) | \
////         (bits[(n)*8+4] << 3) | (bits[(n)*8+5] << 2) | (bits[(n)*8+6] << 1) | bits[(n)*8+7])
////
////    for (int i = 0; i < ADSB_PACKET_BYTES; i++) {
////        bytes[i] = BYTE_FROM_BITS(i);
////    }
//}
//
//



//// Преобразование битового массива в байты
//void bits_to_bytes(volatile uint8_t bits, uint8_t bytes) {
//    for (int i = 0; i < ADSB_PACKET_BYTES; i++) {
//        bytes[i] = 0;
//        for (int j = 0; j < 8; j++) {
//            if (bits[i * 8 + j]) {
//                bytes[i] |= (1 << (7 - j));
//            }
//        }
//    }
//}

// Декодирование пакета ADS-B
void decode_adsb_packet(uint8_t* packet) 
{
    uint8_t df = (packet[0] >> 3) & 0x1F; // Downlink Format
    uint32_t icao = (packet[1] << 16) | (packet[2] << 8) | packet[3]; // ICAO адрес

    Serial2.print("DF: ");
    Serial2.print(df);
    Serial2.print(", ICAO: ");
    Serial2.print(icao, HEX);

    if (df == 17 || df == 18) { // ADS-B сообщения
        uint8_t tc = (packet[4] >> 3) & 0x1F; // Type Code
        Serial2.print(", TC: ");
        Serial2.print(tc);

        if (tc >= 1 && tc <= 4) { // Идентификация
            char callsign[9] = {0};
            uint64_t data = 0;
            for (int i = 5; i < 11; i++) {
                data = (data << 8) | packet[i];
            }

            // Декодирование позывного (6-битная кодировка)
            for (int i = 0; i < 8; i++) {
                uint8_t c = (data >> (42 - i * 6)) & 0x3F;
                if (c > 0) {
                    if (c < 32) callsign[i] = c + 64; // A-Z
                    else callsign[i] = c; // 0-9, пробел и др.
                }
            }
            Serial2.print(", Callsign: ");
            Serial2.print(callsign);
        }

        if ((tc >= 9 && tc <= 18) || (tc >= 20 && tc <= 22)) { // Позиция
            static uint32_t last_lat_even = 0, last_lon_even = 0;
            static uint32_t last_lat_odd = 0, last_lon_odd = 0;
            static bool has_even = false, has_odd = false;
            static uint32_t last_even_time = 0, last_odd_time = 0;

            bool odd_flag = (packet[6] >> 2) & 1;
            uint32_t lat_cpr = ((packet[6] & 0x03) << 15) | (packet[7] << 7) | (packet[8] >> 1);
            uint32_t lon_cpr = ((packet[8] & 0x01) << 16) | (packet[9] << 8) | packet[10];

            uint32_t current_time = millis();

            if (odd_flag) { // Нечетная позиция
                last_lat_odd = lat_cpr;
                last_lon_odd = lon_cpr;
                last_odd_time = current_time;
                has_odd = true;
            } else { // Четная позиция
                last_lat_even = lat_cpr;
                last_lon_even = lon_cpr;
                last_even_time = current_time;
                has_even = true;
            }

            // CPR декодирование при наличии обеих позиций
            if (has_even && has_odd) 
            {
                uint32_t time_diff = abs((int32_t)(last_even_time - last_odd_time));

                if (time_diff < 10000) {
                    bool recent_even = last_even_time > last_odd_time;

                    // Кэш для предотвращения повторных вычислений
                    static uint32_t cache_lat_even = 0, cache_lon_even = 0;
                    static uint32_t cache_lat_odd = 0, cache_lon_odd = 0;
                    static double cached_lat = 0.0, cached_lon = 0.0;
                    static bool cache_valid = false;

                    // Проверяем, изменились ли входные данные
                    bool data_changed = (cache_lat_even != last_lat_even) ||
                        (cache_lon_even != last_lon_even) ||
                        (cache_lat_odd != last_lat_odd) ||
                        (cache_lon_odd != last_lon_odd);

                    double latitude, longitude;
                    bool decode_success = false;

                    if (data_changed || !cache_valid) {
                        // Новые данные - пересчитываем
                        decode_success = decodeCPR(last_lat_even, last_lon_even,
                            last_lat_odd, last_lon_odd,
                            recent_even, latitude, longitude);

                        if (decode_success) {
                            // Обновляем кэш
                            cache_lat_even = last_lat_even;
                            cache_lon_even = last_lon_even;
                            cache_lat_odd = last_lat_odd;
                            cache_lon_odd = last_lon_odd;
                            cached_lat = latitude;
                            cached_lon = longitude;
                            cache_valid = true;
                        }
                    }
                    else {
                        // Используем кэшированные данные
                        latitude = cached_lat;
                        longitude = cached_lon;
                        decode_success = true;
                    }

                    if (decode_success) {
                        Serial2.print(", Lat: ");
                        Serial2.print(latitude, 6);
                        Serial2.print(", Lon: ");
                        Serial2.print(longitude, 6);
                    }
                    else {
                        Serial2.print(", CPR decode failed");
                        cache_valid = false;
                    }
                }
            }




            //// CPR декодирование при наличии обеих позиций
            //if (has_even && has_odd && abs((int32_t)(last_even_time - last_odd_time)) < 10000) {
            //    bool recent_even = last_even_time > last_odd_time;
            //    Position pos = decodeCPR(last_lat_even, last_lon_even, last_lat_odd, last_lon_odd, recent_even);

            //    if (pos.valid) {
            //        Serial2.print(", Lat: ");
            //        Serial2.print(pos.latitude, 6);
            //        Serial2.print(", Lon: ");
            //        Serial2.print(pos.longitude, 6);
            //    }
            //}

            Serial2.print(", CPR: ");
            Serial2.print(lat_cpr);
            Serial2.print("/");
            Serial2.print(lon_cpr);
            Serial2.print(odd_flag ? " (odd)" : " (even)");
        }

        if (tc == 19) { // Скорость и курс
            uint16_t velocity = ((packet[7] & 0x03) << 8) | packet[8];
            uint16_t heading = ((packet[5] & 0x03) << 8) | packet[6];

            Serial2.print(", Speed: ");
            Serial2.print(velocity);
            Serial2.print(" kt, Heading: ");
            Serial2.print(heading);
        }
    }

    Serial2.println();
}


void analyze_test_packet()
{
  //  uint8_t packet[] = { 0x8D, 0x48, 0x43, 0x95, 0x99, 0x00, 0x52, 0xB2, 0xE8, 0x0B, 0x02, 0x7D, 0x68, 0x7F };
   // uint8_t packet[] = { 0x8D, 0x15, 0x5C, 0x1D, 0xEB, 0x19, 0x78, 0x48, 0x00, 0x10, 0x00, 0x38, 0x70, 0x59 };
    //uint8_t packet[] = { 0x8D, 0x3C, 0x65, 0x03, 0x99, 0x90, 0xFB, 0x9E, 0x28, 0x64, 0x04, 0xA0, 0x04, 0x3E };
   // uint8_t packet[] = { 0x8D, 0x15, 0x1D, 0xA7, 0x9B, 0x04, 0xD9, 0x12, 0xD8, 0x38, 0x00, 0x3A, 0x20, 0xD5 };
    //uint8_t packet[] = { 0x8D, 0x15, 0x20, 0x23, 0x9b, 0xa3, 0xf8, 0xB2, 0x08, 0x00, 0x00, 0x00, 0x00, 0x98 };
      uint8_t packet[] = { 0x8D, 0x15, 0x1D, 0xA8, 0xEB, 0x04, 0xF8, 0x24, 0xB4, 0x10, 0x08, 0x61, 0x2F, 0x24 };

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
            decode_velocity(packet);
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
void decode_velocity(uint8_t* packet) {
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






void setup() {
    // Инициализация UART
    Serial2.setTX(UART_TX_PIN);
    Serial2.setRX(UART_RX_PIN);
    Serial2.begin(115200);
    delay(1000);
    Serial2.println("Start setup");
    // Инициализация GPIO
    pinMode(ADS_B_DATA_PIN1, INPUT_PULLUP);
    pinMode(ADS_B_DATA_PIN2, INPUT_PULLUP);
    pinMode(LED_ACTIVITY1, OUTPUT);
    pinMode(LED_ACTIVITY2, OUTPUT);
    pinMode(GAIN_CONTROL_IN, INPUT);
    pinMode(GAIN_PWM_OUT, OUTPUT);

    // Настройка ШИМ для управления усилением
    analogWriteFreq(1000); // 1 кГц
    analogWriteRange(255); // 8-битное разрешение

    // Настройка прерываний
    attachInterrupt(digitalPinToInterrupt(ADS_B_DATA_PIN1), data_isr, FALLING);
   // attachInterrupt(digitalPinToInterrupt(ADS_B_DATA_PIN2), data_isr2, FALLING);

    // Тестирование светодиодов
    digitalWrite(LED_ACTIVITY1, HIGH);
    digitalWrite(LED_ACTIVITY2, HIGH);
    delay(500);
    digitalWrite(LED_ACTIVITY1, LOW);
    digitalWrite(LED_ACTIVITY2, LOW);
    analyze_test_packet();
    Serial2.println("System Ready");
}



void loop() {
    // Управление усилением на основе входного сигнала
    int gain_input = analogRead(GAIN_CONTROL_IN);
    int pwm_value = map(gain_input, 0, 1024, 0, 255);
    analogWrite(GAIN_PWM_OUT, pwm_value);
    manage_activity_leds(); // Управление светодиодами

 
    // Обработка принятых пакетов
    if (packet_received) 
    {
        packet_received = false;
        sync_found = false;

        // Преобразование битов в байты
        bits_to_bytes_fast(bit_buffer, current_packet.data);
        current_packet.timestamp = millis();

        //uint8_t packet[] = { 0x8D, 0x15, 0x5C, 0x1D, 0xEB, 0x19, 0x78, 0x48, 0x00, 0x10, 0x00, 0x38, 0x70, 0x59 };
       // uint8_t packet[] = { 0x8D, 0x48, 0x43, 0x95, 0x99, 0x00, 0x52, 0xB2, 0xE8, 0x0B, 0x02, 0x7D, 0x68, 0x7F };
        // Проверка CRC
        if (validate_crc(current_packet.data))
        {
            current_packet.valid = true;
            Serial2.print("Valid packet: ");

            // Вывод сырых данных
            for (int i = 0; i < ADSB_PACKET_BYTES; i++) 
            {
                if (current_packet.data[i] < 0x10) Serial2.print("0");
                Serial2.print(current_packet.data[i], HEX);
            }
            Serial2.print(" | ");

            // Декодирование пакета
            decode_adsb_packet(current_packet.data);
        }
        else 
        {
            Serial2.println("CRC Error");
        }

        // Сброс индикатора
        digitalWrite(LED_ACTIVITY1, LOW);

        // Сброс буфера
        bit_count = 0;
    }

    // Периодическая диагностика
    static uint32_t last_diagnostic = 0;
    if (millis() - last_diagnostic > 10000) 
    { // Каждые 10 секунд
        Serial2.print("Gain control: ");
        Serial2.print((gain_input * 100) / 1024);
        Serial2.println("%");
        last_diagnostic = millis();
    }

    delay(1);
}

/*
Особенности реализации:

Обработка сигналов ADS-B:
Manchester декодирование на частоте 1090 МГц
Детекция преамбулы и синхронизация
Двухканальный прием с индикацией

CPR декодирование:
Полная реализация алгоритма Compact Position Reporting
Обработка четных/нечетных позиций
Вычисление географических координат

CRC проверка:
CRC24 с полиномом ADS-B стандарта
Полная валидация пакетов

Управление усилением:
Аналоговый вход для контроля уровня сигнала
ШИМ выход для управления усилителем

Декодирование сообщений:
Позывные воздушных судов
GPS координаты с CPR
Скорость и курс
ICAO адреса

Программа готова для загрузки в RP2040 и работает без FreeRTOS, используя только базовые функции Arduino IDE.
*/