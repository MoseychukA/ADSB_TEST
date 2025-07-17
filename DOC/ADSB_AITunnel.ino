Создам для вас программу для приема и расшифровки пакетов ADS-B на RP2040. Вот полный код:


#include <Arduino.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// Конфигурация пинов
#define ADS_B_INPUT_PIN 2    // Пин для входа ADS-B сигнала
#define LED_PIN 25           // Встроенный LED

// Параметры ADS-B
#define PREAMBLE_LENGTH 8    // Длина преамбулы
#define MESSAGE_LENGTH 112   // Длина сообщения в битах
#define SHORT_PULSE 0.5      // Короткий импульс (мкс)
#define LONG_PULSE 1.0       // Длинный импульс (мкс)
#define PULSE_TOLERANCE 0.3  // Допуск для распознавания импульсов

// Буфер для хранения принятых битов
uint8_t message_buffer[14]; // 112 бит = 14 байт
volatile bool message_ready = false;
volatile int bit_count = 0;
volatile uint32_t last_edge_time = 0;
volatile bool receiving = false;

// CRC таблица для проверки ADS-B пакетов
const uint32_t crc_table[256] = {
    0x00000000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    // ... (полная таблица CRC-24 для экономии места показана частично)
    // Полную таблицу можно сгенерировать или найти в стандарте ADS-B
};

void setup() {
    // Инициализация UART1
    Serial1.begin(115200);
    Serial1.println("ADS-B Decoder Starting...");

    // Инициализация LED
    pinMode(LED_PIN, OUTPUT);

    // Настройка входного пина для ADS-B
    pinMode(ADS_B_INPUT_PIN, INPUT);

    // Настройка прерывания на изменение уровня
    attachInterrupt(digitalPinToInterrupt(ADS_B_INPUT_PIN), onEdgeDetect, CHANGE);

    // Инициализация буфера
    memset(message_buffer, 0, sizeof(message_buffer));

    Serial1.println("ADS-B Decoder Ready!");
}

// Обработчик прерывания на изменение уровня сигнала
void onEdgeDetect() {
    uint32_t current_time = micros();
    uint32_t pulse_width = current_time - last_edge_time;

    if (!receiving) {
        // Проверка начала преамбулы
        if (checkPreamble(pulse_width)) {
            receiving = true;
            bit_count = 0;
            memset(message_buffer, 0, sizeof(message_buffer));
        }
    } else {
        // Декодирование битов данных
        if (decodeBit(pulse_width)) {
            if (bit_count >= MESSAGE_LENGTH) {
                receiving = false;
                message_ready = true;
            }
        } else {
            // Ошибка декодирования - сброс
            receiving = false;
            bit_count = 0;
        }
    }

    last_edge_time = current_time;
}

// Проверка преамбулы ADS-B
bool checkPreamble(uint32_t pulse_width) {
    // Упрощенная проверка преамбулы
    // Реальная проверка должна анализировать последовательность импульсов
    return (pulse_width >= 4 && pulse_width <= 8);
}

// Декодирование одного бита
bool decodeBit(uint32_t pulse_width) {
    uint8_t bit_value;

    // Определение значения бита по длительности импульса
    if (abs((int)pulse_width - (int)(SHORT_PULSE  1000000)) < (PULSE_TOLERANCE  1000000)) {
        bit_value = 0;
    } else if (abs((int)pulse_width - (int)(LONG_PULSE  1000000)) < (PULSE_TOLERANCE  1000000)) {
        bit_value = 1;
    } else {
        return false; // Неверная длительность импульса
    }

    // Сохранение бита в буфер
    int byte_index = bit_count / 8;
    int bit_index = 7 - (bit_count % 8);

    if (bit_value) {
        message_buffer[byte_index] |= (1 << bit_index);
    }

    bit_count++;
    return true;
}

// Вычисление CRC-24 для проверки целостности пакета
uint32_t calculateCRC24(uint8_t* data, int length) {
    uint32_t crc = 0;

    for (int i = 0; i < length; i++) {
        crc = ((crc << 8) ^ crc_table[((crc >> 16) ^ data[i]) & 0xFF]) & 0xFFFFFF;
    }

    return crc;
}

// Проверка целостности пакета
bool verifyPacket() {
    uint32_t calculated_crc = calculateCRC24(message_buffer, 11);
    uint32_t received_crc = (message_buffer[11] << 16) |
                           (message_buffer[12] << 8) |
                           message_buffer[13];

    return calculated_crc == received_crc;
}

// Расшифровка пакета ADS-B
void decodeADSB() {
    if (!verifyPacket()) {
        Serial1.println("CRC Error - packet discarded");
        return;
    }

    // Извлечение основных полей
    uint8_t downlink_format = (message_buffer[0] >> 3) & 0x1F;
    uint32_t icao_address = (message_buffer[1] << 16) |
                           (message_buffer[2] << 8) |
                           message_buffer[3];

    Serial1.println("=== ADS-B Packet Decoded ===");
    Serial1.printf("Downlink Format: %d\n", downlink_format);
    Serial1.printf("ICAO Address: %06X\n", icao_address);

    // Расшифровка в зависимости от типа сообщения
    switch (downlink_format) {
        case 17: // ADS-B Extended Squitter
        case 18: // TIS-B Extended Squitter
            decodeExtendedSquitter();
            break;

        case 11: // All-call reply
            Serial1.println("Type: All-call reply");
            break;

        default:
            Serial1.printf("Type: Unknown (DF=%d)\n", downlink_format);
            break;
    }

    // Вывод сырых данных
    Serial1.print("Raw data: ");
    for (int i = 0; i < 14; i++) {
        Serial1.printf("%02X ", message_buffer[i]);
    }
    Serial1.println();
    Serial1.println();
}

// Расшифровка Extended Squitter (наиболее информативные пакеты)
void decodeExtendedSquitter() {
    uint8_t type_code = (message_buffer[4] >> 3) & 0x1F;

    Serial1.println("Type: ADS-B Extended Squitter");
    Serial1.printf("Type Code: %d\n", type_code);

    switch (type_code) {
        case 1 ... 4: // Aircraft identification
            decodeAircraftID();
            break;

        case 9 ... 18: // Airborne position
        case 20 ... 22: // Airborne position
            decodeAirbornePosition();
            break;

        case 19: // Airborne velocity
            decodeAirborneVelocity();
            break;

        default:
            Serial1.printf("Subtype: Unknown (TC=%d)\n", type_code);
            break;
    }
}

// Расшифровка идентификации воздушного судна
void decodeAircraftID() {
    char callsign[9] = {0};

    // Извлечение позывного (6-битное кодирование)
    uint64_t data = 0;
    for (int i = 5; i < 11; i++) {
        data = (data << 8) | message_buffer[i];
    }

    for (int i = 0; i < 8; i++) {
        uint8_t char_code = (data >> (42 - i * 6)) & 0x3F;
        if (char_code == 0) break;

        if (char_code >= 1 && char_code <= 26) {
            callsign[i] = 'A' + char_code - 1;
        } else if (char_code >= 48 && char_code <= 57) {
            callsign[i] = '0' + char_code - 48;
        } else if (char_code == 32) {
            callsign[i] = ' ';
        }
    }

    Serial1.printf("Callsign: %s\n", callsign);
}

// Расшифровка позиции в воздухе
void decodeAirbornePosition() {
    uint16_t altitude_raw = ((message_buffer[5] & 0xFF) << 4) |
                           ((message_buffer[6] & 0xF0) >> 4);

    // Декодирование высоты (упрощенно)
    int altitude = -1000;
    if (altitude_raw != 0) {
        altitude = altitude_raw * 25 - 1000; // Приблизительное декодирование
    }

    Serial1.printf("Altitude: %d feet\n", altitude);

    // CPR координаты (требуют парной обработки для точного определения)
    uint32_t lat_cpr = ((message_buffer[6] & 0x03) << 15) |
                      (message_buffer[7] << 7) |
                      ((message_buffer[8] & 0xFE) >> 1);
    uint32_t lon_cpr = ((message_buffer[8] & 0x01) << 16) |
                      (message_buffer[9] << 8) |
                      message_buffer[10];

    Serial1.printf("CPR Latitude: %d\n", lat_cpr);
    Serial1.printf("CPR Longitude: %d\n", lon_cpr);
}

// Расшифровка скорости в воздухе
void decodeAirborneVelocity() {
    uint8_t subtype = (message_buffer[4] >> 1) & 0x07;

    if (subtype == 1 || subtype == 2) {
        // Скорость относительно земли
        uint16_t ew_velocity = ((message_buffer[5] & 0x03) << 8) | message_buffer[6];
        uint16_t ns_velocity = ((message_buffer[7] & 0x7F) << 3) |
                              ((message_buffer[8] & 0xE0) >> 5);

        Serial1.printf("E-W Velocity: %d kt\n", ew_velocity - 1);
        Serial1.printf("N-S Velocity: %d kt\n", ns_velocity - 1);
    }

    // Вертикальная скорость
    uint16_t vr_raw = ((message_buffer[8] & 0x1F) << 4) |
                     ((message_buffer[9] & 0xF0) >> 4);
    if (vr_raw != 0) {
        int vertical_rate = (vr_raw - 1) * 64;
        Serial1.printf("Vertical Rate: %d ft/min\n", vertical_rate);
    }
}

void loop() {
    // Проверка готовности нового сообщения
    if (message_ready) {
        message_ready = false;

        // Мигание LED при получении пакета
        digitalWrite(LED_PIN, HIGH);

        // Расшифровка пакета
        decodeADSB();

        digitalWrite(LED_PIN, LOW);
    }

    // Небольшая задержка для стабильности
    delay(1);
}

// Дополнительные функции для отладки
void printBinary(uint8_t value) {
    for (int i = 7; i >= 0; i--) {
        Serial1.print((value >> i) & 1);
    }
}

void printPacketBinary() {
    Serial1.println("Packet in binary:");
    for (int i = 0; i < 14; i++) {
        printBinary(message_buffer[i]);
        Serial1.print(" ");
        if ((i + 1) % 4 == 0) Serial1.println();
    }
    Serial1.println();
}

/*
Дополнительная конфигурация для Arduino IDE

Добавьте в boards.txt или используйте следующие настройки компиляции:


// platform.txt дополнения для RP2040
compiler.cpp.extra_flags=-DPICO_STDIO_USB=1 -DPICO_STDIO_UART=1


Подключение аппаратуры


RP2040 Pico    |  ADS-B приемник  |  Назначение
---------------|------------------|------------------
GPIO 2         |  DATA OUT        |  Импульсный сигнал
GND            |  GND             |  Общий провод
3V3            |  VCC             |  Питание (если нужно)
UART1 TX (GP4) |  ---             |  К компьютеру/дисплею
UART1 RX (GP5) |  ---             |  Не используется


Особенности программы

Обработка прерываний - использует аппаратные прерывания для точного захвата импульсов
CRC проверка - валидация целостности пакетов
Множественные типы сообщений - поддержка различных форматов ADS-B
Декодирование позывных - расшифровка идентификации самолетов
Позиционирование - базовое декодирование координат (требует доработки для точности)
Скоростные данные - расшифровка векторов скорости

Примечания по улучшению

Для промышленного использования рекомендуется:
Добавить буферизацию для множественных пакетов
Реализовать полный алгоритм CPR для точных координат
Добавить фильтрацию шумов
Использовать DMA для повышения производительности
*/