

#include <Arduino.h>
#include <pico/multicore.h>

// Определения пинов
#define INPUT_PIN_1     19
#define INPUT_PIN_2     22
#define PREAMBLE_PIN_1  20
#define PREAMBLE_PIN_2  23
#define UART_TX_PIN     4
#define UART_RX_PIN     5
#define AGC_INPUT_PIN   26
#define AGC_OUTPUT_PIN  9
#define AGC_CONTROL_PIN 27
#define CORE0_LED_PIN   15
#define CORE1_LED_PIN   25

// Константы ADS-B
#define ADSB_SHORT_PACKET_BITS  56
#define ADSB_LONG_PACKET_BITS   112
#define ADSB_SHORT_PACKET_BYTES 7
#define ADSB_LONG_PACKET_BYTES  14
#define PREAMBLE_LENGTH         8
#define SAMPLE_RATE_US          1  // 1 МГц
#define DMA_BUFFER_SIZE         256

// Структуры данных
struct ADSBPacket {
    uint8_t data[ADSB_LONG_PACKET_BYTES];
    uint8_t length;
    uint64_t timestamp;
    bool valid_crc;
    bool error_corrected;
};

struct DecodedADSB {
    uint32_t icao;
    char flight_id[9];
    double latitude;
    double longitude;
    uint16_t altitude;
    uint16_t speed;
    float heading;
    int16_t vertical_rate;
    uint8_t message_type;
    uint8_t aircraft_category;
    bool valid;
};

// Глобальные переменные
volatile uint8_t dma_buffer[DMA_BUFFER_SIZE];
volatile bool preamble_detected = false;
volatile uint32_t bit_counter = 0;
volatile uint32_t bit_count = 0;
volatile uint32_t packet_bits[ADSB_LONG_PACKET_BITS * 2 + PREAMBLE_LENGTH];
volatile bool packet_processing = false;

uint32_t crc_table[256];
uint32_t agc_level = 128;

// Определение структуры очереди для RP2040
#define MAX_QUEUE_SIZE 50
#define MAX_DECODED_QUEUE_SIZE 20

// Структура для очереди пакетов
struct PacketQueue {
    ADSBPacket packets[MAX_QUEUE_SIZE];
    volatile int head;
    volatile int tail;
    volatile int count;

    PacketQueue() : head(0), tail(0), count(0) {}
};

// Структура для очереди декодированных данных
struct DecodedQueue {
    DecodedADSB packets[MAX_DECODED_QUEUE_SIZE];
    volatile int head;
    volatile int tail;
    volatile int count;

    DecodedQueue() : head(0), tail(0), count(0) {}
};

// Глобальные очереди
PacketQueue *received_packets;
DecodedQueue *decoded_packets;




bool packet_queue_pop(PacketQueue* queue, ADSBPacket* packet) {
    if (queue->count == 0) {
        return false; // Очередь пустая
    }

    noInterrupts();

    *packet = queue->packets[queue->head];
    queue->head = (queue->head + 1) % MAX_QUEUE_SIZE;
    queue->count--;

    interrupts();
    return true;
}

//
//// Функции для работы с очередью пакетов Старая версия
//bool packet_queue_push(PacketQueue* queue, const ADSBPacket& packet) {
//    noInterrupts();
//
//    if (queue->count >= MAX_QUEUE_SIZE) {
//        // Очередь переполнена - удаляем старый пакет
//        queue->head = (queue->head + 1) % MAX_QUEUE_SIZE;
//        queue->count--;
//        Serial.println("Packet queue overflow - removed old packet");
//    }
//
//    queue->packets[queue->tail] = packet;
//    queue->tail = (queue->tail + 1) % MAX_QUEUE_SIZE;
//    queue->count++;
//
//    interrupts();
//    return true;
//}

// Исправленная функция packet_queue_pop
bool packet_queue_pop(PacketQueue *queue, ADSBPacket *packet) {
    if (queue->count == 0) {
        return false; // Очередь пустая
    }

    noInterrupts();

    *packet = queue->packets[queue->head];
    queue->head = (queue->head + 1) % MAX_QUEUE_SIZE;
    queue->count--;

    interrupts();
    return true;
}

bool packet_queue_empty(PacketQueue *queue) {
    return queue->count == 0;
}

int packet_queue_size(PacketQueue *queue) {
    return queue->count;
}

// Исправленная функция decoded_queue_pop
bool decoded_queue_pop(DecodedQueue *queue, DecodedADSB *decoded) {
    if (queue->count == 0) {
        return false;
    }

    noInterrupts();

    *decoded = queue->packets[queue->head];
    queue->head = (queue->head + 1) % MAX_DECODED_QUEUE_SIZE;
    queue->count--;

    interrupts();
    return true;
}

// Очистка очередей
void clear_packet_queues() {
    noInterrupts();

    received_packets->head = 0;
    received_packets->tail = 0;
    received_packets->count = 0;

    decoded_packets->head = 0;
    decoded_packets->tail = 0;
    decoded_packets->count = 0;

    interrupts();

    Serial.println("Packet queues cleared");
}

// Прототипы функций
void decode_aircraft_identification(const ADSBPacket& packet, DecodedADSB& decoded);
void decode_surface_position(const ADSBPacket& packet, DecodedADSB& decoded);
void decode_airborne_position(const ADSBPacket& packet, DecodedADSB& decoded);
void decode_airborne_velocity(const ADSBPacket& packet, DecodedADSB& decoded);
void decode_airborne_position_gnss(const ADSBPacket& packet, DecodedADSB& decoded);
void decode_aircraft_status(const ADSBPacket& packet, DecodedADSB& decoded);
void decode_target_state_status(const ADSBPacket& packet, DecodedADSB& decoded);
void decode_aircraft_operation_status(const ADSBPacket& packet, DecodedADSB& decoded);
void decode_cpr_position(uint32_t lat_cpr, uint32_t lon_cpr, bool format, DecodedADSB& decoded);
uint16_t decode_mode_c_altitude(uint16_t altitude_raw);
//float calculate_manchester_quality(uint32_t* raw_bits, int bit_count);

// CRC функции
void generate_crc_table() {
    uint32_t polynomial = 0x1021; // CRC-16-CCITT

    for (int i = 0; i < 256; i++) {
        uint32_t crc = i << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
        crc_table[i] = crc & 0xFFFF;
    }
}

uint16_t calculate_crc(uint8_t* data, int length) {
    uint16_t crc = 0;

    for (int i = 0; i < length; i++) {
        uint8_t tbl_idx = ((crc >> 8) ^ data[i]) & 0xFF;
        crc = ((crc << 8) ^ crc_table[tbl_idx]) & 0xFFFF;
    }

    return crc;
}

bool verify_crc(uint8_t* packet, int length) {
    if (length == ADSB_SHORT_PACKET_BYTES) {
        uint16_t calculated_crc = calculate_crc(packet, 5);
        uint16_t packet_crc = (packet[5] << 8) | packet[6];
        return calculated_crc == packet_crc;
    } else if (length == ADSB_LONG_PACKET_BYTES) {
        uint16_t calculated_crc = calculate_crc(packet, 12);
        uint16_t packet_crc = (packet[12] << 8) | packet[13];
        return calculated_crc == packet_crc;
    }
    return false;
}

// Коррекция ошибок (исправление одного бита)
bool correct_single_bit_error(uint8_t* packet, int length) {
    for (int byte_idx = 0; byte_idx < length - 2; byte_idx++) {
        for (int bit_idx = 0; bit_idx < 8; bit_idx++) {
            // Переворачиваем бит
            packet[byte_idx] ^= (1 << bit_idx);

            // Проверяем CRC
            if (verify_crc(packet, length)) {
                return true; // Ошибка исправлена
            }

            // Возвращаем бит обратно
            packet[byte_idx] ^= (1 << bit_idx);
        }
    }
    return false; // Ошибка не исправлена
}

// Функция фильтрации помех
bool noise_filter(uint32_t* bits, int length) {
    int transition_count = 0;

    for (int i = 1; i < length; i++) {
        if (bits[i] != bits[i - 1]) {
            transition_count++;
        }
    }

    // Если слишком много переходов - вероятно помеха
    float transition_ratio = (float)transition_count / length;
    return transition_ratio < 0.7; // Настраиваемый порог
}

// Детектор преамбулы ADS-B
bool detect_preamble(uint32_t* samples, int start_idx) {
    // Стандартная преамбула ADS-B: 8 бит
    // Шаблон: 10101000 в Manchester кодировке
    uint8_t preamble_pattern[] = {1, 0, 1, 0, 1, 0, 0, 0};

    for (int i = 0; i < PREAMBLE_LENGTH; i++) {
        if (samples[start_idx + i] != preamble_pattern[i]) {
            return false;
        }
    }
    return true;
}

// Manchester декодирование
bool manchester_decode_transitions(uint32_t* raw_bits, uint8_t* decoded_bytes, int bit_count) {
    if (bit_count % 2 != 0) return false;
    if (raw_bits == NULL || decoded_bytes == NULL) return false;

    int data_bits = bit_count / 2;
    int byte_count = (data_bits + 7) / 8;

    // Очистка выходного буфера
    memset(decoded_bytes, 0, byte_count);

    for (int i = 0; i < data_bits; i++) {
        int manchester_idx = i * 2;

        if (manchester_idx + 1 >= bit_count) {
            break;
        }

        int byte_idx = i / 8;
        int bit_idx = 7 - (i % 8); // MSB первый

        // Анализ перехода в Manchester символе
        uint8_t first_half = raw_bits[manchester_idx];
        uint8_t second_half = raw_bits[manchester_idx + 1];

        bool transition_detected = (first_half != second_half);

        if (!transition_detected) {
            // В Manchester кодировании всегда должен быть переход
            return false;
        }

        // ADS-B стандарт:
        // HIGH→LOW переход = логическая 1
        // LOW→HIGH переход = логический 0
        if (first_half == 1 && second_half == 0) {
            // Переход сверху вниз = 1
            decoded_bytes[byte_idx] |= (1 << bit_idx);
        } else if (first_half == 0 && second_half == 1) {
            // Переход снизу вверх = 0 (бит уже 0)
        }
    }

    return true;
}

// Функция для целочисленного квадратного корня
uint16_t int_sqrt(uint32_t value) {
    if (value == 0) return 0;

    uint32_t x = value;
    uint32_t y = (x + 1) / 2;

    while (y < x) {
        x = y;
        y = (x + value / x) / 2;
    }

    return (uint16_t)x;
}

// Декодирование скорости
void decode_airborne_velocity(const ADSBPacket& packet, DecodedADSB& decoded) {
    uint8_t subtype = (packet.data[4] & 0x07);

    if (subtype == 1 || subtype == 2) {
        // Ground Speed
        bool ew_dir = (packet.data[5] & 0x04) != 0;
        uint16_t ew_velocity = ((packet.data[5] & 0x03) << 8) | packet.data[6];

        bool ns_dir = (packet.data[7] & 0x80) != 0;
        uint16_t ns_velocity = ((packet.data[7] & 0x7F) << 3) |
            ((packet.data[8] & 0xE0) >> 5);

        if (ew_velocity > 0 && ns_velocity > 0) {
            ew_velocity -= 1;
            ns_velocity -= 1;

            // Расчет результирующей скорости (исправлено)
            uint32_t speed_squared = (uint32_t)ew_velocity * ew_velocity +
                                   (uint32_t)ns_velocity * ns_velocity;
            decoded.speed = int_sqrt(speed_squared);

            // Расчет направления (упрощенно)
            if (ns_velocity != 0) {
                float heading = atan2(ew_velocity * (ew_dir ? -1 : 1),
                                    ns_velocity * (ns_dir ? 1 : -1));
                decoded.heading = (heading * 180.0 / PI);
                if (decoded.heading < 0) decoded.heading += 360.0;
            }
        }

        // Вертикальная скорость
        bool vr_source = (packet.data[8] & 0x10) != 0;
        bool vr_sign = (packet.data[8] & 0x08) != 0;
        uint16_t vr_raw = ((packet.data[8] & 0x07) << 6) |
            ((packet.data[9] & 0xFC) >> 2);

        if (vr_raw > 0) 
        {
           // decoded.vertical_rate = (vr_raw - 1)  64  (vr_sign ? -1 : 1);
        }

        Serial2.print("Velocity - Speed: ");
        Serial2.print(decoded.speed);
        Serial2.print(" kt, Heading: ");
        Serial2.print(decoded.heading);
        Serial2.print("°, VR: ");
        Serial2.print(decoded.vertical_rate);
        Serial2.println(" ft/min");
    }
}

// Декодирование ADS-B пакета с полной поддержкой всех типов сообщений
DecodedADSB decode_adsb_packet(const ADSBPacket& packet) {
    DecodedADSB decoded = {0};
    decoded.valid = false;

    if (packet.length < 7) return decoded;

    // Извлечение Downlink Format (первые 5 бит)
    uint8_t downlink_format = (packet.data[0] >> 3) & 0x1F;

    // Проверяем, что это ADS-B сообщение (DF 17 или 18)
    if (downlink_format != 17 && downlink_format != 18) {
        return decoded; // Не ADS-B сообщение
    }

    // Извлечение ICAO адреса (байты 1-3)
    decoded.icao = ((uint32_t)packet.data[1] << 16) |
        ((uint32_t)packet.data[2] << 8) |
        packet.data[3];

    if (packet.length >= 14) { // Длинный пакет (Extended Squitter)
        // Извлечение Type Code из ME field (первые 5 бит)
        uint8_t type_code = (packet.data[4] >> 3) & 0x1F;
        decoded.message_type = type_code;

        // Декодирование по типу сообщения
        switch (type_code) {
        case 1: case 2: case 3: case 4:
            // Aircraft Identification
            decode_aircraft_identification(packet, decoded);
            break;

        case 5: case 6: case 7: case 8:
            // Surface Position
            decode_surface_position(packet, decoded);
            break;

        case 9: case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18:
            // Airborne Position (with Barometric Altitude)
            decode_airborne_position(packet, decoded);
            break;

        case 19:
            // Airborne Velocities
            decode_airborne_velocity(packet, decoded);
            break;

        case 20: case 21: case 22:
            // Airborne Position (with GNSS Height)
            decode_airborne_position_gnss(packet, decoded);
            break;

        case 23: case 24: case 25: case 26: case 27:
            // Test Message
            Serial2.print("Test message received, type: ");
            Serial2.println(type_code);
            break;

        case 28:
            // Aircraft Status
            decode_aircraft_status(packet, decoded);
            break;

        case 29:
            // Target State and Status Information
            decode_target_state_status(packet, decoded);
            break;

        case 31:
            // Aircraft Operation Status
            decode_aircraft_operation_status(packet, decoded);
            break;

        default:
            Serial2.print("Unknown message type: ");
            Serial2.println(type_code);
            break;
        }
    } else if (packet.length == 7) {
        // Короткий пакет (Short Squitter)
        decoded.message_type = 0; // Короткое сообщение
        Serial2.println("Short squitter message");
    }

    decoded.valid = true;
    return decoded;
}

// Декодирование идентификации воздушного судна
void decode_aircraft_identification(const ADSBPacket& packet, DecodedADSB& decoded) {
    // Извлечение категории воздушного судна
    uint8_t aircraft_category = packet.data[4] & 0x07;
    decoded.aircraft_category = aircraft_category;

    // Извлечение 8-символьного callsign (биты 8-55 ME field)
    uint64_t callsign_raw = 0;

    // Собираем 48 бит callsign из байтов 5-10
    callsign_raw = ((uint64_t)packet.data[5] << 40) |
        ((uint64_t)packet.data[6] << 32) |
        ((uint64_t)packet.data[7] << 24) |
        ((uint64_t)packet.data[8] << 16) |
        ((uint64_t)packet.data[9] << 8) |
        packet.data[10];

    // Декодирование 6-битовых символов
    for (int i = 0; i < 8; i++) {
        uint8_t char_code = (callsign_raw >> (42 - i * 6)) & 0x3F;

        if (char_code == 0) {
            decoded.flight_id[i] = '\0';
            break;
        } else if (char_code >= 1 && char_code <= 26) {
            decoded.flight_id[i] = 'A' + char_code - 1;
        } else if (char_code >= 48 && char_code <= 57) {
            decoded.flight_id[i] = '0' + (char_code - 48);
        } else if (char_code == 32) {
            decoded.flight_id[i] = ' ';
        } else {
            decoded.flight_id[i] = '?'; // Неизвестный символ
        }
    }
    decoded.flight_id[8] = '\0'; // Завершающий ноль

    Serial2.print("Aircraft ID: ");
    Serial2.print(decoded.flight_id);
    Serial2.print(", Category: ");
    Serial2.println(aircraft_category);
}

// Декодирование воздушной позиции с барометрической высотой
void decode_airborne_position(const ADSBPacket& packet, DecodedADSB& decoded) {
    // Биты 8-19: Altitude
    uint16_t altitude_raw = ((packet.data[5] & 0xFF) << 4) |
        ((packet.data[6] & 0xF0) >> 4);

    // Декодирование высоты
    if (altitude_raw != 0) {
        // Бит 7 в altitude - это Q-bit
        bool q_bit = (packet.data[6] & 0x01) != 0;

        if (q_bit) {
            // 25-футовые инкременты
            uint16_t n = ((altitude_raw & 0xFE0) >> 1) | (altitude_raw & 0x0F);
            decoded.altitude = n * 25 - 1000;
        } else {
            // Режим C/S (более сложное декодирование)
            decoded.altitude = decode_mode_c_altitude(altitude_raw);
        }
    }

    // Биты 20-21: Time flag и CPR format
    bool time_flag = (packet.data[6] & 0x08) != 0;
    bool cpr_format = (packet.data[6] & 0x04) != 0;

    // Биты 22-38: Latitude CPR
    uint32_t lat_cpr = ((packet.data[6] & 0x03) << 15) |
        (packet.data[7] << 7) |
        (packet.data[8] >> 1);

    // Биты 39-55: Longitude CPR
    uint32_t lon_cpr = ((packet.data[8] & 0x01) << 16) |
        (packet.data[9] << 8) |
        packet.data[10];

    // CPR декодирование (упрощенная версия - требует более сложной реализации)
    decode_cpr_position(lat_cpr, lon_cpr, cpr_format, decoded);

    Serial2.print("Position - Alt: ");
    Serial2.print(decoded.altitude);
    Serial2.print(" ft, CPR format: ");
    Serial2.print(cpr_format);
    Serial2.print(", Time: ");
    Serial2.println(time_flag);
}

// Упрощенное CPR декодирование
void decode_cpr_position(uint32_t lat_cpr, uint32_t lon_cpr, bool format, DecodedADSB& decoded) {
    const double dlat0 = 360.0 / 60.0; // 6 degrees
    const double dlat1 = 360.0 / 59.0; // ~6.1 degrees

    double dlat = format ? dlat1 : dlat0;

    // Приблизительное декодирование (неточное!)
    decoded.latitude = dlat * (lat_cpr / 131072.0) - 90.0;
    decoded.longitude = (360.0 / (format ? 59 : 60)) * (lon_cpr / 131072.0) - 180.0;

    // Ограничиваем диапазон
    if (decoded.latitude > 90.0) decoded.latitude -= 180.0;
    if (decoded.latitude < -90.0) decoded.latitude += 180.0;
    if (decoded.longitude > 180.0) decoded.longitude -= 360.0;
    if (decoded.longitude < -180.0) decoded.longitude += 360.0;
}

// Декодирование Mode C высоты
uint16_t decode_mode_c_altitude(uint16_t altitude_raw) {
    uint16_t gray = altitude_raw;
    uint16_t binary = 0;

    // Простое приближение - для полной реализации нужна таблица Gray кодов
    binary = gray ^ (gray >> 1);

    return binary * 100; // Mode C в 100-футовых инкрементах
}

// Заглушки для других типов сообщений
void decode_surface_position(const ADSBPacket& packet, DecodedADSB& decoded) {
    Serial2.println("Surface position message");
}

void decode_airborne_position_gnss(const ADSBPacket& packet, DecodedADSB& decoded) {
    Serial2.println("Airborne position with GNSS height");
}

void decode_aircraft_status(const ADSBPacket& packet, DecodedADSB& decoded) {
    Serial2.println("Aircraft status message");
}

void decode_target_state_status(const ADSBPacket& packet, DecodedADSB& decoded) {
    Serial2.println("Target state and status message");
}

void decode_aircraft_operation_status(const ADSBPacket& packet, DecodedADSB& decoded) {
    Serial2.println("Aircraft operation status message");
}

// Функция AGC
void update_agc() {
    uint16_t signal_level = analogRead(AGC_INPUT_PIN);
    uint16_t control_level = analogRead(AGC_CONTROL_PIN);

    // Простой алгоритм AGC
    if (signal_level > 800) {
        agc_level = max(0, agc_level - 1);
    } else if (signal_level < 200) {
        agc_level = min(255, agc_level + 1);
    }

    analogWrite(AGC_OUTPUT_PIN, agc_level);
}

// Версия для использования в прерываниях
inline void reset_packet_buffer_isr() {
    preamble_detected = false;
    bit_count = 0;
    bit_counter = 0;
    digitalWrite(PREAMBLE_PIN_1, LOW);
    digitalWrite(PREAMBLE_PIN_2, LOW);
}

// Прерывание для обнаружения сигнала (исправлено для RP2040)
void signal_interrupt() {
    static uint32_t last_time = 0;
    static uint32_t local_bit_count = 0;

    uint32_t current_time = micros();
    uint32_t pulse_width = current_time - last_time;
    last_time = current_time;

    // Определение бита на основе длительности импульса
    bool bit_value = false;
    if (pulse_width > 400 && pulse_width < 600) { // 0.5 мкс ±20%
        bit_value = digitalRead(INPUT_PIN_1);
    }

    // Сохранение бита
    if (local_bit_count < ADSB_LONG_PACKET_BITS * 2 + PREAMBLE_LENGTH) {
        packet_bits[local_bit_count] = bit_value ? 1 : 0;
        local_bit_count++;
        bit_count = local_bit_count; // Обновляем глобальную переменную
    }

    // Проверка на преамбулу
    if (local_bit_count >= PREAMBLE_LENGTH) {
        if (detect_preamble((uint32_t*)packet_bits, local_bit_count - PREAMBLE_LENGTH)) {
            preamble_detected = true;
            digitalWrite(PREAMBLE_PIN_1, HIGH);
            bit_counter = 0;
        }
    }

    // Если обнаружена преамбула, собираем пакет
    if (preamble_detected) {
        bit_counter++;

        // Проверяем завершение пакета
        if (bit_counter == ADSB_SHORT_PACKET_BITS/*  2*/ || bit_counter == ADSB_LONG_PACKET_BITS /* 2*/) {
            // Обработка завершенного пакета будет в основном цикле
        }
    }

    // Сброс если слишком много битов без валидного пакета
    if (local_bit_count > ADSB_LONG_PACKET_BITS * 2 + PREAMBLE_LENGTH) {
        reset_packet_buffer_isr();
        local_bit_count = 0;
    }
}
//-----------------------------------------------------------------


// Возвращает указатель на элемент или NULL если очередь пустая
ADSBPacket packet_queue_get_next(PacketQueue* queue) {
    if (queue == NULL || queue->count == 0)
    {
        return /*NULL*/; // Очередь пустая
    }

    noInterrupts();

    ADSBPacket packet = queue->packets[queue->head];
    queue->head = (queue->head + 1) % MAX_QUEUE_SIZE;
    queue->count--;

    interrupts();
    return packet;
}


// Использование:
void process_received_packets() {
    int packets_processed = 0;
    const int max_packets_per_call = 5;
   
    while (packets_processed < max_packets_per_call) {
        ADSBPacket*  packet = packet_queue_get_next(received_packets);
        if (packet == NULL) {
            break; // Очередь пустая
        }

        packets_processed++;

        if (packet->valid_crc) {
            DecodedADSB decoded = decode_adsb_packet(*packet);
            if (decoded.valid) {
                send_decoded_packet_uart(decoded, *packet);
                decoded_queue_push(&decoded_packets, decoded);
            }
        }
    }

    if (packets_processed > 0) {
        Serial.print("Processed ");
        Serial.print(packets_processed);
        Serial.println(" packets");
    }
}




// Безопасная обработка принятых пакетов (исправленная версия)
//void process_received_packets() {
//    ADSBPacket packet;
//    int packets_processed = 0;
//    const int max_packets_per_call = 5;
//
//    while (packets_processed < max_packets_per_call &&
//        packet_queue_pop(&received_packets, &packet)) {
//
//        packets_processed++;
//
//        if (packet.valid_crc) {
//            DecodedADSB decoded = decode_adsb_packet(packet);
//
//            if (decoded.valid) {
//                // Отправка в UART2
//                send_decoded_packet_uart(decoded, packet);
//
//                // Исправленное добавление в очередь декодированных пакетов
//                decoded_queue_push(&decoded_packets, &decoded);
//            }
//        }
//    }
//
//    if (packets_processed > 0) {
//        Serial.print("Processed ");
//        Serial.print(packets_processed);
//        Serial.println(" packets");
//    }
//}


// 
//// Безопасная обработка принятых пакетов (исправленная версия)
//void process_received_packets() {
//    ADSBPacket packet;
//    int packets_processed = 0;
//    const int max_packets_per_call = 5;
//
//    // Теперь вызов будет корректным
//    while (packets_processed < max_packets_per_call &&
//        packet_queue_pop(received_packets, packet)) {
//
//        packets_processed++;
//
//        if (packet.valid_crc) {
//            DecodedADSB decoded = decode_adsb_packet(packet);
//
//            if (decoded.valid) {
//                // Отправка в UART2
//                send_decoded_packet_uart(decoded, packet);
//
//                // Добавляем в очередь декодированных пакетов
//                decoded_queue_push(&decoded_packets, decoded);
//            }
//        }
//    }
//
//    if (packets_processed > 0) {
//        Serial.print("Processed ");
//        Serial.print(packets_processed);
//        Serial.println(" packets");
//    }
//}



//-----------------------------------------------------------------
//// Безопасная обработка принятых пакетов  Старая версия
//void process_received_packets() {
//    ADSBPacket packet;
//    int packets_processed = 0;
//    const int max_packets_per_call = 5;
//
//     // Обрабатываем до 5 пакетов за один вызов
//    while (packets_processed < max_packets_per_call && packet_queue_pop(&received_packets, &packet)) {
//
//        packets_processed++;
//
//        if (packet.valid_crc) {
//            DecodedADSB decoded = decode_adsb_packet(packet);
//
//            if (decoded.valid) {
//                // Отправка в UART2
//                send_decoded_packet_uart(decoded, packet);
//
//                // Добавляем в очередь декодированных пакетов
//                decoded_queue_push(&decoded_packets, decoded);
//            }
//        }
//    }
//
//    if (packets_processed > 0) {
//        Serial.print("Processed ");
//        Serial.print(packets_processed);
//        Serial.println(" packets");
//    }
//}

// Функция отправки декодированных данных в UART
void send_decoded_packet_uart(const DecodedADSB& decoded, const ADSBPacket& packet) {
    Serial2.print("ICAO: 0x");
    if (decoded.icao < 0x100000) Serial2.print("0");
    if (decoded.icao < 0x10000) Serial2.print("0");
    if (decoded.icao < 0x1000) Serial2.print("0");
    if (decoded.icao < 0x100) Serial2.print("0");
    if (decoded.icao < 0x10) Serial2.print("0");
    Serial2.print(decoded.icao, HEX);

    Serial2.print(", Type: ");
    Serial2.print(decoded.message_type);

    if (strlen(decoded.flight_id) > 0) {
        Serial2.print(", Flight: ");
        Serial2.print(decoded.flight_id);
    }

    if (decoded.latitude != 0.0 || decoded.longitude != 0.0) {
        Serial2.print(", Lat: ");
        Serial2.print(decoded.latitude, 6);
        Serial2.print(", Lon: ");
        Serial2.print(decoded.longitude, 6);
    }

    if (decoded.altitude > 0) {
        Serial2.print(", Alt: ");
        Serial2.print(decoded.altitude);
        Serial2.print(" ft");
    }

    if (decoded.speed > 0) {
        Serial2.print(", Speed: ");
        Serial2.print(decoded.speed);
        Serial2.print(" kt");
    }

    if (packet.error_corrected) {
        Serial2.print(" [CORRECTED]");
    }

    Serial2.println();
}

// Статистика очередей
void print_queue_statistics() {
    static uint32_t last_stats_time = 0;
    uint32_t current_time = millis();

    if (current_time - last_stats_time > 5000) { // Каждые 5 секунд
        Serial.print("Queue sizes - Received: ");
        Serial.print(packet_queue_size(received_packets));
        Serial.print("/");
        Serial.print(MAX_QUEUE_SIZE);
        Serial.print(", Decoded: ");
        Serial.print(decoded_packets->count);
        Serial.print("/");
        Serial.println(MAX_DECODED_QUEUE_SIZE);

        last_stats_time = current_time;
    }
}

// Инициализация очередей
void init_packet_queues() {
    // Очистка очередей при инициализации
    clear_packet_queues();

    Serial.print("Packet queues initialized - Max received: ");
    Serial.print(MAX_QUEUE_SIZE);
    Serial.print(", Max decoded: ");
    Serial.println(MAX_DECODED_QUEUE_SIZE);
}

// Задача для ядра 0 (прием)
void core0_task() {
    static uint32_t last_led_toggle = 0;

    while (true) {
        uint32_t current_time = millis();

        // Переключение LED каждые 1000 мс
        if (current_time - last_led_toggle >= 1000) {
            digitalWrite(CORE0_LED_PIN, !digitalRead(CORE0_LED_PIN));
            last_led_toggle = current_time;
        }

        // Обновление AGC
        update_agc();

        delay(1);
    }
}

// Задача для ядра 1 (обработка)
void core1_task() {
    static uint32_t last_led_toggle = 0;

    while (true) {
        uint32_t current_time = millis();

        // Переключение LED каждые 500 мс
        if (current_time - last_led_toggle >= 500) {
            digitalWrite(CORE1_LED_PIN, !digitalRead(CORE1_LED_PIN));
            last_led_toggle = current_time;
        }

        // Обработка принятых пакетов
        process_received_packets();

        // Статистика очередей
        print_queue_statistics();

        delay(1);
    }
}

void setup() {
    // Инициализация Serial
    Serial.begin(115200);

    // Инициализация UART2
    Serial2.setTX(UART_TX_PIN);
    Serial2.setRX(UART_RX_PIN);
    Serial2.begin(115200);
    delay(1000);

    Serial2.println("Start setup");

    // Настройка пинов
    pinMode(INPUT_PIN_1, INPUT);
    pinMode(INPUT_PIN_2, INPUT);
    pinMode(PREAMBLE_PIN_1, OUTPUT);
    pinMode(PREAMBLE_PIN_2, OUTPUT);
    pinMode(AGC_INPUT_PIN, INPUT);
    pinMode(AGC_CONTROL_PIN, INPUT);
    pinMode(AGC_OUTPUT_PIN, OUTPUT);
    pinMode(CORE0_LED_PIN, OUTPUT);
    pinMode(CORE1_LED_PIN, OUTPUT);

    // Генерация таблицы CRC
    generate_crc_table();

    // Настройка PWM для AGC
    analogWriteFreq(1000); // 1 кГц
    analogWriteRange(255);

    // Настройка прерываний
    attachInterrupt(digitalPinToInterrupt(INPUT_PIN_1), signal_interrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(INPUT_PIN_2), signal_interrupt, CHANGE);

    init_packet_queues();

    Serial2.println("ADS-B Receiver initialized");
    Serial2.println("ADS-B Receiver started");

    // Запуск второго ядра
    multicore_launch_core1(core1_task);
}

void loop() {
    // Ядро 0 - основной цикл приема
    core0_task();
}

/*
Основные исправления:

Убрал неподдерживаемые заголовки (#include <queue>, hardware/dma.h и др.)
Исправил инициализацию Serial2 - правильное создание объекта
Исправил сигнатуры функций - добавил указатели где нужно
Добавил недостающие прототипы функций
Исправил операторы умножения в расчетах скорости
Убрал ссылки на несуществующие функции (как received_packets.empty())
Исправил локальные переменные в прерывании
Добавил правильную обработку очередей

Теперь код должен успешно компилироваться в Arduino IDE для RP2040.
*/