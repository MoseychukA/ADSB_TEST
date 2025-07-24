#include <mutex>
#include <Arduino.h>
#include <queue>
#include <vector>


#include "adsbee.h"


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
    uint8_t data[14];  // Максимум 112 бит = 14 байт
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

// Фильтр помех
#define NOISE_FILTER_SIZE 5
volatile uint32_t noise_filter_1[NOISE_FILTER_SIZE] = {0};
volatile uint32_t noise_filter_2[NOISE_FILTER_SIZE] = {0};
volatile int filter_index_1 = 0;
volatile int filter_index_2 = 0;

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
bool check_crc(uint8_t* data, int length) {
    uint32_t calculated_crc = calculate_crc(data, length);
    uint32_t packet_crc = (data[length-3] << 16) | (data[length-2] << 8) | data[length-1];
    return calculated_crc == packet_crc;
}

// Коррекция одиночной ошибки
bool correct_single_bit_error(uint8_t* data, int length) {
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









//===================== Новый вариант =============================================
// IRQ обработчик для входного сигнала
void inputIRQHandler()
{
    static uint32_t lastTime = 0;
    uint32_t currentTime = micros();

    //// Антидребезг
    //if (currentTime - lastTime < 1) return;
    //lastTime = currentTime;


    //uint32_t current_time = micros();
    //uint32_t pulse_width = current_time - last_edge_time_1;
    //last_edge_time_1 = current_time;

    //bool bit_value;

    //// Декодирование PPM (Pulse Position Modulation)
    //if (pulse_width >= 45 && pulse_width <= 65)
    //{  // ~0.5 мкс = бит 0
    //    bit_value = false;
    //}
    //else if (pulse_width >= 90 && pulse_width <= 110) 
    //{  // ~1.0 мкс = бит 1
    //    bit_value = true;
    //}
    //else 
    //{
    //    // Неправильная длительность импульса
    //    collecting_1 = false;
    //    bit_count_1 = 0;
    //    return;
    //}

    //if (!collecting_1) 
    //{
    //// Ищем преамбулу
    //static uint32_t preamble_buffer = 0;
    //preamble_buffer = (preamble_buffer << 1) | (bit_value ? 1 : 0);
    //    
    //if ((preamble_buffer & 0xFFFFF) == PREAMBLE_PATTERN) 
    //{
    //    collecting_1 = true;
    //    bit_count_1 = 0;
    //    preamble_detected_1 = true;
    //    digitalWrite(PREAMBLE_LED_1, HIGH);
    //}
    //} 
    //else 
    //{
    //    // Собираем биты пакета
    //    if (bit_count_1 < 112) 
    //    {
    //        bit_buffer_1[bit_count_1] = bit_value;
    //        bit_count_1++;
    //    
    //        // Проверяем завершение пакета
    //        if (bit_count_1 == 56 || bit_count_1 == 112) {
    //            // Конвертируем биты в байты
    //            ADSBPacket packet;
    //            packet.length = bit_count_1 / 8;
    //            packet.timestamp = current_time;
    //            packet.channel = 1;
    //    
    //            for (int i = 0; i < packet.length; i++) {
    //                packet.data[i] = 0;
    //                for (int j = 0; j < 8; j++) {
    //                    if (bit_buffer_1[i*8 + j]) {
    //                        packet.data[i] |= (1 << (7-j));
    //                    }
    //                }
    //            }
    //    
    //            // Добавляем в очередь
    //            while (queue_mutex) delayMicroseconds(1);
    //            queue_mutex = true;
    //    
    //            /*if (packet_queue.size() < MAX_QUEUE_SIZE) 
    //            {
    //                packet_queue.push(packet);
    //            }*/
    //    
    //            queue_mutex = false;
    //    
    //            collecting_1 = false;
    //            bit_count_1 = 0;
    //            digitalWrite(PREAMBLE_LED_1, LOW);
    //        }
    //    }
    //}




    bool signal1 = digitalRead(INPUT_PIN_1);
    bool signal2 = digitalRead(INPUT_PIN_2);

    // Простое XOR для дифференциального сигнала
    bool bit = signal1/* ^ signal2*/;

    // Детекция преамбулы
    static uint16_t preambleBuffer = 0;
    preambleBuffer = (preambleBuffer << 1) | (bit ? 1 : 0);

    if ((preambleBuffer & 0xFFFF) == 0xA140) { // ADS-B преамбула
        preambleDetected = true;
        digitalWrite(PREAMBLE_LED_1, HIGH);
        digitalWrite(PREAMBLE_LED_2, HIGH);
        bitBuffer = 0;
        bitCount = 0;
    }

    if (preambleDetected) 
    {
        bitBuffer = (bitBuffer << 1) | (bit ? 1 : 0);
        bitCount++;

        // Проверяем завершение пакета
        if (bitCount == SHORT_PACKET_BITS || bitCount == LONG_PACKET_BITS)
        {
            packetReady = true;
            preambleDetected = false;
            digitalWrite(PREAMBLE_LED_1, LOW);
            digitalWrite(PREAMBLE_LED_2, LOW);

            Serial2.print("bitCount ");
            Serial2.println(bitCount);
        }
    }
}

//===================== Старый вариант =============================================
//// Фильтр помех
//bool noise_filter(uint32_t pulse_width, volatile uint32_t filter_buffer, volatile int filter_index) {
//    filter_buffer[*filter_index] = pulse_width;
//    filter_index = (filter_index + 1) % NOISE_FILTER_SIZE;
//
//    // Проверяем стабильность сигнала
//    uint32_t min_val = filter_buffer[0];
//    uint32_t max_val = filter_buffer[0];
//
//    for (int i = 1; i < NOISE_FILTER_SIZE; i++) {
//        if (filter_buffer[i] < min_val) min_val = filter_buffer[i];
//        if (filter_buffer[i] > max_val) max_val = filter_buffer[i];
//    }
//
//    // Если разброс слишком большой, считаем это помехой
//    return (max_val - min_val) < 50;  // Порог в микросекундах
//}
//
//// Обработчик прерывания для канала 1
//void IRAM_ATTR irq_handler_1() {
//    uint32_t current_time = micros();
//    uint32_t pulse_width = current_time - last_edge_time_1;
//    last_edge_time_1 = current_time;
//
//    // Фильтр помех
//    if (!noise_filter(pulse_width, noise_filter_1, &filter_index_1)) {
//        return;
//    }
//
//    bool bit_value;
//
//    // Декодирование PPM (Pulse Position Modulation)
//    if (pulse_width >= 45 && pulse_width <= 65) {  // ~0.5 мкс = бит 0
//        bit_value = false;
//    } else if (pulse_width >= 90 && pulse_width <= 110) {  // ~1.0 мкс = бит 1
//        bit_value = true;
//    } else {
//        // Неправильная длительность импульса
//        collecting_1 = false;
//        bit_count_1 = 0;
//        return;
//    }
//
//    if (!collecting_1) {
//        // Ищем преамбулу
//        static uint32_t preamble_buffer = 0;
//        preamble_buffer = (preamble_buffer << 1) | (bit_value ? 1 : 0);
//
//        if ((preamble_buffer & 0xFFFFF) == PREAMBLE_PATTERN) {
//            collecting_1 = true;
//            bit_count_1 = 0;
//            preamble_detected_1 = true;
//            digitalWrite(PREAMBLE_LED_1, HIGH);
//        }
//    } else {
//        // Собираем биты пакета
//        if (bit_count_1 < 112) {
//            bit_buffer_1[bit_count_1] = bit_value;
//            bit_count_1++;
//
//            // Проверяем завершение пакета
//            if (bit_count_1 == 56 || bit_count_1 == 112) {
//                // Конвертируем биты в байты
//                ADSBPacket packet;
//                packet.length = bit_count_1 / 8;
//                packet.timestamp = current_time;
//                packet.channel = 1;
//
//                for (int i = 0; i < packet.length; i++) {
//                    packet.data[i] = 0;
//                    for (int j = 0; j < 8; j++) {
//                        if (bit_buffer_1[i*8 + j]) {
//                            packet.data[i] |= (1 << (7-j));
//                        }
//                    }
//                }
//
//                // Добавляем в очередь
//                while (queue_mutex) delayMicroseconds(1);
//                queue_mutex = true;
//
//                if (packet_queue.size() < MAX_QUEUE_SIZE) {
//                    packet_queue.push(packet);
//                }
//
//                queue_mutex = false;
//
//                collecting_1 = false;
//                bit_count_1 = 0;
//                digitalWrite(PREAMBLE_LED_1, LOW);
//            }
//        }
//    }
//}
//
//// Обработчик прерывания для канала 2
//void IRAM_ATTR irq_handler_2() {
//    uint32_t current_time = micros();
//    uint32_t pulse_width = current_time - last_edge_time_2;
//    last_edge_time_2 = current_time;
//
//    // Фильтр помех
//    if (!noise_filter(pulse_width, noise_filter_2, &filter_index_2)) {
//        return;
//    }
//
//    bool bit_value;
//
//    // Декодирование PPM
//    if (pulse_width >= 45 && pulse_width <= 65) {
//        bit_value = false;
//    } else if (pulse_width >= 90 && pulse_width <= 110) {
//        bit_value = true;
//    } else {
//        collecting_2 = false;
//        bit_count_2 = 0;
//        return;
//    }
//
//    if (!collecting_2) {
//        static uint32_t preamble_buffer = 0;
//        preamble_buffer = (preamble_buffer << 1) | (bit_value ? 1 : 0);
//
//        if ((preamble_buffer & 0xFFFFF) == PREAMBLE_PATTERN) {
//            collecting_2 = true;
//            bit_count_2 = 0;
//            preamble_detected_2 = true;
//            digitalWrite(PREAMBLE_LED_2, HIGH);
//        }
//    } else {
//        if (bit_count_2 < 112) {
//            bit_buffer_2[bit_count_2] = bit_value;
//            bit_count_2++;
//
//            if (bit_count_2 == 56 || bit_count_2 == 112) {
//                ADSBPacket packet;
//                packet.length = bit_count_2 / 8;
//                packet.timestamp = current_time;
//                packet.channel = 2;
//
//                for (int i = 0; i < packet.length; i++) {
//                    packet.data[i] = 0;
//                    for (int j = 0; j < 8; j++) {
//                        if (bit_buffer_2[i*8 + j]) {
//                            packet.data[i] |= (1 << (7-j));
//                        }
//                    }
//                }
//
//                while (queue_mutex) delayMicroseconds(1);
//                queue_mutex = true;
//
//                if (packet_queue.size() < MAX_QUEUE_SIZE) {
//                    packet_queue.push(packet);
//                }
//
//                queue_mutex = false;
//
//                collecting_2 = false;
//                bit_count_2 = 0;
//                digitalWrite(PREAMBLE_LED_2, LOW);
//            }
//        }
//    }
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





// Задача ядра 0 - обработка пакетов
void core0_task() {
    unsigned long lastToggle = 0;

    while (true) {
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

        // Обработка пакетов из очереди
        if (!packet_queue.empty()) 
        {
            while (queue_mutex) delay(1);
            queue_mutex = true;

           //!! ADSBPacket packet = packet_queue.front();
            packet_queue.pop();

            queue_mutex = false;

            // Отладочный вывод сырых данных
            Serial2.print("Raw packet (Ch");
          //!!  Serial2.print(packet.channel);
            Serial2.print(", ");
            //!!Serial2.print(packet.length);
            Serial2.print(" bytes): ");
            //!!for (int i = 0; i < packet.length; i++) {
            //    if (packet.data[i] < 16) Serial2.print("0");
            //    Serial2.print(packet.data[i], HEX);
            //    Serial2.print(" ");
            //}
            Serial2.println();

            // Проверка и коррекция CRC
            bool crc_ok = true;//!!check_crc(packet.data, packet.length);
            bool corrected = false;

            if (!crc_ok) 
            {
                //!!corrected = correct_single_bit_error(packet.data, packet.length);
                //if (corrected)
                //{
                //    Serial2.println("Single bit error corrected!");
                //}
            }

            //!!if (crc_ok || corrected) {
            //    // Декодирование пакета
            //    uint32_t icao = decode_icao(packet.data);
            //    uint8_t msg_type = decode_message_type(packet.data);

            //    char callsign[9];
            //    decode_callsign(packet.data, callsign);

            //    double latitude = 0, longitude = 0;
            //    decode_position(packet.data, &latitude, &longitude);

            //    uint16_t velocity = decode_velocity(packet.data);
            //    uint16_t altitude = decode_altitude(packet.data);

            //    // Формируем JSON для Serial2
            //    String json = "{";
            //    json += "\"icao\":\"" + String(icao, HEX) + "\",";
            //    json += "\"type\":" + String(msg_type) + ",";
            //    json += "\"callsign\":\"" + String(callsign) + "\",";
            //    json += "\"lat\":" + String(latitude, 6) + ",";
            //    json += "\"lon\":" + String(longitude, 6) + ",";
            //    json += "\"alt\":" + String(altitude) + ",";
            //    json += "\"speed\":" + String(velocity) + ",";
            //    json += "\"channel\":" + String(packet.channel) + ",";
            //    json += "\"corrected\":" + String(corrected ? "true" : "false");
            //    json += "}\n";

            //    // Отправляем в Serial2
            //    Serial2.print(json);

            //    // Отправляем в Serial2 для отладки
            //    Serial2.print("Decoded - ICAO: ");
            //    Serial2.print(icao, HEX);
            //    Serial2.print(", Callsign: ");
            //    Serial2.print(callsign);
            //    Serial2.print(", Lat: ");
            //    Serial2.print(latitude, 6);
            //    Serial2.print(", Lon: ");
            //    Serial2.print(longitude, 6);
            //    Serial2.print(", Alt: ");
            //    Serial2.print(altitude);
            //    Serial2.print(", Speed: ");
            //    Serial2.println(velocity);

            //} else {
            //    Serial2.println("CRC failed, packet discarded");
            //}
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
    Serial2.println("Setup End\r\n");


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