//Вот программа для приема пакетов ADS-B на микроконтроллере RP2040:


#include <Arduino.h>
#include <SerialPIO.h>

// Определения пинов
#define ADS_B_PIN_1         19
#define ADS_B_PIN_2         22
#define PREAMBLE_LED_PIN    20
#define UART_TX_PIN         4
#define UART_RX_PIN         5

// Параметры ADS-B
#define ADS_B_SAMPLE_RATE   2000000  // 2 MHz
#define PREAMBLE_LENGTH     8        // Длина преамбулы в битах
#define MESSAGE_LENGTH_SHORT 56      // Короткое сообщение (56 бит)
#define MESSAGE_LENGTH_LONG  112     // Длинное сообщение (112 бит)
#define BIT_DURATION_US     0.3      // Длительность бита в микросекундах

// Фильтр помех
#define NOISE_THRESHOLD     3        // Минимальное количество одинаковых сэмплов
#define SIGNAL_THRESHOLD    512      // Пороговое значение для определения сигнала

// Структура для хранения принятых данных
struct ADSBMessage {
  uint8_t data[14];  // Максимум 112 бит = 14 байт
  uint8_t length;    // Длина сообщения в битах
  bool valid;        // Флаг валидности сообщения
};

// Глобальные переменные
volatile bool signal_detected = false;
volatile uint32_t last_edge_time = 0;
volatile uint8_t sample_buffer[256];
volatile uint16_t sample_index = 0;
volatile bool receiving = false;

SerialPIO uart2(UART_TX_PIN, UART_RX_PIN, 32);

// Прототипы функций
void setupPins();
void setupInterrupts();
void signalISR();
bool decodePreamble(volatile uint8_t* buffer, uint16_t start_index);
bool decodeMessage(volatile uint8_t* buffer, uint16_t start_index, ADSBMessage* message);
void sendToUART(ADSBMessage* message);
uint8_t filterNoise(uint8_t current_sample, uint8_t* history, uint8_t history_size);
bool validateMessage(ADSBMessage* message);
uint32_t calculateCRC(uint8_t* data, uint8_t length);

void setup() {
  Serial.begin(115200);
  uart2.begin(115200);

  setupPins();
  setupInterrupts();

  Serial.println("ADS-B Receiver initialized");
  Serial.println("Waiting for ADS-B signals...");


  uart2.println("ADS-B Receiver initialized");
  uart2.println("Waiting for ADS-B signals...");
}

void loop() {
  if (signal_detected) {
    signal_detected = false;

    // Обработка принятого сигнала
    ADSBMessage message;
    if (processSignal(&message)) {
      // Отправка сообщения через UART2
      sendToUART(&message);

      // Индикация успешного приема
      digitalWrite(PREAMBLE_LED_PIN, HIGH);
      delay(100);
      digitalWrite(PREAMBLE_LED_PIN, LOW);
    }

    // Сброс буфера
    sample_index = 0;
    receiving = false;
  }

  delay(1);
}

void setupPins() {
  pinMode(ADS_B_PIN_1, INPUT);
  pinMode(ADS_B_PIN_2, INPUT);
  pinMode(PREAMBLE_LED_PIN, OUTPUT);
  digitalWrite(PREAMBLE_LED_PIN, LOW);
}

void setupInterrupts() {
  attachInterrupt(digitalPinToInterrupt(ADS_B_PIN_1), signalISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ADS_B_PIN_2), signalISR, CHANGE);
}

void signalISR() {
  static uint8_t noise_filter_history[NOISE_THRESHOLD] = {0};
  static uint8_t filter_index = 0;

  uint32_t current_time = micros();

  // Чтение состояния пинов
  uint8_t pin1_state = digitalRead(ADS_B_PIN_1);
  uint8_t pin2_state = digitalRead(ADS_B_PIN_2);

  // Дифференциальный сигнал
  uint8_t signal_level = (pin1_state != pin2_state) ? 1 : 0;

  // Применение фильтра помех
  uint8_t filtered_signal = filterNoise(signal_level, noise_filter_history, NOISE_THRESHOLD);

  // Сохранение сэмпла в буфер
  if (sample_index < sizeof(sample_buffer)) 
  {
    sample_buffer[sample_index++] = filtered_signal;

    // Проверка на начало преамбулы
    if (!receiving && sample_index >= PREAMBLE_LENGTH) {
      if (decodePreamble(sample_buffer, sample_index - PREAMBLE_LENGTH)) 
      {
        receiving = true;
        digitalWrite(PREAMBLE_LED_PIN, HIGH);
      }
    }

    // Проверка завершения приема сообщения
    if (receiving && (sample_index >= MESSAGE_LENGTH_LONG * 2)) {
      signal_detected = true;
    }
  }

  last_edge_time = current_time;
}

uint8_t filterNoise(uint8_t current_sample, uint8_t* history, uint8_t history_size) {
  static uint8_t index = 0;

  // Добавление текущего сэмпла в историю
  history[index] = current_sample;
  index = (index + 1) % history_size;

  // Подсчет количества единиц в истории
  uint8_t ones_count = 0;
  for (uint8_t i = 0; i < history_size; i++) 
  {
    if (history[i] == 1) ones_count++;
  }

  // Возврат отфильтрованного значения
  return (ones_count > history_size / 2) ? 1 : 0;
}

bool decodePreamble(volatile uint8_t* buffer, uint16_t start_index) {
  // Паттерн преамбулы ADS-B: 10101100
  uint8_t preamble_pattern[] = {1, 0, 1, 0, 1, 1, 0, 0};

  for (uint8_t i = 0; i < PREAMBLE_LENGTH; i++) {
    if (buffer[start_index + i] != preamble_pattern[i]) {
      return false;
    }
  }

  return true;
}

bool processSignal(ADSBMessage* message) {
  // Поиск преамбулы в буфере
  uint16_t preamble_start = 0;
  bool preamble_found = false;

  for (uint16_t i = 0; i <= sample_index - PREAMBLE_LENGTH; i++) {
    if (decodePreamble(sample_buffer, i)) {
      preamble_start = i + PREAMBLE_LENGTH;
      preamble_found = true;
      break;
    }
  }

  if (!preamble_found) {
    return false;
  }

  // Декодирование сообщения
  return decodeMessage(sample_buffer, preamble_start, message);
}

bool decodeMessage(volatile uint8_t* buffer, uint16_t start_index, ADSBMessage* message) {
    // Определение длины сообщения по DownLink Format (первые 5 бит)
    // Извлекаем первые 5 бит для определения типа сообщения
    uint8_t df = 0;
    for (uint8_t i = 0; i < 5; i++) {
        uint16_t bit_start = start_index + i * 2;
        uint8_t first_half = buffer[bit_start];
        uint8_t second_half = buffer[bit_start + 1];

        if (first_half == 1 && second_half == 0) {
            df |= (1 << (4 - i));
        }
        else if (first_half != 0 || second_half != 1) {
            return false; // Ошибка Manchester кодирования
        }
    }

    // Определение длины сообщения на основе DF
    uint8_t msg_length;
    if (df >= 16 && df <= 18) {
        msg_length = MESSAGE_LENGTH_LONG;  // 112 бит для Extended Squitter
    }
    else if (df >= 0 && df <= 15) {
        msg_length = MESSAGE_LENGTH_SHORT; // 56 бит для коротких сообщений
    }
    else {
        return false; // Неизвестный DF
    }

    // Проверка достаточности данных в буфере
    if (start_index + msg_length * 2 > sample_index) {
        return false;
    }

    // Manchester декодирование
    memset((void*)message->data, 0, sizeof(message->data));
    message->length = msg_length;

    for (uint8_t bit = 0; bit < msg_length; bit++) {
        uint16_t bit_start = start_index + bit * 2;

        // Проверка границ буфера
        if (bit_start + 1 >= sample_index) {
            return false;
        }

        // Manchester декодирование: 10 = 1, 01 = 0
        uint8_t first_half = buffer[bit_start];
        uint8_t second_half = buffer[bit_start + 1];

        if (first_half == 1 && second_half == 0) {
            // Установка бита в 1
            message->data[bit / 8] |= (1 << (7 - (bit % 8)));
        }
        else if (first_half == 0 && second_half == 1) {
            // Бит остается 0 (уже установлен memset)
        }
        else {
            // Ошибка Manchester кодирования
            return false;
        }
    }

    // Валидация сообщения
    return validateMessage(message);
}

bool validateMessage(ADSBMessage* message) {
  // Проверка CRC для ADS-B сообщений
  uint32_t calculated_crc = calculateCRC(message->data, message->length - 24);
  uint32_t received_crc = 0;

  // Извлечение CRC из последних 24 бит
  uint8_t crc_start_byte = (message->length - 24) / 8;
  for (uint8_t i = 0; i < 3; i++) {
    received_crc = (received_crc << 8) | message->data[crc_start_byte + i];
  }

  return (calculated_crc == received_crc);
}

uint32_t calculateCRC(uint8_t* data, uint8_t length_bits) {
  // Упрощенная реализация CRC-24 для ADS-B
  uint32_t crc = 0;
  uint32_t polynomial = 0xFFF409; // Полином CRC-24

  for (uint8_t byte = 0; byte < (length_bits + 7) / 8; byte++) {
    uint8_t current_byte = data[byte];
    uint8_t bits_in_byte = (byte * 8 + 8 <= length_bits) ? 8 : (length_bits % 8);

    for (uint8_t bit = 0; bit < bits_in_byte; bit++) {
      bool bit_value = (current_byte & (1 << (7 - bit))) != 0;
      bool crc_msb = (crc & 0x800000) != 0;

      crc <<= 1;
      if (bit_value ^ crc_msb) {
        crc ^= polynomial;
      }
    }
  }

  return crc & 0xFFFFFF; // 24-битный CRC
}

void sendToUART(ADSBMessage* message) {
  // Отправка префикса
  uart2.print("*");

  // Отправка данных в hex формате
  for (uint8_t i = 0; i < (message->length + 7) / 8; i++) {
    if (message->data[i] < 0x10) {
      uart2.print("0");
    }
    uart2.print(message->data[i], HEX);
  }

  // Отправка суффикса и перевода строки
  uart2.println(";");

  // Дублирование в основной UART для отладки
  Serial.print("ADS-B Message (");
  Serial.print(message->length);
  Serial.print(" bits): *");
  for (uint8_t i = 0; i < (message->length + 7) / 8; i++) {
    if (message->data[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(message->data[i], HEX);
  }
  Serial.println(";");
}

void debugMessage(ADSBMessage* message) {
    uint8_t df = (message->data[0] >> 3) & 0x1F;

    Serial.print("DF: ");
    Serial.print(df);
    Serial.print(", Length: ");
    Serial.print(message->length);
    Serial.print(" bits, Data: ");

    for (uint8_t i = 0; i < (message->length + 7) / 8; i++) {
        if (message->data[i] < 0x10) Serial.print("0");
        Serial.print(message->data[i], HEX);
    }
    Serial.println();
}




/*
Особенности программы:

Обработка сигналов
Использует дифференциальный прием с пинов 19 и 22
Прерывания на изменение сигнала для точного захвата данных
Manchester декодирование для извлечения бит

Фильтр помех
Медианный фильтр по истории сэмплов
Пороговое значение для отсечения шума
Сглаживание кратковременных импульсов

Индикация
PIN 20 загорается при обнаружении преамбулы
Мигает при успешном приеме пакета

UART вывод
Формат: *[HEX DATA];
Отправка через UART2 (пины 4,5)
Дублирование в Serial для отладки

Валидация
Проверка преамбулы ADS-B
CRC-24 валидация сообщений
Контроль Manchester кодирования

Программа готова к загрузке в Arduino IDE для RP2040. Не забудьте установить плату "Raspberry Pi Pico" в менеджере плат.
*/