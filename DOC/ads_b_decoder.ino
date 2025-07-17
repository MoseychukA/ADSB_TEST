/*
 * ADS-B Message Decoder for Arduino
 * Полная расшифровка и анализ пакетов ADS-B
 * Поддерживает все основные типы сообщений
 * Автор: [Ваше имя]
 * Дата: 14.07.2025
 */

#include <Arduino.h>

// Константы ADS-B
#define ADS_B_DATA_LENGTH 14
#define ADS_B_CRC_LENGTH 3
#define ADS_B_TOTAL_LENGTH 17
#define CRC24_POLYNOMIAL 0x1FFF409UL

// Структуры для хранения декодированных данных
struct ADSBMessage {
  uint8_t downlink_format;
  uint8_t capability;
  uint32_t icao_address;
  uint8_t type_code;
  uint8_t message_type;
  uint8_t raw_data[ADS_B_DATA_LENGTH];
  bool crc_valid;
};

struct AircraftPosition {
  bool valid;
  double latitude;
  double longitude;
  uint16_t altitude_feet;
  uint8_t cpr_format;
  uint32_t cpr_lat;
  uint32_t cpr_lon;
  uint8_t surveillance_status;
  uint8_t nic_supplement_b;
  uint8_t altitude_type;
  bool time_flag;
  bool cpr_odd_even;
};

struct AircraftVelocity {
  bool valid;
  uint16_t ground_speed_knots;
  uint16_t track_degrees;
  int16_t vertical_rate_fpm;
  uint8_t velocity_type;
  bool intent_change;
  bool ifr_capability;
  uint8_t navigation_accuracy;
  int16_t heading_degrees;
  bool supersonic;
};

struct AircraftIdentification {
  bool valid;
  char callsign[9];
  uint8_t aircraft_category;
  uint8_t emitter_category;
};

// Глобальные переменные
uint32_t crc_table[256];
bool table_initialized = false;

// Инициализация CRC таблицы
void initializeCRCTable() {
  if (table_initialized) return;
  
  for (int i = 0; i < 256; i++) {
    uint32_t crc = (uint32_t)i << 16;
    
    for (int j = 0; j < 8; j++) {
      if (crc & 0x800000UL) {
        crc = ((crc << 1) ^ CRC24_POLYNOMIAL) & 0xFFFFFFUL;
      } else {
        crc = (crc << 1) & 0xFFFFFFUL;
      }
    }
    crc_table[i] = crc;
  }
  table_initialized = true;
}

// Расчет CRC-24
uint32_t calculateCRC24(const uint8_t* data, uint8_t length) {
  if (!table_initialized) initializeCRCTable();
  
  uint32_t crc = 0x000000UL;
  
  for (uint8_t i = 0; i < length; i++) {
    uint8_t table_index = ((crc >> 16) ^ data[i]) & 0xFF;
    crc = ((crc << 8) ^ crc_table[table_index]) & 0xFFFFFFUL;
  }
  
  return crc;
}

// Проверка CRC пакета
bool verifyCRC(const uint8_t* packet) {
  uint32_t calculated_crc = calculateCRC24(packet, ADS_B_DATA_LENGTH);
  uint32_t received_crc = ((uint32_t)packet[14] << 16) |
                          ((uint32_t)packet[15] << 8) |
                          (uint32_t)packet[16];
  return calculated_crc == received_crc;
}

// Извлечение битов из массива байт
uint32_t extractBits(const uint8_t* data, uint8_t start_bit, uint8_t num_bits) {
  uint32_t result = 0;
  
  for (uint8_t i = 0; i < num_bits; i++) {
    uint8_t byte_index = (start_bit + i) / 8;
    uint8_t bit_index = 7 - ((start_bit + i) % 8);
    
    if (data[byte_index] & (1 << bit_index)) {
      result |= (1UL << (num_bits - 1 - i));
    }
  }
  
  return result;
}

// Основная функция декодирования
ADSBMessage decodeADSBMessage(const uint8_t* packet) {
  ADSBMessage msg = {};
  
  // Копируем исходные данные
  memcpy(msg.raw_data, packet, ADS_B_DATA_LENGTH);
  
  // Проверяем CRC
  msg.crc_valid = verifyCRC(packet);
  
  // Извлекаем основные поля
  msg.downlink_format = (packet[0] >> 3) & 0x1F;
  msg.capability = packet[0] & 0x07;
  
  // ICAO адрес (24 бита)
  msg.icao_address = ((uint32_t)packet[1] << 16) |
                     ((uint32_t)packet[2] << 8) |
                     (uint32_t)packet[3];
  
  // Type Code (первые 5 бит ME поля)
  msg.type_code = (packet[4] >> 3) & 0x1F;
  
  // Определяем тип сообщения
  if (msg.type_code >= 1 && msg.type_code <= 4) {
    msg.message_type = 1; // Aircraft Identification
  } else if (msg.type_code >= 9 && msg.type_code <= 18) {
    msg.message_type = 2; // Airborne Position
  } else if (msg.type_code >= 20 && msg.type_code <= 22) {
    msg.message_type = 3; // Airborne Position (GNSS Height)
  } else if (msg.type_code == 19) {
    msg.message_type = 4; // Airborne Velocity
  } else {
    msg.message_type = 0; // Unknown/Other
  }
  
  return msg;
}

// Декодирование идентификации воздушного судна
AircraftIdentification decodeIdentification(const ADSBMessage& msg) {
  AircraftIdentification ident = {};
  
  if (msg.type_code < 1 || msg.type_code > 4) {
    ident.valid = false;
    return ident;
  }
  
  ident.valid = true;
  ident.aircraft_category = msg.type_code;
  
  // Декодирование callsign (символы кодируются в 6 битах каждый)
  const char charset[] = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######";
  
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t char_bits = extractBits(msg.raw_data, 40 + i * 6, 6);
    ident.callsign[i] = charset[char_bits];
  }
  ident.callsign[8] = '\0';
  
  // Удаляем trailing символы
  for (int i = 7; i >= 0; i--) {
    if (ident.callsign[i] == '#' || ident.callsign[i] == '_') {
      ident.callsign[i] = '\0';
    } else {
      break;
    }
  }
  
  return ident;
}

// Декодирование позиции воздушного судна
AircraftPosition decodePosition(const ADSBMessage& msg) {
  AircraftPosition pos = {};
  
  if ((msg.type_code < 9 || msg.type_code > 18) && 
      (msg.type_code < 20 || msg.type_code > 22)) {
    pos.valid = false;
    return pos;
  }
  
  pos.valid = true;
  
  // Surveillance Status (2 бита)
  pos.surveillance_status = extractBits(msg.raw_data, 35, 2);
  
  // NIC Supplement B
  pos.nic_supplement_b = extractBits(msg.raw_data, 37, 1);
  
  // Altitude (12 бит)
  uint16_t alt_encoded = extractBits(msg.raw_data, 40, 12);
  pos.altitude_type = extractBits(msg.raw_data, 47, 1); // Q bit
  
  if (pos.altitude_type) {
    // 25ft increment
    pos.altitude_feet = ((alt_encoded & 0xFF0) >> 1) * 25 - 1000;
  } else {
    // Gillham code (более сложное декодирование)
    pos.altitude_feet = 0; // Упрощенно
  }
  
  // Time flag
  pos.time_flag = extractBits(msg.raw_data, 52, 1);
  
  // CPR format (Odd/Even frame indicator)
  pos.cpr_odd_even = extractBits(msg.raw_data, 53, 1);
  pos.cpr_format = pos.cpr_odd_even;
  
  // Encoded latitude and longitude (17 bits each)
  pos.cpr_lat = extractBits(msg.raw_data, 54, 17);
  pos.cpr_lon = extractBits(msg.raw_data, 71, 17);
  
  // Упрощенное декодирование координат (требует парные кадры для точности)
  // Это базовое приближение
  pos.latitude = (double)pos.cpr_lat / 131072.0 * (pos.cpr_odd_even ? 360.0/59.0 : 360.0/60.0) - 90.0;
  pos.longitude = (double)pos.cpr_lon / 131072.0 * 360.0 - 180.0;
  
  return pos;
}

// Декодирование скорости воздушного судна
AircraftVelocity decodeVelocity(const ADSBMessage& msg) {
  AircraftVelocity vel = {};
  
  if (msg.type_code != 19) {
    vel.valid = false;
    return vel;
  }
  
  vel.valid = true;
  
  // Subtype (3 bits)
  uint8_t subtype = extractBits(msg.raw_data, 37, 3);
  vel.velocity_type = subtype;
  
  // Intent change flag
  vel.intent_change = extractBits(msg.raw_data, 40, 1);
  
  // IFR capability flag  
  vel.ifr_capability = extractBits(msg.raw_data, 41, 1);
  
  // Navigation Accuracy Category for Velocity
  vel.navigation_accuracy = extractBits(msg.raw_data, 42, 3);
  
  if (subtype == 1 || subtype == 2) {
    // Ground speed and track angle
    
    // East-West velocity component
    bool ew_sign = extractBits(msg.raw_data, 45, 1);
    uint16_t ew_velocity = extractBits(msg.raw_data, 46, 10);
    
    // North-South velocity component  
    bool ns_sign = extractBits(msg.raw_data, 56, 1);
    uint16_t ns_velocity = extractBits(msg.raw_data, 57, 10);
    
    if (ew_velocity != 0 && ns_velocity != 0) {
      int16_t ew_vel = ew_sign ? -(ew_velocity - 1) : (ew_velocity - 1);
      int16_t ns_vel = ns_sign ? -(ns_velocity - 1) : (ns_velocity - 1);
      
      // Calculate ground speed and track
      vel.ground_speed_knots = sqrt(ew_vel * ew_vel + ns_vel * ns_vel);
      vel.track_degrees = atan2(ew_vel, ns_vel) * 180.0 / PI;
      if (vel.track_degrees < 0) vel.track_degrees += 360;
    }
    
    // Vertical rate
    bool vr_sign = extractBits(msg.raw_data, 67, 1);
    uint16_t vr_value = extractBits(msg.raw_data, 68, 9);
    
    if (vr_value != 0) {
      vel.vertical_rate_fpm = vr_sign ? -(vr_value - 1) * 64 : (vr_value - 1) * 64;
    }
    
  } else if (subtype == 3 || subtype == 4) {
    // Airspeed and heading
    
    // Heading status
    bool heading_available = extractBits(msg.raw_data, 45, 1);
    
    if (heading_available) {
      uint16_t heading = extractBits(msg.raw_data, 46, 10);
      vel.heading_degrees = heading * 360 / 1024;
    }
    
    // Airspeed type (IAS or TAS)
    bool airspeed_type = extractBits(msg.raw_data, 56, 1);
    uint16_t airspeed = extractBits(msg.raw_data, 57, 10);
    
    if (airspeed != 0) {
      vel.ground_speed_knots = airspeed - 1; // Приближение
    }
    
    // Vertical rate (same as above)
    bool vr_sign = extractBits(msg.raw_data, 67, 1);
    uint16_t vr_value = extractBits(msg.raw_data, 68, 9);
    
    if (vr_value != 0) {
      vel.vertical_rate_fpm = vr_sign ? -(vr_value - 1) * 64 : (vr_value - 1) * 64;
    }
  }
  
  return vel;
}

// Функции вывода результатов
void printHex(const uint8_t* data, uint8_t length) {
  for (uint8_t i = 0; i < length; i++) {
    if (data[i] < 16) Serial.print("0");
    Serial.print(data[i], HEX);
    if (i < length - 1) Serial.print(" ");
  }
}

void printADSBMessage(const ADSBMessage& msg) {
  Serial.println(F("\n=== Анализ ADS-B сообщения ==="));
  
  Serial.print(F("Исходные данные: "));
  printHex(msg.raw_data, ADS_B_DATA_LENGTH);
  Serial.println();
  
  Serial.print(F("CRC статус: "));
  Serial.println(msg.crc_valid ? F("ВАЛИДЕН") : F("ОШИБКА"));
  
  Serial.print(F("Downlink Format: "));
  Serial.println(msg.downlink_format);
  
  Serial.print(F("Capability: "));
  Serial.println(msg.capability);
  
  Serial.print(F("ICAO адрес: 0x"));
  if (msg.icao_address < 0x100000UL) Serial.print("0");
  if (msg.icao_address < 0x10000UL) Serial.print("0");
  if (msg.icao_address < 0x1000UL) Serial.print("0");
  if (msg.icao_address < 0x100UL) Serial.print("0");
  if (msg.icao_address < 0x10UL) Serial.print("0");
  Serial.println(msg.icao_address, HEX);
  
  Serial.print(F("Type Code: "));
  Serial.println(msg.type_code);
  
  Serial.print(F("Тип сообщения: "));
  switch (msg.message_type) {
    case 1: Serial.println(F("Идентификация ВС")); break;
    case 2: Serial.println(F("Позиция ВС (барометрическая)")); break;
    case 3: Serial.println(F("Позиция ВС (GNSS)")); break;
    case 4: Serial.println(F("Скорость ВС")); break;
    default: Serial.println(F("Неизвестный/Другой")); break;
  }
  
  Serial.println();
}

void printIdentification(const AircraftIdentification& ident) {
  if (!ident.valid) {
    Serial.println(F("Данные идентификации недоступны"));
    return;
  }
  
  Serial.println(F("=== Идентификация воздушного судна ==="));
  
  Serial.print(F("Позывной: \""));
  Serial.print(ident.callsign);
  Serial.println("\"");
  
  Serial.print(F("Категория ВС: "));
  switch (ident.aircraft_category) {
    case 1: Serial.println(F("Light")); break;
    case 2: Serial.println(F("Small")); break;
    case 3: Serial.println(F("Large")); break;
    case 4: Serial.println(F("High Vortex Large")); break;
    default: Serial.println(F("Unknown")); break;
  }
  
  Serial.println();
}

void printPosition(const AircraftPosition& pos) {
  if (!pos.valid) {
    Serial.println(F("Данные позиции недоступны"));
    return;
  }
  
  Serial.println(F("=== Позиция воздушного судна ==="));
  
  Serial.print(F("Широта: "));
  Serial.print(pos.latitude, 6);
  Serial.println(F("°"));
  
  Serial.print(F("Долгота: "));
  Serial.print(pos.longitude, 6);
  Serial.println(F("°"));
  
  Serial.print(F("Высота: "));
  Serial.print(pos.altitude_feet);
  Serial.println(F(" футов"));
  
  Serial.print(F("CPR формат: "));
  Serial.println(pos.cpr_odd_even ? F("Нечетный") : F("Четный"));
  
  Serial.print(F("Статус наблюдения: "));
  Serial.println(pos.surveillance_status);
  
  Serial.print(F("Тип высоты: "));
  Serial.println(pos.altitude_type ? F("Барометрическая") : F("Gillham код"));
  
  Serial.println();
}

void printVelocity(const AircraftVelocity& vel) {
  if (!vel.valid) {
    Serial.println(F("Данные скорости недоступны"));
    return;
  }
  
  Serial.println(F("=== Скорость воздушного судна ==="));
  
  Serial.print(F("Путевая скорость: "));
  Serial.print(vel.ground_speed_knots);
  Serial.println(F(" узлов"));
  
  Serial.print(F("Путевой угол: "));
  Serial.print(vel.track_degrees);
  Serial.println(F("°"));
  
  Serial.print(F("Вертикальная скорость: "));
  Serial.print(vel.vertical_rate_fpm);
  Serial.println(F(" фут/мин"));
  
  Serial.print(F("Тип скорости: "));
  Serial.println(vel.velocity_type);
  
  Serial.print(F("Изменение намерений: "));
  Serial.println(vel.intent_change ? F("Да") : F("Нет"));
  
  Serial.print(F("IFR возможность: "));
  Serial.println(vel.ifr_capability ? F("Да") : F("Нет"));
  
  Serial.println();
}

// Полный анализ пакета
void analyzeADSBPacket(const uint8_t* packet) {
  ADSBMessage msg = decodeADSBMessage(packet);
  printADSBMessage(msg);
  
  if (!msg.crc_valid) {
    Serial.println(F("⚠️ ВНИМАНИЕ: CRC не валиден, данные могут быть повреждены"));
    return;
  }
  
  switch (msg.message_type) {
    case 1: {
      AircraftIdentification ident = decodeIdentification(msg);
      printIdentification(ident);
      break;
    }
    
    case 2:
    case 3: {
      AircraftPosition pos = decodePosition(msg);
      printPosition(pos);
      break;
    }
    
    case 4: {
      AircraftVelocity vel = decodeVelocity(msg);
      printVelocity(vel);
      break;
    }
    
    default:
      Serial.println(F("Тип сообщения не поддерживается для детального анализа"));
      break;
  }
}

// Обработка hex строки
void processHexData(String hexData) {
  hexData.trim();
  hexData.toUpperCase();
  hexData.replace(" ", "");
  
  if (hexData.length() != 28 && hexData.length() != 34) {
    Serial.println(F("Ошибка: Неверная длина hex данных"));
    Serial.println(F("Ожидается 28 символов (данные) или 34 символа (пакет)"));
    return;
  }
  
  uint8_t packet[ADS_B_TOTAL_LENGTH] = {0};
  uint8_t length = hexData.length() / 2;
  
  // Конвертируем hex в байты
  for (uint8_t i = 0; i < length; i++) {
    String byteStr = hexData.substring(i * 2, i * 2 + 2);
    packet[i] = strtoul(byteStr.c_str(), NULL, 16);
  }
  
  if (length == ADS_B_DATA_LENGTH) {
    Serial.println(F("⚠️ Данные без CRC - добавляем нулевой CRC"));
    // Добавляем нулевой CRC для анализа структуры
  }
  
  analyzeADSBPacket(packet);
}

// Демонстрационные примеры
void runDemoExamples() {
  Serial.println(F("\n=== Демонстрационные примеры ===\n"));
  
  // Пример 1: Aircraft Identification
  Serial.println(F("📋 Пример 1: Идентификация воздушного судна"));
  uint8_t example1[] = {
    0x8D, 0x48, 0x40, 0x44, 0x20, 0x2C, 0xC3, 0x71, 
    0xC3, 0x2C, 0xE0, 0x57, 0x60, 0x98, 0x2B, 0x7F, 0xB8
  };
  analyzeADSBPacket(example1);
  
  // Пример 2: Airborne Position
  Serial.println(F("🛩️ Пример 2: Позиция воздушного судна"));
  uint8_t example2[] = {
    0x8D, 0x40, 0x62, 0x1D, 0x58, 0xC3, 0x82, 0xD6, 
    0x90, 0xC8, 0xAC, 0x28, 0x00, 0x00, 0xA7, 0x83, 0x2F
  };
  analyzeADSBPacket(example2);
  
  // Пример 3: Velocity
  Serial.println(F("💨 Пример 3: Скорость воздушного судна"));
  uint8_t example3[] = {
    0x8D, 0x40, 0x62, 0x1D, 0x99, 0x11, 0x59, 0x15, 
    0x45, 0x53, 0x20, 0x00, 0x00, 0x00, 0x5D, 0x73, 0x91
  };
  analyzeADSBPacket(example3);
}

// Команды Serial
void processSerialCommand() {
  if (!Serial.available()) return;
  
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();
  
  if (command == "HELP" || command == "?") {
    printHelp();
  }
  else if (command == "DEMO") {
    runDemoExamples();
  }
  else if (command.startsWith("DECODE ")) {
    processHexData(command.substring(7));
  }
  else if (command.startsWith("HEX ")) {
    processHexData(command.substring(4));
  }
  else if (command.length() > 0) {
    // Попытка интерпретировать как hex данные
    if (command.length() == 28 || command.length() == 34) {
      processHexData(command);
    } else {
      Serial.println(F("Неизвестная команда. Введите HELP для справки."));
    }
  }
}

void printHelp() {
  Serial.println(F("\n=== ADS-B Message Decoder - Команды ==="));
  Serial.println(F("HELP или ?           - Эта справка"));
  Serial.println(F("DEMO                 - Демонстрационные примеры"));
  Serial.println(F("DECODE <hex>         - Декодировать hex данные"));
  Serial.println(F("HEX <hex>            - Алиас для DECODE"));
  Serial.println(F("<hex_data>           - Прямой ввод hex данных"));
  Serial.println(F(""));
  Serial.println(F("Примеры:"));
  Serial.println(F("DECODE 8D4840442C71C32CE05760982B7FB8"));
  Serial.println(F("HEX 8D40621D58C382D690C8AC280000A7832F"));
  Serial.println(F("8D40621D99115915455320000005D7391"));
  Serial.println(F(""));
  Serial.println(F("Поддерживаемые типы сообщений:"));
  Serial.println(F("• Идентификация воздушного судна (TC 1-4)"));
  Serial.println(F("• Позиция воздушного судна (TC 9-18, 20-22)"));
  Serial.println(F("• Скорость воздушного судна (TC 19)"));
  Serial.println();
}

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Ждем подключения Serial для Leonardo/Micro
  }
  
  Serial.println(F("ADS-B Message Decoder для Arduino"));
  Serial.println(F("================================="));
  Serial.print(F("Свободная память: "));
  Serial.print(freeMemory());
  Serial.println(F(" байт"));
  Serial.println();
  
  // Инициализация CRC
  initializeCRCTable();
  Serial.println(F("✅ CRC таблица инициализирована"));
  
  Serial.println(F("ℹ️ Готов к декодированию ADS-B сообщений"));
  Serial.println(F("Введите HELP для получения справки"));
  Serial.println(F("Введите DEMO для демонстрационных примеров"));
  Serial.print(F("> "));
}

void loop() {
  processSerialCommand();
  delay(10);
}
