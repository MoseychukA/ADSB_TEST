/*
 * Дополнительные примеры и утилиты для ADS-B CRC
 */

#ifndef ADS_B_EXAMPLES_H
#define ADS_B_EXAMPLES_H

// Структура для хранения примера ADS-B сообщения
struct ADSBExample {
  uint8_t data[ADS_B_DATA_LENGTH];
  uint32_t expected_crc;
  const char* description;
};

// Массив тестовых примеров
const ADSBExample test_examples[] PROGMEM = {
  // Пример 1: Типичное сообщение ADS-B
  {
    {0x8D, 0x40, 0x62, 0x1D, 0x58, 0xC3, 0x82, 0xD6, 0x90, 0xC8, 0xAC, 0x28, 0x00, 0x00},
    0xA7832F,
    "Airborne Position Message"
  },
  
  // Пример 2: Сообщение идентификации
  {
    {0x8D, 0x48, 0x40, 0x44, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0, 0x57, 0x60, 0x98},
    0x2B7FB8,
    "Aircraft Identification"
  },
  
  // Пример 3: Сообщение скорости
  {
    {0x8D, 0x40, 0x62, 0x1D, 0x99, 0x11, 0x59, 0x15, 0x45, 0x53, 0x20, 0x00, 0x00, 0x00},
    0x5D7391,
    "Airborne Velocity"
  }
};

const uint8_t TEST_EXAMPLES_COUNT = sizeof(test_examples) / sizeof(ADSBExample);

// Функция запуска расширенных тестов
void runExtendedTests() {
  Serial.println(F("\n=== Расширенные тесты ==="));
  
  for (uint8_t i = 0; i < TEST_EXAMPLES_COUNT; i++) {
    ADSBExample example;
    memcpy_P(&example, &test_examples[i], sizeof(ADSBExample));
    
    Serial.print(F("Тест "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.println(example.description);
    
    uint32_t calculated_crc = calculateCRC24_fast(example.data, ADS_B_DATA_LENGTH);
    
    Serial.print(F("  Ожидаемый CRC: 0x"));
    if (example.expected_crc < 0x100000UL) Serial.print("0");
    if (example.expected_crc < 0x10000UL) Serial.print("0");
    if (example.expected_crc < 0x1000UL) Serial.print("0");
    if (example.expected_crc < 0x100UL) Serial.print("0");
    if (example.expected_crc < 0x10UL) Serial.print("0");
    Serial.println(example.expected_crc, HEX);
    
    Serial.print(F("  Расчётный CRC:  0x"));
    if (calculated_crc < 0x100000UL) Serial.print("0");
    if (calculated_crc < 0x10000UL) Serial.print("0");
    if (calculated_crc < 0x1000UL) Serial.print("0");
    if (calculated_crc < 0x100UL) Serial.print("0");
    if (calculated_crc < 0x10UL) Serial.print("0");
    Serial.println(calculated_crc, HEX);
    
    bool passed = (calculated_crc == example.expected_crc);
    Serial.print(F("  Результат: "));
    Serial.println(passed ? F("ПРОЙДЕН") : F("ОШИБКА"));
    Serial.println();
  }
}

#endif // ADS_B_EXAMPLES_H
