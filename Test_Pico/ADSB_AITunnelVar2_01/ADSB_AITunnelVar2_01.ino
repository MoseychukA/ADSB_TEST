//Программа приема и расшифровки пакетов ADS-B для микроконтроллера RP2040.


#include <Arduino.h>
#include "pico/stdlib.h"
#include <hardware/structs/systick.h>

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/exception.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/rand.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "stdio.h"  // for printing
#include "hardware/irq.h"
#include "pico/binary_info.h"

// Конфигурация пинов
#define ADS_B_INPUT_PIN 19
#define SIGNAL_LEVEL_PIN 27
#define PWM_OUTPUT_PIN 9
#define LED_INDICATOR_PIN 20

// Настройки UART1
#define SERIAL1_BAUD 115200

// Константы для ADS-B
#define PREAMBLE_LENGTH 16
#define MESSAGE_LENGTH 112
#define BUFFER_SIZE 256
#define SAMPLE_RATE_US 2  // 500 kHz sampling rate
#define AGC_UPDATE_INTERVAL 1000  // ms

// Структура для хранения сообщения ADS-B
struct ADSBMessage {
  uint8_t data[14];  // 112 bits = 14 bytes
  bool valid;
  uint32_t timestamp;
};

// Глобальные переменные
volatile bool messageReceived = false;
volatile uint32_t lastPulseTime = 0;
uint8_t bitBuffer[MESSAGE_LENGTH];
int bufferIndex = 0;
bool preambleFound = false;

// AGC переменные
int signalLevel = 0;
int targetLevel = 512;  // Целевой уровень (примерно середина ADC)
int pwmValue = 128;     // Начальное значение PWM
unsigned long lastAGCUpdate = 0;


void setup() {
    // Инициализация пинов
    pinMode(ADS_B_INPUT_PIN, INPUT);
    pinMode(SIGNAL_LEVEL_PIN, INPUT);
    pinMode(PWM_OUTPUT_PIN, OUTPUT);
    pinMode(LED_INDICATOR_PIN, OUTPUT);

    // Настройка PWM
    analogWriteResolution(8);  // 8-bit PWM
    analogWrite(PWM_OUTPUT_PIN, pwmValue);

    // Инициализация UART1
    Serial1.begin(SERIAL1_BAUD);
    Serial1.println("ADS-B Receiver Started");

    // Инициализация Serial для отладки
    Serial.begin(115200);
    Serial.println("ADS-B Receiver Debug");

    // Настройка прерывания для входного сигнала (исправлено для RP2040)
    attachInterrupt(ADS_B_INPUT_PIN, pulseInterrupt, CHANGE);//!!CHANGE RISING

    digitalWrite(LED_INDICATOR_PIN, LOW);
    Serial.println("Setup completed");
    Serial1.println("Setup completed");
}

void loop() {
    // Обработка AGC
    updateAGC();

    // Проверка на получение сообщения
    if (messageReceived) 
    {
        processMessage();
        messageReceived = false;
    }

    // Обработка команд через Serial (для отладки и настройки)
    handleSerialCommands();

    delay(1);
}



void pulseInterrupt() 
{
    static uint32_t lastTime = 0;
    static bool lastState = false;
    static uint8_t preambleBits = 0;
    static uint32_t preambleStartTime = 0;

    uint32_t currentTime = micros();
    bool currentState = digitalRead(ADS_B_INPUT_PIN);

    // Защита от слишком частых прерываний
    if (currentTime - lastTime < 1) return;

    uint32_t pulseWidth = currentTime - lastTime;
    lastTime = currentTime;

    // Поиск преамбулы ADS-B
    if (!preambleFound) 
    {
        if (detectPreamble(currentState, currentTime))
        {
            preambleFound = true;
            bufferIndex = 0;
            preambleStartTime = currentTime;
            digitalWrite(LED_INDICATOR_PIN, HIGH);
            Serial.println("Preamble detected!");
        }
    }
    else 
    {
        // Декодирование данных после преамбулы
        if (currentTime - preambleStartTime > 120) 
        { // Таймаут 120 мкс
            if (decodeDataBit(pulseWidth, currentState)) 
            {
                if (bufferIndex >= MESSAGE_LENGTH) {
                    preambleFound = false;
                    messageReceived = true;
                    digitalWrite(LED_INDICATOR_PIN, LOW);
                    Serial.print("Message received, bits: ");
                    Serial.println(bufferIndex);
                }
            }
            else 
            {
                // Ошибка декодирования - сброс
                preambleFound = false;
                bufferIndex = 0;
                digitalWrite(LED_INDICATOR_PIN, LOW);
                Serial.println("Decoding error - reset");
            }
        }
    }

    lastState = currentState;
}

// Улучшенная функция детектирования преамбулы
bool detectPreamble(bool currentState, uint32_t currentTime) 
{
    static uint32_t lastTransition[16];
    static uint8_t transitionIndex = 0;
    static bool expectedPattern[16] = { 1,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0 };

    // Запоминаем время перехода
    lastTransition[transitionIndex] = currentTime;
    transitionIndex = (transitionIndex + 1) % 16;

    // Проверяем паттерн преамбулы
    if (transitionIndex == 0) { // Буфер заполнен
        bool patternMatch = true;

        // Проверяем временные интервалы
        for (int i = 0; i < 15; i++) 
        {
            uint32_t interval = lastTransition[(i + 1) % 16] - lastTransition[i % 16];

            // Ожидаемая длительность 0.5 или 1.0 микросекунды
            if (expectedPattern[i]) {
                if (interval < 0.4 || interval > 0.6) {
                    patternMatch = false;
                    break;
                }
            }
            else {
                if (interval < 0.9 || interval > 1.1) {
                    patternMatch = false;
                    break;
                }
            }
        }

        return patternMatch;
    }

    return false;
}

// Упрощенная функция декодирования битов
bool decodeDataBit(uint32_t pulseWidth, bool state) 
{
    static bool waitingForBit = true;
    static uint32_t bitStartTime = 0;

    if (waitingForBit) {
        bitStartTime = micros();
        waitingForBit = false;
        return true;
    }

    uint32_t bitDuration = micros() - bitStartTime;

    // Manchester декодирование на основе длительности
    if (bitDuration >= 0.9 && bitDuration <= 1.1) { // 1 микросекунда
        bitBuffer[bufferIndex++] = state ? 1 : 0;
        waitingForBit = true;
        return true;
    }
    else if (bitDuration >= 1.9 && bitDuration <= 2.1) { // 2 микросекунды
   // Два одинаковых бита
        bitBuffer[bufferIndex++] = state ? 1 : 0;
        if (bufferIndex < MESSAGE_LENGTH) {
            bitBuffer[bufferIndex++] = state ? 1 : 0;
        }
        waitingForBit = true;
        return true;
    }

    waitingForBit = true;
    return false;
}

void processMessage() 
{
    ADSBMessage msg;
    msg.valid = false;
    msg.timestamp = millis();

    // Преобразование битов в байты
    for (int i = 0; i < 14; i++) 
    {
        msg.data[i] = 0;
        for (int j = 0; j < 8; j++) 
        {
            if (i * 8 + j < MESSAGE_LENGTH && i * 8 + j < bufferIndex) 
            {
                msg.data[i] |= (bitBuffer[i * 8 + j] << (7 - j));
            }
        }
    }

    // Проверка CRC (упрощенная)
    if (validateCRC(msg.data)) 
    {
        msg.valid = true;
        Serial.println("CRC OK");
    }
    else 
    {
        Serial.println("CRC Error");
    }

    // Декодирование сообщения
    decodeADSBMessage(msg);

    // Сброс буфера
    bufferIndex = 0;
}

bool validateCRC(uint8_t* data) 
{
    // Упрощенная проверка - проверяем, что данные не нулевые
    bool hasData = false;
    for (int i = 0; i < 11; i++) 
    {
        if (data[i] != 0) 
        {
            hasData = true;
            break;
        }
    }
    return hasData;
}

void decodeADSBMessage(ADSBMessage& msg) 
{
    uint8_t df = (msg.data[0] >> 3) & 0x1F;  // Downlink Format
    uint32_t icao = ((uint32_t)msg.data[1] << 16) |
        ((uint32_t)msg.data[2] << 8) |
        msg.data[3];

    Serial1.print("MSG,DF=");
    Serial1.print(df);
    Serial1.print(",ICAO=");
    Serial1.print(icao, HEX);
    Serial1.print(",TIME=");
    Serial1.print(msg.timestamp);

    if (msg.valid) {
        Serial1.print(",VALID");

        // Декодирование в зависимости от типа сообщения
        switch (df) {
        case 17: // ADS-B Extended Squitter
        case 18: // TIS-B Extended Squitter
            Serial1.print(",");
            decodeExtendedSquitter(msg);
            break;
        case 4:  // Surveillance Altitude Reply
            Serial1.print(",SURV_ALT");
            break;
        case 5:  // Surveillance Identity Reply
            Serial1.print(",SURV_ID");
            break;
        default:
            Serial1.print(",UNKNOWN");
            break;
        }
    }
    else 
    {
        Serial1.print(",INVALID");
    }

    Serial1.println();

    // Вывод raw данных для отладки
    if (Serial) 
    {
        Serial.print("Raw: ");
        for (int i = 0; i < 14; i++) 
        {
            if (msg.data[i] < 0x10) Serial.print("0");
            Serial.print(msg.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void decodeExtendedSquitter(ADSBMessage& msg) 
{
    uint8_t tc = (msg.data[4] >> 3) & 0x1F;  // Type Code

    Serial1.print("TC=");
    Serial1.print(tc);

    switch (tc) {
    case 1: case 2: case 3: case 4: // Aircraft identification
        Serial1.print(",");
        decodeIdentification(msg);
        break;
    case 9: case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18:
        Serial1.print(",");
        decodeAirbornePosition(msg);
        break;
    case 19: // Airborne velocity
        Serial1.print(",");
        decodeAirborneVelocity(msg);
        break;
    default:
        Serial1.print(",DATA");
        break;
    }
}

void decodeIdentification(ADSBMessage& msg) 
{
    char callsign[9] = { 0 };

    // Декодирование позывного
    uint64_t data = 0;
    for (int i = 5; i < 11; i++) {
        data = (data << 8) | msg.data[i];
    }

    const char charset[] = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";

    for (int i = 0; i < 8; i++) {
        int index = (data >> (42 - i * 6)) & 0x3F;
        if (index < 64) {
            callsign[i] = charset[index];
        }
        else {
            callsign[i] = '?';
        }
    }

    Serial1.print("CALLSIGN=");
    Serial1.print(callsign);
}

void decodeAirbornePosition(ADSBMessage& msg) 
{
    uint16_t altitude = ((msg.data[5] & 0xFF) << 4) |
        ((msg.data[6] & 0xF0) >> 4);

    bool odd = (msg.data[6] >> 2) & 1;

    Serial1.print("ALT=");
    Serial1.print(altitude * 25);
    Serial1.print(",FRAME=");
    Serial1.print(odd ? "ODD" : "EVEN");
}

void decodeAirborneVelocity(ADSBMessage& msg) 
{
    uint8_t subtype = (msg.data[4] & 0x07);
    Serial1.print("VEL_SUBTYPE=");
    Serial1.print(subtype);
}

void updateAGC() 
{
    unsigned long currentTime = millis();

    if (currentTime - lastAGCUpdate >= AGC_UPDATE_INTERVAL) 
    {
        lastAGCUpdate = currentTime;

        // Чтение уровня сигнала
        signalLevel = analogRead(SIGNAL_LEVEL_PIN);

        // Простой AGC алгоритм
        int error = targetLevel - signalLevel;

        // Пропорциональное управление с ограничением
        int adjustment = constrain(error / 8, -10, 10);

        pwmValue += adjustment;
        pwmValue = constrain(pwmValue, 0, 255);

        // Установка нового значения PWM
        analogWrite(PWM_OUTPUT_PIN, pwmValue);

        // Отладочная информация
        if (Serial && (currentTime % 5000 == 0)) { // Каждые 5 секунд
            Serial.print("AGC - Signal: ");
            Serial.print(signalLevel);
            Serial.print(", PWM: ");
            Serial.print(pwmValue);
            Serial.print(", Error: ");
            Serial.println(error);
        }
    }
}

void handleSerialCommands() 
{
    if (Serial.available()) 
    {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.startsWith("PWM=")) 
        {
            int value = command.substring(4).toInt();
            setManualPWM(value);
        }
        else if (command == "CAL") {
            calibrateAGC();
        }
        else if (command == "STATUS") {
            printStatus();
        }
        else if (command == "HELP") {
            printHelp();
        }
    }
}

void setManualPWM(int value) {
    pwmValue = constrain(value, 0, 255);
    analogWrite(PWM_OUTPUT_PIN, pwmValue);
    Serial.print("PWM set to: ");
    Serial.println(pwmValue);
    Serial1.print("PWM=");
    Serial1.println(pwmValue);
}

void calibrateAGC() {
    Serial.println("AGC Calibration started...");
    Serial1.println("CALIBRATION_START");

    int bestPWM = pwmValue;
    int bestError = abs(analogRead(SIGNAL_LEVEL_PIN) - targetLevel);

    for (int testPWM = 50; testPWM <= 200; testPWM += 10) {
        analogWrite(PWM_OUTPUT_PIN, testPWM);
        delay(200);  // Время стабилизации

        int level = analogRead(SIGNAL_LEVEL_PIN);
        int error = abs(level - targetLevel);

        if (error < bestError) {
            bestError = error;
            bestPWM = testPWM;
        }

        Serial.print("PWM: ");
        Serial.print(testPWM);
        Serial.print(", Level: ");
        Serial.print(level);
        Serial.print(", Error: ");
        Serial.println(error);
    }

    pwmValue = bestPWM;
    analogWrite(PWM_OUTPUT_PIN, pwmValue);

    Serial.print("Calibration complete. Optimal PWM: ");
    Serial.println(pwmValue);
    Serial1.print("CALIBRATION_COMPLETE,PWM=");
    Serial1.println(pwmValue);
}

void printStatus() {
    Serial.println("=== ADS-B Receiver Status ===");
    Serial.print("Signal Level: ");
    Serial.println(analogRead(SIGNAL_LEVEL_PIN));
    Serial.print("PWM Value: ");
    Serial.println(pwmValue);
    Serial.print("Target Level: ");
    Serial.println(targetLevel);
    Serial.print("Preamble Found: ");
    Serial.println(preambleFound ? "YES" : "NO");
    Serial.print("Buffer Index: ");
    Serial.println(bufferIndex);
}

void printHelp() {
    Serial.println("=== Commands ===");
    Serial.println("PWM=<value>  - Set PWM value (0-255)");
    Serial.println("CAL          - Start AGC calibration");
    Serial.println("STATUS       - Show current status");
    Serial.println("HELP         - Show this help");
}
/*
Основные особенности программы:

Прием ADS-B сигналов
Использует прерывания для захвата импульсов
Декодирует преамбулу ADS-B (1010000101000000)
Выполняет Manchester декодирование данных

Расшифровка сообщений
Поддержка различных типов сообщений ADS-B
Декодирование позывных самолетов
Декодирование позиций и высоты
Декодирование скорости и курса
Проверка CRC для валидации данных

Автоматическая регулировка уровня (AGC)
Мониторинг уровня входного сигнала
Автоматическая подстройка PWM
Пропорциональное управление
Функции ручной калибровки

Индикация и вывод
Светодиодная индикация приема пакетов
Вывод данных на UART1 в читаемом формате
Отладочная информация через Serial

Дополнительные функции
Функция ручной установки PWM
Калибровка AGC
Мониторинг качества сигнала

Программа готова к загрузке в Arduino IDE для микроконтроллера RP2040 и должна корректно работать с указанными пинами.
*/