//Вы правы, в конфигурации DMA есть ошибка. DMA не может читать напрямую с GPIO пинов таким образом. Вот исправленная версия настройки DMA для ADS-B приемника:


#include <Arduino.h>
#include <hardware/dma.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/irq.h>
#include <hardware/pwm.h>
#include <hardware/adc.h>
#include <hardware/gpio.h>
#include <pico/multicore.h>
#include <queue>

// Определения пинов
#define INPUT_PIN_1         19
#define INPUT_PIN_2         22
#define PREAMBLE_LED_1      20
#define PREAMBLE_LED_2      23
#define UART_TX_PIN         4
#define UART_RX_PIN         5
#define AGC_INPUT_PIN       26  // ADC0
#define AGC_OUTPUT_PIN      9
#define AGC_CONTROL_PIN     27  // ADC1
#define STATUS_LED_CORE0    15
#define STATUS_LED_CORE1    25

// Константы ADS-B
#define PREAMBLE_LENGTH     8
#define SHORT_PACKET_BITS   56
#define LONG_PACKET_BITS    112
#define SHORT_PACKET_BYTES  7
#define LONG_PACKET_BYTES   14
#define SAMPLE_RATE         2000000  // 2 MHz
#define BIT_RATE           1000000   // 1 MHz
#define DMA_BUFFER_SIZE     1024

// Структура пакета ADS-B
struct ADSBPacket {
    uint8_t data[14];
    uint8_t length;
    bool isValid;
    uint32_t timestamp;
};

// Структура декодированных данных
struct DecodedData {
    uint32_t icao;
    char flightNumber[9];
    double latitude;
    double longitude;
    uint16_t speed;
    uint16_t altitude;
    uint8_t messageType;
};

// Глобальные переменные
volatile bool preambleDetected = false;
volatile uint32_t bitBuffer = 0;
volatile uint8_t bitCount = 0;
volatile bool packetReady = false;
volatile uint32_t dmaBufferIndex = 0;

// Очереди для пакетов
std::queue<ADSBPacket> rawPacketQueue;
std::queue<ADSBPacket> processedPacketQueue;

// DMA каналы и буферы
int dma_channel_adc;
uint16_t dma_buffer_adc[DMA_BUFFER_SIZE];
volatile bool dma_buffer_ready = false;
volatile uint32_t dma_transfer_count = 0;

// Таблица CRC для ADS-B
uint32_t crc_table[256];

// Генерация таблицы CRC
void generateCRCTable() {
    uint32_t polynomial = 0xFFF409;

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

//========================================================
void dma_irq_handler() 
{
    // Проверяем прерывание для нашего канала
    if (dma_hw->ints0 & (1u << dma_channel_adc)) {
        // Очистка прерывания - правильный способ
        dma_hw->ints0 = 1u << dma_channel_adc;

        dma_buffer_ready = true;
        dma_transfer_count++;

        // Для циклического буфера не нужно перезапускать адрес записи
        // Ring buffer автоматически возвращается к началу

        // Если используется обычный буфер, то нужно перезапустить DMA:
        // dma_channel_set_write_addr(dma_channel_adc, dma_buffer_adc, true);
    }
}
//======================================================



//// DMA interrupt handler  Старая версия
//void dma_irq_handler()
//{
//    if (dma_channel_hw_addr(dma_channel_adc)->ints & (1u << dma_channel_adc)) 
//    {
//        // Очистка прерывания
//        dma_channel_hw_addr(dma_channel_adc)->ints = 1u << dma_channel_adc;
//
//        dma_buffer_ready = true;
//        dma_transfer_count++;
//
//        // Перезапуск DMA для непрерывной передачи
//        dma_channel_set_write_addr(dma_channel_adc, dma_buffer_adc, true);
//    }
//}

// Настройка DMA для ADC
void setupDMA() {
    // Инициализация ADC
    adc_init();
    adc_gpio_init(AGC_INPUT_PIN);
    adc_gpio_init(AGC_CONTROL_PIN);
    adc_select_input(0); // Канал 0 (pin 26)

    // Настройка ADC для свободного режима с максимальной скоростью
    adc_set_clkdiv(1.0f); // Максимальная скорость
    adc_fifo_setup(true, true, 1, false, false); // FIFO включен, DMA запросы, порог 1

    // Получение DMA канала
    dma_channel_adc = dma_claim_unused_channel(true);

    // Конфигурация DMA
    dma_channel_config c = dma_channel_get_default_config(dma_channel_adc);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16); // 16-битные данные ADC
    channel_config_set_read_increment(&c, false); // Чтение из одного места (ADC FIFO)
    channel_config_set_write_increment(&c, true);  // Запись с инкрементом в буфер
    channel_config_set_dreq(&c, DREQ_ADC); // DMA запрос от ADC

    // Настройка циклического буфера
    channel_config_set_ring(&c, true, 11); // 2^11 = 2048 байт = 1024 uint16_t

    dma_channel_configure(
        dma_channel_adc,
        &c,
        dma_buffer_adc,           // Назначение (буфер)
        &adc_hw->fifo,           // Источник (ADC FIFO)
        DMA_BUFFER_SIZE,         // Количество передач
        false                    // Не запускать сразу
    );

    // Включение прерывания DMA
    dma_channel_set_irq0_enabled(dma_channel_adc, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Запуск ADC и DMA
    adc_run(true);
    dma_channel_start(dma_channel_adc);
}

// Альтернативная настройка DMA с PIO для цифровых входов
PIO pio = pio0;
uint sm = 0;
uint offset;

// PIO программа для чтения GPIO пинов
uint16_t pio_program_instructions[] = {
    0x4001, // in pins, 1        ; Чтение 1 бита с входных пинов
    0x8080, // push block         ; Помещение в FIFO с блокировкой
};

struct pio_program pio_gpio_program = {
    .instructions = pio_program_instructions,
    .length = 2,
    .origin = -1
};

void setupPIODMA() {
    // Добавление программы в PIO
    offset = pio_add_program(pio, &pio_gpio_program);

    // Конфигурация state machine
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset, offset + pio_gpio_program.length - 1);
    sm_config_set_in_pins(&c, INPUT_PIN_1); // Базовый пин для входа
    sm_config_set_in_shift(&c, false, false, 32); // Сдвиг влево, без автопуша
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / SAMPLE_RATE);

    // Инициализация GPIO для PIO
    pio_gpio_init(pio, INPUT_PIN_1);
    pio_gpio_init(pio, INPUT_PIN_2);
    gpio_set_dir(INPUT_PIN_1, GPIO_IN);
    gpio_set_dir(INPUT_PIN_2, GPIO_IN);

    // Запуск state machine
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    // Настройка DMA для PIO
    dma_channel_adc = dma_claim_unused_channel(true);

    dma_channel_config dma_c = dma_channel_get_default_config(dma_channel_adc);
    channel_config_set_transfer_data_size(&dma_c, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_c, false);
    channel_config_set_write_increment(&dma_c, true);
    channel_config_set_dreq(&dma_c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(
        dma_channel_adc,
        &dma_c,
        dma_buffer_adc,                    // Назначение
        &pio->rxf[sm],                     // Источник (PIO RX FIFO)
        DMA_BUFFER_SIZE / 2,               // Количество 32-битных слов
        false
    );

    // Включение прерывания DMA
    dma_channel_set_irq0_enabled(dma_channel_adc, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(dma_channel_adc);
}

// Обработка данных из DMA буфера
void processDMABuffer() {
    if (!dma_buffer_ready) return;

    static uint8_t filteredSamples[5];
    static uint8_t sampleIndex = 0;

    for (uint32_t i = 0; i < DMA_BUFFER_SIZE; i++) {
        // Преобразование ADC данных в цифровой сигнал
        uint16_t sample = dma_buffer_adc[i];
        uint8_t digitalBit = (sample > 2048) ? 1 : 0; // Пороговое значение

        // Фильтрация помех
        filteredSamples[sampleIndex] = digitalBit;
        sampleIndex = (sampleIndex + 1) % 5;

        // Применение медианного фильтра
        uint8_t filtered = noiseFilter(filteredSamples, 5);

        // Детекция преамбулы и обработка битов
        processReceivedBit(filtered);
    }

    dma_buffer_ready = false;
}

// Обработка принятого бита
void processReceivedBit(uint8_t bit) {
    static uint16_t preambleBuffer = 0;

    preambleBuffer = (preambleBuffer << 1) | (bit & 1);

    // Детекция преамбулы ADS-B: 1010000101000000
    if ((preambleBuffer & 0xFFFF) == 0xA140) {
        preambleDetected = true;
        digitalWrite(PREAMBLE_LED_1, HIGH);
        digitalWrite(PREAMBLE_LED_2, HIGH);
        bitBuffer = 0;
        bitCount = 0;
        return;
    }

    if (preambleDetected) {
        bitBuffer = (bitBuffer << 1) | (bit & 1);
        bitCount++;

        // Проверяем завершение пакета
        if (bitCount == SHORT_PACKET_BITS || bitCount == LONG_PACKET_BITS) {
            packetReady = true;
            preambleDetected = false;
            digitalWrite(PREAMBLE_LED_1, LOW);
            digitalWrite(PREAMBLE_LED_2, LOW);
        }
    }
}

// Вычисление CRC
uint32_t calculateCRC(uint8_t* data, int length) {
    uint32_t crc = 0;

    for (int i = 0; i < length - 3; i++) {
        uint8_t index = ((crc >> 16) ^ data[i]) & 0xFF;
        crc = ((crc << 8) ^ crc_table[index]) & 0xFFFFFF;
    }

    return crc;
}

// Проверка и коррекция CRC
bool checkAndCorrectCRC(ADSBPacket& packet) {
    uint32_t receivedCRC = (packet.data[packet.length-3] << 16) |
                          (packet.data[packet.length-2] << 8) |
                          packet.data[packet.length-1];

    uint32_t calculatedCRC = calculateCRC(packet.data, packet.length);

    if (receivedCRC == calculatedCRC) {
        return true;
    }

    // Попытка коррекции одного бита
    for (int bytePos = 0; bytePos < packet.length - 3; bytePos++) {
        for (int bitPos = 0; bitPos < 8; bitPos++) {
            packet.data[bytePos] ^= (1 << bitPos);

            uint32_t newCRC = calculateCRC(packet.data, packet.length);
            if (newCRC == receivedCRC) {
                Serial2.println("Corrected single bit error");
                return true;
            }

            packet.data[bytePos] ^= (1 << bitPos);
        }
    }

    return false;
}

// Фильтр помех
uint8_t noiseFilter(uint8_t* samples, int count) {
    if (count < 3) return samples[0];

    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (samples[j] > samples[j + 1]) {
                uint8_t temp = samples[j];
                samples[j] = samples[j + 1];
                samples[j + 1] = temp;
            }
        }
    }

    return samples[count / 2];
}

// Декодирование пакета ADS-B
DecodedData decodeADSBPacket(const ADSBPacket& packet) {
    DecodedData decoded = {0};

    if (packet.length < 7) return decoded;

    decoded.messageType = (packet.data[0] >> 3) & 0x1F;
    decoded.icao = (packet.data[1] << 16) | (packet.data[2] << 8) | packet.data[3];

    switch (decoded.messageType) {
        case 1: case 2: case 3: case 4:
            for (int i = 0; i < 8; i++) {
                uint8_t char_code = 0;
                if (i < 4) {
                    char_code = (packet.data[4 + i/2] >> (4 * (1 - i%2))) & 0x0F;
                } else {
                    char_code = (packet.data[6 + (i-4)/2] >> (4 * (1 - (i-4)%2))) & 0x0F;
                }

                if (char_code >= 1 && char_code <= 26) {
                    decoded.flightNumber[i] = 'A' + char_code - 1;
                } else if (char_code >= 48 && char_code <= 57) {
                    decoded.flightNumber[i] = char_code;
                } else {
                    decoded.flightNumber[i] = ' ';
                }
            }
            decoded.flightNumber[8] = '\0';
            break;

        case 9: case 10: case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18:
            if (packet.length >= 14) {
                uint32_t lat_cpr = ((packet.data[6] & 0x03) << 15) |
                                  (packet.data[7] << 7) |
                                  (packet.data[8] >> 1);
                uint32_t lon_cpr = ((packet.data[8] & 0x01) << 16) |
                                  (packet.data[9] << 8) |
                                  packet.data[10];

                decoded.latitude = (lat_cpr / 131072.0) * 90.0;
                decoded.longitude = (lon_cpr / 131072.0) * 180.0;

                uint16_t alt_code = ((packet.data[5] & 0x0F) << 8) | packet.data[6];
                if (alt_code & 0x40) {
                    decoded.altitude = ((alt_code & 0x7F8) >> 3) * 25 - 1000;
                }
            }
            break;

        case 19:
            if (packet.length >= 14) {
                decoded.speed = ((packet.data[7] & 0x03) << 8) | packet.data[8];
            }
            break;
    }

    return decoded;
}

// Управление AGC
void setupAGC() {
    pinMode(AGC_INPUT_PIN, INPUT);
    pinMode(AGC_CONTROL_PIN, INPUT);

    gpio_set_function(AGC_OUTPUT_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(AGC_OUTPUT_PIN);
    pwm_set_wrap(slice_num, 255);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 128);
    pwm_set_enabled(slice_num, true);
}

void updateAGC() {
    static uint32_t lastAGCUpdate = 0;

    if (millis() - lastAGCUpdate > 10) {
        adc_select_input(0);
        int agcInput = adc_read();

        adc_select_input(1);
        int controlLevel = adc_read();

        int targetLevel = 2048;
        int error = targetLevel - agcInput;

        static int agcOutput = 128;
        agcOutput += error / 128;
        agcOutput = constrain(agcOutput, 0, 255);
        agcOutput = (agcOutput * controlLevel) / 4096;

        uint slice_num = pwm_gpio_to_slice_num(AGC_OUTPUT_PIN);
        pwm_set_chan_level(slice_num, PWM_CHAN_A, agcOutput);

        lastAGCUpdate = millis();
    }
}

// Функция для ядра 1 (прием пакетов)
void core1_entry() {
    pinMode(STATUS_LED_CORE1, OUTPUT);

    while (true) {
        static uint32_t lastBlink = 0;

        if (millis() - lastBlink > 500) {
            digitalWrite(STATUS_LED_CORE1, !digitalRead(STATUS_LED_CORE1));
            lastBlink = millis();
        }

        // Обработка DMA буфера
        processDMABuffer();

        // Обработка готовых пакетов
        if (packetReady) {
            ADSBPacket packet;
            packet.timestamp = micros();
            packet.isValid = false;

            if (bitCount == SHORT_PACKET_BITS) {
                packet.length = SHORT_PACKET_BYTES;
            } else {
                packet.length = LONG_PACKET_BYTES;
            }

            // Преобразование битов в байты
            for (int i = 0; i < packet.length; i++) {
                packet.data[i] = (bitBuffer >> ((packet.length - 1 - i) * 8)) & 0xFF;
            }

            Serial2.print("Raw packet (");
            Serial2.print(packet.length);
            Serial2.print(" bytes): ");
            for (int i = 0; i < packet.length; i++) {
                Serial2.print(packet.data[i], HEX);
                Serial2.print(" ");
            }
            Serial2.println();

           //!! rawPacketQueue.push(packet);
            packetReady = false;
        }

        updateAGC();
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


    pinMode(INPUT_PIN_1, INPUT);
    pinMode(INPUT_PIN_2, INPUT);
    pinMode(PREAMBLE_LED_1, OUTPUT);
    pinMode(PREAMBLE_LED_2, OUTPUT);
    pinMode(STATUS_LED_CORE0, OUTPUT);
    pinMode(STATUS_LED_CORE1, OUTPUT);

    generateCRCTable();

    // Выбор метода DMA: ADC или PIO
    #ifdef USE_ADC_DMA
        setupDMA();        // Для аналогового входа через ADC
    #else
        setupPIODMA();     // Для цифрового входа через PIO
    #endif

    setupAGC();

    multicore_launch_core1(core1_entry);

    Serial2.println("ADS-B receiver started");
}

void loop() {
    static uint32_t lastBlink = 0;

    if (millis() - lastBlink > 1000) {
        digitalWrite(STATUS_LED_CORE0, !digitalRead(STATUS_LED_CORE0));
        lastBlink = millis();
    }

    if (!rawPacketQueue.empty()) 
    {
       //!! ADSBPacket packet = rawPacketQueue.front();
        rawPacketQueue.pop();

        //if (checkAndCorrectCRC(packet)) 
        //{
        //    packet.isValid = true;

        //    DecodedData decoded = decodeADSBPacket(packet);

        //    if (decoded.icao != 0) {
        //        String output = "ICAO: " + String(decoded.icao, HEX) +
        //                      ", Flight: " + String(decoded.flightNumber) +
        //                      ", Lat: " + String(decoded.latitude, 6) +
        //                      ", Lon: " + String(decoded.longitude, 6) +
        //                      ", Speed: " + String(decoded.speed) +
        //                      ", Alt: " + String(decoded.altitude) +
        //                      ", Type: " + String(decoded.messageType);

        //        Serial2.println(output);
        //       // Serial.println(output);
        //    }

        //    processedPacketQueue.push(packet);
        //}
        //else 
        //{
        //    Serial2.println("CRC failed and couldn't correct packet");
        //}
    }

    delay(1);
}

/*
Основные исправления в DMA:

ADC DMA: Настроен для чтения аналогового сигнала через ADC FIFO
PIO DMA: Альтернативный вариант для цифрового сигнала через PIO
Правильные источники: &adc_hw->fifo для ADC, &pio->rxf[sm] для PIO
Циклический буфер: Использование ring buffer для непрерывной работы
Прерывания DMA: Корректная обработка завершения передач

Теперь DMA правильно настроен для захвата данных с высокой скоростью.
*/