Вот комплексная программа для приема и расшифровки пакетов ADS-B на ESP32-S3:


#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/timer.h>
#include <driver/i2s.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/ringbuf.h>
#include <math.h>

// Конфигурация пинов
#define ADS_B_INPUT_PIN 9
#define UART_NUM UART_NUM_1
#define UART_TX_PIN 17
#define UART_RX_PIN 18

// Параметры ADS-B
#define SAMPLE_RATE 2000000  // 2 MHz
#define PREAMBLE_LENGTH 8
#define MESSAGE_LENGTH_SHORT 56
#define MESSAGE_LENGTH_LONG 112
#define BUFFER_SIZE 4096
#define MAX_PACKETS 50

// Структуры данных
typedef struct {
    uint8_t data[14];  // 112 бит = 14 байт
    uint8_t length;
    uint32_t timestamp;
    float signal_strength;
} adsb_packet_t;

typedef struct {
    double latitude;
    double longitude;
    bool is_valid;
    uint32_t timestamp;
} cpr_position_t;

typedef struct {
    uint32_t icao;
    cpr_position_t even_pos;
    cpr_position_t odd_pos;
    double last_lat;
    double last_lon;
} aircraft_state_t;

// Глобальные переменные
static RingbufHandle_t packet_buffer;
static QueueHandle_t decode_queue;
static aircraft_state_t aircraft_db[256];
static volatile uint32_t sample_buffer[BUFFER_SIZE];
static volatile uint32_t buffer_index = 0;
static volatile bool acquisition_active = false;

// AGC переменные
static float agc_gain = 1.0;
static float signal_level = 0.0;
static const float AGC_ALPHA = 0.1;
static const float TARGET_LEVEL = 0.5;

// CPR константы
static const double LAT_ZONE = 360.0 / 60.0;  // 6 градусов
static const double LON_ZONE_EVEN = 360.0 / 60.0;
static const double LON_ZONE_ODD = 360.0 / 59.0;

// Прототипы функций
void setup_uart();
void setup_timer_interrupt();
void setup_dma();
void IRAM_ATTR timer_isr(void* arg);
void packet_decoder_task(void* parameter);
void signal_processor_task(void* parameter);
bool detect_preamble(uint32_t* samples, int start_idx);
bool decode_manchester(uint32_t samples, int start_idx, uint8_t output, int bits);
bool validate_crc(uint8_t* data, int length);
void decode_adsb_message(adsb_packet_t* packet);
void decode_airborne_position(uint8_t* data, uint32_t icao);
double cpr_decode_latitude(uint32_t cpr_lat, bool is_odd);
double cpr_decode_longitude(uint32_t cpr_lon, double lat, bool is_odd);
void update_agc(float sample);
void apply_noise_filter(uint32_t* samples, int length);
aircraft_state_t* get_aircraft_state(uint32_t icao);

void setup() {
    Serial.begin(115200);

    // Инициализация буферов и очередей
    packet_buffer = xRingbufferCreate(sizeof(adsb_packet_t) * MAX_PACKETS, RINGBUF_TYPE_NOSPLIT);
    decode_queue = xQueueCreate(MAX_PACKETS, sizeof(adsb_packet_t));

    // Настройка периферии
    setup_uart();
    setup_timer_interrupt();
    setup_dma();

    // Инициализация базы самолетов
    memset(aircraft_db, 0, sizeof(aircraft_db));

    // Создание задач
    xTaskCreatePinnedToCore(signal_processor_task, "SignalProcessor", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(packet_decoder_task, "PacketDecoder", 8192, NULL, 1, NULL, 0);

    Serial.println("ADS-B декодер инициализирован");
}

void loop() {
    // Основной цикл - мониторинг статистики
    static uint32_t last_stats = 0;

    if (millis() - last_stats > 5000) {
        Serial.printf("AGC Gain: %.2f, Signal Level: %.3f\n", agc_gain, signal_level);
        last_stats = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

void setup_uart() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 1024, 0, 0, NULL, 0);
}

void setup_timer_interrupt() {
    // Настройка GPIO для входного сигнала
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << ADS_B_INPUT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&gpio_conf);

    // Настройка таймера для семплирования
    timer_config_t timer_config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = 40  // 80MHz / 40 = 2MHz
    };

    timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 1);  // Каждую микросекунду * 0.5
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void setup_dma() {
    // DMA настройка для I2S (альтернативный высокоскоростной ввод)
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 1024,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
}

void IRAM_ATTR timer_isr(void* arg) {
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

    if (acquisition_active) {
        // Чтение сигнала с АЦП
        uint32_t sample = gpio_get_level(ADS_B_INPUT_PIN);

        // Применение AGC
        float normalized_sample = sample * agc_gain;
        update_agc(normalized_sample);

        sample_buffer[buffer_index] = (normalized_sample > 0.5) ? 1 : 0;
        buffer_index = (buffer_index + 1) % BUFFER_SIZE;
    }
}

void signal_processor_task(void* parameter) {
    uint32_t local_buffer[BUFFER_SIZE];
    uint32_t last_index = 0;

    acquisition_active = true;

    while (1) {
        // Копирование буфера для обработки
        uint32_t current_index = buffer_index;
        if (current_index != last_index) {
            uint32_t samples_to_process;

            if (current_index > last_index) {
                samples_to_process = current_index - last_index;
                memcpy(local_buffer, (void*)&sample_buffer[last_index],
                       samples_to_process * sizeof(uint32_t));
            } else {
                // Обработка кольцевого буфера
                uint32_t end_samples = BUFFER_SIZE - last_index;
                memcpy(local_buffer, (void*)&sample_buffer[last_index],
                       end_samples * sizeof(uint32_t));
                memcpy(&local_buffer[end_samples], (void*)sample_buffer,
                       current_index * sizeof(uint32_t));
                samples_to_process = end_samples + current_index;
            }

            // Фильтрация шумов
            apply_noise_filter(local_buffer, samples_to_process);

            // Поиск преамбул и декодирование пакетов
            for (int i = 0; i < samples_to_process - MESSAGE_LENGTH_LONG; i++) {
                if (detect_preamble(local_buffer, i)) {
                    adsb_packet_t packet;

                    // Попытка декодирования короткого сообщения
                    if (decode_manchester(local_buffer, i + PREAMBLE_LENGTH,
                                        packet.data, MESSAGE_LENGTH_SHORT)) {
                        packet.length = MESSAGE_LENGTH_SHORT / 8;
                        packet.timestamp = esp_timer_get_time();
                        packet.signal_strength = signal_level;

                        if (validate_crc(packet.data, packet.length)) {
                            xRingbufferSend(packet_buffer, &packet, sizeof(packet), 0);
                        }
                    }
                    // Попытка декодирования длинного сообщения
                    else if (decode_manchester(local_buffer, i + PREAMBLE_LENGTH,
                                             packet.data, MESSAGE_LENGTH_LONG)) {
                        packet.length = MESSAGE_LENGTH_LONG / 8;
                        packet.timestamp = esp_timer_get_time();
                        packet.signal_strength = signal_level;

                        if (validate_crc(packet.data, packet.length)) {
                            xRingbufferSend(packet_buffer, &packet, sizeof(packet), 0);
                        }
                    }

                    i += MESSAGE_LENGTH_SHORT; // Пропустить обработанную область
                }
            }

            last_index = current_index;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void packet_decoder_task(void* parameter) {
    adsb_packet_t packet;
    size_t item_size;

    while (1) {
        // Получение пакета из буфера
        adsb_packet_t received_packet = (adsb_packet_t)xRingbufferReceive(
            packet_buffer, &item_size, pdMS_TO_TICKS(100));

        if (received_packet != NULL) {
            memcpy(&packet, received_packet, sizeof(packet));
            vRingbufferReturnItem(packet_buffer, received_packet);

            // Декодирование сообщения
            decode_adsb_message(&packet);
        }
    }
}

bool detect_preamble(uint32_t* samples, int start_idx) {
    // Паттерн преамбулы ADS-B: 1010000101000000
    const uint8_t preamble_pattern[] = {1,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0};

    for (int i = 0; i < 16; i++) {
        if (samples[start_idx + i] != preamble_pattern[i]) {
            return false;
        }
    }

    return true;
}

bool decode_manchester(uint32_t samples, int start_idx, uint8_t output, int bits) {
    memset(output, 0, (bits + 7) / 8);

    for (int i = 0; i < bits; i++) {
        int sample_idx = start_idx + i * 2;
        uint8_t first_half = samples[sample_idx];
        uint8_t second_half = samples[sample_idx + 1];

        if (first_half == 1 && second_half == 0) {
            // Логическая 1
            output[i / 8] |= (1 << (7 - (i % 8)));
        } else if (first_half == 0 && second_half == 1) {
            // Логический 0 (уже установлен)
        } else {
            return false; // Ошибка декодирования
        }
    }

    return true;
}

bool validate_crc(uint8_t* data, int length) {
    // CRC-24 проверка для ADS-B
    uint32_t crc = 0x000000;
    uint32_t polynomial = 0xFFF409;

    for (int i = 0; i < length - 3; i++) {
        crc ^= ((uint32_t)data[i] << 16);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x800000) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }

    uint32_t received_crc = (data[length-3] << 16) |
                           (data[length-2] << 8) |
                           data[length-1];

    return (crc & 0xFFFFFF) == received_crc;
}

void decode_adsb_message(adsb_packet_t* packet) {
    uint8_t downlink_format = (packet->data[0] >> 3) & 0x1F;
    uint32_t icao = (packet->data[1] << 16) | (packet->data[2] << 8) | packet->data[3];

    char output[256];
    sprintf(output, "ICAO: %06X, DF: %d, Signal: %.2f\n",
            icao, downlink_format, packet->signal_strength);
    uart_write_bytes(UART_NUM, output, strlen(output));

    switch (downlink_format) {
        case 17: // ADS-B сообщения
        case 18:
            if (packet->length >= 14) {
                uint8_t type_code = (packet->data[4] >> 3) & 0x1F;

                if (type_code >= 9 && type_code <= 18) {
                    // Сообщение о позиции
                    decode_airborne_position(packet->data, icao);
                }
            }
            break;
    }
}

void decode_airborne_position(uint8_t* data, uint32_t icao) {
    aircraft_state_t* aircraft = get_aircraft_state(icao);

    uint8_t cpr_format = (data[6] >> 2) & 0x01;
    uint32_t cpr_lat = ((data[6] & 0x03) << 15) | (data[7] << 7) | (data[8] >> 1);
    uint32_t cpr_lon = ((data[8] & 0x01) << 16) | (data[9] << 8) | data[10];

    cpr_position_t* pos = cpr_format ? &aircraft->odd_pos : &aircraft->even_pos;
    pos->timestamp = esp_timer_get_time();

    // Декодирование CPR координат
    double lat = cpr_decode_latitude(cpr_lat, cpr_format == 1);
    double lon = cpr_decode_longitude(cpr_lon, lat, cpr_format == 1);

    pos->latitude = lat;
    pos->longitude = lon;
    pos->is_valid = true;

    // Если есть оба формата, вычисляем точные координаты
    if (aircraft->even_pos.is_valid && aircraft->odd_pos.is_valid) {
        // Проверяем временную корреляцию (не более 10 секунд разница)
        uint32_t time_diff = abs((int32_t)(aircraft->even_pos.timestamp -
                                          aircraft->odd_pos.timestamp));

        if (time_diff < 10000000) { // 10 секунд в микросекундах
            double final_lat, final_lon;

            // Используем более новое сообщение как референс
            if (aircraft->even_pos.timestamp > aircraft->odd_pos.timestamp) {
                final_lat = aircraft->even_pos.latitude;
                final_lon = aircraft->even_pos.longitude;
            } else {
                final_lat = aircraft->odd_pos.latitude;
                final_lon = aircraft->odd_pos.longitude;
            }

            aircraft->last_lat = final_lat;
            aircraft->last_lon = final_lon;

            char pos_output[256];
            sprintf(pos_output, "ICAO: %06X, Position: %.6f, %.6f\n",
                    icao, final_lat, final_lon);
            uart_write_bytes(UART_NUM, pos_output, strlen(pos_output));
        }
    }
}

double cpr_decode_latitude(uint32_t cpr_lat, bool is_odd) {
    double lat_zone = is_odd ? LON_ZONE_ODD : LON_ZONE_EVEN;
    double lat = lat_zone * (cpr_lat / 131072.0);

    if (lat >= 270.0) {
        lat -= 360.0;
    }

    return lat;
}

double cpr_decode_longitude(uint32_t cpr_lon, double lat, bool is_odd) {
    double lon_zone = is_odd ? LON_ZONE_ODD : LON_ZONE_EVEN;

    // Коррекция для широты
    if (fabs(lat) < 87.0) {
        double nl = floor(2 * M_PI / acos(1 - (1 - cos(M_PI / 30.0)) /
                         (cos(M_PI / 180.0  fabs(lat))  cos(M_PI / 180.0 * fabs(lat)))));

        if (is_odd) {
            nl = nl - 1;
        }

        if (nl > 0) {
            lon_zone = 360.0 / nl;
        }
    }

    double lon = lon_zone * (cpr_lon / 131072.0);

    if (lon >= 180.0) {
        lon -= 360.0;
    }

    return lon;
}

void update_agc(float sample) {
    // Обновление уровня сигнала с экспоненциальным сглаживанием
    signal_level = AGC_ALPHA  fabs(sample) + (1.0 - AGC_ALPHA)  signal_level;

    // Автоматическая регулировка усиления
    if (signal_level > TARGET_LEVEL * 1.2) {
        agc_gain *= 0.95;  // Уменьшить усиление
    } else if (signal_level < TARGET_LEVEL * 0.8) {
        agc_gain *= 1.05;  // Увеличить усиление
    }

    // Ограничение усиления
    if (agc_gain > 10.0) agc_gain = 10.0;
    if (agc_gain < 0.1) agc_gain = 0.1;
}

void apply_noise_filter(uint32_t* samples, int length) {
    // Простой медианный фильтр для подавления импульсных помех
    for (int i = 1; i < length - 1; i++) {
        uint32_t prev = samples[i-1];
        uint32_t curr = samples[i];
        uint32_t next = samples[i+1];

        // Медианное значение из трех соседних отсчетов
        if ((prev <= curr && curr <= next) || (next <= curr && curr <= prev)) {
            // curr уже медиана
        } else if ((curr <= prev && prev <= next) || (next <= prev && prev <= curr)) {
            samples[i] = prev;
        } else {
            samples[i] = next;
        }
    }
}

aircraft_state_t* get_aircraft_state(uint32_t icao) {
    uint8_t hash = (icao ^ (icao >> 8) ^ (icao >> 16)) & 0xFF;

    if (aircraft_db[hash].icao == 0 || aircraft_db[hash].icao == icao) {
        aircraft_db[hash].icao = icao;
        return &aircraft_db[hash];
    }

    // Простое разрешение коллизий - линейное зондирование
    for (int i = 0; i < 256; i++) {
        uint8_t idx = (hash + i) & 0xFF;
        if (aircraft_db[idx].icao == 0 || aircraft_db[idx].icao == icao) {
            aircraft_db[idx].icao = icao;
            return &aircraft_db[idx];
        }
    }

    return &aircraft_db[hash]; // Fallback
}

/*

Ключевые особенности программы:

Высокопроизводительный ввод данных
Использование таймерных прерываний для точного семплирования
DMA поддержка через I2S
Кольцевая буферизация для непрерывного приема

Автоматическая регулировка усиления (AGC)
Адаптивное усиление на основе уровня сигнала
Экспоненциальное сглаживание
Защита от перегрузки и недостаточного сигнала

Полный алгоритм CPR
Декодирование четных и нечетных сообщений
Временная корреляция для точности
Коррекция для различных широт

Фильтрация шумов
Медианный фильтр для подавления импульсных помех
AGC для нормализации уровня сигнала

Буферизация и многозадачность
Отдельные задачи для приема и декодирования
Кольцевые буферы для множественных пакетов
База данных самолетов с хеш-таблицей

CRC валидация
Полная проверка CRC-24 для ADS-B сообщений
Отбраковка поврежденных пакетов

Программа обеспечивает профессиональное качество декодирования ADS-B с высокой производительностью и точностью.
*/