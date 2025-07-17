

#include <WiFi.h>
#include <USB.h>
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"
#include <HardwareSerial.h>

// WiFi настройки
const char* wifi_ssid = "TPLINK";
const char* wifi_password = "panasonic";

// UART настройки
HardwareSerial UART1_Port(1);

// RTL-SDR VID/PID
#define RTL_SDR_VID 0x0bda
#define RTL_SDR_PID 0x2838

// USB Host переменные
usb_host_client_handle_t client_hdl = NULL;
usb_device_handle_t device_hdl = NULL;
volatile bool usb_device_connected = false;
volatile bool system_running = true;

// Синхронизация transfer'ов
volatile bool g_transfer_done = false;
volatile usb_transfer_status_t g_transfer_status = USB_TRANSFER_STATUS_ERROR;
SemaphoreHandle_t transfer_mutex = NULL;

// Буферы (статические для стабильности)
static uint8_t sample_buffer[1024];
static uint8_t queue_buffer[1024];
QueueHandle_t sample_queue = NULL;

// Статистика
uint32_t packets_processed = 0;
uint32_t usb_errors = 0;

void setup() {
    Serial.begin(115200);
    delay(3000);  // Больше времени на стабилизацию

    Serial.println("=== Стабильная версия ADS-B приемника ===");
    Serial.printf("Свободной heap памяти: %d байт\n", ESP.getFreeHeap());

    // Инициализация UART
    UART1_Port.begin(115200, SERIAL_8N1, 18, 17);
    UART1_Port.println("SYSTEM: Stable ADS-B Decoder v2.0");

    // Создание семафора для синхронизации
    transfer_mutex = xSemaphoreCreateMutex();
    if (transfer_mutex == NULL) {
        Serial.println("КРИТИЧЕСКАЯ ОШИБКА: Не удалось создать семафор");
        return;
    }

    // Создание очереди
    sample_queue = xQueueCreate(3, sizeof(uint8_t) * 1024);
    if (sample_queue == NULL) {
        Serial.println("КРИТИЧЕСКАЯ ОШИБКА: Не удалось создать очередь");
        return;
    }

    // WiFi подключение
    connectToWiFi();

    // USB Host инициализация
    if (!initUSBHostSafe()) {
        Serial.println("КРИТИЧЕСКАЯ ОШИБКА: USB Host");
        return;
    }

    // Создание задач с проверкой
    BaseType_t result;

    result = xTaskCreatePinnedToCore(
        safeUSBTask,
        "SafeUSB",
        4096,
        NULL,
        3,
        NULL,
        0
    );

    if (result != pdPASS) {
        Serial.println("КРИТИЧЕСКАЯ ОШИБКА: Не удалось создать USB задачу");
        return;
    }

    result = xTaskCreatePinnedToCore(
        safeADSBTask,
        "SafeADSB",
        3072,
        NULL,
        2,
        NULL,
        1
    );

    if (result != pdPASS) {
        Serial.println("КРИТИЧЕСКАЯ ОШИБКА: Не удалось создать ADS-B задачу");
        return;
    }

    Serial.println("Система инициализирована успешно");
    Serial.println("Ожидание RTL-SDR устройства...");
}

void loop() {
    static uint32_t last_status_time = 0;
    static uint32_t last_heap_check = 0;

    // Проверка системы каждые 3 секунды
    if (millis() - last_status_time > 3000) {
        Serial.printf("USB: %s | Очередь: %d | Пакетов: %d | Ошибок: %d\n",
            usb_device_connected ? "OK" : "NO",
            uxQueueMessagesWaiting(sample_queue),
            packets_processed,
            usb_errors);
        last_status_time = millis();
    }

    // Проверка памяти каждые 10 секунд
    if (millis() - last_heap_check > 10000) {
        uint32_t free_heap = ESP.getFreeHeap();
        Serial.printf("Свободной памяти: %d байт\n", free_heap);

        if (free_heap < 50000) {  // Предупреждение при малом объеме памяти
            Serial.println("ПРЕДУПРЕЖДЕНИЕ: Мало свободной памяти!");
        }
        last_heap_check = millis();
    }

    // Проверка работоспособности системы
    if (!system_running) {
        Serial.println("КРИТИЧЕСКАЯ ОШИБКА: Система остановлена");
        ESP.restart();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

void connectToWiFi() {
    Serial.print("Подключение к WiFi");
    WiFi.begin(wifi_ssid, wifi_password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi подключен: " + WiFi.localIP().toString());
    }
    else {
        Serial.println("\nWiFi не подключен (продолжаем без WiFi)");
    }
}

bool initUSBHostSafe() {
    Serial.println("Инициализация USB Host...");

    const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };

    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        Serial.printf("Ошибка usb_host_install: %s\n", esp_err_to_name(err));
        return false;
    }

    Serial.println("USB Host инициализирован");
    return true;
}

// Безопасный callback для transfer'ов
void IRAM_ATTR safeTransferCallback(usb_transfer_t* transfer) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (xSemaphoreTakeFromISR(transfer_mutex, &xHigherPriorityTaskWoken) == pdTRUE) {
        g_transfer_status = transfer->status;
        g_transfer_done = true;
        xSemaphoreGiveFromISR(transfer_mutex, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Безопасная USB задача
void safeUSBTask(void* pvParameters) {
    Serial.println("Безопасная USB задача запущена");

    // Регистрация клиента
    usb_host_client_config_t client_config;
    memset(&client_config, 0, sizeof(client_config));
    client_config.is_synchronous = true;
    client_config.max_num_event_msg = 3;

    esp_err_t err = usb_host_client_register(&client_config, &client_hdl);
    if (err != ESP_OK) {
        Serial.printf("Ошибка регистрации клиента: %s\n", esp_err_to_name(err));
        system_running = false;
        vTaskDelete(NULL);
        return;
    }

    Serial.println("USB клиент зарегистрирован");

    uint32_t last_device_check = 0;

    while (system_running) {
        // Обработка событий USB
        usb_host_lib_handle_events(10, NULL);

        if (client_hdl) {
            usb_host_client_handle_events(client_hdl, 5);
        }

        // Поиск устройств каждые 2 секунды
        if (!usb_device_connected && (millis() - last_device_check) > 2000) {
            checkForRTLSDRSafe();
            last_device_check = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    vTaskDelete(NULL);
}

void checkForRTLSDRSafe() {
    uint8_t dev_addr_list[3];  // Ограничиваем количество устройств
    int num_dev = 0;

    esp_err_t err = usb_host_device_addr_list_fill(sizeof(dev_addr_list), dev_addr_list, &num_dev);

    if (err == ESP_OK && num_dev > 0) {
        Serial.printf("Найдено %d USB устройств\n", num_dev);

        for (int i = 0; i < num_dev && i < 3; i++) {
            if (connectToRTLSDRSafe(dev_addr_list[i])) {
                break;
            }
        }
    }
}

bool connectToRTLSDRSafe(uint8_t dev_addr) {
    Serial.printf("Проверка устройства на адресе %d\n", dev_addr);

    esp_err_t err = usb_host_device_open(client_hdl, dev_addr, &device_hdl);
    if (err != ESP_OK) {
        Serial.printf("Не удалось открыть устройство: %s\n", esp_err_to_name(err));
        return false;
    }

    // Получение дескриптора с проверкой
    const usb_device_desc_t* device_desc;
    err = usb_host_get_device_descriptor(device_hdl, &device_desc);

    if (err != ESP_OK) {
        Serial.printf("Не удалось получить дескриптор: %s\n", esp_err_to_name(err));
        usb_host_device_close(client_hdl, device_hdl);
        device_hdl = NULL;
        return false;
    }

    Serial.printf("VID:PID = %04X:%04X\n", device_desc->idVendor, device_desc->idProduct);

    // Проверка RTL-SDR
    if (device_desc->idVendor == RTL_SDR_VID && device_desc->idProduct == RTL_SDR_PID) {
        Serial.println("RTL-SDR найден!");

        if (initRTLSDRSafe()) {
            usb_device_connected = true;
            Serial.println("RTL-SDR успешно инициализирован");
            return true;
        }
    }

    // Закрытие если не RTL-SDR или ошибка инициализации
    usb_host_device_close(client_hdl, device_hdl);
    device_hdl = NULL;
    return false;
}

bool initRTLSDRSafe() {
    Serial.println("Безопасная инициализация RTL-SDR...");

    // Пауза для стабилизации
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Базовая настройка без агрессивных команд
    Serial.println("1. Установка конфигурации USB...");
    if (sendUSBCommandSafe(0x00, 0x09, 1, 0, NULL, 0)) {
        Serial.println("✓ USB конфигурация установлена");
    }
    else {
        Serial.println("! Не удалось установить USB конфигурацию");
    }

    vTaskDelay(pdMS_TO_TICKS(200));

    Serial.println("2. Установка интерфейса...");
    if (sendUSBCommandSafe(0x01, 0x0B, 0, 0, NULL, 0)) {
        Serial.println("✓ Интерфейс установлен");
    }
    else {
        Serial.println("! Не удалось установить интерфейс");
    }

    vTaskDelay(pdMS_TO_TICKS(200));

    Serial.println("3. Базовая настройка RTL чипа...");

    // Простейшие команды RTL
    if (sendUSBCommandSafe(0x40, 0x10, 0x0000, 0x2D, NULL, 0)) {
        Serial.println("✓ RTL FIFO сброшен");
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    if (sendUSBCommandSafe(0x40, 0x10, 0x0002, 0x2F, NULL, 0)) {
        Serial.println("✓ RTL USB включен");
    }

    Serial.println("4. Запуск потока данных...");

    if (sendUSBCommandSafe(0x40, 0x10, 0x0000, 0x01, NULL, 0)) {
        Serial.println("✓ Поток данных запущен");

        // Запуск задачи чтения данных
        if (xTaskCreate(safeDataReadTask, "SafeRead", 2048, NULL, 2, NULL) == pdPASS) {
            Serial.println("✓ Задача чтения данных запущена");
            return true;
        }
        else {
            Serial.println("! Не удалось запустить задачу чтения");
        }
    }

    return false;
}


bool sendUSBCommandSafe(uint8_t bmRequestType, uint8_t bRequest,
    uint16_t wValue, uint16_t wIndex,
    uint8_t* data, uint16_t wLength) {

    if (!device_hdl || !transfer_mutex) {
        return false;
    }

    // Захват семафора
    if (xSemaphoreTake(transfer_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }

    usb_transfer_t* transfer = NULL;
    esp_err_t err = usb_host_transfer_alloc(wLength + sizeof(usb_setup_packet_t), 0, &transfer);

    if (err != ESP_OK || transfer == NULL) {
        xSemaphoreGive(transfer_mutex);
        return false;
    }

    // ИСПРАВЛЕНО: Правильное объявление указателя
    usb_setup_packet_t *setup = (usb_setup_packet_t*)transfer->data_buffer;
    setup->bmRequestType = bmRequestType;
    setup->bRequest = bRequest;
    setup->wValue = wValue;
    setup->wIndex = wIndex;
    setup->wLength = wLength;

    // Копирование данных для OUT запросов
    if (data && wLength > 0 && !(bmRequestType & 0x80)) {
        memcpy(transfer->data_buffer + sizeof(usb_setup_packet_t), data, wLength);
    }

    transfer->num_bytes = wLength + sizeof(usb_setup_packet_t);
    transfer->device_handle = device_hdl;
    transfer->bEndpointAddress = 0x00;
    transfer->callback = safeTransferCallback;
    transfer->timeout_ms = 3000;

    // Сброс флагов
    g_transfer_done = false;
    g_transfer_status = USB_TRANSFER_STATUS_ERROR;

    // Отправка команды
    err = usb_host_transfer_submit_control(client_hdl, transfer);

    bool success = false;

    if (err == ESP_OK) {
        // Ожидание завершения
        int timeout_count = 0;
        while (!g_transfer_done && timeout_count < 300) {
            vTaskDelay(pdMS_TO_TICKS(10));
            timeout_count++;
        }

        if (g_transfer_done && g_transfer_status == USB_TRANSFER_STATUS_COMPLETED) {
            success = true;

            // Копирование данных для IN запросов
            if (data && wLength > 0 && (bmRequestType & 0x80)) {
                memcpy(data, transfer->data_buffer + sizeof(usb_setup_packet_t), wLength);
            }
        }
    }
    else {
        usb_errors++;
    }

    // Освобождение ресурсов
    usb_host_transfer_free(transfer);
    xSemaphoreGive(transfer_mutex);

    return success;
}
bool sendSimpleUSBCommand(uint8_t bmRequestType, uint8_t bRequest,
    uint16_t wValue, uint16_t wIndex,
    uint8_t* data, uint16_t wLength) {

    if (!device_hdl) return false;

    usb_transfer_t* transfer;
    esp_err_t err = usb_host_transfer_alloc(wLength + sizeof(usb_setup_packet_t), 0, &transfer);
    if (err != ESP_OK) return false;

    // ИСПРАВЛЕНО: Правильное объявление указателя
    usb_setup_packet_t *setup = (usb_setup_packet_t*)transfer->data_buffer;
    setup->bmRequestType = bmRequestType;
    setup->bRequest = bRequest;
    setup->wValue = wValue;
    setup->wIndex = wIndex;
    setup->wLength = wLength;

    // Копирование данных если нужно
    if (data && wLength > 0 && !(bmRequestType & 0x80)) {
        memcpy(transfer->data_buffer + sizeof(usb_setup_packet_t), data, wLength);
    }

    transfer->num_bytes = wLength + sizeof(usb_setup_packet_t);
    transfer->device_handle = device_hdl;
    transfer->bEndpointAddress = 0x00;
    transfer->callback = safeTransferCallback;
    transfer->timeout_ms = 1000;

    // Сброс флагов
    g_transfer_done = false;
    g_transfer_status = USB_TRANSFER_STATUS_ERROR;

    // Отправка
    err = usb_host_transfer_submit_control(client_hdl, transfer);
    if (err != ESP_OK) {
        usb_host_transfer_free(transfer);
        return false;
    }

    // Ожидание с простым таймаутом
    int timeout = 0;
    while (!g_transfer_done && timeout < 100) {
        vTaskDelay(pdMS_TO_TICKS(10));
        timeout++;
    }

    bool success = (g_transfer_done && g_transfer_status == USB_TRANSFER_STATUS_COMPLETED);

    // Копирование данных для IN запросов
    if (success && data && wLength > 0 && (bmRequestType & 0x80)) {
        memcpy(data, transfer->data_buffer + sizeof(usb_setup_packet_t), wLength);
    }

    usb_host_transfer_free(transfer);
    return success;
}



// Дополнительные проверки для безопасности
bool validateUSBTransfer(usb_transfer_t* transfer) {
    if (transfer == NULL) {
        Serial.println("ОШИБКА: transfer == NULL");
        return false;
    }

    if (transfer->data_buffer == NULL) {
        Serial.println("ОШИБКА: data_buffer == NULL");
        return false;
    }

    if (transfer->device_handle == NULL) {
        Serial.println("ОШИБКА: device_handle == NULL");
        return false;
    }

    return true;
}

// Улучшенная функция с дополнительными проверками
bool sendUSBCommandWithValidation(uint8_t bmRequestType, uint8_t bRequest,
    uint16_t wValue, uint16_t wIndex,
    uint8_t* data, uint16_t wLength) {

    // Проверка входных параметров
    if (!device_hdl || !transfer_mutex) {
        Serial.println("ОШИБКА: Устройство не инициализировано");
        return false;
    }

    if (wLength > 1024) {  // Ограничение размера данных
        Serial.println("ОШИБКА: Слишком большой размер данных");
        return false;
    }

    // Захват семафора с проверкой
    if (xSemaphoreTake(transfer_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        Serial.println("ОШИБКА: Не удалось захватить семафор");
        return false;
    }

    usb_transfer_t* transfer = NULL;
    size_t transfer_size = wLength + sizeof(usb_setup_packet_t);

    esp_err_t err = usb_host_transfer_alloc(transfer_size, 0, &transfer);

    if (err != ESP_OK || transfer == NULL) {
        Serial.printf("ОШИБКА: Выделение transfer неудачно: %s\n", esp_err_to_name(err));
        xSemaphoreGive(transfer_mutex);
        return false;
    }

    // Проверка валидности transfer
    if (!validateUSBTransfer(transfer)) {
        usb_host_transfer_free(transfer);
        xSemaphoreGive(transfer_mutex);
        return false;
    }

    // Безопасная настройка setup пакета
    usb_setup_packet_t *setup = (usb_setup_packet_t*)transfer->data_buffer;

    // Обнуляем структуру для безопасности
    memset(setup, 0, sizeof(usb_setup_packet_t));

    setup->bmRequestType = bmRequestType;
    setup->bRequest = bRequest;
    setup->wValue = wValue;
    setup->wIndex = wIndex;
    setup->wLength = wLength;

    // Копирование данных для OUT запросов с проверкой
    if (data && wLength > 0 && !(bmRequestType & 0x80)) {
        if (transfer_size >= sizeof(usb_setup_packet_t) + wLength) {
            memcpy(transfer->data_buffer + sizeof(usb_setup_packet_t), data, wLength);
        }
        else {
            Serial.println("ОШИБКА: Недостаточно места в буфере");
            usb_host_transfer_free(transfer);
            xSemaphoreGive(transfer_mutex);
            return false;
        }
    }

    // Настройка transfer
    transfer->num_bytes = transfer_size;
    transfer->device_handle = device_hdl;
    transfer->bEndpointAddress = 0x00;
    transfer->callback = safeTransferCallback;
    transfer->timeout_ms = 3000;

    // Сброс флагов
    g_transfer_done = false;
    g_transfer_status = USB_TRANSFER_STATUS_ERROR;

    // Отправка команды
    err = usb_host_transfer_submit_control(client_hdl, transfer);

    bool success = false;

    if (err == ESP_OK) {
        // Ожидание завершения с проверкой
        int timeout_count = 0;
        const int max_timeout = 300;  // 3 секунды

        while (!g_transfer_done && timeout_count < max_timeout) {
            vTaskDelay(pdMS_TO_TICKS(10));
            timeout_count++;

            // Проверка системы каждые 100мс
            if (timeout_count % 10 == 0 && !system_running) {
                Serial.println("Система остановлена, прерывание USB операции");
                break;
            }
        }

        if (g_transfer_done && g_transfer_status == USB_TRANSFER_STATUS_COMPLETED) {
            success = true;

            // Безопасное копирование данных для IN запросов
            if (data && wLength > 0 && (bmRequestType & 0x80)) {
                if (transfer->actual_num_bytes >= sizeof(usb_setup_packet_t)) {
                    size_t data_size = transfer->actual_num_bytes - sizeof(usb_setup_packet_t);
                    size_t copy_size = (data_size < wLength) ? data_size : wLength;
                    memcpy(data, transfer->data_buffer + sizeof(usb_setup_packet_t), copy_size);
                }
            }
        }
        else if (!g_transfer_done) {
            Serial.printf("TIMEOUT: USB команда 0x%02X превысила время ожидания\n", bRequest);
            usb_errors++;
        }
        else {
            Serial.printf("ОШИБКА: USB команда 0x%02X, статус: %d\n", bRequest, g_transfer_status);
            usb_errors++;
        }
    }
    else {
        Serial.printf("ОШИБКА: Отправка USB команды 0x%02X: %s\n", bRequest, esp_err_to_name(err));
        usb_errors++;
    }

    // Освобождение ресурсов
    usb_host_transfer_free(transfer);
    xSemaphoreGive(transfer_mutex);

    return success;
}

//
//Основные исправления :
//
//Правильное приведение типов - (usb_setup_packet_t*)вместо(usb_setup_packet_t)
//Дополнительные проверки валидности transfer'ов
//Обнуление памяти перед использованием structure setup
//Проверки размеров буферов перед копированием
//Улучшенная обработка ошибок с детальным логированием
//Защита от переполнения буферов
//
//Теперь код должен работать без ошибок компиляции и runtime ошибок.

void safeDataReadTask(void* pvParameters) {
    Serial.println("Безопасная задача чтения данных запущена");

    uint32_t successful_reads = 0;
    uint32_t failed_reads = 0;

    while (usb_device_connected && system_running) {
        if (readBulkDataSafe(0x81, sample_buffer, sizeof(sample_buffer))) {
            // Копирование в буфер очереди
            memcpy(queue_buffer, sample_buffer, sizeof(queue_buffer));

            if (xQueueSend(sample_queue, queue_buffer, pdMS_TO_TICKS(10)) == pdPASS) {
                successful_reads++;
                failed_reads = 0;
            }
        }
        else {
            failed_reads++;
            if (failed_reads > 100) {
                Serial.println("ПРЕДУПРЕЖДЕНИЕ: Множественные ошибки чтения USB");
                failed_reads = 0;
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Serial.printf("Задача чтения завершена. Успешно: %d, Ошибок: %d\n",
        successful_reads, failed_reads);
    vTaskDelete(NULL);
}

bool readBulkDataSafe(uint8_t endpoint, uint8_t* buffer, size_t length) {
    if (!device_hdl || !transfer_mutex) {
        return false;
    }

    if (xSemaphoreTake(transfer_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }

    usb_transfer_t* transfer = NULL;
    esp_err_t err = usb_host_transfer_alloc(length, 0, &transfer);

    if (err != ESP_OK) {
        xSemaphoreGive(transfer_mutex);
        return false;
    }

    transfer->device_handle = device_hdl;
    transfer->bEndpointAddress = endpoint;
    transfer->callback = safeTransferCallback;
    transfer->timeout_ms = 100;
    transfer->num_bytes = length;

    g_transfer_done = false;
    g_transfer_status = USB_TRANSFER_STATUS_ERROR;

    err = usb_host_transfer_submit(transfer);

    bool success = false;

    if (err == ESP_OK) {
        int timeout = 0;
        while (!g_transfer_done && timeout < 20) {  // 200ms таймаут
            vTaskDelay(pdMS_TO_TICKS(10));
            timeout++;
        }

        if (g_transfer_done && g_transfer_status == USB_TRANSFER_STATUS_COMPLETED) {
            memcpy(buffer, transfer->data_buffer,
                transfer->actual_num_bytes < length ? transfer->actual_num_bytes : length);
            success = true;
        }
    }

    usb_host_transfer_free(transfer);
    xSemaphoreGive(transfer_mutex);

    return success;
}

void safeADSBTask(void* pvParameters) {
    Serial.println("Безопасная ADS-B задача запущена");

    uint8_t processing_buffer[1024];

    while (system_running) {
        if (xQueueReceive(sample_queue, processing_buffer, pdMS_TO_TICKS(1000)) == pdPASS) {

            // Простая обработка - подсчет активности сигнала
            if (processSignalActivity(processing_buffer, sizeof(processing_buffer))) {
                packets_processed++;

                if (packets_processed % 50 == 0) {
                    outputActivityReport();
                }
            }
        }
    }

    vTaskDelete(NULL);
}

bool processSignalActivity(uint8_t* data, size_t length) {
    int high_activity = 0;

    for (size_t i = 0; i < length - 1; i += 2) {
        int16_t i_sample = (int16_t)data[i] - 128;
        int16_t q_sample = (int16_t)data[i + 1] - 128;

        int16_t magnitude = abs(i_sample) + abs(q_sample);

        if (magnitude > 100) {
            high_activity++;
        }
    }

    // Если высокая активность - возможно ADS-B сигнал
    return (high_activity > 20);
}

void outputActivityReport() {
    String report = "ACTIVITY,";
    report += String(millis()) + ",";
    report += String(packets_processed) + ",";
    report += String(ESP.getFreeHeap()) + ",";
    report += String(uxQueueMessagesWaiting(sample_queue));

    UART1_Port.println(report);
    Serial.println("Activity: " + report);
}

/*
Основные улучшения :

Статические буферы - избежим проблем с динамической памятью
Mutex для синхронизации transfer'ов
IRAM_ATTR для callback - размещение в быстрой памяти
Проверка системы на каждом этапе
Контроль памяти и автоматический перезапуск при критических ошибках
Увеличенные таймауты для стабильности
Упрощенная обработка без сложных алгоритмов

Этот код должен быть намного стабильнее и избежать panic ошибок.
*/