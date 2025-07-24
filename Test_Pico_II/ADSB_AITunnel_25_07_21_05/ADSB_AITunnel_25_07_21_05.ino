#include <mutex>
#include "pico/multicore.h"
#include "unit_conversions.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "comms.h"
#include "transponder_packet.h"
#include "packet_decoder.h"
#include "unit_conversions.h"
#include "bsp.h"
#include "decode_utils.h"
#include "object_dictionary.h"
#include "aircraft_dictionary.h"
#include "adsbee.h"
#include "packet_decoder.h"
#include "data_structures.h"  // For PFBQueue.

#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define AGC_INPUT_PIN 26
#define AGC_OUTPUT_PIN 9
#define AGC_CONTROL_PIN 27
#define CORE0_LED_PIN 15
#define CORE1_LED_PIN 25

 
BSP bsp = BSP({});
ADSBee adsbee = ADSBee({});
SettingsManager settings_manager;
PacketDecoder decoder = PacketDecoder({ .enable_1090_error_correction = true });

/*Настройки только для теста*/
const int ledPin = 15;                       // the number of the LED pin
int ledState = LOW;                          // ledState used to set the LED
unsigned long previousMillis = 0;            // will store last time LED was updated
const long interval = 1000;                  // interval at which to blink (milliseconds)

const int ledPin1 = 25;
int ledState1 = LOW;
unsigned long previousMillis1 = 0;
const long interval1 = 300;


// AGC переменные
volatile int agc_level = 512;
volatile uint32_t last_agc_update = 0;


// Мьютексы для защиты очередей
volatile bool queue_mutex = false;
volatile bool processed_mutex = false;

// Структура пакета
struct ADSBPacket1 {
    uint8_t data[14];  // Максимум 112 бит = 14 байт
    uint8_t length;    // Длина в байтах
    uint32_t timestamp;
    uint8_t channel;   // 1 или 2
};





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
        }
        else {
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
            }
            else {
                break;
            }
        }
    }
    else {
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

Decoded1090Packet decoded_1090_packet_out_queue_buffer_[100U];
PFBQueue<Decoded1090Packet> decoded_1090_packet_out_queue = PFBQueue<Decoded1090Packet>({ .buf_len_num_elements = 100U, .buffer = decoded_1090_packet_out_queue_buffer_ });





void setup() 
{
    bi_decl(bi_program_description("ADSBee 1090 ADSB Receiver"));

   //!! Serial.begin(115200);
   //!! sleep_ms(3000);
    comms_manager.Init();  //Сначала настроим вывод в КОМ порт
    sleep_ms(500);
    adsbee.Init();
    comms_manager.console_printf("Software ");
   //!! Serial.print("Software ");
    String ver_soft = __FILE__;
    int val_srt = ver_soft.lastIndexOf('\\');
    ver_soft.remove(0, val_srt + 1);
    val_srt = ver_soft.lastIndexOf('.');
    ver_soft.remove(val_srt);
    //!!Serial.println(ver_soft);
    comms_manager.console_printf(ver_soft.c_str());
 
    settings_manager.Load();    // Загрузить настройки по умолчанию. Нужно еще поработать с этой функцией.

   //!! Serial.println("Setup End\r\n");
    comms_manager.console_printf("\r\nSetup End\r\n");
}






void setup1()
{
    pinMode(ledPin1, OUTPUT);
}



void loop() 
{
   // decoder.UpdateLogLoop(); // Вывод сырых пакетов (работает, но контрольная сумма не работает.).
    adsbee.Update();
  



    uint16_t num_messages = decoded_1090_packet_out_queue.Length(); //
           // Обработка пакетов из очереди
    if (!num_messages)
    {
        while (queue_mutex) delay(1);
        queue_mutex = true;

        Decoded1090Packet packet;
       
        decoded_1090_packet_out_queue.Pop(packet);


        //!! ADSBPacket packet = packet_queue.front();
        

        queue_mutex = false;

        // Отладочный вывод сырых данных
        Serial2.print("Raw packet (Ch");
        //Serial2.print(packet.channel);
        Serial2.print(", ");
        //Serial2.print(packet.rx_buffer_len);
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


//---------------------------------------------------------------------

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) 
    {
    previousMillis = currentMillis;
    if (ledState == LOW) 
    {
        ledState = HIGH;
    }
    else 
    {
        ledState = LOW;
    }
    digitalWrite(ledPin, ledState);
    }
}


void loop1()
{
   decoder.UpdateDecoderLoop();   //PacketDecoder Контроль CRC пакетов (не работает)

    unsigned long currentMillis1 = millis();
    if (currentMillis1 - previousMillis1 >= interval1)
    { 
        previousMillis1 = currentMillis1;
        if (ledState1 == LOW)
        {
            ledState1 = HIGH;
        }
        else
        {
            ledState1 = LOW;
        }
        digitalWrite(ledPin1, ledState1);
    }


}
