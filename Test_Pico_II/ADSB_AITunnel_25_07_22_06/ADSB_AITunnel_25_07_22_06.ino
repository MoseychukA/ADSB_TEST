#include <mutex>
#include "pico/multicore.h"
#include "unit_conversions.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "comms.h"
#include "transponder_packet.h"
#include "packet_decoder.h"
#include "hal.h"
#include "unit_conversions.h"
#include "bsp.h"
#include "awb_utils.h"
#include "decode_utils.h"
#include "object_dictionary.h"
#include "aircraft_dictionary.h"
#include "nasa_cpr.h"
#include "adsbee.h"
#include "beast_tables.h"
#include "data_structures.h"


const uint16_t kStatusLEDBootupBlinkPeriodMs = 200; 
const uint32_t kESP32BootupTimeoutMs         = 10000;
const uint32_t kESP32BootupCommsRetryMs      = 500;
 
BSP bsp = BSP({});
ADSBee adsbee = ADSBee({});
SettingsManager settings_manager;
ObjectDictionary object_dictionary;
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

// Структура пакета
struct ADSBPacket1 {
    uint8_t data[14];  // Максимум 112 бит = 14 байт
    uint8_t length;    // Длина в байтах
    uint32_t timestamp;
    uint8_t channel;   // 1 или 2
};


Decoded1090Packet decoded_1090_packet_out_queue_buffer_[100U];
// Выходные очереди.
PFBQueue<Decoded1090Packet> decoded_1090_packet_out_queue = PFBQueue<Decoded1090Packet>({ .buf_len_num_elements = 100U, .buffer = decoded_1090_packet_out_queue_buffer_ });




void setup() 
{
    bi_decl(bi_program_description("ADSBee 1090 ADSB Receiver"));

   //!! Serial.begin(115200);
   //!! sleep_ms(3000);
    comms_manager.Init();  //Сначала настроим вывод в КОМ порт
    sleep_ms(500);
    adsbee.Init();
    comms_manager.console_printf("ADSBee 1090 Version %d.%d.%d\r\n",
        object_dictionary.kFirmwareVersionMajor, object_dictionary.kFirmwareVersionMinor,
        object_dictionary.kFirmwareVersionPatch);

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

    for (uint16_t i = 0; i < 5; i++)
    {
        adsbee.SetStatusLED(true);
        sleep_ms(kStatusLEDBootupBlinkPeriodMs / 2);
        adsbee.SetStatusLED(false);
        sleep_ms(kStatusLEDBootupBlinkPeriodMs / 2);
    }
   //!! Serial.println("Setup End\r\n");
    comms_manager.console_printf("\r\nSetup End\r\n");
}


void setup1()
{
    pinMode(ledPin1, OUTPUT);
}



void loop() 
{
    decoder.UpdateLogLoop(); // Вывод сырых пакетов ().
   // comms_manager.Update();  // Вывод расшифрованных пакетов (не работает).
   // adsbee.Update();
 

  /*  Raw1090Packet raw_packet1;
    Decoded1090Packet decoded_packet1 = Decoded1090Packet(raw_packet1);*/


    Raw1090Packet raw_packet;
    Decoded1090Packet decoded_packet;

    while (decoder.decoded_1090_packet_out_queue.Pop(decoded_packet) /*raw_1090_packet_queue.Pop(raw_packet)*/)
    {
        comms_manager.console_printf("ADSBee::Update %d\r\n ", raw_packet.buffer_len_bits);
        //  comms_manager.console_printf("!!!raw_packet.buffer_len_bits %d \r\n", raw_packet.buffer_len_bits);
        if (raw_packet.buffer_len_bits == Raw1090Packet::kExtendedSquitterPacketLenBits)  // 112;
        {
            comms_manager.console_printf("ADSBee::Update New message: 0x%08x|%08x|%08x|%04x SRC=%d SIGS=%ddBm SIGQ=%ddB MLAT=%u\r\n",
                         raw_packet.buffer[0], raw_packet.buffer[1], raw_packet.buffer[2],
                         (raw_packet.buffer[3]) >> (4 * kBitsPerNibble), raw_packet.source, raw_packet.sigs_dbm, 
                         raw_packet.sigq_db, raw_packet.mlat_48mhz_64bit_counts);

            //CONSOLE_INFO("ADSBee::Update", "New message: 0x%08x|%08x|%08x|%04x SRC=%d SIGS=%ddBm SIGQ=%ddB MLAT=%u",
            //    raw_packet.buffer[0], raw_packet.buffer[1], raw_packet.buffer[2],
            //    (raw_packet.buffer[3]) >> (4 * kBitsPerNibble), raw_packet.source, raw_packet.sigs_dbm,
            //    raw_packet.sigq_db, raw_packet.mlat_48mhz_64bit_counts);

        }
        else
        {
            //comms_manager.console_printf("ADSBee::Update New message: 0x%08x|%06x SRC=%d SIGS=%ddBm SIGQ=%ddB MLAT=%u\r\n",
            //             raw_packet.buffer[0], (raw_packet.buffer[1]) >> (2 * kBitsPerNibble), raw_packet.source,
            //             raw_packet.sigs_dbm, raw_packet.sigq_db, raw_packet.mlat_48mhz_64bit_counts);

            //CONSOLE_INFO("ADSBee::Update", "New message: 0x%08x|%06x SRC=%d SIGS=%ddBm SIGQ=%ddB MLAT=%u",
            //    raw_packet.buffer[0], (raw_packet.buffer[1]) >> (2 * kBitsPerNibble), raw_packet.source,
            //    raw_packet.sigs_dbm, raw_packet.sigq_db, raw_packet.mlat_48mhz_64bit_counts);
        }

        //Decoded1090Packet decoded_packet = Decoded1090Packet(raw_packet);

        ////while (queue_mutex) delay(1);
        ////queue_mutex = true;
        //comms_manager.console_printf("ADSBee::Update\tdf=%d\t icao_address=0x%06x\r\n", decoded_packet.GetDownlinkFormat(), decoded_packet.GetICAOAddress());
        ////queue_mutex = false;

        //if (aircraft_dictionary.IngestDecoded1090Packet(decoded_packet)) // Принять декодированный пакет 1090
        //{
        //    // Пакет был использован для обновления словаря или был молча проигнорирован (но предположительно действителен).
        //    FlashStatusLED();  // Мигнуть светодиодом
        //    // ПРИМЕЧАНИЕ: Отправка в очередь отчетов здесь означает, что мы будем сообщать только о проверенных пакетах!
        //    comms_manager.transponder_packet_reporting_queue.Push(decoded_packet);
        //    // comms_manager.console_printf("ADSBee::Update \taircraft %d \r\n", aircraft_dictionary.GetNumAircraft());
        //}
        //!! comms_manager.transponder_packet_reporting_queue.Push(decoded_packet);
    }









   // ADSBPacket packet = decoder.decoded_1090_packet_out_queue.Pop(decoded_packet);

 /*       if (decoded_packet.IsValid())
        {

            comms_manager.console_printf("IsValid\r\n");

        }*/
    //  if (decoded_1090_packet_out_queue.Pop(decoded_packet))

    //if (decoded_1090_packet_out_queue.Pop(decoded_packet1))
    //{
    //    if (raw_packet1.buffer_len_bits == Raw1090Packet::kExtendedSquitterPacketLenBits)  // 112;
    //    {

    //        comms_manager.console_printf("0x%08x%08x%08x%04x\r\n",
    //            raw_packet1.buffer[0], raw_packet1.buffer[1], raw_packet1.buffer[2],
    //            (raw_packet1.buffer[3]) >> (4 * kBitsPerNibble));

    //        /*
    //           uint32_t buffer[kMaxPacketLenWords32] = {0};
    //           uint16_t buffer_len_bits = 0;
    //           int8_t source = -1;                    // Источник пакета ADS-B (номер конечного автомата PIO).
    //           int16_t sigs_dbm = INT16_MIN;          // Уровень сигнала, в дБм.
    //           int16_t sigq_db = INT16_MIN;           // Качество сигнала (дБ выше уровня шума), в дБ.
    //           uint64_t mlat_48mhz_64bit_counts = 0;  // Счетчик MLAT высокого разрешения.
    //        */

    //    }
    //    else
    //    {
    //        //comms_manager.console_printf("+++Update New message: 0x%08x|%06x SRC=%d SIGS=%ddBm SIGQ=%ddB MLAT=%u\r\n",
    //        //             raw_packet.buffer[0], (raw_packet.buffer[1]) >> (2 * kBitsPerNibble), raw_packet.source,
    //        //             raw_packet.sigs_dbm, raw_packet.sigq_db, raw_packet.mlat_48mhz_64bit_counts);

    //        //CONSOLE_INFO("ADSBee::Update", "New message: 0x%08x|%06x SRC=%d SIGS=%ddBm SIGQ=%ddB MLAT=%u",
    //        //    raw_packet.buffer[0], (raw_packet.buffer[1]) >> (2 * kBitsPerNibble), raw_packet.source,
    //        //    raw_packet.sigs_dbm, raw_packet.sigq_db, raw_packet.mlat_48mhz_64bit_counts);
    //    }

    //}



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
   // digitalWrite(ledPin, ledState);
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
