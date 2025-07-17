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
    decoder.UpdateLogLoop(); // Вывод сырых пакетов (работает, но контрольная сумма не работает.).
    comms_manager.Update();  // Вывод расшифрованных пакетов (не работает).
    adsbee.Update();
  
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
