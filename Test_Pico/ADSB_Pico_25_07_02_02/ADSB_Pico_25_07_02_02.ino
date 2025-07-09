#include <mutex>
#include "comms.h"
#include "packet_decoder.h"
#include "transponder_packet.h"
#include "unit_conversions.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hal.h"
#include "core1.h"  // Functions for runningon core1.
#include "unit_conversions.h"
#include "eepromPico.h"
#include "bsp.h"
#include "awb_utils.h"
#include "decode_utils.h"
#include "object_dictionary.h"
#include "aircraft_dictionary.h"
#include "nasa_cpr.h"
#include "adsbee.h"
#include "beast_tables.h"
#include "flash_utils.h"
#include "firmware_update.h"

const uint16_t kStatusLEDBootupBlinkPeriodMs = 200; 
const uint32_t kESP32BootupTimeoutMs         = 10000;
const uint32_t kESP32BootupCommsRetryMs      = 500;

// Здесь можно переопределить параметры конфигурации по умолчанию.
EEPROMPICO eeprom_pico = EEPROMPICO({});
// BSP настраивается по-разному, если подключена или нет EEPROM. Попытаемся инициализировать EEPROM, чтобы выяснить,
// какую конфигурацию платы следует загрузить (настройки во флэш-памяти или настройки в EEPROM).
BSP bsp = BSP(eeprom_pico.Init());
 
ADSBee adsbee = ADSBee({});
SettingsManager settings_manager;
ObjectDictionary object_dictionary;
PacketDecoder decoder = PacketDecoder({ .enable_1090_error_correction = true });

const int ledPin = 15;//!! LED_BUILTIN;     // the number of the LED pin
int ledState = LOW;                         // ledState used to set the LED
unsigned long previousMillis = 0;           // will store last time LED was updated
const long interval = 1000;                 // interval at which to blink (milliseconds)



void setup() 
{
    bi_decl(bi_program_description("ADSBee 1090 ADSB Receiver"));

    comms_manager.Init();  //Сначала настроим вывод в КОМ порт
    sleep_ms(1000);
    adsbee.Init();
    comms_manager.console_printf("ADSBee 1090 Version %d.%d.%d\r\n",
        object_dictionary.kFirmwareVersionMajor, object_dictionary.kFirmwareVersionMinor,
        object_dictionary.kFirmwareVersionPatch);

    comms_manager.console_printf("Software ");
    String ver_soft = __FILE__;
    int val_srt = ver_soft.lastIndexOf('\\');
    ver_soft.remove(0, val_srt + 1);
    val_srt = ver_soft.lastIndexOf('.');
    ver_soft.remove(val_srt);
    comms_manager.console_printf(ver_soft.c_str());
 
    settings_manager.Load();

    for (uint16_t i = 0; i < 5; i++)
    {
        adsbee.SetStatusLED(true);
        sleep_ms(kStatusLEDBootupBlinkPeriodMs / 2);
        adsbee.SetStatusLED(false);
        sleep_ms(kStatusLEDBootupBlinkPeriodMs / 2);
    }

    comms_manager.console_printf("\r\nSetup End\r\n");
}

void loop() 
{

   // decoder.UpdateDecoderLoop();
    decoder.UpdateLogLoop();
    comms_manager.Update();
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
