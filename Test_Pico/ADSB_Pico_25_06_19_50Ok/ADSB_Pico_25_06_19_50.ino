#include <mutex>
#include "packet_decoder.h"
#include "transponder_packet.h"
#include "unit_conversions.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hal.h"
#include "core1.h"  // Functions for runningon core1.
#include "comms.h"
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
//!!#include "beast_utils.h"
#include "flash_utils.h"


const uint16_t kStatusLEDBootupBlinkPeriodMs = 200;
const uint32_t kESP32BootupTimeoutMs = 10000;
const uint32_t kESP32BootupCommsRetryMs = 500;

// Override default config params here.
EEPROMPICO eeprom_pico = EEPROMPICO({});
// BSP настраивается по-разному, если подключена или нет EEPROM. Попытаемся инициализировать EEPROM, чтобы выяснить,
// какую конфигурацию платы следует загрузить (настройки во флэш-памяти или настройки в EEPROM).
BSP bsp = BSP(eeprom_pico.Init());
 
ADSBee adsbee = ADSBee({});
SettingsManager settings_manager;
ObjectDictionary object_dictionary;
PacketDecoder decoder = PacketDecoder({ .enable_1090_error_correction = true });

const int ledPin =  LED_BUILTIN;     // the number of the LED pin
int ledState = LOW;                  // ledState used to set the LED
unsigned long previousMillis = 0;    // will store last time LED was updated
const long interval = 500;           // interval at which to blink (milliseconds)

void setup() 
{
    // pinMode(ledPin, OUTPUT);
    bi_decl(bi_program_description("ADSBee 1090 ADSB Receiver"));

    adsbee.Init();

    //Serial2.begin(115200);
    //Serial2.println("ASCII Table ~ Character Map");

    comms_manager.Init();
    comms_manager.console_printf("ADSBee 1090\r\nSoftware Version %d.%d.%d\r\n",
        object_dictionary.kFirmwareVersionMajor, object_dictionary.kFirmwareVersionMinor,
        object_dictionary.kFirmwareVersionPatch);

    //!!settings_manager.Load();

    for (uint16_t i = 0; i < 5; i++)
    {
        adsbee.SetStatusLED(true);
       // delay(100);
        sleep_ms(kStatusLEDBootupBlinkPeriodMs / 2);
        adsbee.SetStatusLED(false);
        sleep_ms(kStatusLEDBootupBlinkPeriodMs / 2);
        // delay(100);
    }

}

void loop() 
{





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
