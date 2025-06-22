
#include <mutex>

#include "adsbee.h"
#include "comms.h"
#include "core1.h"  // Functions for runningon core1.
#include "eeprom.h"
#include "esp32_flasher.h"
#include "firmware_update.h"  // For figuring out which flash partition we're in.
#include "hal.h"
//#include "hardware_unit_tests.h"  // For testing only!
#include "packet_decoder.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "spi_coprocessor.h"
#include "transponder_packet.h"
#include "unit_conversions.h"

// #define DEBUG_DISABLE_ESP32_FLASH  // Uncomment this to stop the RP2040 from flashing the ESP32.

// For testing only
#include "hardware/gpio.h"


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
