#include "flash_utils.h"

#include "adsbee.h"
#include "core1.h"
#include "hardware/sync.h"

// Initialize static variables.
uint32_t FlashUtils::stored_interrupts_ = 0;

void FlashUtils::FlashSafe() 
{
    stored_interrupts_ = save_and_disable_interrupts();
    StopCore1();               // Reset core 1 to prevent it from executing while we erase flash.
    adsbee.DisableWatchdog();  // Flash erase can take a while, prevent watchdog from rebooting us.
}
void FlashUtils::FlashUnsafe() 
{
   //!! adsbee.EnableWatchdog();
    StartCore1();
    restore_interrupts(stored_interrupts_);
}