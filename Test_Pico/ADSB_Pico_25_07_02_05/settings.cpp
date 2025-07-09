#include "settings.h"

#include "adsbee.h"
#include "comms.h"
//!!#include "core1.h"
#include "eepromPico.h"
#include "hardware/flash.h"
#include "flash_utils.h"
#include "bsp.h"
#include "firmware_update.h"


/* Original flash length: 16384k Bytes.
   FLASH MAP
       0x10000000	(176k)	FLASH_BL	Bootloader
       0x1002C000	(4k)	FLASH_HDR0	    Application 0 Header
       0x1002D000	(8096k)	FLASH_APP0	    Application 0 Data
       0x10815000	(4k)	FLASH_HDR1	    Application 1 Header
       0x10816000	(8096k)	FLASH_APP1	    Application 1 Data
       0x10FFE000	(8k)	FLASH_SETTINGS	Settings
*/
// Использовать половину FLASH_SETTINGS для настроек, половину для информации об устройстве. 
// За один раз мы можем стереть только полный сектор (4096 байт),
//поэтому лучше не трогать сектор с информацией об устройстве, чтобы не повредить его.
static_assert(sizeof(SettingsManager::Settings) < FlashUtils::kFlashSectorSizeBytes);
static_assert(sizeof(SettingsManager::DeviceInfo) < FlashUtils::kFlashSectorSizeBytes);
const uint32_t kDeviceInfoMaxSizeBytes = sizeof(SettingsManager::DeviceInfo);

// Информация об устройстве хранится с небольшим смещением от конца флэш-памяти, чтобы избежать некоторых адресов, по которым невозможно выполнить запись.
const uint32_t kFlashSettingsStartAddr = 0x10FFE000;
const uint32_t kFlashEndAddr = 0x10FFFD00;          // Здесь есть некоторые жестко запрограммированные вещи, которые вызывают проблемы.
static_assert(kFlashEndAddr % kBytesPerWord == 0);  // Необходимо выровнять по словам.
const uint32_t kFlashDeviceInfoStartAddr = kFlashSettingsStartAddr + FlashUtils::kFlashSectorSizeBytes;
// Необходимо выровнять по слову, чтобы разрешить прямое приведение указателя без memcpy.
static_assert(kFlashDeviceInfoStartAddr % kBytesPerWord == 0);

// Информация об устройстве хранится в самом конце EEPROM для обратной совместимости.
const uint32_t kEEPROMSizeBytes = 8e3;  // 8000 Bytes for backwards compatibility.
const uint32_t kEEPROMDeviceInfoOffset = kEEPROMSizeBytes - kDeviceInfoMaxSizeBytes;

bool SettingsManager::Load() 
{
    //if (bsp.has_eeprom) 
    //{
    //    // Загрузка настроек из внешней EEPROM.
    //    //if (!eeprom_Pico.Load(settings)) 
    //    //{
    //    //    comms_manager.console_printf("settings.cpp::Load Failed load settings from EEPROM.");
    //    //    return false;
    //    //}
    //}
    //else
    //{
    //    // Загрузить настройки из флэш-памяти.
    //    comms_manager.console_printf("\r\nload settings from EEPROM.");
    //    FlashUtils::FlashSafe();
    //    settings = *(Settings *)kFlashSettingsStartAddr;
    //    FlashUtils::FlashUnsafe();
    //}

    //// Сброс к настройкам по умолчанию при загрузке с чистого EEPROM.
    //if (settings.settings_version != kSettingsVersion) 
    //{
    //    ResetToDefaults();
    //    //if (bsp.has_eeprom && !eeprom_Pico.Save(settings)) 
    //    //{
    //    ////    CONSOLE_ERROR("settings.cc::Load", "Failed to save default settings.");
    //    ////    return false;
    //    //}
    //    //else 
    //    //{
    //        FlashUtils::FlashSafe();
    //        //!!flash_range_erase(FirmwareUpdateManager::FlashAddrToOffset(kFlashSettingsStartAddr), FlashUtils::kFlashSectorSizeBytes);
    //        //!!flash_range_program(FirmwareUpdateManager::FlashAddrToOffset(kFlashSettingsStartAddr), (uint8_t *)&settings, sizeof(settings));
    //        FlashUtils::FlashUnsafe();
    //    //}
    //}

    Apply();

    return true;
}

bool SettingsManager::Save()
{
    settings.receiver_enabled = adsbee.Receiver1090IsEnabled();
    settings.tl_mv = adsbee.GetTLMilliVolts();
    settings.bias_tee_enabled = adsbee.BiasTeeIsEnabled();
    settings.watchdog_timeout_sec = adsbee.GetWatchdogTimeoutSec();

    // Save log level.
    settings.log_level = comms_manager.log_level;

    // Save reporting protocols.
    comms_manager.GetReportingProtocol(SerialInterface::kCommsUART, settings.reporting_protocols[SerialInterface::kCommsUART]);
    comms_manager.GetReportingProtocol(SerialInterface::kConsole, settings.reporting_protocols[SerialInterface::kConsole]);

    // Save baud rates.
    comms_manager.GetBaudrate(SerialInterface::kCommsUART, settings.comms_uart_baud_rate);
    comms_manager.GetBaudrate(SerialInterface::kGNSSUART, settings.gnss_uart_baud_rate);

    //if (bsp.has_eeprom) 
    //{
    //    return eeprom_Pico.Save(settings);
    //}
    //else 
    //{
    //    FlashUtils::FlashSafe();
    //    //flash_range_erase(FirmwareUpdateManager::FlashAddrToOffset(kFlashSettingsStartAddr),
    //    //                  FlashUtils::kFlashSectorSizeBytes);
    //    //flash_range_program(FirmwareUpdateManager::FlashAddrToOffset(kFlashSettingsStartAddr), (uint8_t *)&settings,
    //    //                    sizeof(settings));
    //    FlashUtils::FlashUnsafe();
    //    return true;
    //}

    return true;//!! Нужно удалить
}

void SettingsManager::ResetToDefaults() 
{
    Settings default_settings;
    settings = default_settings;
    Apply();
}

//!!bool SettingsManager::SetDeviceInfo(const DeviceInfo &device_info) 
//{
//    if (bsp.has_eeprom) 
//    {
//        // Device Info is stored on external EEPROM.
//        if (eeprom_Pico.RequiresInit()) return false;
//        return eeprom_Pico.Save(device_info, kEEPROMDeviceInfoOffset);
//    }
//    else 
//    {
//        // Device Info is stored in flash.
//        FlashUtils::FlashSafe();
//        //!!flash_range_erase(FirmwareUpdateManager::FlashAddrToOffset(kFlashDeviceInfoStartAddr),
//        //                  FlashUtils::kFlashSectorSizeBytes);
//        //!!flash_range_program(FirmwareUpdateManager::FlashAddrToOffset(kFlashDeviceInfoStartAddr),
//        //                    (uint8_t *)&device_info, sizeof(device_info));
//        FlashUtils::FlashUnsafe();
//        return true;
//    }
//}

//!!bool SettingsManager::GetDeviceInfo(DeviceInfo &device_info) 
//{
//    if (bsp.has_eeprom) 
//    {
//        // Device Info is stored on external EEPROM.
//        if (eeprom_Pico.RequiresInit()) return false;
//        return eeprom_Pico.Load(device_info, kEEPROMDeviceInfoOffset);
//    }
//    else 
//    {
//        // Device Info is stored in flash.
//         FlashUtils::FlashSafe();
//        device_info = *(DeviceInfo *)(kFlashDeviceInfoStartAddr);
//        FlashUtils::FlashUnsafe();
//        return true;
//    }
//}

bool SettingsManager::Apply() 
{
    adsbee.SetReceiver1090Enable(settings.receiver_enabled);
    adsbee.SetTLMilliVolts(settings.tl_mv);
    adsbee.SetBiasTeeEnable(settings.bias_tee_enabled);
   //!! adsbee.SetWatchdogTimeoutSec(settings.watchdog_timeout_sec); //Запуск сторожевого таймера

    // Apply log level.
    comms_manager.log_level = settings.log_level;

    // Применить протоколы отчетности.
    comms_manager.SetReportingProtocol(SerialInterface::kCommsUART, settings.reporting_protocols[SerialInterface::kCommsUART]);
   //!! comms_manager.SetReportingProtocol(SerialInterface::kConsole, settings.reporting_protocols[SerialInterface::kConsole]); //!!Не нужно в данном случае

    // Apply baud rates.
    comms_manager.SetBaudrate(SerialInterface::kCommsUART, settings.comms_uart_baud_rate);
    comms_manager.SetBaudrate(SerialInterface::kGNSSUART, settings.gnss_uart_baud_rate);

    return true;  // В настоящее время не выполняется проверка ошибок, полагаясь на AT-команды для ограничения параметров
                  // допустимыми диапазонами. Может возникнуть проблема при загрузке из поврежденного EEPROM.
}
void SettingsManager::PrintPico()
{
    //Пока не задейсвован
}