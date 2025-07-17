#include "object_dictionary.h"
#include "comms.h"

const uint8_t ObjectDictionary::kFirmwareVersionMajor = 1;
const uint8_t ObjectDictionary::kFirmwareVersionMinor = 2;
const uint8_t ObjectDictionary::kFirmwareVersionPatch = 3;
// NOTE: Indicate a final release with RC = 0.
const uint8_t ObjectDictionary::kFirmwareVersionReleaseCandidate = 3;

const uint32_t ObjectDictionary::kFirmwareVersion = (kFirmwareVersionMajor << 24) | (kFirmwareVersionMinor << 16) |
                                                    (kFirmwareVersionPatch << 8) | kFirmwareVersionReleaseCandidate;

bool ObjectDictionary::SetBytes(Address addr, uint8_t *buf, uint16_t buf_len, uint16_t offset) {
    switch (addr) {
        case kAddrScratch:
            // Warning: printing here will cause a timeout and tests will fail.
             CONSOLE_INFO("ObjectDictionary::SetBytes", "Setting %d settings Bytes at offset %d.", buf_len,
             offset);
            memcpy((uint8_t *)&scratch_ + offset, buf, buf_len);
            break;
        case kAddrSettingsData:
            // Предупреждение: печать здесь приведет к тайм-ауту и ​​тесты не будут пройдены.
            // CONSOLE_INFO("ObjectDictionary::SetBytes", "Setting %d settings Bytes at offset %d.", buf_len,
            // offset);
            memcpy((uint8_t *)&(settings_manager.settings) + offset, buf, buf_len);
            if (offset + buf_len == sizeof(SettingsManager::Settings)) 
            {
                CONSOLE_INFO("SPICoprocessor::SetBytes", "Wrote last chunk of settings data. Applying new values.");
                settings_manager.Apply();
                //!!settings_manager.PrintPico(); //Пока не задейсвован
            }
            break;
        default:
            CONSOLE_ERROR("SPICoprocessor::SetBytes", "No behavior implemented for writing to address 0x%x.", addr);
            return false;
    }
    return true;
}

bool ObjectDictionary::GetBytes(Address addr, uint8_t *buf, uint16_t buf_len, uint16_t offset) 
{
    switch (addr) {
        case kAddrFirmwareVersion:
            memcpy(buf, (uint8_t *)(&kFirmwareVersion) + offset, buf_len);
            break;
        case kAddrScratch:
            // Предупреждение: печать здесь приведет к тайм-ауту и ​​тесты не будут пройдены.
             CONSOLE_INFO("ObjectDictionary::GetBytes", "Getting %d scratch Bytes at offset %d.", buf_len,
             offset);
            memcpy(buf, (uint8_t *)(&scratch_) + offset, buf_len);
            break;
        case kAddrSettingsData:
            // Предупреждение: печать здесь приведет к тайм-ауту и ​​тесты не будут пройдены.
             CONSOLE_INFO("ObjectDictionary::GetBytes", "Getting %d settings Bytes at offset %d.", buf_len,
             offset);
            memcpy(buf, (uint8_t *)&(settings_manager.settings) + offset, buf_len);
            break;
        default:
            CONSOLE_ERROR("SPICoprocessor::SetBytes", "No behavior implemented for reading from address 0x%x.", addr);
            return false;
    }
    return true;
}