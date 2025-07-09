#ifndef SETTINGS_HH_
#define SETTINGS_HH_

#include <stdlib.h>
#include <cstdint>
#include <cstring>     // for memset
#include <functional>  // for strtoull

#include "macros.h"
#include "stdio.h"
//!!#include "pico/rand.h"  //Генератор случайных чисел


#ifdef ON_PICO
//!!#include "pico/rand.h"
#endif

static constexpr uint32_t kSettingsVersion   = 0x7;  // Измените это при изменении формата настроек!
static constexpr uint32_t kDeviceInfoVersion = 0x2;

class SettingsManager 
{
   public:
    // Serial Interface enum and string conversion array.
    enum SerialInterface : uint16_t { kConsole = 0, kCommsUART, kGNSSUART, kNumSerialInterfaces };
    static constexpr uint16_t kSerialInterfaceStrMaxLen = 30;
    static const char kSerialInterfaceStrs[SerialInterface::kNumSerialInterfaces][kSerialInterfaceStrMaxLen];

    enum LogLevel : uint16_t { kSilent = 0, kErrors, kWarnings, kInfo, kNumLogLevels };
    static constexpr uint16_t kConsoleLogLevelStrMaxLen = 30;
    static const char kConsoleLogLevelStrs[LogLevel::kNumLogLevels][kConsoleLogLevelStrMaxLen];

    // Reporting Protocol enum and string conversion array.
    enum ReportingProtocol : uint16_t {
        kNoReports = 0,
        kRaw,
        kBeast,
        kBeastRaw,
        kCSBee,
        kMAVLINK1,
        kMAVLINK2,
        kGDL90,
        kNumProtocols
    };
    static constexpr uint16_t kReportingProtocolStrMaxLen = 30;
    static const char kReportingProtocolStrs[ReportingProtocol::kNumProtocols][kReportingProtocolStrMaxLen];

    static constexpr uint8_t kWiFiAPChannelMax = 11;  // В США избегают работы на каналах 12-14.

    enum EnableState : int8_t {
        kEnableStateExternal = -1,  // Enable GPIO pin is high impedance.
        kEnableStateDisabled = 0,
        kEnableStateEnabled = 1
    };

    // Эта структура содержит неизменяемые настройки, которые должны сохраняться при перезагрузках, но могут быть перезаписаны во время
    // обновления прошивки, если формат структуры настроек изменится.
    struct Settings {
        static constexpr int kDefaultTLMV = 1300;  // [mV]
        static constexpr uint16_t kMaxNumTransponderPackets = 100;  // Определяет размер кольцевого буфера ADSBPacket (PFBQueue).
        static constexpr uint32_t kDefaultWatchdogTimeoutSec = 10;
        // ПРИМЕЧАНИЕ: Длина не включает нулевой терминатор.
        static constexpr uint16_t kHostnameMaxLen = 32;
        static constexpr uint16_t kWiFiSSIDMaxLen = 32;
        static constexpr uint16_t kWiFiPasswordMaxLen = 64;
        static constexpr uint16_t kWiFiMaxNumClients = 6;
        static constexpr uint32_t kDefaultCommsUARTBaudrate = 115200;
        static constexpr uint32_t kDefaultGNSSUARTBaudrate = 9600;
        static constexpr uint16_t kMaxNumFeeds = 10;
        static constexpr uint16_t kFeedURIMaxNumChars = 63;
        static constexpr uint16_t kFeedReceiverIDNumBytes = 8;
        static constexpr uint16_t kIPAddrStrLen = 16;   // XXX.XXX.XXX.XXX (does not include null terminator)
        static constexpr uint16_t kMACAddrStrLen = 18;  // XX:XX:XX:XX:XX:XX (does not include null terminator)
        static constexpr uint16_t kMACAddrNumBytes = 6;

        uint32_t settings_version = kSettingsVersion;

        // ADSBee settings
        bool receiver_enabled = true;
        int tl_mv = kDefaultTLMV;
        bool bias_tee_enabled = false;
        uint32_t watchdog_timeout_sec = kDefaultWatchdogTimeoutSec;

        // Настройки менеджера коммуникаций
        LogLevel log_level = LogLevel::kWarnings;
        ReportingProtocol reporting_protocols[SerialInterface::kNumSerialInterfaces - 1] = {
            ReportingProtocol::kNoReports, ReportingProtocol::kMAVLINK1};
        uint32_t comms_uart_baud_rate = 115200;
        uint32_t gnss_uart_baud_rate = 9600;

        // Sub-GHz settings
        EnableState subg_enabled = EnableState::kEnableStateExternal;  // Состояние высокого импеданса по умолчанию.

        bool ethernet_enabled = false;

        char feed_uris[kMaxNumFeeds][kFeedURIMaxNumChars + 1];
        uint16_t feed_ports[kMaxNumFeeds];
        bool feed_is_active[kMaxNumFeeds];
        ReportingProtocol feed_protocols[kMaxNumFeeds];
        uint8_t feed_receiver_ids[kMaxNumFeeds][kFeedReceiverIDNumBytes];

        /**
         * Default constructor.
         */
        Settings() 
        {
#ifdef ON_PICO
            DeviceInfo device_info;
            if (GetDeviceInfo(device_info)) {
                // If able to load device info from EEPROM, use the last 16 characters in the part code as part of the
                // WiFi SSID.
                device_info.GetDefaultSSID(wifi_ap_ssid);
                // Reuse the WiFi SSID as the hostname.
                strncpy(hostname, wifi_ap_ssid, 32);
                snprintf(wifi_ap_password, kWiFiPasswordMaxLen, "yummyflowers");
            }

            wifi_ap_channel = get_rand_32() % kWiFiAPChannelMax + 1;  // Randomly select channel 1-11.
#endif

            for (uint16_t i = 0; i < kMaxNumFeeds; i++) 
            {
                memset(feed_uris[i], '\0', kFeedURIMaxNumChars + 1);
                feed_ports[i] = 0;
                feed_is_active[i] = false;
                feed_protocols[i] = kNoReports;
#ifdef ON_PICO
                // Pico has access to EEPROM for receiver ID in device info.
                device_info.GetDefaultFeedReceiverID(feed_receiver_ids[i]);
#else
                // ESP32 придется запросить идентификатор приемника позже.
                memset(feed_receiver_ids[i], 0, kFeedReceiverIDNumBytes);
#endif
            }

            // Set default feed URIs.
            // adsb.fi: feed.adsb.fi:30004, Beast
            strncpy(feed_uris[kMaxNumFeeds - 1], "feed.adsb.fi", kFeedURIMaxNumChars);
            feed_uris[kMaxNumFeeds - 1][kFeedURIMaxNumChars] = '\0';
            feed_ports[kMaxNumFeeds - 1] = 30004;
            feed_is_active[kMaxNumFeeds - 1] = true;
            feed_protocols[kMaxNumFeeds - 1] = kBeast;
            // airplanes.live: feed.airplanes.live:30004, Beast
            strncpy(feed_uris[kMaxNumFeeds - 2], "feed.airplanes.live", kFeedURIMaxNumChars);
            feed_uris[kMaxNumFeeds - 2][kFeedURIMaxNumChars] = '\0';
            feed_ports[kMaxNumFeeds - 2] = 30004;
            feed_is_active[kMaxNumFeeds - 2] = true;
            feed_protocols[kMaxNumFeeds - 2] = kBeast;
            // adsb.lol: feed.adsb.lol:30004, Beast
            strncpy(feed_uris[kMaxNumFeeds - 3], "feed.adsb.lol", kFeedURIMaxNumChars);
            feed_uris[kMaxNumFeeds - 3][kFeedURIMaxNumChars] = '\0';
            feed_ports[kMaxNumFeeds - 3] = 30004;
            feed_is_active[kMaxNumFeeds - 3] = true;
            feed_protocols[kMaxNumFeeds - 3] = kBeast;
            // whereplane.xyz: feed.whereplane.xyz:30004, Beast
            strncpy(feed_uris[kMaxNumFeeds - 4], "feed.whereplane.xyz", kFeedURIMaxNumChars);
            feed_uris[kMaxNumFeeds - 4][kFeedURIMaxNumChars] = '\0';
            feed_ports[kMaxNumFeeds - 4] = 30004;
            feed_is_active[kMaxNumFeeds - 4] = false;  // Not active by default.
            feed_protocols[kMaxNumFeeds - 4] = kBeast;
        }
    };

    // Эта структура содержит информацию об устройстве, которая должна сохраняться при обновлениях прошивки.
    struct DeviceInfo 
    {
        // NOTE: Lengths do not include null terminator.
        static constexpr uint16_t kPartCodeLen = 26;  // NNNNNNNNNR-YYYYMMDD-VVXXXX (not counting end of string char).
        static constexpr uint16_t kOTAKeyMaxLen = 128;
        static constexpr uint16_t kNumOTAKeys = 2;

        uint32_t device_info_version = kDeviceInfoVersion;
        char part_code[kPartCodeLen + 1];
        char ota_keys[kNumOTAKeys][kOTAKeyMaxLen + 1];

        /**
         * Default constructor.
         */
        DeviceInfo() 
        {
            memset(part_code, '\0', kPartCodeLen + 1);
            for (uint16_t i = 0; i < kNumOTAKeys; i++) 
            {
                memset(ota_keys[i], '\0', kOTAKeyMaxLen + 1);
            }
        }

        //static constexpr uint16_t kDefaultSSIDLenChars = 24;  // ADSBee1090-YYYMMDDVVXXXX
        ///**
        //* Записывает значение по умолчанию для сетевого SSID в буфер. Буфер должен быть не менее kDefaultSSIDLenChars+1, чтобы
        //* было место для символа конца строки. Это значение сетевого SSID по умолчанию не должно
        //* конфликтовать с другими устройствами ADSBee или будущими продуктами Pants for Birds.
        //* @param[out] buf Буфер для записи сетевого SSID.
        //*/
        //void GetDefaultSSID(char *buf) 
        //{
        //    memcpy(buf, "ADSBee1090-", 11);       // [0:10] ADSBee1090-
        //    memcpy(buf + 11, part_code + 12, 7);  // [11:17] YYYMMDD
        //    memcpy(buf + 18, part_code + 20, 6);  // [18:23] VVXXXX
        //    buf[kDefaultSSIDLenChars] = '\0';
        //}
        /**
        * Записывает уникальный идентификатор по умолчанию длиной 8 байт в буфер. Буфер должен быть длиной не менее 8 байт. UID в двоичном формате
        * (не читается человеком) и имеет вид 0xBE 0xE0 NN NN NN NN, где NN представляют байты в уникальном
        * значении, сформированном из целого числа Base-10 YYYMMDDVVXXXX, извлеченного из производственного кода ADSBee 1090, отправленного
        * MSB первым. Уникальный идентификатор не должен конфликтовать между двумя устройствами ADSBee 1090. Новые устройства в линейке
        * ADSBee могут иметь префикс 0xBE EN, где N — значение больше 0.
        * @param[out] buf Буфер для записи уникального идентификатора длиной 8 байт.
        */
        void GetDefaultFeedReceiverID(uint8_t *buf) 
        {
            // 0xBE 0xE0 <6 Byte Binary UID, MSB first.>
            buf[0] = 0xBE;
            buf[1] = 0xE0;
            // Base the rest of the UID off of a 13-digit Base 10 number.
            char uid_digits[14];                        // YYYMMDDVVXXXX
            memcpy(uid_digits, part_code + 12, 7);      // YYYMMDD
            memcpy(uid_digits + 7, part_code + 20, 6);  // VVXXXX
            uid_digits[13] = '\0';
            // log2(10^13) = 43.18, so we need 44 (6 Bytes) bits to store the UID.
            uint64_t uid_value = strtoull(uid_digits, nullptr, 10);
            for (uint16_t i = 0; i < 6; i++) {
                buf[2 + i] = (uid_value >> (8 * (5 - i))) & 0xFF;
            }
        }
    };

    /**
     * Applies internal settings to the relevant objects. This is only used after the settings struct has been updated
     * by loading it from EEPROM or by overwriting it via the coprocessor SPI bus.
     */
    bool Apply();

    /////**
    //// * Helper function for reconstructing an AT command value for a given EnableState.
    //// * @param[in] state EnableState to convert to a string.
    //// * @retval String representation of the EnableState, as it would be used in an AT command.
    //// */
    //static inline const char *EnableStateToATValueStr(EnableState state) 
    //{
    //    switch (state) 
    //    {
    //        case kEnableStateExternal:
    //            return "EXTERNAL";
    //        case kEnableStateEnabled:
    //            return "1";
    //        case kEnableStateDisabled:
    //            return "0";
    //        default:
    //            return "?";
    //    }
    //}

    /**
    * Загружает настройки из EEPROM. Предполагается, что настройки хранятся по адресу 0x0, и не выполняет проверку целостности.
    * @retval True в случае успеха, в противном случае false.
    */
    bool Load();

    /**
   * Распечатать настройки в удобном для восприятия формате.
   */
    void PrintPico();

    ///**
    //* Принимает пароль как строку и заполняет буфер соответствующим количеством звездочек.
    //* @param[in] password_buf Буфер для чтения пароля. Должен быть не менее password_len+1 символов.
    //* @param[out] redacted_password_buf Буфер для записи звездочки. Должен быть не менее password_len+1 символов.
    //* @param[in] buf_len Максимально допустимое количество символов в пароле. Используется для защиты от выпадения
    //* конца строки. Не используется для фактического поиска количества звездочек для печати.
    //*/
    //static inline void RedactPassword(char *password_buf, char *redacted_password_buf, uint16_t buf_len) {
    //    uint16_t password_len = MIN(strlen(password_buf), buf_len);
    //    memset(redacted_password_buf, '*', password_len);
    //    redacted_password_buf[password_len] = '\0';
    //}

        /**
     * Saves settings to EEPROM. Stores settings at address 0x0 and performs no integrity check.
     * @retval True if succeeded, false otherwise.
     */
    bool Save();
    ///**
    //* Выводит 8-байтовый идентификатор приемника в строковый буфер.
    //* @param[in] receiver_id Указатель на первый байт 8-байтового идентификатора приемника.
    //* @param[in] buf Буфер для записи строки идентификатора приемника. Должен быть не менее 17 символов (включая нулевой терминатор).
    //*/
    //static inline void ReceiverIDToStr(uint8_t *receiver_id, char *buf) 
    //{
    //    for (int16_t i = 0; i < SettingsManager::Settings::kFeedReceiverIDNumBytes; i++) 
    //    {
    //        snprintf(buf, 2 * SettingsManager::Settings::kFeedReceiverIDNumBytes, "%02x%02x%02x%02x%02x%02x%02x%02x",
    //                 receiver_id[0], receiver_id[1], receiver_id[2], receiver_id[3], receiver_id[4], receiver_id[5],
    //                 receiver_id[6], receiver_id[7]);
    //    }
    //}

    /**
     * Restores settings to factory default values.
     */
    void ResetToDefaults();


    /**
    * Используется для записи информации об устройстве в EEPROM во время производства. Доступно только на Pico, так как это единственный
    * имеющий прямой доступ к EEPROM через I2C.
    * @param[in] device_info Ссылка на структуру DeviceInfo с информацией для установки в EEPROM.
    * @retval True, если информация об устройстве была успешно установлена, в противном случае false.
    */
    //!!static bool SetDeviceInfo(const DeviceInfo &device_info);

    /**
    * Используется для извлечения информации об устройстве, либо напрямую из EEPROM, либо через межпроцессорную шину SPI.
    * @param[in] device_info Структура DeviceInfo для установки.
    * @retval True, если информация об устройстве была успешно извлечена, в противном случае false.
    */
    //!!static bool GetDeviceInfo(DeviceInfo &device_info);

    Settings settings;

   private:
};

extern SettingsManager settings_manager;

#endif /* SETTINGS_HH_ */