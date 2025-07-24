#ifndef SETTINGS_HH_
#define SETTINGS_HH_

#include <stdlib.h>
#include <cstdint>
#include <cstring>     // for memset
#include <functional>  // for strtoull

#include "stdio.h"


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
            for (uint16_t i = 0; i < kMaxNumFeeds; i++) 
            {
                memset(feed_uris[i], '\0', kFeedURIMaxNumChars + 1);
                feed_ports[i] = 0;
                feed_is_active[i] = false;
                feed_protocols[i] = kNoReports;

                // ESP32 придется запросить идентификатор приемника позже.
                memset(feed_receiver_ids[i], 0, kFeedReceiverIDNumBytes);

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

 
    /**
     * Applies internal settings to the relevant objects. This is only used after the settings struct has been updated
     * by loading it from EEPROM or by overwriting it via the coprocessor SPI bus.
     */
    bool Apply();

    /**
    * Загружает настройки из EEPROM. Предполагается, что настройки хранятся по адресу 0x0, и не выполняет проверку целостности.
    * @retval True в случае успеха, в противном случае false.
    */
    bool Load();

    /**
   * Распечатать настройки в удобном для восприятия формате.
   */
    void PrintPico();

        /**
     * Saves settings to EEPROM. Stores settings at address 0x0 and performs no integrity check.
     * @retval True if succeeded, false otherwise.
     */
    bool Save();
     /**
     * Restores settings to factory default values.
     */
    void ResetToDefaults();

    Settings settings;

   private:
};

extern SettingsManager settings_manager;

#endif /* SETTINGS_HH_ */