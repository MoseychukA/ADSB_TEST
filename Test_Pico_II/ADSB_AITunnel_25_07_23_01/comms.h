//#pragma once

#ifndef COMMS_H_
#define COMMS_H_

#include "Arduino.h"
#include "data_structures.h"  // For PFBQueue.
#include "hardware/uart.h"
#include "settings.h"
#include "transponder_packet.h"


class CommsManager 
{

public:

       CommsManager();
       ~CommsManager();
    static constexpr uint16_t kATCommandBufMaxLen         = 1000;
    static constexpr uint16_t kNetworkConsoleBufMaxLen    = 4096;
    static constexpr uint16_t kPrintfBufferMaxSize        = 500;
    static constexpr uint32_t kRawReportingIntervalMs     = 50;  // Report packets internally at 20Hz.
    static constexpr uint32_t kMAVLINKReportingIntervalMs = 1000;
    static constexpr uint32_t kCSBeeReportingIntervalMs   = 1000;
    static constexpr uint32_t kGDL90ReportingIntervalMs   = 1000;

    static constexpr uint32_t kOTAWriteTimeoutMs = 5000;  // ms until OTA write command exits if all bytes not received.

    struct CommsManagerConfig 
    {
        uart_inst_t* gnss_uart_handle = uart0;
        uint16_t gnss_uart_tx_pin = 0;
        uint16_t gnss_uart_rx_pin = 1;
        uart_inst_t *comms_uart_handle = uart1;
        uint16_t comms_uart_tx_pin = 4;
        uint16_t comms_uart_rx_pin = 5;
        uint16_t uart_timeout_us = 0;  // Time to wait for a character if there isn't one alredy available.
    };


    CommsManager(CommsManagerConfig config_in);

    /**
     * Initialize the CommsManager. Sets up UARTs and other necessary peripherals.
     * @retval True if initialization succeeded, false otherwise.
     */
    bool Init();
    /**
    * �������� CommsManager. ��������� ��� ������������ ����������, ����������� ��� ���������� ������.
    * @retval True, ���� ���������� ������ �������, � ��������� ������ � false.
    */
    void Update();
    //!!bool Update();

    /**
     * Update incoming and outgoing buffers for the ESP32 network console. Called as part of Update(), or can be called
     * separately if desired (e.g. while printing to the console without forwarding incoming data to the AT command
     * parser).
     * @retval True if update succeeded, false otherwise.
     */
    bool UpdateNetworkConsole();

    int console_printf(const char *format, ...);
    int console_level_printf(SettingsManager::LogLevel level, const char *format, ...);
    int iface_printf(SettingsManager::SerialInterface iface, const char *format, ...);
    int iface_vprintf(SettingsManager::SerialInterface iface, const char *format, va_list args);
    bool iface_putc(SettingsManager::SerialInterface iface, char c);
    bool iface_getc(SettingsManager::SerialInterface iface, char &c);
    bool iface_puts(SettingsManager::SerialInterface iface, const char *buf);

    bool network_console_putc(char c);
    bool network_console_puts(const char *buf, uint16_t len = UINT16_MAX);

    void SendBuf(SettingsManager::SerialInterface iface, char *buf, uint16_t buf_len) 
    {
        for (uint16_t i = 0; i < buf_len; i++) 
        {
            iface_putc(iface, buf[i]);
        }
    }

    /**
     * Sets the baudrate for a serial interface.
     * @param[in] iface SerialInterface to set baudrate for.
     * @param[in] baudrate Baudrate to set.
     * @retval True if the baudrate could be set, false if the interface specified does not support a baudrate.
     */
    bool SetBaudrate(SettingsManager::SerialInterface iface, uint32_t baudrate) 
    {
        switch (iface) 
        {
            case SettingsManager::kCommsUART:
                // Save the actual set value as comms_uart_baudrate_.
                comms_uart_baudrate_ = uart_set_baudrate(config_.comms_uart_handle, baudrate);
                return true;
                break;
            case SettingsManager::kGNSSUART:
                // Save the actual set value as gnss_uart_baudrate_.
                gnss_uart_baudrate_ = uart_set_baudrate(config_.gnss_uart_handle, baudrate);
                return true;
                break;
            default:
                return false;  // Other interfaces don't have a baudrate.
        }
        return false;  // Should never get here.
    }

    /**
     * Returns the currently set baudrate for a serial interface.
     * @param[in] iface SerialInterface to get the baudrate for.
     * @param[out] baudrate Reference to uint32_t to fill with retrieved value.
     * @retval True if baudrate retrieval succeeded, false if iface does not support a baudrate.
     */
    bool GetBaudrate(SettingsManager::SerialInterface iface, uint32_t &baudrate) 
    {
        switch (iface) 
        {
            case SettingsManager::kCommsUART:
                // Save the actual set value as comms_uart_baudrate_.
                baudrate = comms_uart_baudrate_;
                return true;
                break;
            case SettingsManager::kGNSSUART:
                // Save the actual set value as gnss_uart_baudrate_.
                baudrate = gnss_uart_baudrate_;
                return true;
                break;
            default:
                return false;  // Other interfaces don't have a baudrate.
        }
        return false;  // Should never get here.
    }

    /**
     * Specify the reporting protocol for a given serial interface.
     * @param[in] iface SerialInterface to set reporting protocol on.
     * @param[in] protocol Reporting protocol to set on iface.
     * @retval True if succeeded, false otherwise.
     */
    bool SetReportingProtocol(SettingsManager::SerialInterface iface, SettingsManager::ReportingProtocol protocol) 
    {
        reporting_protocols_[iface] = protocol;
        return true;
    }

    /**
     * Get the reporting protocol for a given serial interface.
     * @param[in] iface SerialInterface to get the reporting protocol from.
     * @param[out] protocol reference to ReportingProtocol to fill with result.
     * @retval True if reportig protocol could be retrieved, false otherwise.
     */
    bool GetReportingProtocol(SettingsManager::SerialInterface iface, SettingsManager::ReportingProtocol &protocol) 
    {
        protocol = reporting_protocols_[iface];
        return true;
    }

    // Public console settings.
    SettingsManager::LogLevel log_level = SettingsManager::LogLevel::kInfo;  // Start with highest verbosity by default.

    // Queue for storing transponder packets before they get reported.
    PFBQueue<Decoded1090Packet> transponder_packet_reporting_queue =
        PFBQueue<Decoded1090Packet>({.buf_len_num_elements = SettingsManager::Settings::kMaxNumTransponderPackets,
                                     .buffer = transponder_packet_reporting_queue_buffer_});

    // Queues for incoming / outgoing network characters.
    PFBQueue<char> esp32_console_rx_queue =
        PFBQueue<char>({.buf_len_num_elements = kNetworkConsoleBufMaxLen, .buffer = esp32_console_rx_queue_buffer_});
    PFBQueue<char> esp32_console_tx_queue =
        PFBQueue<char>({.buf_len_num_elements = kNetworkConsoleBufMaxLen, .buffer = esp32_console_tx_queue_buffer_});

   private:
    // AT Functions
    //bool InitAT();
    //bool UpdateAT();

    // Reporting Functions
   // bool InitReporting();
    bool UpdateReporting();

    bool ReportRaw(SettingsManager::SerialInterface iface, const Decoded1090Packet packets_to_report_1090[],
                   uint16_t num_packets_to_report);

    /**
     * Sends out Mode S Beast formatted transponder data on the selected serial interface. Reports all transponder
     * packets in the provided packets_to_report array, which is used to allow printing arbitrary blocks of transponder
     * packets received via the CommsManager's built-in transponder_packet_reporting_queue_.
     * @param[in] iface SerialInterface to broadcast Mode S Beast messages on.
     * @param[in] packets_to_report Array of transponder packets to report.
     * @param[in] num_packets_to_report Number of packets to report from the packets_to_report array.
     * @retval True if successful, false if something broke.
     */
    bool ReportBeast(SettingsManager::SerialInterface iface, const Decoded1090Packet packets_to_report_1090[],
                     uint16_t num_packets_to_report);

    /**
     * Sends out comma separated aircraft information for each aircraft in the aircraft dictionary.
     * @param[in] iface SerialInterface to broadcast aircraft information on.
     * @retval True if successful, false if something broke.
     */
    bool ReportCSBee(SettingsManager::SerialInterface iface);

    CommsManagerConfig config_;

    // ������� ��� ��������/��������� �������� ������� �������.
    char esp32_console_rx_queue_buffer_[kNetworkConsoleBufMaxLen];
    char esp32_console_tx_queue_buffer_[kNetworkConsoleBufMaxLen];

    // Queue for holding new transponder packets before they get reported.
    Decoded1090Packet transponder_packet_reporting_queue_buffer_[SettingsManager::Settings::kMaxNumTransponderPackets];

    // Reporting Settings
    uint32_t comms_uart_baudrate_ = SettingsManager::Settings::kDefaultCommsUARTBaudrate;
    uint32_t gnss_uart_baudrate_ = SettingsManager::Settings::kDefaultGNSSUARTBaudrate;
    SettingsManager::ReportingProtocol
        reporting_protocols_[SettingsManager::SerialInterface::kNumSerialInterfaces - 1] = {
            SettingsManager::ReportingProtocol::kNoReports,
            SettingsManager::ReportingProtocol::kMAVLINK1};  // GNSS_UART not included.

    // Reporting protocol timestamps
    // NOTE: Raw reporting interval used for RAW and BEAST protocols as well as internal functions.
    uint32_t last_raw_report_timestamp_ms_ = 0;
    uint32_t last_csbee_report_timestamp_ms_ = 0;
    uint32_t last_mavlink_report_timestamp_ms_ = 0;
    uint32_t last_gdl90_report_timestamp_ms_ = 0;

    // OTA configuration. Used to ignore incoming UART commands while processing OTA data.
    uint32_t ota_transfer_begin_timestamp_ms_ = 0;
    uint32_t ota_transfer_bytes_remaining_ = 0;
};

extern CommsManager comms_manager;
////!!extern const CppAT::ATCommandDef_t at_command_list[];  // Initialized in comms_at.cc
//extern const uint16_t at_command_list_num_commands;
//
//#define TEXT_COLOR_RED              "\033[31m"
//#define TEXT_COLOR_GREEN            "\033[32m"
//#define TEXT_COLOR_YELLOW           "\033[33m" /* orange on some systems */
//#define TEXT_COLOR_BLUE             "\033[34m"
//#define TEXT_COLOR_MAGENTA          "\033[35m"
//#define TEXT_COLOR_CYAN             "\033[36m"
//#define TEXT_COLOR_LIGHT_GRAY       "\033[37m"
//#define TEXT_COLOR_DARK_GRAY        "\033[90m"
//#define TEXT_COLOR_BRIGHT_RED       "\033[91m"
//#define TEXT_COLOR_BRIGHT_GREEN     "\033[92m"
//#define TEXT_COLOR_BRIGHT_YELLOW    "\033[93m"
//#define TEXT_COLOR_BRIGHT_BLUE      "\033[94m"
//#define TEXT_COLOR_BRIGHT_MAGENTA   "\033[95m"
//#define TEXT_COLOR_BRIGHT_CYAN      "\033[96m"
//#define TEXT_COLOR_WHITE            "\033[97m"
//
//#define TEXT_COLOR_RESET            "\033[0m"
//
//
//#define CONSOLE_PRINTF(format, ...) comms_manager.console_printf(format __VA_OPT__(, ) __VA_ARGS__);
//#define CONSOLE_INFO(tag, format, ...) comms_manager.console_level_printf(SettingsManager::LogLevel::kInfo, tag ": " format "\r\n" __VA_OPT__(, ) __VA_ARGS__);
//#define CONSOLE_WARNING(tag, format, ...) comms_manager.console_level_printf(SettingsManager::LogLevel::kWarnings,tag ": " TEXT_COLOR_YELLOW format TEXT_COLOR_RESET "\r\n" __VA_OPT__(, ) __VA_ARGS__);
//#define CONSOLE_ERROR(tag, format, ...) comms_manager.console_level_printf(SettingsManager::LogLevel::kErrors, tag ": " TEXT_COLOR_RED format TEXT_COLOR_RESET "\r\n" __VA_OPT__(, ) __VA_ARGS__);
//



#endif /* COMMS_H_ */