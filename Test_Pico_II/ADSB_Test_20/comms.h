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
     static constexpr uint16_t kNetworkConsoleBufMaxLen    = 4096;
    static constexpr uint16_t kPrintfBufferMaxSize        = 500;

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
 
    int console_printf(const char *format, ...);
    int console_level_printf(SettingsManager::LogLevel level, const char *format, ...);
    int iface_printf(SettingsManager::SerialInterface iface, const char *format, ...);
    int iface_vprintf(SettingsManager::SerialInterface iface, const char *format, va_list args);
    bool iface_putc(SettingsManager::SerialInterface iface, char c);
    bool iface_getc(SettingsManager::SerialInterface iface, char &c);
    bool iface_puts(SettingsManager::SerialInterface iface, const char *buf);

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

    CommsManagerConfig config_;

    // Очереди для входящих/исходящих символов сетевой консоли.
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

#endif /* COMMS_H_ */