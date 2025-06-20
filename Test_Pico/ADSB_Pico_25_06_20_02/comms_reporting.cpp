#include "adsbee.h"
#include "beast_utils.h"
#include "comms.h"
#include "csbee_utils.h"
#include "hal.h"  // For timestamping.
#include "raw_utils.h"
#include "unit_conversions.h"

extern ADSBee adsbee;

bool CommsManager::InitReporting() { return true; }

bool CommsManager::UpdateReporting() 
{
    bool ret = true;
    uint32_t timestamp_ms = get_time_since_boot_ms();

    if (timestamp_ms - last_raw_report_timestamp_ms_ <= kRawReportingIntervalMs) 
    {
        return true;  // Nothing to update.
    }
    // Proceed with update and record timestamp.
    last_raw_report_timestamp_ms_ = timestamp_ms;

    Decoded1090Packet packets_to_report[SettingsManager::Settings::kMaxNumTransponderPackets];
    /**
     * Raw packet reporting buffer used to transfer multiple packets at once over SPI.
     * [<uint8_t num_packets to report> <packet 1> <packet 2> ...]
     */
    uint8_t spi_raw_packet_reporting_buffer[sizeof(uint8_t) + SettingsManager::Settings::kMaxNumTransponderPackets];

    // Fill up the array of Decoded1090Packets for internal functions, and the buffer of Raw1090Packets to
    // send to the ESP32 over SPI. Raw1090Packets are used instead of Decoded1090Packets over the SPI link
    // in order to preserve bandwidth.
    uint16_t num_packets_to_report = 0;
    for (; num_packets_to_report < SettingsManager::Settings::kMaxNumTransponderPackets &&
           transponder_packet_reporting_queue.Pop(packets_to_report[num_packets_to_report]);
         num_packets_to_report++) 
    {
        //if (esp32.IsEnabled()) {
        //    // Pop all the packets to report (up to max limit of the buffer).
        //    Raw1090Packet raw_packet = packets_to_report[num_packets_to_report].GetRaw();
        //    spi_raw_packet_reporting_buffer[0] = num_packets_to_report + 1;
        //    memcpy(spi_raw_packet_reporting_buffer + sizeof(uint8_t) + sizeof(Raw1090Packet) * num_packets_to_report,
        //           &raw_packet, sizeof(Raw1090Packet));
        //}
    }
    //if (esp32.IsEnabled() && num_packets_to_report > 0) {
    //    // Write packet to ESP32 with a forced ACK.
    //    esp32.Write(ObjectDictionary::kAddrRaw1090PacketArray,                       // addr
    //                spi_raw_packet_reporting_buffer,                                 // buf
    //                true,                                                            // require_ack
    //                sizeof(uint8_t) + num_packets_to_report * sizeof(Raw1090Packet)  // len
    //    );
    //}

    //!!for (uint16_t i = 0; i < SettingsManager::SerialInterface::kGNSSUART; i++) 
    //{
    //    SettingsManager::SerialInterface iface = static_cast<SettingsManager::SerialInterface>(i);
    //    switch (reporting_protocols_[i]) {
    //        case SettingsManager::kNoReports:
    //            break;
    //        case SettingsManager::kRaw:
    //            ret = ReportRaw(iface, packets_to_report, num_packets_to_report);
    //            break;
    //        case SettingsManager::kBeast:
    //            ret = ReportBeast(iface, packets_to_report, num_packets_to_report);
    //            break;
    //        case SettingsManager::kCSBee:
    //            if (timestamp_ms - last_csbee_report_timestamp_ms_ >= kCSBeeReportingIntervalMs) {
    //                ret = ReportCSBee(iface);
    //                last_csbee_report_timestamp_ms_ = timestamp_ms;
    //            }
    //            break;
    //         case SettingsManager::kNumProtocols:
    //        default:
    //            CONSOLE_WARNING("CommsManager::UpdateReporting",
    //                            "Invalid reporting protocol %d specified for interface %d.", reporting_protocols_[i],
    //                            i);
    //            ret = false;
    //            break;
    //    }
    //}

    return ret;
}

bool CommsManager::ReportRaw(SettingsManager::SerialInterface iface, const Decoded1090Packet packets_to_report_1090[],
                             uint16_t num_packets_to_report) 
{
    for (uint16_t i = 0; i < num_packets_to_report; i++) 
    {
        char raw_frame_buf[kRaw1090FrameMaxNumChars];
        uint16_t num_bytes_in_frame = BuildRaw1090Frame(packets_to_report_1090[i].GetRaw(), raw_frame_buf);
        SendBuf(iface, (char *)raw_frame_buf, num_bytes_in_frame);
        comms_manager.iface_puts(iface, (char *)"\r\n");  // Send delimiter.
    }
    return true;
}

bool CommsManager::ReportBeast(SettingsManager::SerialInterface iface, const Decoded1090Packet packets_to_report_1090[],
                               uint16_t num_packets_to_report) {
    for (uint16_t i = 0; i < num_packets_to_report; i++) 
    {
        uint8_t beast_frame_buf[kBeastFrameMaxLenBytes];
        uint16_t num_bytes_in_frame = Build1090BeastFrame(packets_to_report_1090[i], beast_frame_buf);
        comms_manager.iface_putc(iface, char(0x1a));  // Send beast escape char to denote beginning of frame.
        SendBuf(iface, (char *)beast_frame_buf, num_bytes_in_frame);
    }
    return true;
}

bool CommsManager::ReportCSBee(SettingsManager::SerialInterface iface)
{
    // Write out a CSBee Aircraft message for each aircraft in the aircraft dictionary.
    for (auto &itr : adsbee.aircraft_dictionary.dict) 
    {
        const Aircraft1090 &aircraft = itr.second;

        char message[kCSBeeMessageStrMaxLen];
        int16_t message_len_bytes = WriteCSBeeAircraftMessageStr(message, aircraft);
        if (message_len_bytes < 0) {
            //CONSOLE_ERROR("CommsManager::ReportCSBee",
            //              "Encountered an error in WriteCSBeeAircraftMessageStr, error code %d.", message_len_bytes);
            return false;
        }
        SendBuf(iface, message, message_len_bytes);
    }

    //// Write a CSBee Statistics message.
    char message[kCSBeeMessageStrMaxLen];
    int16_t message_len_bytes =
        WriteCSBeeStatisticsMessageStr(message,                                                 // Buffer to write into.
                                       adsbee.aircraft_dictionary.metrics.demods_1090,          // DPS
                                       adsbee.aircraft_dictionary.metrics.raw_squitter_frames,  // RAW_SFPS
                                       adsbee.aircraft_dictionary.metrics.valid_squitter_frames,           // SFPS
                                       adsbee.aircraft_dictionary.metrics.raw_extended_squitter_frames,    // RAW_ESFPS
                                       adsbee.aircraft_dictionary.metrics.valid_extended_squitter_frames,  // ESFPS
                                       adsbee.aircraft_dictionary.GetNumAircraft(),  // NUM_AIRCRAFT
                                       0u,                                           // TSCAL
                                       get_time_since_boot_ms() / 1000               // UPTIME
        );
    if (message_len_bytes < 0) 
    {
        //CONSOLE_ERROR("CommsManager::ReportCSBee",
        //              "Encountered an error in WriteCSBeeStatisticsMessageStr, error code %d.", message_len_bytes);
        return false;
    }
    SendBuf(iface, message, message_len_bytes);
    return true;
}
