#include "packet_decoder.h"
#include "comms.h"
#include "crc.h"

bool PacketDecoder::UpdateLogLoop() 
{
    uint16_t num_messages = debug_message_out_queue.Length();

    for (uint16_t i = 0; i < num_messages; i++) 
    {
        DebugMessage message;
        debug_message_out_queue.Pop(message);
        switch (message.log_level) 
        {
            case SettingsManager::LogLevel::kInfo:
               //!! comms_manager.console_printf("PacketDecoder1..  %s \r\n", message.message);

                break;
            case SettingsManager::LogLevel::kWarnings:
                comms_manager.console_printf("PacketDecoder2..  ", "%s \r\n", message.message);
                break;
            case SettingsManager::LogLevel::kErrors:
                comms_manager.console_printf("PacketDecoderp3..  " , "%s \r\n", message.message);
                break;
            default:
                break;  // Don't do anything when logs are silent.
        }
    }
    uint16_t bit_flip_index;
    while (decoded_1090_packet_bit_flip_locations_out_queue.Pop(bit_flip_index))
    {
       // comms_manager.console_printf("PacketDecoder::DecoderLoop4.. ", "Corrected single bit error at bit index %d.", bit_flip_index);
    }
    return true;
}


bool PacketDecoder::UpdateDecoderLoop() 
{
    uint16_t num_packets_to_process = raw_1090_packet_in_queue.Length(); // Проверить, были пакеты или нет
    if (num_packets_to_process == 0)                                     // Пакетов нет
    {
        return true;  // Nothing to do.
    }
 
    for (uint16_t i = 0; i < num_packets_to_process; i++) 
    {
        Raw1090Packet raw_packet;
 
        if (!raw_1090_packet_in_queue.Pop(raw_packet)) 
        {
            debug_message_out_queue.Push(DebugMessage
                {
                .message = "",//"Failed to pop raw packet from input queue.",  //Не удалось извлечь необработанный пакет из входной очереди. 
                .log_level = SettingsManager::LogLevel::kErrors,
            });
            return false;
        }
 
        Decoded1090Packet decoded_packet = Decoded1090Packet(raw_packet); //
        DebugMessage decode_debug_message = DebugMessage
        {
            .message = "",
            .log_level = SettingsManager::LogLevel::kInfo,
        };
        if (decoded_packet.IsValid())  // Когда выставлен флаг?
        {
            decoded_1090_packet_out_queue.Push(decoded_packet);  // 

            strncpy(decode_debug_message.message, "[VALID ] ", DebugMessage::kMessageMaxLen); 
           // strncpy(decode_debug_message.message, "", DebugMessage::kMessageMaxLen);
        } 
        else if (config_.enable_1090_error_correction && decoded_packet.GetBufferLenBits() == Raw1090Packet::kExtendedSquitterPacketLenBits) 
        {
            // Коррекция контрольной суммы включена, и у нас есть пакет, который стоит исправить.
            Raw1090Packet* raw_packet_ptr = decoded_packet.GetRawPtr();
            uint16_t packet_len_bytes = raw_packet_ptr->buffer_len_bits / kBitsPerByte;
            uint8_t raw_buffer[packet_len_bytes];
            WordBufferToByteBuffer(raw_packet_ptr->buffer, raw_buffer, packet_len_bytes);
            int16_t bit_flip_index = crc24_find_single_bit_error(crc24_syndrome(raw_buffer, packet_len_bytes), raw_packet_ptr->buffer_len_bits);
            if (bit_flip_index > 0) 
            {
                // Обнаружена ошибка в одном бите: переворачиваем ее и помещаем исправленный пакет в выходную очередь.
                flip_bit(raw_packet_ptr->buffer, bit_flip_index);
                decoded_1090_packet_bit_flip_locations_out_queue.Push(bit_flip_index);
                decoded_1090_packet_out_queue.Push(Decoded1090Packet(*raw_packet_ptr));

               strncpy(decode_debug_message.message, "[1FIXD ] ", DebugMessage::kMessageMaxLen);
            }
            else 
            {
                // Ошибка исправления контрольной суммы.
              strncpy(decode_debug_message.message, "[ NOFIX] ", DebugMessage::kMessageMaxLen);
            }
        }
        else 
        {
            // Неверно и не подлежит исправлению.
            strncpy(decode_debug_message.message, "[ INVLD] ", DebugMessage::kMessageMaxLen);
        }

        //// Добавить содержимое пакета к отладочному сообщению.
        //if (decoded_packet.GetDownlinkFormat() == 17)
        //{
            uint16_t message_len = strlen(decode_debug_message.message);
           //!! message_len += snprintf(decode_debug_message.message + message_len, DebugMessage::kMessageMaxLen - message_len, "df=%02d icao=0x%06x 0x", decoded_packet.GetDownlinkFormat(), decoded_packet.GetICAOAddress());
            // Добавить распечатку содержимого пакета.
            raw_packet.PrintBuffer(decode_debug_message.message + message_len, DebugMessage::kMessageMaxLen - message_len);
            debug_message_out_queue.Push(decode_debug_message);
           //!! comms_manager.console_printf("UpdateDecoderLoop..  %s \r\n", decode_debug_message.message);
          //  comms_manager.console_printf("%s \r\n", decode_debug_message.message);

            // Добавляем новые пакеты в словарь.
          //  Raw1090Packet raw_packet;
          //  Decoded1090Packet decoded_packet;

            while (decoded_1090_packet_out_queue.Pop(decoded_packet) /*raw_1090_packet_queue.Pop(raw_packet)*/)
            {
               //!! comms_manager.console_printf("!!!raw_packet.buffer_len_bits %d \r\n", raw_packet.buffer_len_bits);
                if (raw_packet.buffer_len_bits == Raw1090Packet::kExtendedSquitterPacketLenBits)  // 112;
                {
 /*                   comms_manager.console_printf("ADSBee::Update New message: 0x%08x|%08x|%08x|%04x SRC=%d SIGS=%ddBm SIGQ=%ddB MLAT=%u\r\n",
                                 raw_packet.buffer[0], raw_packet.buffer[1], raw_packet.buffer[2],
                                 (raw_packet.buffer[3]) >> (4 * kBitsPerNibble), raw_packet.source, raw_packet.sigs_dbm, 
                                 raw_packet.sigq_db, raw_packet.mlat_48mhz_64bit_counts);*/

                    //CONSOLE_INFO("ADSBee::Update", "New message: 0x%08x|%08x|%08x|%04x SRC=%d SIGS=%ddBm SIGQ=%ddB MLAT=%u",
                    //    raw_packet.buffer[0], raw_packet.buffer[1], raw_packet.buffer[2],
                    //    (raw_packet.buffer[3]) >> (4 * kBitsPerNibble), raw_packet.source, raw_packet.sigs_dbm,
                    //    raw_packet.sigq_db, raw_packet.mlat_48mhz_64bit_counts);

                     //!!comms_manager.console_printf("0x%08x%08x%08x%04x\r\n",
                     //            raw_packet.buffer[0], raw_packet.buffer[1], raw_packet.buffer[2],
                     //            (raw_packet.buffer[3]) >> (4 * kBitsPerNibble));

                     /*
                        uint32_t buffer[kMaxPacketLenWords32] = {0};
                        uint16_t buffer_len_bits = 0;
                        int8_t source = -1;                    // Источник пакета ADS-B (номер конечного автомата PIO).
                        int16_t sigs_dbm = INT16_MIN;          // Уровень сигнала, в дБм.
                        int16_t sigq_db = INT16_MIN;           // Качество сигнала (дБ выше уровня шума), в дБ.
                        uint64_t mlat_48mhz_64bit_counts = 0;  // Счетчик MLAT высокого разрешения.
                     */

                }
                else
                {
                    //comms_manager.console_printf("+++Update New message: 0x%08x|%06x SRC=%d SIGS=%ddBm SIGQ=%ddB MLAT=%u\r\n",
                    //             raw_packet.buffer[0], (raw_packet.buffer[1]) >> (2 * kBitsPerNibble), raw_packet.source,
                    //             raw_packet.sigs_dbm, raw_packet.sigq_db, raw_packet.mlat_48mhz_64bit_counts);

                    //CONSOLE_INFO("ADSBee::Update", "New message: 0x%08x|%06x SRC=%d SIGS=%ddBm SIGQ=%ddB MLAT=%u",
                    //    raw_packet.buffer[0], (raw_packet.buffer[1]) >> (2 * kBitsPerNibble), raw_packet.source,
                    //    raw_packet.sigs_dbm, raw_packet.sigq_db, raw_packet.mlat_48mhz_64bit_counts);
                }

                Decoded1090Packet decoded_packet = Decoded1090Packet(raw_packet);

                // comms_manager.console_printf("Update\tdf=%d icao_address=0x%06x\r\n", decoded_packet.GetDownlinkFormat(), decoded_packet.GetICAOAddress());
                 // CONSOLE_INFO("ADSBee::Update", "\tdf=%d icao_address=0x%06x", decoded_packet.GetDownlinkFormat(), decoded_packet.GetICAOAddress());

                //if (aircraft_dictionary.IngestDecoded1090Packet(decoded_packet))
                //{
                //    // Пакет был использован для обновления словаря или был молча проигнорирован (но предположительно действителен).
                //    // FlashStatusLED();  // Мигнуть светодиодом
                //    // ПРИМЕЧАНИЕ: Отправка в очередь отчетов здесь означает, что мы будем сообщать только о проверенных пакетах!
                //    comms_manager.transponder_packet_reporting_queue.Push(decoded_packet);
                //    comms_manager.console_printf("Update\t aircraft_dictionary: %d\t aircraft\r\n", aircraft_dictionary.GetNumAircraft());
                //}
                comms_manager.transponder_packet_reporting_queue.Push(decoded_packet);
            }
       // }
    }

    return true;
}