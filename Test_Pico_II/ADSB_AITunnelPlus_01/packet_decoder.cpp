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
                comms_manager.console_printf("PacketDecoder1..  %s \r\n", message.message);
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
                .message = "Failed to pop raw packet from input queue.",  //Не удалось извлечь необработанный пакет из входной очереди.
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

        // Добавить содержимое пакета к отладочному сообщению.
        uint16_t message_len = strlen(decode_debug_message.message);
        message_len += snprintf(decode_debug_message.message + message_len, DebugMessage::kMessageMaxLen - message_len, "df=%02d icao=0x%06x 0x", decoded_packet.GetDownlinkFormat(), decoded_packet.GetICAOAddress());
        // Добавить распечатку содержимого пакета.
        raw_packet.PrintBuffer(decode_debug_message.message + message_len, DebugMessage::kMessageMaxLen - message_len);
        debug_message_out_queue.Push(decode_debug_message);
   
    }

    return true;
}