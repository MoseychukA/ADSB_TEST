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
                //!!comms_manager.console_printf("PacketDecoder1..  %s \r\n", message.message);
                break;
            case SettingsManager::LogLevel::kWarnings:
                comms_manager.console_printf("PacketDecoder2..  %s \r\n", message.message);
                break;
            case SettingsManager::LogLevel::kErrors:
                comms_manager.console_printf("PacketDecoderp3.. %s \r\n", message.message);
                break;
            default:
                break;  // Don't do anything when logs are silent.
        }
    }
    uint16_t bit_flip_index;
    while (decoded_1090_packet_bit_flip_locations_out_queue.Pop(bit_flip_index))
    {
        //!!comms_manager.console_printf("PacketDecoder::DecoderLoop4.. Corrected single bit error at bit index %d.\r\n", bit_flip_index);
    }
    return true;
}

//==============================================================================

bool PacketDecoder::UpdateDecoderLoop() 
{
    uint16_t num_packets_to_process = raw_1090_packet_in_queue.Length();
    if (num_packets_to_process == 0) 
    {
        return true;  // Nothing to do.
    }

    for (uint16_t i = 0; i < num_packets_to_process; i++) 
    {
        Raw1090Packet raw_packet;
        if (!raw_1090_packet_in_queue.Pop(raw_packet)) 
        {
            debug_message_out_queue.Push(DebugMessage{
                .message = "Failed to pop raw packet from input queue.",
                .log_level = SettingsManager::LogLevel::kErrors,
                });
            return false;
        }

        Decoded1090Packet decoded_packet = Decoded1090Packet(raw_packet);
        DebugMessage decode_debug_message = DebugMessage
        {
            .message = "",
            .log_level = SettingsManager::LogLevel::kInfo,
        };

        if (decoded_packet.IsValid()) 
        {
            decoded_1090_packet_out_queue.Push(decoded_packet);

            strncpy(decode_debug_message.message, "[VALID     ] ", DebugMessage::kMessageMaxLen);
        }
        else if (config_.enable_1090_error_correction && decoded_packet.GetBufferLenBits() == Raw1090Packet::kExtendedSquitterPacketLenBits) 
        {
            // Checksum correction is enabled, and we have a packet worth correcting.
            Raw1090Packet* raw_packet_ptr = decoded_packet.GetRawPtr();
            uint16_t packet_len_bytes = raw_packet_ptr->buffer_len_bits / kBitsPerByte;
            uint8_t raw_buffer[packet_len_bytes];
            WordBufferToByteBuffer(raw_packet_ptr->buffer, raw_buffer, packet_len_bytes);
            int16_t bit_flip_index = crc24_find_single_bit_error(crc24_syndrome(raw_buffer, packet_len_bytes),
                raw_packet_ptr->buffer_len_bits);
            if (bit_flip_index > 0) 
            {
                // Found a single bit error: flip it and push the corrected packet to the output queue.
                flip_bit(raw_packet_ptr->buffer, bit_flip_index);
                decoded_1090_packet_bit_flip_locations_out_queue.Push(bit_flip_index);
                decoded_1090_packet_out_queue.Push(Decoded1090Packet(*raw_packet_ptr));

                strncpy(decode_debug_message.message, "[1FIXD     ] ", DebugMessage::kMessageMaxLen);
            }
            else 
            {
                // Checksum correction failed.
                strncpy(decode_debug_message.message, "[     NOFIX] ", DebugMessage::kMessageMaxLen);
            }
        }
        else
        {
            // Invalid and not worth correcting.
            strncpy(decode_debug_message.message, "[     INVLD] ", DebugMessage::kMessageMaxLen);
        }

        // Append packet contents to debug message.
        uint16_t message_len = strlen(decode_debug_message.message);
        message_len += snprintf(decode_debug_message.message + message_len, DebugMessage::kMessageMaxLen - message_len,
            "df=%02d icao=0x%06x 0x", decoded_packet.GetDownlinkFormat(), decoded_packet.GetICAOAddress());
        // Append a print of the packet contents.
        raw_packet.PrintBuffer(decode_debug_message.message + message_len, DebugMessage::kMessageMaxLen - message_len);
        debug_message_out_queue.Push(decode_debug_message);
    }

    return true;
}



//==============================================================================

//bool PacketDecoder::UpdateDecoderLoop() 
//{
//    uint16_t num_packets_to_process = raw_1090_packet_in_queue.Length(); // Проверить, были пакеты или нет
//    if (num_packets_to_process == 0)                                     // Пакетов нет
//    {
//        return true;  // Nothing to do.
//    }
// 
//    for (uint16_t i = 0; i < num_packets_to_process; i++) 
//    {
//        Raw1090Packet raw_packet;
// 
//        if (!raw_1090_packet_in_queue.Pop(raw_packet)) 
//        {
//            debug_message_out_queue.Push(DebugMessage
//                {
//                .message = "",//"Failed to pop raw packet from input queue.",  //Не удалось извлечь необработанный пакет из входной очереди. 
//                .log_level = SettingsManager::LogLevel::kErrors,
//            });
//            return false;
//        }
// 
//        Decoded1090Packet decoded_packet = Decoded1090Packet(raw_packet); //
//        DebugMessage decode_debug_message = DebugMessage
//        {
//            .message = "",
//            .log_level = SettingsManager::LogLevel::kInfo,
//        };
//        if (decoded_packet.IsValid())  // Когда выставлен флаг?
//        {
//            decoded_1090_packet_out_queue.Push(decoded_packet);  // 
//
//           //!! strncpy(decode_debug_message.message, "[VALID ] ", DebugMessage::kMessageMaxLen); 
//           // strncpy(decode_debug_message.message, "", DebugMessage::kMessageMaxLen);
//        } 
//        else if (config_.enable_1090_error_correction && decoded_packet.GetBufferLenBits() == Raw1090Packet::kExtendedSquitterPacketLenBits) 
//        {
//            // Коррекция контрольной суммы включена, и у нас есть пакет, который стоит исправить.
//            Raw1090Packet* raw_packet_ptr = decoded_packet.GetRawPtr();
//            uint16_t packet_len_bytes = raw_packet_ptr->buffer_len_bits / kBitsPerByte;
//            uint8_t raw_buffer[packet_len_bytes];
//            WordBufferToByteBuffer(raw_packet_ptr->buffer, raw_buffer, packet_len_bytes);
//            int16_t bit_flip_index = crc24_find_single_bit_error(crc24_syndrome(raw_buffer, packet_len_bytes), raw_packet_ptr->buffer_len_bits);
//            if (bit_flip_index > 0) 
//            {
//                // Обнаружена ошибка в одном бите: переворачиваем ее и помещаем исправленный пакет в выходную очередь.
//                flip_bit(raw_packet_ptr->buffer, bit_flip_index);
//                decoded_1090_packet_bit_flip_locations_out_queue.Push(bit_flip_index);
//                decoded_1090_packet_out_queue.Push(Decoded1090Packet(*raw_packet_ptr));
//
//              //!! strncpy(decode_debug_message.message, "[1FIXD ] ", DebugMessage::kMessageMaxLen);
//            }
//            else 
//            {
//                // Ошибка исправления контрольной суммы.
//             //!! strncpy(decode_debug_message.message, "[ NOFIX] ", DebugMessage::kMessageMaxLen);
//            }
//        }
//        else 
//        {
//            // Неверно и не подлежит исправлению.
//            //!!strncpy(decode_debug_message.message, "[ INVLD] ", DebugMessage::kMessageMaxLen);
//        }
//
//        // Добавить содержимое пакета к отладочному сообщению.
//        uint16_t message_len = strlen(decode_debug_message.message);
//        //!! message_len += snprintf(decode_debug_message.message + message_len, DebugMessage::kMessageMaxLen - message_len, "df=%02d icao=0x%06x 0x", decoded_packet.GetDownlinkFormat(), decoded_packet.GetICAOAddress());
//        // Добавить распечатку содержимого пакета.
//       //!! raw_packet.PrintBuffer(decode_debug_message.message + message_len, DebugMessage::kMessageMaxLen - message_len);
//       //!! debug_message_out_queue.Push(decode_debug_message);
//        //!! comms_manager.console_printf("UpdateDecoderLoop..  %s \r\n", decode_debug_message.message);
//        //  comms_manager.console_printf("%s \r\n", decode_debug_message.message);
//
//        // Добавляем новые пакеты в словарь.
//        //  Raw1090Packet raw_packet;
//        //  Decoded1090Packet decoded_packet;
//
//        while (decoded_1090_packet_out_queue.Pop(decoded_packet) /*raw_1090_packet_queue.Pop(raw_packet)*/)
//        {
//            if (raw_packet.buffer_len_bits == Raw1090Packet::kExtendedSquitterPacketLenBits)  // 112;
//            {
///*                   comms_manager.console_printf("ADSBee::Update New message: 0x%08x|%08x|%08x|%04x SRC=%d SIGS=%ddBm SIGQ=%ddB MLAT=%u\r\n",
//                                raw_packet.buffer[0], raw_packet.buffer[1], raw_packet.buffer[2],
//                                (raw_packet.buffer[3]) >> (4 * kBitsPerNibble), raw_packet.source, raw_packet.sigs_dbm, 
//                                raw_packet.sigq_db, raw_packet.mlat_48mhz_64bit_counts);*/
//
//                    //comms_manager.console_printf("0x%08x%08x%08x%04x\r\n",
//                    //            raw_packet.buffer[0], raw_packet.buffer[1], raw_packet.buffer[2],
//                    //            (raw_packet.buffer[3]) >> (4 * kBitsPerNibble));
//
//
//              
//                    decode_debug_message.message[201U] = { '\0' };
//
//                    uint16_t message_len1 = strlen(decode_debug_message.message);
//                    //message_len += snprintf("0x%08x%08x%08x%04x\r\n", raw_packet.buffer[0], raw_packet.buffer[1], raw_packet.buffer[2],
//                    //    (raw_packet.buffer[3]) >> (4 * kBitsPerNibble));
//                    raw_packet.PrintBuffer(decode_debug_message.message + message_len1, DebugMessage::kMessageMaxLen - message_len1);
//                    comms_manager.console_printf("%s \r\n", decode_debug_message.message);
//
//                    //uint8_t packet_data[16];
//
//                    //// Преобразуем в массив
//                    //int packet_length = hexStringToBytes(decode_debug_message.message, packet_data, 16);
//                    ////comms_manager.console_printf("%08x%08x%08x \r\n", packet_data);
//
//                    //  // Проверка CRC
//                    //if (validate_crc(packet_data))
//                    //{
//                    //    comms_manager.console_printf("CRC: ✓ Valid\r\n");
//                    //}
//                    //else
//                    //{
//                    //    comms_manager.console_printf("CRC: ✗ Invalid\r\n");
//                    //}
//
//                    //if (validate_crc(packet_data)) 
//                    //{
//                    //    // Декодирование пакета
//                    //    uint32_t icao = decode_icao(packet_data);
//                    //    uint8_t msg_type = decode_message_type(packet_data);
//
//                    //    char callsign[9];
//                    //    decode_callsign(packet_data, callsign);
//
//                    //    double latitude = 0, longitude = 0;
//                    //    decode_position(packet_data, &latitude, &longitude);
//
//                    //    uint16_t velocity = decode_velocity(packet_data);
//                    //    uint16_t altitude = decode_altitude(packet_data);
//
//                    //    // Формируем JSON для UART2
//                    //    String json = "{";
//                    //    json += "\"icao\":\"" + String(icao, HEX) + "\",";
//                    //    json += "\"type\":" + String(msg_type) + ",";
//                    //    json += "\"callsign\":\"" + String(callsign) + "\",";
//                    //    json += "\"lat\":" + String(latitude, 6) + ",";
//                    //    json += "\"lon\":" + String(longitude, 6) + ",";
//                    //    json += "\"alt\":" + String(altitude) + ",";
//                    //    json += "\"speed\":" + String(velocity) + ",";
//                    //    //json += "\"channel\":" + String(packet.channel) + ",";
//                    //    //json += "\"corrected\":" + String(corrected ? "true" : "false");
//                    //    json += "}\n";
//
//                    //    // Отправляем в UART2
//                    //   //!! comms_manager.console_printf("%s",json);
//
//                    //    // Отправляем в Serial для отладки
//                    //    comms_manager.console_printf("Decoded - ICAO: %x \r\n", icao);
//                    //    comms_manager.console_printf("Callsign: %s\r\n", callsign);
//                    //    comms_manager.console_printf("Lat: , Lon: %6f %6f\r\n", latitude, longitude);
//                    //    comms_manager.console_printf("Alt: %d\r\n", altitude);
//                    //    comms_manager.console_printf("Speed: %d\r\n", velocity);
//                    // }
//                    //else 
//                    //{
//                    //    comms_manager.console_printf("CRC failed, packet discarded\r\n");
//                    //}
//                    //comms_manager.console_printf("========================\n");
//
//                   
//
//                    /*
//                    uint32_t buffer[kMaxPacketLenWords32] = {0};
//                    uint16_t buffer_len_bits = 0;
//                    int8_t source = -1;                    // Источник пакета ADS-B (номер конечного автомата PIO).
//                    int16_t sigs_dbm = INT16_MIN;          // Уровень сигнала, в дБм.
//                    int16_t sigq_db = INT16_MIN;           // Качество сигнала (дБ выше уровня шума), в дБ.
//                    uint64_t mlat_48mhz_64bit_counts = 0;  // Счетчик MLAT высокого разрешения.
//                    */
//
//            }
//            else
//            {
//                //comms_manager.console_printf("+++Update New message: 0x%08x|%06x SRC=%d SIGS=%ddBm SIGQ=%ddB MLAT=%u\r\n",
//                //             raw_packet.buffer[0], (raw_packet.buffer[1]) >> (2 * kBitsPerNibble), raw_packet.source,
//                //             raw_packet.sigs_dbm, raw_packet.sigq_db, raw_packet.mlat_48mhz_64bit_counts);
//
//            }
//
//           
//
//            Decoded1090Packet decoded_packet = Decoded1090Packet(raw_packet);
//
//            // comms_manager.console_printf("Update\tdf=%d icao_address=0x%06x\r\n", decoded_packet.GetDownlinkFormat(), decoded_packet.GetICAOAddress());
//                // CONSOLE_INFO("ADSBee::Update", "\tdf=%d icao_address=0x%06x", decoded_packet.GetDownlinkFormat(), decoded_packet.GetICAOAddress());
//
//            if (aircraft_dictionary.IngestDecoded1090Packet(decoded_packet))
//            {
//                // Пакет был использован для обновления словаря или был молча проигнорирован (но предположительно действителен).
//                // FlashStatusLED();  // Мигнуть светодиодом
//                // ПРИМЕЧАНИЕ: Отправка в очередь отчетов здесь означает, что мы будем сообщать только о проверенных пакетах!
//                comms_manager.transponder_packet_reporting_queue.Push(decoded_packet);
//               // comms_manager.console_printf("Update\t aircraft_dictionary: %d\t aircraft\r\n", aircraft_dictionary.GetNumAircraft());
//            }
//            comms_manager.transponder_packet_reporting_queue.Push(decoded_packet);
//        }
//    }
//
//    return true;
//}


// Функция для преобразования HEX символа в число
uint8_t PacketDecoder::hexCharToValue(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    else if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    else if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return 0xFF;  // Ошибка
}

// Улучшенная функция преобразования
int PacketDecoder::hexStringToBytes(/*const */char* hex_string, uint8_t* output, int max_output_size)
{
    int len = strlen(hex_string);

    if (len % 2 != 0 || len / 2 > max_output_size)
    {
        return -1;  // Ошибка
    }

    int byte_count = 0;

    for (int i = 0; i < len; i += 2) {
        uint8_t high = hexCharToValue(hex_string[i]);
        uint8_t low = hexCharToValue(hex_string[i + 1]);

        if (high == 0xFF || low == 0xFF) {
            return -1;  // Недопустимый символ
        }

        output[byte_count] = (high << 4) | low;
        byte_count++;
    }

    if (byte_count > 0) {
        //comms_manager.console_printf("Converted ");
        //comms_manager.console_printf("%d",byte_count);
        //comms_manager.console_printf(" bytes:");

        for (int i = 0; i < byte_count; i++) {
            if (output[i] < 16) comms_manager.console_printf("0");
            comms_manager.console_printf("%x",output[i]);
           // comms_manager.console_printf(" ");
        }
        comms_manager.console_printf("\r\n\r\n");
    }



    return byte_count;
}



//-------------------------------------------------------------
// Функции для работы с CRC
uint32_t PacketDecoder::crc24(uint8_t* data, int len) {
    uint32_t crc = 0;
    uint32_t generator = 0x1FFF409; // ADS-B CRC24 полином

    for (int i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i] << 16;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x800000) {
                crc = (crc << 1) ^ generator;
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc & 0xFFFFFF;
}

bool PacketDecoder::validate_crc(uint8_t* packet) {
    uint32_t received_crc = (packet[11] << 16) | (packet[12] << 8) | packet[13];
    uint32_t calculated_crc = crc24(packet, 11);
    return received_crc == calculated_crc;
}

//-----------------------------------------------------------------------

// Декодирование ICAO адреса
uint32_t PacketDecoder::decode_icao(uint8_t* data)
{
    return (data[1] << 16) | (data[2] << 8) | data[3];
}

// Декодирование типа сообщения
uint8_t PacketDecoder::decode_message_type(uint8_t* data)
{
    return data[0] >> 3;
}

// Декодирование позиции
void PacketDecoder::decode_position(uint8_t* data, double* latitude, double* longitude)
{
    uint8_t msg_type = decode_message_type(data);

    if (msg_type >= 9 && msg_type <= 22) {  // Airborne position
        uint32_t raw_lat = ((data[6] & 0x03) << 15) | (data[7] << 7) | (data[8] >> 1);
        uint32_t raw_lon = ((data[8] & 0x01) << 16) | (data[9] << 8) | data[10];

        // Упрощенное декодирование (требует CPR декодирование для точности)
        //latitude = (raw_lat / 131072.0) /* 90.0*/;
        //longitude = (raw_lon / 131072.0)  /*180.0*/;

        //if (latitude > 90) latitude -= 180;
        //if (longitude > 180) longitude -= 360;
    }
}

// Декодирование скорости
uint16_t PacketDecoder::decode_velocity(uint8_t* data)
{
    uint8_t msg_type = decode_message_type(data);

    if (msg_type == 19) {  // Velocity message
        uint16_t ew_vel = ((data[5] & 0x03) << 8) | data[6];
        uint16_t ns_vel = ((data[7] & 0x7F) << 3) | (data[8] >> 5);

        return sqrt(ew_vel/*  ew_vel*/ + ns_vel  /*ns_vel*/);
    }

    return 0;
}

// Декодирование высоты
uint16_t PacketDecoder::decode_altitude(uint8_t* data)
{
    uint8_t msg_type = decode_message_type(data);

    if (msg_type >= 9 && msg_type <= 22) {
        uint16_t alt_code = ((data[5] & 0x1F) << 7) | (data[6] >> 1);

        if (alt_code == 0) return 0;

        // Q-бит проверка
        if (data[5] & 0x01) {
            return (alt_code - 1) * 25;  // футы
        }
        else {
            // Gillham кодирование (упрощено)
            return alt_code * 100;
        }
    }

    return 0;
}

//// Декодирование номера рейса
//void PacketDecoder::decode_callsign(uint8_t* data, char* callsign)
//{
//    uint8_t msg_type = decode_message_type(data);
//
//    if (msg_type >= 1 && msg_type <= 4) {  // Identification message
//        const char charset[] = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######";
//
//        for (int i = 0; i < 8; i++) {
//            uint8_t char_code;
//
//            switch (i) {
//            case 0: char_code = (data[5] >> 2) & 0x3F; break;
//            case 1: char_code = ((data[5] & 0x03) << 4) | (data[6] >> 4); break;
//            case 2: char_code = ((data[6] & 0x0F) << 2) | (data[7] >> 6); break;
//            case 3: char_code = data[7] & 0x3F; break;
//            case 4: char_code = (data[8] >> 2) & 0x3F; break;
//            case 5: char_code = ((data[8] & 0x03) << 4) | (data[9] >> 4); break;
//            case 6: char_code = ((data[9] & 0x0F) << 2) | (data[10] >> 6); break;
//            case 7: char_code = data[10] & 0x3F; break;
//            default: char_code = 0; break;
//            }
//
//            callsign[i] = charset[char_code];
//        }
//        callsign[8] = '\0';
//
//        // Удаляем завершающие пробелы
//        for (int i = 7; i >= 0; i--) {
//            if (callsign[i] == '_') {
//                callsign[i] = '\0';
//            }
//            else {
//                break;
//            }
//        }
//    }
//    else 
//    {
//        strcpy(callsign, "N/A");
//    }
//}

uint8_t PacketDecoder::extract_6bits(const uint8_t* data, int bit_offset) 
{
    int byte_idx = bit_offset / 8;
    int bit_idx = bit_offset % 8;

    if (bit_idx <= 2) {
        return (data[byte_idx] >> (2 - bit_idx)) & 0x3F;
    }
    else {
        return ((data[byte_idx] << (bit_idx - 2)) |
            (data[byte_idx + 1] >> (10 - bit_idx))) & 0x3F;
    }
}




void PacketDecoder::decode_callsign(uint8_t* data, char* callsign)
{
    if (!data || !callsign) 
    {
        if (callsign) strcpy(callsign, "N/A");
        return;
    }

    uint8_t msg_type = decode_message_type(data);
    comms_manager.console_printf("msg_type = %d\r\n", msg_type);

    if (msg_type >= 1 && msg_type <= 4)
    {
        static constexpr const char charset[] =  "#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######";

        // Извлекаем 8 символов по 6 бит каждый, начиная с бита 40
        for (int i = 0; i < 8; i++) 
        {
            uint8_t char_code = extract_6bits(data, 40 + i * 6);
            callsign[i] = charset[char_code];
        }
        callsign[8] = '\0';

        // Удаляем trailing пробелы
        for (int i = 7; i >= 0 && callsign[i] == '_'; i--) 
        {
            callsign[i] = '\0';
        }
    }
    else 
    {
        strcpy(callsign, "N/A...");
    }
}
