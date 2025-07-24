#ifndef PACKET_DECODER_H_
#define PACKET_DECODER_H_

#include "data_structures.h"
#include "settings.h"
#include "transponder_packet.h"
#include "aircraft_dictionary.h"

class PacketDecoder 
{
   public:
    static constexpr uint16_t kPacketQueueLen = 100;       // Длина очереди пакетов
    static constexpr uint16_t kDebugMessageQueueLen = 100;//!! 20;  // Длина очереди сообщений отладки

    AircraftDictionary aircraft_dictionary;

    struct PacketDecoderConfig 
    {
        bool enable_1090_error_correction = true;
        bool max_1090_error_correction_num_bits = 1;
    };

    /**
    * Структура, используемая для отправки сообщений отладки в основное ядро, поскольку PacketDecoder предназначен для работы на нескольких ядрах.
     */
    struct DebugMessage
    {
        static const uint16_t kMessageMaxLen = 200;
        char message[kMessageMaxLen + 1] = {'\0'};
        SettingsManager::LogLevel log_level = SettingsManager::LogLevel::kInfo; 
    };

    PacketDecoder(PacketDecoderConfig config_in) : config_(config_in) {};

    bool Init() { return true; }
    /**
    * Вывести все отладочные сообщения, сгенерированные UpdateDecoderLoop(). Следует запускать из основного цикла, чтобы избежать
    * смешивания отладочных сообщений от декодера с отладочными сообщениями из основного цикла.
    */
    bool UpdateLogLoop();

    /**
    * Считывание RawTransponderPackets, попытка исправления контрольной суммы и передача их в выходную очередь как
    * DecodedTransponderPackets.
    * @retval True в случае успеха, false, если что-то сломалось.
    */
    bool UpdateDecoderLoop();

    uint8_t hexCharToValue(char c);
    int hexStringToBytes(/*const */char* hex_string, uint8_t* output, int max_output_size);
    uint32_t crc24(uint8_t* data, int len);
    bool validate_crc(uint8_t* packet);
    uint32_t decode_icao(uint8_t* data);
    uint8_t decode_message_type(uint8_t* data);
    void decode_position(uint8_t* data, double* latitude, double* longitude);
    uint16_t decode_velocity(uint8_t* data);
    uint16_t decode_altitude(uint8_t* data);
    uint8_t extract_6bits(const uint8_t* data, int bit_offset);
    void decode_callsign(uint8_t* data, char* callsign);
    
   
    


    // Входные очереди.  необработанный пакет 1090 в очереди
    PFBQueue<Raw1090Packet> raw_1090_packet_in_queue = PFBQueue<Raw1090Packet>({.buf_len_num_elements = kPacketQueueLen, .buffer = raw_1090_packet_in_queue_buffer_});

    // Выходные очереди.
    PFBQueue<Decoded1090Packet> decoded_1090_packet_out_queue = PFBQueue<Decoded1090Packet>({.buf_len_num_elements = kPacketQueueLen, .buffer = decoded_1090_packet_out_queue_buffer_});

    PFBQueue<uint16_t> decoded_1090_packet_bit_flip_locations_out_queue = PFBQueue<uint16_t>(
        {.buf_len_num_elements = kPacketQueueLen, .buffer = decoded_1090_packet_bit_flip_locations_out_queue_buffer_});
    PFBQueue<DebugMessage> debug_message_out_queue = PFBQueue<DebugMessage>({
        .buf_len_num_elements = kDebugMessageQueueLen,
        .buffer = debug_message_out_queue_buffer_,
    });

   private:
    PacketDecoderConfig config_;
    Raw1090Packet raw_1090_packet_in_queue_buffer_[kPacketQueueLen];
    Decoded1090Packet decoded_1090_packet_out_queue_buffer_[kPacketQueueLen];
    uint16_t decoded_1090_packet_bit_flip_locations_out_queue_buffer_[kPacketQueueLen];
    DebugMessage debug_message_out_queue_buffer_[kDebugMessageQueueLen];
};

extern PacketDecoder decoder;

#endif /* PACKET_DECODER_HH_ */