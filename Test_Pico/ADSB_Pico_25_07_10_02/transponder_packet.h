#ifndef _ADSB_PACKET_HH_
#define _ADSB_PACKET_HH_

#include <cstdint>

#include "buffer_utils.h"
#include "unit_conversions.h"

// Useful resource: https://mode-s.org/decode/content/ads-b/1-basics.html

class Raw1090Packet {
   public:
    static const uint16_t kMaxPacketLenWords32 = 4;
    static const uint16_t kSquitterPacketLenBits = 56;
    static const uint16_t kSquitterPacketNumWords32 = 2;  // 56 bits = 1.75 words, round up to 2.
    static const uint16_t kExtendedSquitterPacketLenBits = 112;
    static const uint16_t kExtendedSquitterPacketNumWords32 = 4;  // 112 bits = 3.5 words, round up to 4.

    Raw1090Packet(char *rx_string, int16_t source_in = -1, int16_t sigs_dbm_in = INT16_MIN,
                  int16_t sigq_db_in = INT16_MIN, uint64_t mlat_48mhz_64bit_counts = 0);

    Raw1090Packet(uint32_t rx_buffer[kMaxPacketLenWords32], uint16_t rx_buffer_len_words32, int16_t source_in = -1,
                  int16_t sigs_dbm_in = INT16_MIN, int16_t sigq_db_in = INT16_MIN,
                  uint64_t mlat_48mhz_64bit_counts = 0);
    /**
     * Default constructor.
     */
    Raw1090Packet() 
    {
        for (uint16_t i = 0; i < kMaxPacketLenWords32; i++) 
        {
            buffer[i] = 0;
        }
    }

    /**
    * Вспомогательная функция, возвращающая временную метку в секундах. Используется для абстрагирования разрешения временной метки MLAT для
    * функций, которым это не нужно.
    * @return Временная метка в секундах.
    */
    uint64_t GetTimestampMs() { return mlat_48mhz_64bit_counts / 48'000; }

    /**
    * Вывести содержимое буфера в строку.
    * @param[in] buf Буфер для печати.
    * @param[in] buf_len_bytes Длина буфера в символах.
    * @return Количество символов, записанных в буфер.
    */
    uint16_t PrintBuffer(char *buf, uint16_t buf_len_bytes) const;

    uint32_t buffer[kMaxPacketLenWords32] = {0};
    uint16_t buffer_len_bits = 0;
    int8_t source = -1;                    // Источник пакета ADS-B (номер конечного автомата PIO).
    int16_t sigs_dbm = INT16_MIN;          // Уровень сигнала, в дБм.
    int16_t sigq_db = INT16_MIN;           // Качество сигнала (дБ выше уровня шума), в дБ.
    uint64_t mlat_48mhz_64bit_counts = 0;  // Счетчик MLAT высокого разрешения.
};

class Decoded1090Packet {
   public:
    static const uint16_t kMaxPacketLenWords32 = Raw1090Packet::kMaxPacketLenWords32;
    static const uint16_t kDFNUmBits = 5;     // [1-5] Downlink Format bitlength.
    static const uint16_t kMaxDFStrLen = 50;  // Max length of TypeCode string.
    static const uint16_t kDebugStrLen = 200;

    // Bits 1-5: Downlink Format (DF)
    enum DownlinkFormat {
        kDownlinkFormatInvalid = -1,
        // DF 0-11 = short messages (56 bits)
        kDownlinkFormatShortRangeAirToAirSurveillance = 0,  // All-call request from ground station.
        kDownlinkFormatAltitudeReply = 4,
        kDownlinkFormatIdentityReply = 5,
        kDownlinkFormatAllCallReply = 11,
        // DF 16-24 = long messages (112 bits)
        kDownlinkFormatLongRangeAirToAirSurveillance = 16,   // ACAS/TCAS message.
        kDownlinkFormatExtendedSquitter = 17,                // Aircraft (taxiing or airborne).
        kDownlinkFormatExtendedSquitterNonTransponder = 18,  // Surface vehicle or aircraft in special ground operation.
        kDownlinkFormatMilitaryExtendedSquitter = 19,        // Used by military aircraft, protocol is not public.
        // We don't currently do anything with CommB, CommC, CommD.
        kDownlinkFormatCommBAltitudeReply = 20,  // Data being sent as reply to request from Comm-B ground station.
        kDownlinkFormatCommBIdentityReply = 21,  // Reply to ground station via Comm-C (higher capacity than Comm-B).
        kDownlinkFormatCommDExtendedLengthMessage = 24
        // DF 1-3, 6-10, 11-15, 22-23 not used
    };

    // Bits 6-8 in DF=11 and DF=17-18: Transponder Capability (CA)
    enum Capability : uint8_t {
        kCALevel1Transponder = 0,
        // CA = 1-3 reserved for backwards compatibility.
        kCALevel2PlusTransponderOnSurfaceCanSetCA7 = 4,
        kCALevel2PlusTransponderAirborneCanSetCA7 = 5,
        kCALevel2PlusTransponderOnSurfaceOrAirborneCanSetCA7 = 6,
        kCADRNot0OrFSEquals2345OnSurfaceOrAirborne = 7  // Indicates aircraft is under enhanced surveillance by ATC.
    };

    // Конструкторы
    /**
    * Конструктор декодированного пакета Decoded1090Packet.
    * @param[in] rx_buffer Буфер для чтения. Должен быть упакован таким образом, чтобы все 32 бита каждого слова были заполнены, а
    * каждое слово слева (MSb) было выровнено так, чтобы общее количество битов составляло 112. Слова должны быть в порядке от старшего к младшему (big endian), причем MSb
    * первого слова является самым старшим битом.
    * @param[in] rx_buffer_len_words32 Количество 32-битных слов для чтения из rx_buffer.
    * @param[in] sigs_dbm RSSI принятого пакета в дБм. По умолчанию INT32_MIN, если не установлено.
    * @param[in] mlat_48mhz_64bit_counts Количество тактовых импульсов 12 МГц, используемых для 6-байтовой временной метки мультилатерации.
    */
    Decoded1090Packet(uint32_t rx_buffer[kMaxPacketLenWords32], uint16_t rx_buffer_len, int16_t source = -1,
                      int32_t sigs_dbm = INT32_MIN, int32_t sigq_db = INT32_MIN, uint64_t mlat_48mhz_64bit_counts = 0);
    /**
    * Конструктор декодированного пакета 1090Packet из строки.
    * @param[in] rx_string Строка полубайтов в шестнадцатеричном формате. Порядок байтов от старшего к старшему (MSB) — первый.
    * @param[in] sigs_dbm RSSI принятого пакета в дБм. По умолчанию INT32_MIN, если не установлено.
    * @param[in] mlat_12mhz_counts Количество импульсов 12 МГц, используемых для 6-байтовой временной метки мультилатерации.
    */
    Decoded1090Packet(char *rx_string, int16_t source = -1, int32_t sigs_dbm = INT32_MIN, int32_t sigq_db = INT32_MIN,
                      uint64_t mlat_48mhz_64bit_counts = 0);
   
    /**
    * Конструктор Decoded1090Packet из Raw1090Packet. Использует неявный конструктор Raw1090Packet.
    * @param[in] packet_in Raw1090Packet для использования при создании Decoded1090Packet.
    */
    Decoded1090Packet(const Raw1090Packet &packet_in);

    /**
     * Default constructor.
     */
    Decoded1090Packet() : raw_((char *)"") { debug_string[0] = '\0'; };

    bool IsValid() const { return is_valid_; };

    /**
    * Принудительно устанавливает валидность пакета. Используется для маркировки 56-битных (сквиттерных) пакетов как валидных после их
    * внешней проверки по словарю бортового компьютера, поскольку их CRC не равен 0, и поэтому они считаются невалидными
    * при создании.
    */  
    
    void ForceValid() { is_valid_ = true; }

    int GetBufferLenBits() const { return raw_.buffer_len_bits; }
    int GetRSSIdBm() const { return raw_.sigs_dbm; }
    uint64_t GetMLAT12MHzCounter() const { return (raw_.mlat_48mhz_64bit_counts >> 2) & 0xFFFFFFFFFFFF; }
    uint64_t GetTimestampMs() { return raw_.GetTimestampMs(); }
    uint16_t GetDownlinkFormat() const { return downlink_format_; }
    //!!uint16_t GetDownlinkFormatString(char str_buf[kMaxDFStrLen]) const;
    DownlinkFormat GetDownlinkFormatEnum();
    uint32_t GetICAOAddress() const { return icao_address_; }
    uint16_t GetPacketBufferLenBits() const { return raw_.buffer_len_bits; }
    Raw1090Packet GetRaw() const { return raw_; }
    Raw1090Packet *GetRawPtr() { return &raw_; }
    /**
    * Выводит содержимое внутреннего буфера пакетов в целевой буфер и возвращает количество записанных байтов.
    * @param[in] to_buffer Буфер назначения, должен иметь длину kMaxPacketLenWords32 или больше.
    * @retval Количество байтов, записанных в буфер назначения.
    */
    uint16_t DumpPacketBuffer(uint32_t to_buffer[kMaxPacketLenWords32]) const;
    uint16_t DumpPacketBuffer(uint8_t to_buffer[kMaxPacketLenWords32 * kBytesPerWord]) const;

    // Exposed for testing only.
    uint32_t Get24BitWordFromPacketBuffer(uint16_t first_bit_index) const {
        return GetNBitWordFromBuffer(24, first_bit_index, raw_.buffer);
    };
    /**
    * Вычисляет 24-битную контрольную сумму CRC пакета ADS-B и возвращает её значение. Возвращаемое
    * значение должно совпадать с последними 24 битами 112-битного пакета ADS-B, если пакет корректен.
    * @retval Контрольная сумма CRC.
    */
    uint32_t CalculateCRC24(uint16_t packet_len_bits = Raw1090Packet::kExtendedSquitterPacketLenBits) const;

    char debug_string[kDebugStrLen] = "";

   protected:
    bool is_valid_ = false;
    Raw1090Packet raw_;

    uint32_t icao_address_ = 0;
    uint16_t downlink_format_ = static_cast<uint16_t>(kDownlinkFormatInvalid);

    uint32_t parity_interrogator_id = 0;

   private:
    void ConstructTransponderPacket();
};

class ADSBPacket : public Decoded1090Packet {
   public:
    static const uint16_t kMaxTCStrLen = 50;

    // Bitlengths of each field in the ADS-B frame. See Table 3.1 in The 1090MHz Riddle (Junzi Sun) pg. 35.
    static const uint16_t kCANumBits = 3;     // [6-8] Capability bitlength.
    static const uint16_t kICAONumBits = 24;  // [9-32] ICAO Address bitlength.
    static const uint16_t kMENumBits = 56;    // [33-88] Extended Squitter Message bitlength.
    static const uint16_t kTCNumBits = 5;     // [33-37] Type code bitlength. Not always included.
    static const uint16_t kPINumBits = 24;    // Parity / Interrogator ID bitlength.

    static const uint16_t kMEFirstBitIndex = kDFNUmBits + kCANumBits + kICAONumBits;

    /**
     * Constructor. Can only create an ADSBPacket from an existing Decoded1090Packet, which is is referenced as
     * the parent of the ADSBPacket. Think of this as a way to use the ADSBPacket as a "window" into the contents of the
     * parent Decoded1090Packet. The ADSBPacket cannot exist without the parent Decoded1090Packet!
     */
    ADSBPacket(const Decoded1090Packet &decoded_packet) : Decoded1090Packet(decoded_packet) { ConstructADSBPacket(); };

    // Bits 6-8 [3]: Capability (CA)
    // Bits 9-32 [24]: ICAO Aircraft Address (ICAO)
    // Bits 33-88 [56]: Message, Extended Squitter (ME)
    // (Bits 33-37 [5]): Type code (TC)
    enum TypeCode : uint8_t {
        kTypeCodeInvalid = 0,
        kTypeCodeAircraftID = 1,                 // 1–4     Aircraft identification
        kTypeCodeSurfacePosition = 5,            // 5–8     Surface position
        kTypeCodeAirbornePositionBaroAlt = 9,    // 9–18	Airborne position (w/Baro Altitude)
        kTypeCodeAirborneVelocities = 19,        // 19      Airborne velocities
        kTypeCodeAirbornePositionGNSSAlt = 20,   // 20–22	Airborne position (w/GNSS Height)
        kTypeCodeReserved = 23,                  // 23–27	Reserved
        kTypeCodeAircraftStatus = 28,            // 28      Aircraft status
        kTypeCodeTargetStateAndStatusInfo = 29,  // 29      Target state and status information
        kTypeCodeAircraftOperationStatus = 31    // 31      Aircraft operation status
    };
    // Bits 89-112 [24]: Parity / Interrogator ID (PI)

    // Subtype enums used for specific packet types (not instantiated as part of the ADSBPacket class).
    // Airborne Velocitites (TC = 19)
    enum AirborneVelocitiesSubtype : uint8_t {
        kAirborneVelocitiesGroundSpeedSubsonic = 1,
        kAirborneVelocitiesGroundSpeedSupersonic = 2,
        kAirborneVelocitiesAirspeedSubsonic = 3,
        kAirborneVelocitiesAirspeedSupersonic = 4
    };

    // Operation Status (TC = 31)
    enum OperationStatusSubtype : uint8_t { kOperationStatusSubtypeAirborne = 0, kOperationStatusSubtypeSurface = 1 };

    inline Capability GetCapability() const { return capability_; };
    inline TypeCode GetTypeCode() const { return typecode_; };
    TypeCode GetTypeCodeEnum() const;

    // Exposed for testing only.
    inline uint32_t GetNBitWordFromMessage(uint16_t n, uint16_t first_bit_index) const {
        return GetNBitWordFromBuffer(n, kMEFirstBitIndex + first_bit_index, raw_.buffer);
    };

   private:
    Capability capability_ = kCALevel1Transponder;  // Default to most basic capability.

    TypeCode typecode_ = kTypeCodeInvalid;

    void ConstructADSBPacket();
};

class AllCallReplyPacket : public Decoded1090Packet {
   public:
    AllCallReplyPacket(const Decoded1090Packet &decoded_packet) : Decoded1090Packet(decoded_packet) {
        capability_ = static_cast<Capability>(GetNBitWordFromBuffer(3, 5, raw_.buffer));
    }

    Capability GetCapability() const { return capability_; }

   private:
    Capability capability_ = kCALevel1Transponder;  // Default to most basic capability.
};

class AltitudeReplyPacket : public Decoded1090Packet {
   public:
    enum DownlinkRequest : uint8_t {
        kDownlinkRequestNone = 0b00000,
        kDownlinkRequestSendCommBMessage = 0b00001,
        kDownlinkRequestCommBBroadcastMessage1Available = 0b00100,
        kDownlinkRequestCommBBroadcastMessage2Available = 0b00101
    };

    enum UtilityMessageType : uint8_t {
        kUtilityMessageNoInformation = 0b00,
        kUtilityMessageCommBInterrogatorIdentifierCode = 0b10,
        kUtilityMessageCommCInterrogatorIdentifierCode = 0b10,
        kUtilityMessageCommDInterrogatorIdentifierCode = 0b11
    };

    AltitudeReplyPacket(const Decoded1090Packet &decoded_packet);

    bool IsAirborne() const { return is_airborne_; }
    bool HasAlert() const { return has_alert_; }
    bool HasIdent() const { return has_ident_; }
    DownlinkRequest GetDownlinkRequest() const { return downlink_request_; }
    uint8_t GetUtilityMessage() const { return utility_message_; }
    int32_t GetAltitudeFt() const { return altitude_ft_; }

   private:
    bool is_airborne_ = false;
    bool has_alert_ = false;
    bool has_ident_ = false;

    DownlinkRequest downlink_request_ = kDownlinkRequestNone;
    uint8_t utility_message_ = 0b0;
    UtilityMessageType utility_message_type_ = kUtilityMessageNoInformation;

    int32_t altitude_ft_ = -1;
};

class IdentityReplyPacket : public Decoded1090Packet {
   public:
    enum DownlinkRequest : uint8_t {
        kDownlinkRequestNone = 0b00000,
        kDownlinkRequestSendCommBMessage = 0b00001,
        kDownlinkRequestCommBBroadcastMessage1Available = 0b00100,
        kDownlinkRequestCommBBroadcastMessage2Available = 0b00101
    };

    enum UtilityMessageType : uint8_t {
        kUtilityMessageNoInformation = 0b00,
        kUtilityMessageCommBInterrogatorIdentifierCode = 0b10,
        kUtilityMessageCommCInterrogatorIdentifierCode = 0b10,
        kUtilityMessageCommDInterrogatorIdentifierCode = 0b11
    };

    IdentityReplyPacket(const Decoded1090Packet &decoded_packet);

    bool IsAirborne() const { return is_airborne_; }
    bool HasAlert() const { return has_alert_; }
    bool HasIdent() const { return has_ident_; }
    DownlinkRequest GetDownlinkRequest() const { return downlink_request_; }
    uint8_t GetUtilityMessage() const { return utility_message_; }
    uint16_t GetSquawk() const { return squawk_; }

   private:
    bool is_airborne_ = false;
    bool has_alert_ = false;
    bool has_ident_ = false;

    DownlinkRequest downlink_request_ = kDownlinkRequestNone;
    uint8_t utility_message_ = 0b0;
    UtilityMessageType utility_message_type_ = kUtilityMessageNoInformation;

    uint16_t squawk_ = 0xFFFF;
};

#endif /* _ADSB_PACKET_HH_ */