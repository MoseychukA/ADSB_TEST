#include "transponder_packet.h"
#include <stdio.h>
#include <stdlib.h> // ��� �������� �� ������� ����
#include <cstdint>
#include <cstdio>   // for snprintf
#include <cstring>  // for strlen
//#include "comms.h"  // For debug prints.
#include "decode_utils.h"

#define BYTES_PER_WORD_32 4
#define BITS_PER_WORD_32  32
#define BYTES_PER_WORD_24 3
#define BITS_PER_WORD_24  24
#define BITS_PER_WORD_25  25
#define BITS_PER_BYTE     8
#define NIBBLES_PER_BYTE  2
#define BITS_PER_NIBBLE   4

#define MASK_MSBIT_WORD24 (0b1 << (BITS_PER_WORD_24 - 1))
#define MASK_MSBIT_WORD25 (0b1 << BITS_PER_WORD_24)
#define MASK_WORD24       0xFFFFFF

#define CHAR_TO_HEX(c)    ((c >= 'A') ? (c >= 'a') ? (c - 'a' + 10) : (c - 'A' + 10) : (c - '0'))

const uint32_t kExtendedSquitterLastWordIngestionMask = 0xFFFF0000;
const uint32_t kExtendedSquitterLastWordPopCount = 16;
const uint32_t kSquitterLastWordIngestionMask = 0xFFFFFF00;
const uint32_t kSquitterLastWordPopCount = 24;

#define CRC24_USE_TABLE
#ifndef CRC24_USE_TABLE
const uint32_t kCRC24Generator = 0x1FFF409;
const uint16_t kCRC24GeneratorNumBits = 25;
#else
  #include "crc.h"
#endif

/** Decoded1090Packet **/

Raw1090Packet::Raw1090Packet(uint32_t rx_buffer[kMaxPacketLenWords32], uint16_t rx_buffer_len_words32,
                             int16_t source_in, int16_t sigs_dbm_in, int16_t sigq_db_in,
                             uint64_t mlat_48mhz_64bit_counts_in)
    : source(source_in),
      sigs_dbm(sigs_dbm_in),
      sigq_db(sigq_db_in),
      mlat_48mhz_64bit_counts(mlat_48mhz_64bit_counts_in) 
{
    // // ������������� ��������� ��������� ���������� ����� �� ������ ����� ������.
    uint32_t last_word_ingestion_mask, last_word_popcount;
    if (rx_buffer_len_words32 > 2) 
    {
        // 112-������ ����� (����������� ��������)
        last_word_ingestion_mask = kExtendedSquitterLastWordIngestionMask;
        last_word_popcount = kExtendedSquitterLastWordPopCount;
    }
    else 
    {
        // 56-������ ����� (��������)
        last_word_ingestion_mask = kSquitterLastWordIngestionMask;
        last_word_popcount = kSquitterLastWordPopCount;
    }

    // Pack the packet buffer.
    for (uint16_t i = 0; i < rx_buffer_len_words32 && i < kMaxPacketLenWords32; i++) 
    {
        if (i == rx_buffer_len_words32 - 1) 
        {
            buffer[i] = rx_buffer[i] & last_word_ingestion_mask;  // Trim any crap off of last word.
            buffer_len_bits += last_word_popcount;
        }
        else 
        {
            buffer[i] = rx_buffer[i];
            buffer_len_bits += BITS_PER_WORD_32;
        }
    }


}

Raw1090Packet::Raw1090Packet(char *rx_string, int16_t source_in, int16_t sigs_dbm_in, int16_t sigq_db_in,
                             uint64_t mlat_48mhz_64bit_counts_in)
    : source(source_in),
      sigs_dbm(sigs_dbm_in),
      sigq_db(sigq_db_in),
      mlat_48mhz_64bit_counts(mlat_48mhz_64bit_counts_in) 
{
    uint16_t rx_num_bytes = strlen(rx_string) / NIBBLES_PER_BYTE;
    for (uint16_t i = 0; i < rx_num_bytes && i < kMaxPacketLenWords32 * BYTES_PER_WORD_32; i++) 
    {
        uint8_t byte = (CHAR_TO_HEX(rx_string[i * NIBBLES_PER_BYTE]) << BITS_PER_NIBBLE) |
                       CHAR_TO_HEX(rx_string[i * NIBBLES_PER_BYTE + 1]);
        uint16_t byte_offset = i % BYTES_PER_WORD_32;  // number of Bytes to shift right from MSB of current word
        if (byte_offset == 0) 
        {
            buffer[i / BYTES_PER_WORD_32] = byte << (3 * BITS_PER_BYTE);  // need to clear out the word
        }
        else 
        {
            buffer[i / BYTES_PER_WORD_32] |= byte << ((3 - byte_offset) * BITS_PER_BYTE);
        }
        buffer_len_bits += BITS_PER_BYTE;
    }
}

uint16_t Raw1090Packet::PrintBuffer(char *buf, uint16_t buf_len_bytes) const {
    uint16_t len = 0;
    switch (buffer_len_bits) 
    {
        case kSquitterPacketLenBits:
            len = snprintf(buf, buf_len_bytes, "%08lX%06lX", buffer[0], buffer[1] >> (2 * kBitsPerNibble));
            break;
        case kExtendedSquitterPacketLenBits:
            len = snprintf(buf, buf_len_bytes, "%08lX%08lX%08lX%04lX", buffer[0], buffer[1], buffer[2],
                           buffer[3] >> (4 * kBitsPerNibble));
            break;
            // ������ �� ��������, ���� ����� ����� ������������ �����.
    }
    return len;
}

Decoded1090Packet::Decoded1090Packet(uint32_t rx_buffer[kMaxPacketLenWords32], uint16_t rx_buffer_len_words32,
                                     int16_t source, int32_t sigs_dbm, int32_t sigq_db,
                                     uint64_t mlat_48mhz_64bit_counts)
    : raw_(rx_buffer, rx_buffer_len_words32, source, sigs_dbm, sigq_db, mlat_48mhz_64bit_counts) 
{
    ConstructTransponderPacket();
}

Decoded1090Packet::Decoded1090Packet(char *rx_string, int16_t source, int32_t sigs_dbm, int32_t sigq_db,
                                     uint64_t mlat_48mhz_64bit_counts)
    : raw_(rx_string, source, sigs_dbm, sigq_db, mlat_48mhz_64bit_counts) {
    ConstructTransponderPacket();
}

Decoded1090Packet::Decoded1090Packet(const Raw1090Packet &packet_in) : raw_(packet_in) { ConstructTransponderPacket(); }

Decoded1090Packet::DownlinkFormat Decoded1090Packet::GetDownlinkFormatEnum() 
{
    switch (downlink_format_) 
    {
        // DF 0-11 = short messages (56 bits)
        case DownlinkFormat::kDownlinkFormatShortRangeAirToAirSurveillance:
        case DownlinkFormat::kDownlinkFormatAltitudeReply:
        case DownlinkFormat::kDownlinkFormatIdentityReply:
        case DownlinkFormat::kDownlinkFormatAllCallReply:
        // DF 16-24 = long messages (112 bits)
        case DownlinkFormat::kDownlinkFormatLongRangeAirToAirSurveillance:
        case DownlinkFormat::kDownlinkFormatExtendedSquitter:
        case DownlinkFormat::kDownlinkFormatExtendedSquitterNonTransponder:
        case DownlinkFormat::kDownlinkFormatMilitaryExtendedSquitter:
        case DownlinkFormat::kDownlinkFormatCommBAltitudeReply:
        case DownlinkFormat::kDownlinkFormatCommBIdentityReply:
        case DownlinkFormat::kDownlinkFormatCommDExtendedLengthMessage:
            // DF 1-3, 6-10, 11-15, 22-23 not used
            return static_cast<DownlinkFormat>(downlink_format_);
            break;
        default:
            return DownlinkFormat::kDownlinkFormatInvalid;
    }
    // Bits 1-5: Downlink Format (DF)
    enum DownlinkFormat 
    {
        kDownlinkFormatInvalid = -1,
        // DF 0-11 = short messages (56 bits)
        kDownlinkFormatShortRangeAirToAirSurveillance = 0,
        kDownlinkFormatAltitudeReply = 4,
        kDownlinkFormatIdentityReply = 5,
        DF_ALL_CALL_REPLY = 11,
        // DF 16-24 = long messages (112 bits)
        DF_LONG_RANGE_AIR_SURVEILLANCE = 16,
        DF_EXTENDED_SQUITTER = 17,
        DF_EXTENDED_SQUITTER_NON_TRANSPONDER = 18,
        DF_MILITARY_EXTENDED_SQUITTER = 19,
        DF_COMM_B_ALTITUDE_REPLY = 20,
        DF_COMM_B_IDENTITY_REPLY = 21,
        DF_COMM_D_EXTENDED_LENGTH_MESSAGE = 24
        // DF 1-3, 6-10, 11-15, 22-23 not used
    };
}

uint16_t Decoded1090Packet::DumpPacketBuffer(uint32_t to_buffer[kMaxPacketLenWords32]) const {
    uint16_t bytes_written = raw_.buffer_len_bits / BITS_PER_BYTE;
    for (uint16_t i = 0; i < kMaxPacketLenWords32; i++) 
    {
        to_buffer[i] = raw_.buffer[i];
    }
    return bytes_written;
}

uint16_t Decoded1090Packet::DumpPacketBuffer(uint8_t to_buffer[kMaxPacketLenWords32 * kBytesPerWord]) const {
    uint16_t bytes_written = raw_.buffer_len_bits / BITS_PER_BYTE;
    for (uint16_t i = 0; i < kMaxPacketLenWords32; i++) 
    {
        // ������ ���������� ��� � ������� ���.
        to_buffer[i * kBytesPerWord] = raw_.buffer[i] >> 24;
        to_buffer[i * kBytesPerWord + 1] = (raw_.buffer[i] >> 16) & 0xFF;
        to_buffer[i * kBytesPerWord + 2] = (raw_.buffer[i] >> 8) & 0xFF;
        to_buffer[i * kBytesPerWord + 3] = raw_.buffer[i] & 0xFF;
    }
    return bytes_written;
}

uint32_t Decoded1090Packet::CalculateCRC24(uint16_t packet_len_bits) const 
{
#ifndef CRC24_USE_TABLE
    // CRC calculation algorithm from https://mode-s.org/decode/book-the_1090mhz_riddle-junzi_sun.pdf pg. 91.
    // Must be called on buffer that does not have extra bit ingested at end and has all words left-aligned.
    uint32_t crc_buffer[kMaxPacketLenWords32];
    for (uint16_t i = 0; i < kMaxPacketLenWords32; i++) 
    {
        crc_buffer[i] = raw_.buffer[i];
    }

    // Overwrite 24-bit parity word with zeros.
    SetNBitWordInBuffer(BITS_PER_WORD_24, 0x0, packet_len_bits - BITS_PER_WORD_24, crc_buffer);

    // CRC is a conditional convolve operation using the 25-bit generator word.
    for (uint16_t i = 0; i < packet_len_bits - BITS_PER_WORD_24; i++) 
    {
        uint32_t word = GetNBitWordFromBuffer(BITS_PER_WORD_25, i, crc_buffer);
        if (word & MASK_MSBIT_WORD25) 
        {
            // Most significant bit is a 1. XOR with generator!
            SetNBitWordInBuffer(BITS_PER_WORD_25, word ^ kCRC24Generator, i, crc_buffer);
        }
    }

    return GetNBitWordFromBuffer(BITS_PER_WORD_24, packet_len_bits - BITS_PER_WORD_24, crc_buffer);
#else
    // �������������� 32-������ ����� ������ ���� � �������� �����.
    uint16_t packet_len_bytes = packet_len_bits / kBitsPerByte;
    uint8_t raw_buffer[packet_len_bytes];

    WordBufferToByteBuffer(raw_.buffer, raw_buffer, packet_len_bytes);

   //!! comms_manager.console_printf("raw_.buffer %06x \r\n", raw_.buffer);
    // �������� ����� ������ � ��������� ����������� CRC.


    return crc24(raw_buffer, packet_len_bytes - 3);  // �� ��������� ��� CRC.


   // comms_manager.console_printf("packet_len_bytes %d raw_buffer %06x \r\n", packet_len_bytes, raw_buffer);

#endif
}

void Decoded1090Packet::ConstructTransponderPacket() 
{
    if (raw_.buffer_len_bits != Raw1090Packet::kExtendedSquitterPacketLenBits &&   raw_.buffer_len_bits != Raw1090Packet::kSquitterPacketLenBits) // �� ����� 112 ��� 56
    {
        snprintf( debug_string, kDebugStrLen, "Bit number mismatch while decoding packet. Expected %d or %d but got %d!\r\n",
            Raw1090Packet::kExtendedSquitterPacketLenBits, Raw1090Packet::kSquitterPacketLenBits, raw_.buffer_len_bits);

        return;  // �������� is_valid_ ��� false
    }

    downlink_format_ = raw_.buffer[0] >> 27;  // �������� DF �� ������
   // comms_manager.console_printf("downlink_format 27 %d \r\n", raw_.buffer[0] >> 27);
   // comms_manager.console_printf("icao_address %06x \r\n", Get24BitWordFromBuffer(8, raw_.buffer));

    
    uint32_t calculated_checksum = CalculateCRC24(raw_.buffer_len_bits);
    uint32_t parity_value = Get24BitWordFromBuffer(raw_.buffer_len_bits - BITS_PER_WORD_24, raw_.buffer);

  //!!  comms_manager.console_printf("raw_.buffer[0] %d\r\n", raw_.buffer[0]); // 
  //!! comms_manager.console_printf("raw_.buffer_len_bits %d\r\n", raw_.buffer_len_bits); // ����� ������
  // comms_manager.console_printf("calculated_checksum %x\r\n", calculated_checksum); //!!
  //  comms_manager.console_printf("parity_value %x \r\n", parity_value);              //����������� ����� CRC � ����� ������

    //comms_manager.console_printf("icao_address_ %x \r\n", parity_value ^ calculated_checksum);

    switch (static_cast<DownlinkFormat>(downlink_format_)) 
    {
        case kDownlinkFormatShortRangeAirToAirSurveillance:  // DF = 0
        case kDownlinkFormatAltitudeReply:                   // DF = 4
        case kDownlinkFormatIdentityReply:                   // DF = 5
        {
            // Process a 56-bit message.
            is_valid_ = false;  // ����������� ����������� ����� ������������ � ������� ICAO � ������� �������� XOR. ��. ����������� �� ������������� ADS-B, ���. 22.
            // ����� ���� � ��� �������� ��������� �������, ��� ���������� ����������� � ������� ������������ �������.
            icao_address_ = parity_value ^ calculated_checksum;  // ��������� ����� ������� � ��� ������, ���� �������� ����������
            break;
        }
        case kDownlinkFormatAllCallReply:  // DF = 11
        {
            icao_address_ = Get24BitWordFromBuffer(8, raw_.buffer);
            uint16_t interrogator_id = parity_value ^ calculated_checksum;
            if (interrogator_id == 0) 
            {
                // ����� �� ���������� ������ �� ��������� ������.
                is_valid_ = true;
            }
            else 
            {
                // �� ���� �������������� �����������, ������� �� ���� �������, ������������ �� ��.
                is_valid_ = false;
            }
            break;
        }

        default:  // ��� ��������� DF. ����������: DF=17-19 ��� ADS-B.
        {
            // Process a 112-bit message.
            icao_address_ = raw_.buffer[0] & 0xFFFFFF;

           //!! comms_manager.console_printf("icao_address_ %x\r\n", icao_address_);
            if (/*downlink_format_ = 17*/calculated_checksum == parity_value)
            {
               //!! comms_manager.console_printf("is_valid_ = true\r\n");
                is_valid_ = true;  // �������� ����� ��� ��������������, ���� CRC ������������� ����� ��������
            }
            else 
            {
                // is_valid_ is set to false by default
               //!! snprintf(debug_string, kDebugStrLen, "Invalid checksum, expected %06lx but calculated %06lx.\r\n", parity_value, calculated_checksum);
              //!! comms_manager.console_printf("!!!Invalid checksum, parity_value calculated %06lx %06lx.\r\n", parity_value, calculated_checksum);
            }
        }
    }
}

/** ADSBPacket **/

/**
 * Helper function used by constructors.
 */
void ADSBPacket::ConstructADSBPacket() 
{
    capability_ = static_cast<ADSBPacket::Capability>((raw_.buffer[0] >> 24) & 0b111);
    typecode_ = static_cast<ADSBPacket::TypeCode>(raw_.buffer[1] >> 27);
    parity_interrogator_id = raw_.buffer[1] & 0xFFFFFF;
}

ADSBPacket::TypeCode ADSBPacket::GetTypeCodeEnum() const {
    // Table 3.3 from The 1090Mhz Riddle (Junzi Sun), pg. 37.
    switch (static_cast<uint16_t>(typecode_)) 
    {
        case 1:
        case 2:
        case 3:
        case 4:
            return kTypeCodeAircraftID;
            break;
        case 5:
        case 6:
        case 7:
        case 8:
            return kTypeCodeSurfacePosition;
            break;
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 18:
            return kTypeCodeAirbornePositionBaroAlt;
            break;
        case 19:
            return kTypeCodeAirborneVelocities;
            break;
        case 20:
        case 21:
        case 22:
            return kTypeCodeAirbornePositionGNSSAlt;
            break;
        case 23:
        case 24:
        case 25:
        case 26:
        case 27:
            return kTypeCodeReserved;
            break;
        case 28:
            return kTypeCodeAircraftStatus;
            break;
        case 29:
            return kTypeCodeTargetStateAndStatusInfo;
            break;
        case 31:
            return kTypeCodeAircraftOperationStatus;
            break;
        default:
            return kTypeCodeInvalid;
    }
}

AltitudeReplyPacket::AltitudeReplyPacket(const Decoded1090Packet &decoded_packet) : Decoded1090Packet(decoded_packet) 
{
    uint8_t flight_status = GetNBitWordFromBuffer(3, 5, raw_.buffer);  // FS = Bits 5-7.
    switch (flight_status) 
    {
        case 0b000:  // No alert, no SPI, aircraft is airborne.
            has_alert_ = false;
            has_ident_ = false;
            is_airborne_ = true;
            break;
        case 0b001:  // No alert, no SPI, aircraft is on ground.
            has_alert_ = false;
            has_ident_ = false;
            is_airborne_ = false;
            break;
        case 0b010:  // Alert, no SPI, aircraft is airborne.
            has_alert_ = true;
            has_ident_ = false;
            is_airborne_ = true;
            break;
        case 0b011:  // Alert, no SPI, aircraft is on ground.
            has_alert_ = true;
            has_ident_ = false;
            is_airborne_ = false;
            break;
        case 0b100:  // Alert, SPI, aircraft is airborne or on ground.
            has_alert_ = true;
            has_ident_ = true;
            // Default is_airborne_ to false when not known.
            break;
        case 0b101:
            // No alert, SPI, aircaft is airborne or on ground.
            has_ident_ = true;
            has_alert_ = false;
            // Default is_airborne_ to false when not known.
            break;
        case 0b110:  // Reserved.
            break;
        case 0b111:  // Not assigned.
            break;
    }

    downlink_request_ = static_cast<DownlinkRequest>(GetNBitWordFromBuffer(5, 8, raw_.buffer));
    utility_message_ = GetNBitWordFromBuffer(4, 13, raw_.buffer);
    utility_message_type_ = static_cast<UtilityMessageType>(GetNBitWordFromBuffer(2, 17, raw_.buffer));
   // altitude_ft_ = AltitudeCodeToAltitudeFt(GetNBitWordFromBuffer(13, 19, raw_.buffer));
};

IdentityReplyPacket::IdentityReplyPacket(const Decoded1090Packet &decoded_packet) : Decoded1090Packet(decoded_packet) 
{
    uint8_t flight_status = GetNBitWordFromBuffer(3, 5, raw_.buffer);  // FS = Bits 5-7.
    switch (flight_status) {
        case 0b000:  // No alert, no SPI, aircraft is airborne.
            has_alert_ = false;
            has_ident_ = false;
            is_airborne_ = true;
            break;
        case 0b001:  // No alert, no SPI, aircraft is on ground.
            has_alert_ = false;
            has_ident_ = false;
            is_airborne_ = false;
            break;
        case 0b010:  // Alert, no SPI, aircraft is airborne.
            has_alert_ = true;
            has_ident_ = false;
            is_airborne_ = true;
            break;
        case 0b011:  // Alert, no SPI, aircraft is on ground.
            has_alert_ = true;
            has_ident_ = false;
            is_airborne_ = false;
            break;
        case 0b100:  // Alert, SPI, aircraft is airborne or on ground.
            has_alert_ = true;
            has_ident_ = true;
            // Default is_airborne_ to false when not known.
            break;
        case 0b101:
            // No alert, SPI, aircaft is airborne or on ground.
            has_ident_ = true;
            has_alert_ = false;
            // Default is_airborne_ to false when not known.
            break;
        case 0b110:  // Reserved.
            break;
        case 0b111:  // Not assigned.
            break;
    }

    downlink_request_ = static_cast<DownlinkRequest>(GetNBitWordFromBuffer(5, 8, raw_.buffer));
    utility_message_ = GetNBitWordFromBuffer(4, 13, raw_.buffer);
    utility_message_type_ = static_cast<UtilityMessageType>(GetNBitWordFromBuffer(2, 17, raw_.buffer));
   // squawk_ = IdentityCodeToSquawk(GetNBitWordFromBuffer(13, 19, raw_.buffer));
};




#include <stdio.h>
#include <stdlib.h>
#include <string.h> //-strlen()
#include <stdint.h> //uint32_t

//------------  passing VALUE instead of POINTER to function(), it's safer,but cost time.!!!!!! -----------------
//---  return 24bits CRC in uint32_t 
uint32_t Decoded1090Packet::adsb_crc_88bits(uint32_t* bin32_divid) // for 88bits message
{
    //--- bin32_divid[0-2]: message data, totally will be 88-bits 

    uint32_t tbin32_divid[3];
    int j;
    uint32_t crc32_GP = 0xFFFA0480;//0x04C11DB7; // ������� ���������� CRC ��� �������� � ���� ����� ������� ������ ���, ����� ��� ����������� CRC ������ ����������.

    //---�������� ������ ��������� �� ��������� ������, ����� ����������� �������� �� ��������� �������� ������
    tbin32_divid[0] = bin32_divid[0]; tbin32_divid[1] = bin32_divid[1]; tbin32_divid[2] = bin32_divid[2];
    printf("INPUT CRC DIVIDEND (22x4bits) : %08x%08x%06x \n", tbin32_divid[0], tbin32_divid[1], tbin32_divid[2] >> 8);

    for (j = 0; j < 88; j++)
    {
        if (*tbin32_divid & 0x80000000)
            (*tbin32_divid) ^= crc32_GP; // xor CRC generator polynomial
        *tbin32_divid = *tbin32_divid << 1;
        if (*(tbin32_divid + 1) & 0x80000000)  *(tbin32_divid) |= 1; // shift code[1] to code[0]
        *(tbin32_divid + 1) = *(tbin32_divid + 1) << 1;
        if (*(tbin32_divid + 2) & 0x80000000)  *(tbin32_divid + 1) |= 1; // shift code[2] to code[1]
        *(tbin32_divid + 2) = *(tbin32_divid + 2) << 1;
    }
    printf("CRC24: %06x \n", (*tbin32_divid) >> 8);
    printf("after calculation tbin32_divid[0]=%06x \n", bin32_divid[0] >> 8);
    return (*tbin32_divid) >> 8;
}


/*========================================================================================
                            CRC CALCULATION FOR VARIABLE WIDTH
 adsb_crc(uint32_t *bin32_divid, int nbits)
 *bin32_divid :  pointer to full message
 nbits:     width of messsage for CRC check
 Return:  24bits CRC
 Note: original data uncontaminated.
========================================================================================*/
uint32_t adsb_crc(uint32_t* bin32_divid, int nbits)
{
    //--- bin32_divid[0-3]: message data, Max. to be 112-bits 
    uint32_t tbin32_divid[4] = { 0,0,0,0 };
    int32_t mask = 0xffffffff;
    int i, j, k, m;
    uint32_t crc32_GP = 0xFFFA0480;//0x04C11DB7; // CRC Generator Polynomial alread inlucdes the highest first bit, while standard CRC usually omitted. 

    //---copy message data to a temp. array tbin32_divid[], so following proceeding  will not contaminate original data
    if (nbits > 112) //---nbits Max 112
    {
        printf("Message length great than 28*4=112bits, use 112 as Max. value.\n");
        nbits = 112;
    }
    k = nbits / 32;
    m = nbits % 32;
    //printf("k=%d, m=%d \n",k,m);
    if (k > 0)
    {
        for (i = 0; i < k; i++)
            tbin32_divid[i] = bin32_divid[i];
    }
    mask = mask << (32 - m);
    tbin32_divid[k] = bin32_divid[k] & mask;
    //printf("INPUT CRC DIVIDEND : %08x%08x%08x%04x \n",tbin32_divid[0],tbin32_divid[1],tbin32_divid[2],tbin32_divid[3]>>16);
    for (j = 0; j < nbits; j++) //------CRC calculation
    {
        if (*tbin32_divid & 0x80000000)
            (*tbin32_divid) ^= crc32_GP; // xor CRC generator polynomial
        *tbin32_divid = *tbin32_divid << 1;
        if (*(tbin32_divid + 1) & 0x80000000)  *(tbin32_divid) |= 1; // shift code[1] to code[0]
        *(tbin32_divid + 1) = *(tbin32_divid + 1) << 1;
        if (*(tbin32_divid + 2) & 0x80000000)  *(tbin32_divid + 1) |= 1; // shift code[2] to code[1]
        *(tbin32_divid + 2) = *(tbin32_divid + 2) << 1;
        if (*(tbin32_divid + 3) & 0x80000000)  *(tbin32_divid + 2) |= 1; // shift code[2] to code[1]
        *(tbin32_divid + 3) = *(tbin32_divid + 3) << 1;
    }
    // printf("***  CRC24: %06x \n",(*tbin32_divid)>>8);
    return (tbin32_divid[0]) >> 8; //---���������� 24-������� CRC, �������� ����� ����� �������� �� ��������� ������ ������.
}


/*==========================================================================
    -------  FIX 1BIT ERROR IN MESSAGE ---------
    return position of the error bit starting from 0
    *bin32_code : full message
     nbits:   width of messsage for CRC check
     Return:   -1:error unfixable;     0: no error;     1: error fixed;
     Note: original data will be modified! !!! ensure pointer safe !!!!
===========================================================================*/
int adsb_fixerror_slow(uint32_t* bin32_code)
{
    int ret = -1;
    int i, j, k, m;
    uint32_t mask = 0x80000000;
    if (!adsb_crc(bin32_code, 112))
    {
        //      printf("CRC check OK.  No error in original message.\n");
        return 0;
    }

    for (i = 0; i < 112; i++)
    {
        k = i / 32;
        m = i % 32;
        mask = (0x80000000 >> m);
        bin32_code[k] ^= mask; // exchange 1 and 0 
        if (!adsb_crc(bin32_code, 112))
        {
            //        printf("1bit error in bin32_code[%d] fixed! \n",32*k+m);
            return 1;
        }
        bin32_code[k] ^= mask; // exchange 1 and 0, resume data
    }

    //    printf("More than 1 bit error, can't fix the error!\n");
    return -1;
}

