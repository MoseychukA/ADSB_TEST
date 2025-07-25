#include "buffer_utils.h"

#include "stdio.h"

#define BITMASK_32_ALL   0xFFFFFFFF
#define WORD_32_NUM_BITS 32

// ����������: �������� � ������� ����������� � ������� �� �������� � �������� (������� ���� ����������� � ������� �������� ����), ��������� ������� ����� ���������� �����.

uint32_t Get24BitWordFromBuffer(uint32_t first_bit_index, const uint32_t buffer[]) 
{
    return GetNBitWordFromBuffer(24, first_bit_index, buffer);
}

/**
* ������� n-������ ����� �� ������ big-endian �� 32-������ ����. �� �������� �� ��������� �� �������
* ������, ������� ������ ���������!
* @param[in] n ����� ���� ����� ��� ����������.
* @param[in] first_bit_index ������ ����, � �������� ���������� ������ (������ MSb ����� ��� ������). MSb ������� ����� � ������
* � ��� ��� 0.
* @param[in] ����� ����� ��� ������.
* @retval ����������� �� ������� ���� n-������ �����, ������� ���� ��������� �� ������.
*/
uint32_t GetNBitWordFromBuffer(uint16_t n, uint32_t first_bit_index, const uint32_t buffer[]) 
{
    // ����������: ��� 0 �������� ������� ����� � ���� �������, ��������� ������� ������� ������ ���������� ����� (������� ��� � ��� ������� ���).
    if (n > WORD_32_NUM_BITS || n < 1) 
    {
        printf("GetNBitWordFromBuffer: Tried to get %d bit word from buffer, but word bitlength must be between 1 "
            "and 32.\r\n",
            n);
        return 0;
    }
    uint32_t first_word_index_32 = first_bit_index / WORD_32_NUM_BITS;
    uint16_t bit_offset_32 = first_bit_index % WORD_32_NUM_BITS;
    // �������� 32-������ �����, ����� ��������� ��� �� n ���.
    // ����������� ������ ����� �����.
    uint32_t word_n = ((buffer[first_word_index_32] << bit_offset_32));
    uint16_t first_subword_length = WORD_32_NUM_BITS - bit_offset_32;
    if (first_subword_length < n) 
    {
        // ����������� ������� ����� ����� �� ���������� 32-������� ����� � ������.
        word_n |= (buffer[first_word_index_32 + 1] >> (first_subword_length));
    }
    word_n &= BITMASK_32_ALL << (32 - n);  // mask to the upper n bits
    word_n >>= (WORD_32_NUM_BITS - n);     // right-align
    return word_n;
}

/**
* �������� n-������ ����� � ����� big-endian �� 32-������ ����. �� �������� �� ��������� �� �������
* ������, ������� ������ ���������!
* @param[in] n ����� ��� ����� ��� �������.
* @param[in] word ����� ��� �������. ������ ���� ��������� �� ������� ����.
* @param[in] first_bit_index ������ ����, ���� ������ ���� �������� MSb �����. MSb ������� ����� � ������ � ��� ��� 0.
* @param[in] buffer ����� ��� �������.
*/
void SetNBitWordInBuffer(uint16_t n, uint32_t word, uint32_t first_bit_index, uint32_t buffer[]) 
{
    if (n > WORD_32_NUM_BITS || n < 1) 
    {
        printf(
            "SetNBitWordInBuffer: Tried to set %d-bit word in buffer, but word bitlength must be between 1 and "
            "32.\r\n",
            n);
        return;
    }

    word &= BITMASK_32_ALL >> (WORD_32_NUM_BITS - n);  // mask insertion word to n LSb's.
    word <<= (WORD_32_NUM_BITS - n);                   // left-align insertion word for easier handling.

    uint32_t first_word_index_32 = first_bit_index / WORD_32_NUM_BITS;
    uint16_t bit_offset_32 = first_bit_index % WORD_32_NUM_BITS;
    // ������� �� n ������� ����� ������� �����, ������� � bit_offset_32.
    buffer[first_word_index_32] &= ~((BITMASK_32_ALL << (WORD_32_NUM_BITS - n)) >> bit_offset_32);
    buffer[first_word_index_32] |= word >> bit_offset_32;  // insert insertion word into first word of buffer
    uint16_t first_subword_length = WORD_32_NUM_BITS - bit_offset_32;
    if (first_subword_length < n) 
    {
        // ����������� ����� ���������� �� ������ ����� ������.
        // ������ n-first_subword_length ������� �������� ���� ������� �����.
        buffer[first_word_index_32 + 1] &= BITMASK_32_ALL >> (n - first_subword_length);
        buffer[first_word_index_32 + 1] |= word << first_subword_length;
    }
}

void PrintBinary32(uint32_t value) 
{
    printf("\t0b");
    for (int j = 31; j >= 0; j--) {
        printf(value & (0b1 << j) ? "1" : "0");
    }
    printf("\r\n");
}

/**
* ������ ������� ������ 16-������� ��������.
* @param[in] value �������� ��� ������ MSB � LSB.
* @retval �������� � ���������� �������� ������.
*/
uint16_t swap16(uint16_t value) { return (value << 8) | (value >> 8); }

uint16_t CalculateCRC16(const uint8_t *buf, int32_t buf_len_bytes) 
{
    uint8_t x;
    uint16_t crc = 0xFFFF;
    while (buf_len_bytes--) {
        x = crc >> 8 ^ *buf++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x);
    }
    return swap16(crc);
}