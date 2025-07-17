#include "buffer_utils.h"

#include "stdio.h"

#define BITMASK_32_ALL   0xFFFFFFFF
#define WORD_32_NUM_BITS 32

// ПРИМЕЧАНИЕ: Операции с буфером выполняются в порядке от старшего к младшему (старшие биты сохраняются в старшем значащем бите), поскольку входной буфер сдвигается влево.

uint32_t Get24BitWordFromBuffer(uint32_t first_bit_index, const uint32_t buffer[]) 
{
    return GetNBitWordFromBuffer(24, first_bit_index, buffer);
}

/**
* Извлечь n-битное слово из буфера big-endian из 32-битных слов. НЕ защищает от выпадения за пределы
* буфера, поэтому будьте осторожны!
* @param[in] n Длина бита слова для извлечения.
* @param[in] first_bit_index Индекс бита, с которого начинается чтение (индекс MSb слова для чтения). MSb первого слова в буфере
* — это бит 0.
* @param[in] буфер Буфер для чтения.
* @retval Выровненное по правому краю n-битное слово, которое было прочитано из буфера.
*/
uint32_t GetNBitWordFromBuffer(uint16_t n, uint32_t first_bit_index, const uint32_t buffer[]) 
{
    // ПРИМЕЧАНИЕ: Бит 0 является старшим битом в этом формате, поскольку входной регистр сдвига сдвигается влево (старший бит — это старший бит).
    if (n > WORD_32_NUM_BITS || n < 1) 
    {
        printf("GetNBitWordFromBuffer: Tried to get %d bit word from buffer, but word bitlength must be between 1 "
            "and 32.\r\n",
            n);
        return 0;
    }
    uint32_t first_word_index_32 = first_bit_index / WORD_32_NUM_BITS;
    uint16_t bit_offset_32 = first_bit_index % WORD_32_NUM_BITS;
    // Получаем 32-битное слово, затем маскируем его до n бит.
    // Захватываем нижнюю часть слова.
    uint32_t word_n = ((buffer[first_word_index_32] << bit_offset_32));
    uint16_t first_subword_length = WORD_32_NUM_BITS - bit_offset_32;
    if (first_subword_length < n) 
    {
        // Захватываем верхнюю часть слова из следующего 32-битного слова в буфере.
        word_n |= (buffer[first_word_index_32 + 1] >> (first_subword_length));
    }
    word_n &= BITMASK_32_ALL << (32 - n);  // mask to the upper n bits
    word_n >>= (WORD_32_NUM_BITS - n);     // right-align
    return word_n;
}

/**
* Вставьте n-битное слово в буфер big-endian из 32-битных слов. НЕ защищает от выпадения за пределы
* буфера, поэтому будьте осторожны!
* @param[in] n Длина бит слова для вставки.
* @param[in] word Слово для вставки. Должно быть выровнено по правому краю.
* @param[in] first_bit_index Индекс бита, куда должен быть вставлен MSb слова. MSb первого слова в буфере — это бит 0.
* @param[in] buffer Буфер для вставки.
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
    // Удалить до n младших битов первого слова, начиная с bit_offset_32.
    buffer[first_word_index_32] &= ~((BITMASK_32_ALL << (WORD_32_NUM_BITS - n)) >> bit_offset_32);
    buffer[first_word_index_32] |= word >> bit_offset_32;  // insert insertion word into first word of buffer
    uint16_t first_subword_length = WORD_32_NUM_BITS - bit_offset_32;
    if (first_subword_length < n) 
    {
        // Вставленное слово перетекает во второе слово буфера.
        // Пустые n-first_subword_length старшие значимые биты второго слова.
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
* Меняет порядок байтов 16-битного значения.
* @param[in] value Значение для замены MSB и LSB.
* @retval Значение с измененным порядком байтов.
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