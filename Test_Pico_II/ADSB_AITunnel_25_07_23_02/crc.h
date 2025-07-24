#ifndef CRC_HH_
#define CRC_HH_

#include <cstdint>
/**
* Вычисляет CRC24 буфера. Обратите внимание, что CRC рассчитывается для всего буфера, поэтому buffer_len_bytes должна
* включать только полезную нагрузку, без конечного CRC.
* @param[in] buffer Указатель на буфер для расчета CRC.
* @param[in] buffer_len_bytes Количество байтов для расчета CRC (только полезная нагрузка).
* @param[in] initial_value Начальное значение CRC.
* @retval 24-битный CRC.
*/
uint32_t crc24(const uint8_t *buffer, uint16_t buffer_len_bytes, uint32_t initial_value = 0x0);

/**
* Вычисляет синдром CRC24 буфера. Возвращает 0, если вычисленный CRC совпадает с CRC, содержащимся в
* последних 3 словах буфера. Если результат ненулевой, его можно использовать для поиска места битовых ошибок.
* @param[in] buffer Указатель на буфер для вычисления CRC.
* @param[in] buffer_len_bytes Количество байтов для вычисления CRC плюс CRC для проверки (полезная нагрузка + CRC).
* @param[in] initial_value Начальное значение CRC.
* @retval синдром CRC24.
*/
uint32_t crc24_syndrome(const uint8_t *buffer, uint16_t buffer_len_bytes, uint32_t initial_value = 0x0);

/**
* Находит индекс однобитовой ошибки в сообщении CRC24.
* @param[in] синдром CRC24 синдром.
* @param[in] message_len_bits Длина сообщения в битах.
* @retval Индекс однобитовой ошибки или -1, если ошибок не обнаружено. Возвращается -2, если количество битов недопустимо.
*/
int16_t crc24_find_single_bit_error(uint32_t syndrome, uint16_t message_len_bits);

/**
* Инвертирует один бит сообщения по заданному индексу в массиве байтов.
*/
void flip_bit(uint8_t *message, uint16_t index);

/**
* Меняет один бит сообщения на противоположный по указанному индексу в массиве слов.
*/void flip_bit(uint32_t *message, uint16_t index);

/**
* Вычисляет CRC32 буфера. Использует таблицы из crc_tables.hh, которые заполняются генератором IEEE 802.3
* полиномом (аналогично реализации zlib по умолчанию). Обратите внимание, что эта функция CRC использует начальное значение 0xFFFFFFFF для
* соответствия стандарту IEEE 802.3, а также включает в себя завершающую операцию XOR с 0xFFFFFFFF.
* @param[in] buffer Указатель на буфер для вычисления CRC.
* @param[in] buffer_len_bytes Количество байтов для вычисления CRC.
* @param[in] initial_value Начальное значение CRC.
* @retval 32-битный CRC.
*/
uint32_t crc32_ieee_802_3(const uint8_t *buffer, uint32_t buffer_len_bytes, uint32_t initial_value = 0xFFFFFFFF);

#endif /* CRC_HH_ */