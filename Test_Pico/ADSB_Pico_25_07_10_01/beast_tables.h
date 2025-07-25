// ���� ���� ������������� ������������ generate_beast_tables.py.
#ifndef BEAST_TABLES_H_
#define BEAST_TABLES_H_

#include <cstdint>

static const int kMinRSSIdBm = -100;
static const int kMaxRSSIdBm = -40;
// �������, ������� ������������ 60 �������� RSSI � ���������������� �� ���������� � ���� ����� dBFS Mode S Beast.
static const uint8_t kRSSIdBmToRSSIdBFS[61] = {1,  2,  2,   2,   2,   2,   2,   3,   3,   3,   3,   4,   4,  4,  5,  5,
                                               6,  6,  7,   7,   8,   9,   10,  10,  11,  12,  14,  15,  16, 18, 19, 21,
                                               23, 25, 27,  29,  32,  35,  38,  42,  45,  49,  54,  59,  64, 70, 76, 83,
                                               90, 99, 108, 117, 128, 139, 152, 166, 181, 197, 215, 234, 255};

#endif /* BEAST_TABLES_HH_ */
