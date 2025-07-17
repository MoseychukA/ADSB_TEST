/*
 * CPR (Compact Position Reporting) декодер для точных координат
 * Требует два последовательных кадра (четный и нечетный)
 */

#ifndef ADS_B_CPR_H
#define ADS_B_CPR_H

#include <Arduino.h>
#include <math.h>

// Константы для CPR декодирования
#define DLAT_EVEN (360.0 / 60.0)   // 6.0
#define DLAT_ODD  (360.0 / 59.0)   // 6.101694915...
#define CPR_MAX   131072.0         // 2^17

struct CPRFrame {
  bool valid;
  bool odd_even;  // 0 = even, 1 = odd
  uint32_t cpr_lat;
  uint32_t cpr_lon;
  uint32_t timestamp;
};

struct DecodedPosition {
  bool valid;
  double latitude;
  double longitude;
  bool globally_unambiguous;
};

class CPRDecoder {
private:
  CPRFrame even_frame;
  CPRFrame odd_frame;
  bool has_even_frame;
  bool has_odd_frame;
  
  static double mod(double a, double b) {
    return a - b * floor(a / b);
  }
  
  static uint8_t nl(double lat) {
    if (abs(lat) >= 87.0) return 1;
    return floor(2.0 * PI / acos(1.0 - (1.0 - cos(PI / (2.0 * 15.0))) / 
                                 pow(cos(PI * lat / 180.0), 2.0)));
  }
  
public:
  CPRDecoder() : has_even_frame(false), has_odd_frame(false) {}
  
  void addFrame(bool odd_even, uint32_t cpr_lat, uint32_t cpr_lon, uint32_t timestamp) {
    if (odd_even) {
      odd_frame = {true, true, cpr_lat, cpr_lon, timestamp};
      has_odd_frame = true;
    } else {
      even_frame = {true, false, cpr_lat, cpr_lon, timestamp};
      has_even_frame = true;
    }
  }
  
  DecodedPosition decodeGlobal() {
    DecodedPosition result = {};
    
    if (!has_even_frame || !has_odd_frame) {
      result.valid = false;
      return result;
    }
    
    // Проверяем, что кадры не слишком старые (максимум 10 секунд разница)
    if (abs((long)(odd_frame.timestamp - even_frame.timestamp)) > 10000) {
      result.valid = false;
      return result;
    }
    
    double dlat_even = DLAT_EVEN;
    double dlat_odd = DLAT_ODD;
    
    // Вычисляем индекс широты
    double j = floor(((59.0 * even_frame.cpr_lat - 60.0 * odd_frame.cpr_lat) / CPR_MAX) + 0.5);
    
    double rlat_even = dlat_even * (mod(j, 60.0) + even_frame.cpr_lat / CPR_MAX);
    double rlat_odd = dlat_odd * (mod(j, 59.0) + odd_frame.cpr_lat / CPR_MAX);
    
    // Приводим к диапазону [-90, 90]
    if (rlat_even >= 270.0) rlat_even -= 360.0;
    if (rlat_odd >= 270.0) rlat_odd -= 360.0;
    
    // Проверяем совместимость широт
    if (abs(rlat_even - rlat_odd) > 1.0) {
      result.valid = false;
      return result;
    }
    
    // Используем более свежий кадр для определения широты
    double rlat;
    bool use_odd;
    if (odd_frame.timestamp > even_frame.timestamp) {
      rlat = rlat_odd;
      use_odd = true;
    } else {
      rlat = rlat_even;
      use_odd = false;
    }
    
    // Вычисляем долготу
    uint8_t nl_lat = nl(rlat);
    uint8_t nl_lat_1 = nl(rlat - (use_odd ? dlat_odd : dlat_even));
    
    if (nl_lat != nl_lat_1) {
      result.valid = false;
      return result;
    }
    
    if (nl_lat == 0) nl_lat = 1;  // Избегаем деления на ноль
    
    double dlon = 360.0 / nl_lat;
    double m = floor(((even_frame.cpr_lon * (nl_lat - 1) - odd_frame.cpr_lon * nl_lat) / CPR_MAX) + 0.5);
    
    double rlon;
    if (use_odd) {
      rlon = dlon * (mod(m, nl_lat) + odd_frame.cpr_lon / CPR_MAX);
    } else {
      rlon = dlon * (mod(m, nl_lat) + even_frame.cpr_lon / CPR_MAX);
    }
    
    // Приводим к диапазону [-180, 180]
    if (rlon > 180.0) rlon -= 360.0;
    if (rlon < -180.0) rlon += 360.0;
    
    result.valid = true;
    result.latitude = rlat;
    result.longitude = rlon;
    result.globally_unambiguous = true;
    
    return result;
  }
  
  // Локальное декодирование (требует известную позицию)
  DecodedPosition decodeLocal(double ref_lat, double ref_lon, bool odd_even, 
                             uint32_t cpr_lat, uint32_t cpr_lon) {
    DecodedPosition result = {};
    
    double dlat = odd_even ? DLAT_ODD : DLAT_EVEN;
    double j = floor(ref_lat / dlat) + floor(0.5 + mod(ref_lat, dlat) / dlat - cpr_lat / CPR_MAX);
    
    double rlat = dlat * (j + cpr_lat / CPR_MAX);
    
    uint8_t nl_lat = nl(rlat);
    if (nl_lat == 0) nl_lat = 1;
    
    double dlon = 360.0 / nl_lat;
    double m = floor(ref_lon / dlon) + floor(0.5 + mod(ref_lon, dlon) / dlon - cpr_lon / CPR_MAX);
    
    double rlon = dlon * (m + cpr_lon / CPR_MAX);
    
    result.valid = true;
    result.latitude = rlat;
    result.longitude = rlon;
    result.globally_unambiguous = false;
    
    return result;
  }
  
  void reset() {
    has_even_frame = false;
    has_odd_frame = false;
  }
};

#endif // ADS_B_CPR_H
