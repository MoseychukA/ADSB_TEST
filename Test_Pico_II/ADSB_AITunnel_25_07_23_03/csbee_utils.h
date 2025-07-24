#ifndef CSBEE_UTILS_H_
#define CSBEE_UTILS_H_

#include "aircraft_dictionary.h"
#include "macros.h"
#include "stdio.h"

const uint16_t kCSBeeMessageStrMaxLen = 200;
const uint16_t kCRCMaxNumChars = 4;  // 16 bits = 4 hex characters.
const uint16_t kEOLNumChars = 2;

/**
* Выводит объект Aircraft в строковый буфер в формате CSBee. Длина строкового буфера должна быть
* kCSBeeMessageStrMaxLen.
* @param[out] message_buf Массив символов для записи.
* @param[in] aircraft Объект Aircraft, содержимое которого выводится.
* @retval Количество символов, записанных в строковый буфер, или отрицательное значение в случае ошибки.
*/
inline int16_t WriteCSBeeAircraftMessageStr(char message_buf[], const Aircraft1090 &aircraft) 
{
    // #A:ICAO,FLAGS,CALL,SQ,LAT,LON,ALT_BARO,TRACK,VELH,VELV,SIGS,SIGQ,FPS,NICNAC,ALT_GEO,ECAT,CRC\r\n

    // Создать битовое поле SYSINFO.
    // Преобразовать длину и ширину самолёта в максимальное значение.
    uint32_t sysinfo = MAX(aircraft.length_m, aircraft.width_m) << 22;  // MDIM bitfield.
    // Convert GNSS antenna offset value to CSBee formatted bitfield.
    if (aircraft.gnss_antenna_offset_right_of_roll_axis_m != INT8_MAX) 
    {
        sysinfo |= (((aircraft.gnss_antenna_offset_right_of_roll_axis_m > 0) & 0b1) << 21);         // GAOR bitfield.
        sysinfo |= (((ABS(aircraft.gnss_antenna_offset_right_of_roll_axis_m) >> 1) & 0b11) << 19);  // GAOD bitfield.
        sysinfo |= (0b1 << 18);                                                                     // GAOK bitfield.
    }
    sysinfo |= ((aircraft.system_design_assurance & 0b11) << 16);                 // SDA bitfield.
    sysinfo |= ((aircraft.source_integrity_level & 0b11) << 14);                  // SIL bitfield.
    sysinfo |= ((aircraft.geometric_vertical_accuracy & 0b11) << 12);             // GVA bitfield
    sysinfo |= ((aircraft.navigation_accuracy_category_position & 0b1111) << 8);  // NAC_p bitfield.
    sysinfo |= ((aircraft.navigation_accuracy_category_velocity & 0b111) << 5);   // NAC_v bitfield.
    sysinfo |= ((aircraft.navigation_integrity_category_baro & 0b1) << 4);        // NIC_baro bitfield.
    sysinfo |= ((aircraft.navigation_integrity_category & 0b1111));               // NIC bitfield.

    int16_t num_chars =  // Вывести все, кроме CRC, в строковый буфер.
        snprintf(message_buf, kCSBeeMessageStrMaxLen - kCRCMaxNumChars - 1,
                 "%06X,"                                           // ICAO, e.g. 3C65AC
                 "%s,"                                             // CALL, e.g. N61ZP
                 "%04o,"                                           // SQUAWK, e.g. 7232
                 "%.5f,"                                           // LAT, e.g. 57.57634
                 "%.5f,"                                           // LON, e.g. 17.59554
                 "%d,"                                             // ALT_BARO, e.g. 5000
                 "%d,"                                             // ALT_GEO, e.g. 5000
                 "%.0f,"                                           // TRACK, e.g. 35
                 "%.0f,"                                           // VELH, e.g. 464
                 "%d\r\n"                                          // VELV, e.g. -1344
                 ,                                            
                 aircraft.icao_address,                            // ICAO
                 aircraft.callsign,                                // CALL
                 aircraft.squawk,                                  // SQUAWK
                 aircraft.latitude_deg,                            // LAT
                 aircraft.longitude_deg,                           // LON
                 aircraft.baro_altitude_ft,                        // ALT_BARO
                 aircraft.gnss_altitude_ft,                        // ALT_GEO
                 aircraft.direction_deg,                           // TRACK
                 aircraft.velocity_kts,                            // VELH
                 aircraft.vertical_rate_fpm                        // VELV
         );
    if (num_chars < 0) return num_chars;  // Check if snprintf call got busted.

    // Append a CRC.
   // uint16_t crc = CalculateCRC16((uint8_t *)message_buf, num_chars);
   // uint16_t crc_num_chars = snprintf(message_buf + num_chars, kCRCMaxNumChars + kEOLNumChars + 1, "%X\r\n", crc);  // Add CRC to the message buffer.
   // uint16_t crc_num_chars = snprintf(message_buf , "%X\r\n");  // Add CRC to the message buffer.
    return num_chars/* + crc_num_chars*/;
}

inline int16_t WriteCSBeeStatisticsMessageStr(char message_buf[], uint16_t dps, uint16_t raw_sfps, uint16_t sfps,
                                              uint16_t raw_esfps, uint16_t esfps, uint16_t num_aircraft, uint32_t tscal,
                                              uint32_t uptime) 
{
    int16_t num_chars =  // Print everything except for CRC into string buffer.
        snprintf(message_buf, kCSBeeMessageStrMaxLen - kCRCMaxNumChars - 1,
                 "#S:%d,"  // DPS, e.g. 106
                 "%d,"     // RAW_SFPS e.g. 30
                 "%d,"     // SFPS, e.g. 20
                 "%d,"     // RAW_ESFPS e.g. 15
                 "%d,"     // ESFPS, e.g. 13
                 "%d,"     // NUM_AIRCRAFT, e.g. 13
                 "%d,"     // TSCAL, e.g. 13999415
                 "%d,",    // UPTIME, e.g. 134
                 dps, raw_sfps, sfps, raw_esfps, esfps, num_aircraft, tscal, uptime);
    if (num_chars < 0) return num_chars;  // Check if snprintf call got busted.

    // Append a CRC.
    uint16_t crc = CalculateCRC16((uint8_t *)message_buf, num_chars);
    uint16_t crc_num_chars = snprintf(message_buf + num_chars, kCRCMaxNumChars + kEOLNumChars + 1, "%X\r\n",
                                      crc);  // Add CRC to the message buffer.

    return num_chars + crc_num_chars;
}

#endif /* CSBEE_UTILS_HH_ */