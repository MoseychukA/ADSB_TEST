#ifndef AIRCRAFT_DICTIONARY_HH_
#define AIRCRAFT_DICTIONARY_HH_

#include <cstdio>
#include <cstring>
#include <unordered_map>

#include "transponder_packet.h"

class AircraftDictionary {
   public:
    static const uint16_t kMaxNumAircraft = 400;
    static const uint16_t kMaxNumSources = 3;

    struct AircraftDictionaryConfig_t 
    {
        uint32_t aircraft_prune_interval_ms = 60e3;
    };

    struct Metrics 
    {
 
        uint32_t raw_squitter_frames = 0;
        uint32_t valid_squitter_frames = 0;
        uint32_t raw_extended_squitter_frames = 0;
        uint32_t valid_extended_squitter_frames = 0;
        uint32_t demods_1090 = 0;

        uint16_t raw_squitter_frames_by_source[kMaxNumSources] = {0};
        uint16_t valid_squitter_frames_by_source[kMaxNumSources] = {0};
        uint16_t raw_extended_squitter_frames_by_source[kMaxNumSources] = {0};
        uint16_t valid_extended_squitter_frames_by_source[kMaxNumSources] = {0};
        uint16_t demods_1090_by_source[kMaxNumSources] = {0};

 
    };

    /**
    * ������������ ���������� 56-������ ��������-����. ������������ ��� ������ ���������� ������������������. ���������� ����� �������������
    * ������������ �� ����� �����. ��� ������� ����������� �� �����, ����� �������������� ����� ��� ������� �����
    * ����� ���� ��������. �������� ��������, ��� ���������� �� ����� ����� �� ���������� ���������� �������.
    */
    inline void Record1090RawSquitterFrame(int16_t source = -1) 
    {
        metrics_counter_.raw_squitter_frames++;
        if (source > 0) 
        {
            metrics_counter_.raw_squitter_frames_by_source[source]++;
        }
    }
    /**
    * ������������ ���������� 112-������ ����������� ��������-����. ������������ ��� ������ ���������� ������������������. ���������� �����
    * ������������� ������������ �� ����� �����. ��� ������� ����������� �� �����, ��� ��� �������������� ����� ��� ������� �����
    * ����� ���� ��������. �������� ��������, ��� ���������� �� ����� ����� �� ���������� ���������� �������.
    */
    inline void Record1090RawExtendedSquitterFrame(int16_t source = -1) 
    {
        metrics_counter_.raw_extended_squitter_frames++;
        if (source > 0) {
            metrics_counter_.raw_extended_squitter_frames_by_source[source]++;
        }
    }

    Metrics metrics;

   private:

    Metrics metrics_counter_;
};

#endif /* AIRCRAFT_DICTIONARY_HH_ */