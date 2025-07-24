#include "settings.h"

// ��� ������ ���������������� �����, ��������� �� ������ ���������������� � settings.h, ��������� ��� �����������.
// ESP32 � RP2040 ����� ��������� ����� settings.cpp, �� ����� ��������� ������������ ��� ����������� ����������� �����.
const char SettingsManager::kConsoleLogLevelStrs[SettingsManager::LogLevel::kNumLogLevels]
                                                [SettingsManager::kConsoleLogLevelStrMaxLen] = {"SILENT", "ERRORS",
                                                                                                "WARNINGS", "INFO"};
const char SettingsManager::kSerialInterfaceStrs[SettingsManager::SerialInterface::kNumSerialInterfaces]
                                                [SettingsManager::kSerialInterfaceStrMaxLen] = {"CONSOLE", "COMMS_UART",
                                                                                                "GNSS_UART"};
const char SettingsManager::kReportingProtocolStrs[SettingsManager::ReportingProtocol::kNumProtocols]
                                                  [SettingsManager::kReportingProtocolStrMaxLen] = {
                                                      "NONE",  "RAW",      "BEAST",    "BEAST_RAW",
                                                      "CSBEE", "MAVLINK1", "MAVLINK2", "GDL90"};