#include "settings.h"

// Эти строки инициализируются здесь, поскольку их нельзя инициализировать в settings.h, поскольку они статические.
// ESP32 и RP2040 имеют отдельные файлы settings.cpp, но хотят совместно использовать эти статические определения строк.
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