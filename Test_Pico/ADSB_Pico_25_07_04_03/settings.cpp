#include "settings.h"

#include "adsbee.h"
#include "comms.h"
#include "eepromPico.h"
#include "bsp.h"

bool SettingsManager::Load() 
{
 
    Apply();

    return true;
}

bool SettingsManager::Save()
{
    settings.receiver_enabled = adsbee.Receiver1090IsEnabled();
    settings.tl_mv = adsbee.GetTLMilliVolts();
    settings.bias_tee_enabled = adsbee.BiasTeeIsEnabled();
    settings.watchdog_timeout_sec = adsbee.GetWatchdogTimeoutSec();

    // Save log level.
    settings.log_level = comms_manager.log_level;

    // Save reporting protocols.
    comms_manager.GetReportingProtocol(SerialInterface::kCommsUART, settings.reporting_protocols[SerialInterface::kCommsUART]);
    comms_manager.GetReportingProtocol(SerialInterface::kConsole, settings.reporting_protocols[SerialInterface::kConsole]);

    // Save baud rates.
    comms_manager.GetBaudrate(SerialInterface::kCommsUART, settings.comms_uart_baud_rate);
    comms_manager.GetBaudrate(SerialInterface::kGNSSUART, settings.gnss_uart_baud_rate);

    return true;//!! ����� �������
}

void SettingsManager::ResetToDefaults() 
{
    Settings default_settings;
    settings = default_settings;
    Apply();
}



bool SettingsManager::Apply() 
{
    adsbee.SetReceiver1090Enable(settings.receiver_enabled);
    adsbee.SetTLMilliVolts(settings.tl_mv);
    adsbee.SetBiasTeeEnable(settings.bias_tee_enabled);
   //!! adsbee.SetWatchdogTimeoutSec(settings.watchdog_timeout_sec); //������ ����������� �������

    // Apply log level.
    comms_manager.log_level = settings.log_level;

    // ��������� ��������� ����������.
    comms_manager.SetReportingProtocol(SerialInterface::kCommsUART, settings.reporting_protocols[SerialInterface::kCommsUART]);
   //!! comms_manager.SetReportingProtocol(SerialInterface::kConsole, settings.reporting_protocols[SerialInterface::kConsole]); //!!�� ����� � ������ ������

    // Apply baud rates.
    comms_manager.SetBaudrate(SerialInterface::kCommsUART, settings.comms_uart_baud_rate);
    comms_manager.SetBaudrate(SerialInterface::kGNSSUART, settings.gnss_uart_baud_rate);

    return true;  // � ��������� ����� �� ����������� �������� ������, ��������� �� AT-������� ��� ����������� ����������
                  // ����������� �����������. ����� ���������� �������� ��� �������� �� ������������� EEPROM.
}
void SettingsManager::PrintPico()
{
    //���� �� �����������
}