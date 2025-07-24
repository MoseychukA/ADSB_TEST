#include "comms.h"
#include <cstdarg>  // For debug printf.
#include <cstdio>   // Regular pico/stdio.h doesn't support vprint functions.
#include "pico/stdlib.h"
#include <stdio.h>  // for printing

#include <cstring>   // for strcat
#include <iostream>  // for AT command ingestion


CommsManager comms_manager;

CommsManager::CommsManager()
{

}

CommsManager::~CommsManager()
{

}

CommsManager::CommsManager(CommsManagerConfig config_in)
    : config_(config_in)/*!!, at_parser_(CppAT(at_command_list, at_command_list_num_commands, true))*/ {}

bool CommsManager::Init() 
{
   // InitReporting();
  
    gpio_set_function(config_.comms_uart_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(config_.comms_uart_rx_pin, GPIO_FUNC_UART);
    uart_set_translate_crlf(config_.comms_uart_handle, true);     //Если true, преобразовать перевод строки в возврат каретки при передаче
    uart_init(config_.comms_uart_handle, SettingsManager::Settings::kDefaultCommsUARTBaudrate);
   
    gpio_set_function(config_.gnss_uart_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(config_.gnss_uart_rx_pin, GPIO_FUNC_UART);
    uart_set_translate_crlf(config_.gnss_uart_handle, false);
    uart_init(config_.gnss_uart_handle, SettingsManager::Settings::kDefaultGNSSUARTBaudrate);

     
    // Не трогайте здесь GPIO включения/сброса ESP32, так как их нужно переключать программисту. 
    // Инициализируйте их только в том случае, если программирование не требуется. 
    // Не трогайте вывод ESP32 wifi, пока мы не будем готовы попробовать обновления прошивки.

    //!!stdio_init_all();
    //!!stdio_set_translate_crlf(&stdio_usb, false);
    return true;
}

int CommsManager::console_printf(const char *format, ...) 
{
    va_list args;
    va_start(args, format);
    int res = iface_vprintf(SettingsManager::SerialInterface::kCommsUART, format, args);
    va_end(args);
    return res;
}

int CommsManager::console_level_printf(SettingsManager::LogLevel level, const char *format, ...) 
{
    if (log_level < level) return 0;
    va_list args;
    va_start(args, format);
    int res = iface_vprintf(SettingsManager::SerialInterface::kCommsUART, format, args);
    va_end(args);
    return res;
}

int CommsManager::iface_printf(SettingsManager::SerialInterface iface, const char *format, ...) 
{
    va_list args;
    va_start(args, format);
    int res = iface_vprintf(iface, format, args);
    va_end(args);
    return res;
}

int CommsManager::iface_vprintf(SettingsManager::SerialInterface iface, const char *format, va_list args) 
{
    char buf[kPrintfBufferMaxSize];

    // Formatted print to buffer.
    int res = vsnprintf(buf, kPrintfBufferMaxSize, format, args);
    if (res <= 0) {
        return res;  // vsnprintf failed.
    }
    // Send buffer to interface, then manually push messages (otherwise they only pop out when the buffer gets full).
    if (iface_puts(iface, buf)) 
    {
        return res;  // Return number of characters written.
    }

    return -1;  // puts failed.
}

bool CommsManager::iface_putc(SettingsManager::SerialInterface iface, char c) 
{
    switch (iface) {
        case SettingsManager::kCommsUART:
            uart_putc_raw(config_.comms_uart_handle, c);
            return true;  // Function is void so we won't know if it succeeds.
            break;
        case SettingsManager::kGNSSUART:
            uart_putc_raw(config_.gnss_uart_handle, c);
            return true;  // Function is void so we won't know if it succeeds.
            break;
        case SettingsManager::kConsole:
            return putchar(c) >= 0;
            break;
        case SettingsManager::kNumSerialInterfaces:
        default:
  /*         CONSOLE_WARNING("CommsManager::iface_putc", "Unrecognized iface %d.", iface);*/
            return false;
    }
    return false;  // Should never get here.
}

bool CommsManager::iface_getc(SettingsManager::SerialInterface iface, char &c) 
{
    switch (iface) {
        case SettingsManager::kCommsUART:
            if (uart_is_readable_within_us(config_.comms_uart_handle, config_.uart_timeout_us)) 
            {
                c = uart_getc(config_.comms_uart_handle);
                return true;
            }
            return false;  // No chars to read.
            break;
        case SettingsManager::kGNSSUART:
            if (uart_is_readable_within_us(config_.gnss_uart_handle, config_.uart_timeout_us)) 
            {
                c = uart_getc(config_.gnss_uart_handle);
                return true;
            }
            return false;  // No chars to read.
            break;
        case SettingsManager::kConsole: {
            int ret = getchar_timeout_us(config_.uart_timeout_us);
            if (ret >= 0) {
                c = (char)ret;
                return true;
            }
            return false;  // Failed to read character.
            break;
        }
        case SettingsManager::kNumSerialInterfaces:
        default:
            //CONSOLE_WARNING("CommsManager::iface_getc", "Unrecognized iface %d.", iface);
            return false;  // Didn't match an interface.
            break;
    }
    return false;  // Should never get here.
}

bool CommsManager::iface_puts(SettingsManager::SerialInterface iface, const char *buf) 
{
    switch (iface) {
        case SettingsManager::kCommsUART:
            uart_puts(config_.comms_uart_handle, buf);
            return true;  // Function is void so we won't know if it succeeds.
            break;
        case SettingsManager::kGNSSUART:
            uart_puts(config_.gnss_uart_handle, buf);
            return true;  // Function is void so we won't know if it succeeds.
            break;
         default:
 /*           CONSOLE_WARNING("CommsManager::iface_puts", "Unrecognized iface %d.", iface);*/
            return false;  // Didn't match an interface.
            break;
    }
    return false;  // Should never get here.
}

