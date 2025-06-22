//!!#include "Arduino.h"
#include "comms.h"
#include <cstdarg>  // For debug printf.
#include <cstdio>   // Regular pico/stdio.h doesn't support vprint functions.
#include "pico/stdlib.h"
#include <stdio.h>  // for printing

#include <cstring>   // for strcat
#include <iostream>  // for AT command ingestion

//======================================
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

//#define LOG_BUFF_SIZE   1024

#define LOG_UART_ID     uart1
#define LOG_BAUD_RATE   115200
#define LOG_DATA_BITS   8
#define LOG_STOP_BITS   1
#define LOG_PARITY      UART_PARITY_NONE
#define LOG_PIN         0

uart_inst_t* mUart;
uint mUartIRQ;

bool need_start;

uint8_t buffer[LOG_BUFF_SIZE];
uint16_t head_index;
uint16_t size;
//size_t Log_non_blocking(const uint8_t* src, size_t len);
//size_t Logs(const char* str);
//
//
//static inline size_t putCharactersToQueue(const uint8_t* src, size_t len);

void on_tx();


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

    gpio_set_function(LOG_PIN, UART_FUNCSEL_NUM(LOG_UART_ID, LOG_PIN));
    uart_set_hw_flow(LOG_UART_ID, false, false);

    uart_set_format(LOG_UART_ID, LOG_DATA_BITS, LOG_STOP_BITS, LOG_PARITY);

    Log_init(LOG_UART_ID);

    //sleep_ms(700);
    //Logs("Boot...\r\n");

       
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

bool CommsManager::Update() 
{
 //!!   UpdateAT();
  //!!  UpdateNetworkConsole();
    UpdateReporting();
    return true;
}

bool CommsManager::UpdateNetworkConsole() 
{
    static bool recursion_alert = false;
    if (recursion_alert) 
    {
        return false;
    }
    //if (esp32.IsEnabled()) 
    //{
        recursion_alert = true;
        // Send outgoing network console characters.
        char esp32_console_tx_buf[1024]; //
        char c = '\0';
        while (esp32_console_tx_queue.Length() > 0) 
        {
            uint16_t message_len = 0;
            //!!for (; message_len < SPICoprocessor::SCWritePacket::kDataMaxLenBytes && esp32_console_tx_queue.Pop(c); message_len++) 
            //{
            //    esp32_console_tx_buf[message_len] = c;
            //}
            // Ran out of characters to send, or hit the max packet length.
            if (message_len > 0) 
            {
                // Don't send empty messages.
                //!!if (!esp32.Write(ObjectDictionary::kAddrConsole, esp32_console_tx_buf, true, message_len)) 
                //{
                //    // Don't enter infinite loop of error messages if writing to the ESP32 isn't working.
                //    break;
                //}
            }
        }
        recursion_alert = false;
   // }
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
    if (iface_puts(iface, buf) && comms_manager.UpdateNetworkConsole()) 
    {
        return res;  // Return number of characters written.
    }

    return -1;  // puts failed.
}

bool CommsManager::iface_putc(SettingsManager::SerialInterface iface, char c) {
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
            return putchar(c) >= 0/* && (!esp32.IsEnabled() || network_console_putc(c) >= 0)*/;
            break;
        case SettingsManager::kNumSerialInterfaces:
        default:
         //!! CONSOLE_WARNING("CommsManager::iface_putc", "Unrecognized iface %d.", iface);
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
            //!!CONSOLE_WARNING("CommsManager::iface_getc", "Unrecognized iface %d.", iface);
            return false;  // Didn't match an interface.
            break;
    }
    return false;  // Should never get here.
}

bool CommsManager::iface_puts(SettingsManager::SerialInterface iface, const char *buf) {
    switch (iface) {
        case SettingsManager::kCommsUART:
            uart_puts(config_.comms_uart_handle, buf);
            return true;  // Function is void so we won't know if it succeeds.
            break;
        case SettingsManager::kGNSSUART:
            uart_puts(config_.gnss_uart_handle, buf);
            return true;  // Function is void so we won't know if it succeeds.
            break;
        case SettingsManager::kConsole:
            // Note: Using fputs instead of standard puts, since puts adds a line feed.
           //!! return fputs(buf, stdout) >= 0 && (!esp32.IsEnabled() || network_console_puts(buf) >= 0);
            break;
        case SettingsManager::kNumSerialInterfaces:
        default:
        //!!    CONSOLE_WARNING("CommsManager::iface_puts", "Unrecognized iface %d.", iface);
            return false;  // Didn't match an interface.
            break;
    }
    return false;  // Should never get here.
}

bool CommsManager::network_console_putc(char c) 
{
    //static bool recursion_alert = false;
    //if (recursion_alert) {
    //    return false;  // Don't get into infinite loops in case UpdateAT or Push() create error messages that would in
    //                   // turn create more network_console_putc calls.
    //}
    //recursion_alert = true;
    //if (!comms_manager.esp32_console_tx_queue.Push(c)) 
    //{
    //    // Try flushing the buffer before dumping it.
    //    comms_manager.UpdateAT();
    //    if (comms_manager.esp32_console_tx_queue.Push(c)) 
    //    {
    //        recursion_alert = false;
    //        return true;  // Crisis averted! Phew.
    //    }
    //    // Flush failed, clear the buffer.
    //    comms_manager.esp32_console_tx_queue.Clear();
    //    recursion_alert = false;
    //  //!! CONSOLE_ERROR("CommsManager::network_console_putc", "Overflowed buffer for outgoing network console chars.");
    //    return false;
    //}
    //recursion_alert = false;
    return true;
}
bool CommsManager::network_console_puts(const char *buf, uint16_t len) 
{
    for (uint16_t i = 0; i < strlen(buf) && i < len; i++) 
    {
        if (!network_console_putc(buf[i])) {
            return false;
        }
    }
    return true;
}


//============ мой вариант вывода в КОМ порт ==============================


void CommsManager::Log_init(uart_inst_t* uart)
{
    mUart = uart;
    uart_set_fifo_enabled(uart, false);
    mUartIRQ = (uart == uart0) ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(mUartIRQ, on_tx);
    uart_set_irq_enables(uart, /*rx_has_data*/false, /*tx_needs_data*/true);
    need_start = true;
    head_index = 0;
    size = 0;
}

size_t CommsManager::Logs(const char* str)
{
    size_t len = 0;
    while (str[len++]);
    return Log_non_blocking((const uint8_t*)str, --len);
}

size_t CommsManager::Log_non_blocking(const uint8_t* src, size_t len)
{
    size_t nStored;

    nStored = putCharactersToQueue(src, len);

    if ((nStored > 0) && need_start) {
        need_start = false;
        on_tx();
    }
    return nStored;
}

inline size_t CommsManager::putCharactersToQueue(const uint8_t* src, size_t len)
{
    size_t nSaved;
    uint32_t next_index;

    if (len == 0) return 0;

    irq_set_enabled(mUartIRQ, false);
    nSaved = LOG_BUFF_SIZE - size;
    if (nSaved > len)
    {
        nSaved = len;
    }

    if (nSaved > 0)
    {
        next_index = (head_index + size) % LOG_BUFF_SIZE;
        for (size_t i = 0; i < nSaved; i++, size++)
        {
            buffer[next_index++] = src[i];
            if (next_index >= LOG_BUFF_SIZE)
            {
                next_index = 0;
            }
        }
    }
    irq_set_enabled(mUartIRQ, true);
    return nSaved;
}

void on_tx()
{
    uint8_t ch;
    if (uart_is_writable(mUart))
    {
        if (size > 0)
        {
            ch = buffer[head_index++];
            if (head_index >= LOG_BUFF_SIZE)
            {
                head_index = 0;
            }
            uart_putc_raw(mUart, ch);
            size--;
            if (size == 0)
            {
                need_start = true;
            }
        }
        else {
            ((uart_hw_t*)mUart)->icr = UART_UARTICR_TXIC_BITS;
        }
    }
}
