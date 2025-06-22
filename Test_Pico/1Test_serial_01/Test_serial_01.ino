#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define LOG_BUFF_SIZE   1024

uart_inst_t* mUart;
uint mUartIRQ;

bool need_start;

uint8_t buffer[LOG_BUFF_SIZE];
uint16_t head_index;
uint16_t size;
size_t Log_non_blocking(const uint8_t* src, size_t len);
size_t Logs(const char* str);


static inline size_t putCharactersToQueue(const uint8_t* src, size_t len);

#define LOG_UART_ID     uart1
#define LOG_BAUD_RATE   115200
#define LOG_DATA_BITS   8
#define LOG_STOP_BITS   1
#define LOG_PARITY      UART_PARITY_NONE
#define LOG_PIN         0


void Log_init(uart_inst_t* uart);
void on_tx();


void setup()
{
    //stdio_init_all();




    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);
    uart_set_translate_crlf(LOG_UART_ID, false);
    uart_init(LOG_UART_ID, LOG_BAUD_RATE);
    gpio_set_function(LOG_PIN, UART_FUNCSEL_NUM(LOG_UART_ID, LOG_PIN));
    uart_set_hw_flow(LOG_UART_ID, false, false);

    uart_set_format(LOG_UART_ID, LOG_DATA_BITS, LOG_STOP_BITS, LOG_PARITY);

    Log_init(LOG_UART_ID);

    sleep_ms(700);
    Logs("Boot...\r\n");

}

void loop()
{
    Logs("aaaaaaaaaabbbbbbbbbbccccccccccddddddddddeeeeeeeeeeffffffffffgggggggggghhhhhhhhhhiiiiiiiiiijjjjjjjjjjkkkkkkkkkkllllllllllmmmmmmmmmmnnnnnnnnnnooooooooooppppppppppqqqqqqqqqqrrrrrrrrrrssssssssssttttttttttuuuuuuuuuuvvvvvvvvvvwwwwwwwwwwxxxxxxxxxxyyyyyyyyyyzzzzzzzzzz\r\n");
    Logs("Hello-0  ");
    Logs("Hello-1  ");
    Logs("Hello-2  ");
    Logs("Hello-3  ");
    Logs("Hello-4  ");
    Logs("Hello-5  ");
    Logs("Hello-6  ");
    Logs("Hello-7  ");
    Logs("Hello-8  ");
    Logs("Hello-9  \r\n\n");
    sleep_ms(1000);
}

void Log_init(uart_inst_t* uart)
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

size_t Logs(const char* str)
{
    size_t len = 0;
    while (str[len++]);
    return Log_non_blocking((const uint8_t*)str, --len);
}

size_t Log_non_blocking(const uint8_t* src, size_t len)
{
    size_t nStored;

    nStored = putCharactersToQueue(src, len);

    if ((nStored > 0) && need_start) {
        need_start = false;
        on_tx();
    }
    return nStored;
}

static inline size_t putCharactersToQueue(const uint8_t* src, size_t len)
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
