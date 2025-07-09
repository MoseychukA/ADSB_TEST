
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include <cstdarg>  // For debug printf.
#include <cstring>   // for strcat
#include <iostream>  // for AT command ingestion


#define GEN_PWM_SIGNAL_PIN1    2
#define GEN_PWM_SIGNAL_PIN2    3

void setup()
{
    //  stdio_init_all();
    Serial1.begin(115200);
    Serial1.println("Starting!");
    // Tell GPIO 0 and 1 they are allocated to the PWM
  
    //pinMode(GEN_PWM_SIGNAL_PIN1, OUTPUT);
    //digitalWrite(GEN_PWM_SIGNAL_PIN1, LOW);



    gpio_set_function(GEN_PWM_SIGNAL_PIN1, GPIO_FUNC_PWM);
    gpio_set_function(GEN_PWM_SIGNAL_PIN2, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(GEN_PWM_SIGNAL_PIN1);

    pwm_set_clkdiv(slice_num, 100); // pwm clock should now be running at 1MHz

    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_wrap(slice_num, 1);
    // Set channel A output high for one cycle before dropping
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 1);
    // Set initial B output high for three cycles before dropping
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 3);
    // Set the PWM running
    pwm_set_enabled(slice_num, true);
    sleep_us(1);
    pwm_set_enabled(slice_num, false);
    sleep_us(1);
    pwm_set_enabled(slice_num, true);
    sleep_us(1);
    pwm_set_enabled(slice_num, false);
    sleep_us(1);
    pwm_set_enabled(slice_num, true);
    sleep_us(1);
    pwm_set_enabled(slice_num, false);

 


}


//
//
//// Этот пример управляет выходом ШИМ в диапазоне рабочих циклов и использует
//// другой срез ШИМ в режиме входа для измерения рабочего цикла. Вам нужно будет
//// соединить эти два контакта перемычкой:
//const uint OUTPUT_PIN = 2;
//const uint MEASURE_PIN = 5;
//
//float measure_duty_cycle(uint gpio) 
//{
//    // Only the PWM B pins can be used as inputs.
//    assert(pwm_gpio_to_channel(gpio) == PWM_CHAN_B);
//    uint slice_num = pwm_gpio_to_slice_num(gpio);
//
//    // Count once for every 100 cycles the PWM B input is high
//    pwm_config cfg = pwm_get_default_config();
//    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_HIGH);
//    pwm_config_set_clkdiv(&cfg, 100);
//    pwm_init(slice_num, &cfg, false);
//    gpio_set_function(gpio, GPIO_FUNC_PWM);
//
//    pwm_set_enabled(slice_num, true);
//    sleep_ms(10);
//    pwm_set_enabled(slice_num, false);
//    float counting_rate = clock_get_hz(clk_sys) / 100;
//    float max_possible_count = counting_rate * 0.01;
//    return pwm_get_counter(slice_num) / max_possible_count;
//}
//
//const float test_duty_cycles[] = {
//        0.f,
//        0.1f,
//        0.5f,
//        0.9f,
//        1.f
//};
//
//
//void setup() 
//{
//  //  stdio_init_all();
//    Serial1.begin(115200);
//    Serial1.println("Starting!");
//
//    //gpio_set_function(0, GPIO_FUNC_UART);
//    //gpio_set_function(1, GPIO_FUNC_UART);
//    //uart_set_translate_crlf(uart0, true);
//    //uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
//    //uart_init(uart0, 115200);
//    //sleep_ms(700);
//
//    //printf("\nPWM duty cycle measurement example\n");
//
//    // Configure PWM slice and set it running
//    const uint count_top = 200;
//    pwm_config cfg = pwm_get_default_config();
//    pwm_config_set_wrap(&cfg, count_top);
//    pwm_init(pwm_gpio_to_slice_num(OUTPUT_PIN), &cfg, true);
//
//    // Обратите внимание, что мы пока не трогаем другой вывод — выводы ШИМ являются выходами по умолчанию, 
//    // но меняются на входы после изменения режима делителя с
//   // свободного хода. Неразумно подключать два выхода напрямую вместе!
//    gpio_set_function(OUTPUT_PIN, GPIO_FUNC_PWM); //
//
//    // Для каждого из наших тестовых рабочих циклов управляйте выходным штифтом на этом уровне,
//   // и считывайте фактический выходной рабочий цикл с помощью другого штифта. Два
//   // значения должны быть очень близки!
//    //for (int i = 0; i < count_of(test_duty_cycles); ++i) 
//    //{
//    //    float output_duty_cycle = test_duty_cycles[i];
//    //    pwm_set_gpio_level(OUTPUT_PIN, (uint16_t)(output_duty_cycle * (count_top + 1)));
//    //    float measured_duty_cycle = measure_duty_cycle(MEASURE_PIN);
//    //    //printf("Output duty cycle = %.1f%%, measured input duty cycle = %.1f%%\n",
//    //    //    output_duty_cycle * 100.f, measured_duty_cycle * 100.f);
//    //}
//
//    float output_duty_cycle = test_duty_cycles[2];
//    pwm_set_gpio_level(OUTPUT_PIN, (uint16_t)(output_duty_cycle * (count_top + 1)));
//
//    //delay(1);
//    pwm_set_enabled(OUTPUT_PIN, false);
//   
//   //// pwm_set_counter(OUTPUT_PIN, 10);
//    int count_imp = pwm_get_counter(OUTPUT_PIN);
//    Serial1.println(count_imp);
//
//    //if (count_imp == 10)
//    //{
//
//    //    Serial1.println(count_imp);
//    //    pwm_set_enabled(OUTPUT_PIN, false);
//    //}
//}

void loop() 
{
  //  sleep_ms(10);

    //digitalWrite(GEN_PWM_SIGNAL_PIN1, HIGH);
    //digitalWrite(GEN_PWM_SIGNAL_PIN1, LOW);


    //int count_imp = pwm_get_counter(OUTPUT_PIN);
    //Serial1.println(count_imp);

    //if (count_imp == 1000)
    //{

    //    Serial1.println(count_imp);
    //  //  pwm_set_enabled(OUTPUT_PIN, false);
    //}
}


/*
* // Сообщаем GPIO 0 и 1, что они выделены для ШИМ
gpio_set_function(GEN_PWM_SIGNAL_PIN1, GPIO_FUNC_PWM);
gpio_set_function(GEN_PWM_SIGNAL_PIN2, GPIO_FUNC_PWM);

// Выясняем, какой слайс ШИМ подключен к GPIO 0 (это слайс 0)
uint slice_num = pwm_gpio_to_slice_num(GEN_PWM_SIGNAL_PIN1);

pwm_set_clkdiv(slice_num, 125); // Теперь тактовая частота ШИМ должна быть 1 МГц

// Устанавливаем период в 4 цикла (от 0 до 3 включительно)
pwm_set_wrap(slice_num, 3);
// Установить выход канала A на высокий уровень на один цикл перед сбросом
pwm_set_chan_level(slice_num, PWM_CHAN_A, 1);
// Установить начальный выход B на высокий уровень на три цикла перед сбросом
pwm_set_chan_level(slice_num, PWM_CHAN_B, 3);
// Установить работу ШИМ
pwm_set_enabled(slice_num, true);

// Обратите внимание, что мы также могли бы использовать pwm_set_gpio_level(gpio, x), который ищет
// правильный срез и канал для данного GPIO.
* 
* 
 // Tell GPIO 0 and 1 they are allocated to the PWM
  gpio_set_function(GEN_PWM_SIGNAL_PIN1, GPIO_FUNC_PWM);
  gpio_set_function(GEN_PWM_SIGNAL_PIN2, GPIO_FUNC_PWM);

  // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
  uint slice_num = pwm_gpio_to_slice_num(GEN_PWM_SIGNAL_PIN1);

  pwm_set_clkdiv(slice_num, 125); // pwm clock should now be running at 1MHz

  // Set period of 4 cycles (0 to 3 inclusive)
  pwm_set_wrap(slice_num, 3);
  // Set channel A output high for one cycle before dropping
  pwm_set_chan_level(slice_num, PWM_CHAN_A, 1);
  // Set initial B output high for three cycles before dropping
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 3);
  // Set the PWM running
  pwm_set_enabled(slice_num, true);

  // Note we could also use pwm_set_gpio_level(gpio, x) which looks up the
  // correct slice and channel for a given GPIO.
*/