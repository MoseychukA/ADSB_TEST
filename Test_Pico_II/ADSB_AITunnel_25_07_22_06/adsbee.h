#ifndef _ADS_BEE_H_
#define _ADS_BEE_H_

#include "aircraft_dictionary.h"
#include "bsp.h"
#include "data_structures.h"  // For PFBQueue.
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"
#include "macros.h"  // For MAX / MIN.
#include "settings.h"
#include "stdint.h"
#include "transponder_packet.h"
#include "comms.h"

class ADSBee 
{
   public:
    static constexpr uint16_t kTLMaxPWMCount = 5000;  // Тактовая частота 125 МГц, рекомендуемая частота ШИМ 25 кГц.
    static constexpr int kVDDMV = 3300;               // [mV] Voltage of positive supply rail.
    static constexpr int kTLMaxMV = 3300;             // [mV]
    static constexpr int kTLMinMV = 0;                // [mV]
    static constexpr uint32_t kStatusLEDOnMs = 10;

    static constexpr uint32_t kTLLearningIntervalMs         = 10000;  // [ms] Length of Simulated Annealing interval for learning trigger level.
    static constexpr uint16_t kTLLearningNumCycles          = 100;    // Number of simulated annealing cycles for learning trigger level.
    static constexpr uint16_t kTLLearningStartTemperatureMV = 1000;   // [mV] Starting value for simulated annealing temperature when learning triger level. This corresponds
               // to the maximum value that the trigger level could be moved (up or down) when exploring a neighbor
               // state.

    static constexpr int32_t kNoiseFloorExpoFilterPercent    = 50;  // [%] Weight to use for low pass expo filter of noise floor ADC counts. 0 = no filter, 100 = hold value.
    static constexpr uint32_t kNoiseFloorADCSampleIntervalMs = 1;   // [ms] Interval between ADC samples to approximate noise floor value.

    struct ADSBeeConfig 
    {
        PIO preamble_detector_pio = pio0;
        uint preamble_detector_demod_pin_irq = IO_IRQ_BANK0;
        PIO message_demodulator_pio = pio1;
        uint preamble_detector_demod_complete_irq = PIO0_IRQ_0;

        uint16_t r1090_led_pin = 25;
        // Чтение ADS-B на GPIO19. Будет искать DE// Чтение ADS-B на GPIO19. Будет искать сигнал DEMOD на GPIO20.Сигнал MOD на GPIO20.
        uint16_t* pulses_pins = bsp.r1090_pulses_pins;
        uint16_t* demod_pins = bsp.r1090_demod_pins;
        // Используйте GPIO22 для декодирования программы PIO, чтобы вывести ее восстановленные часы (только для отладки).
        uint16_t* recovered_clk_pins = bsp.r1090_recovered_clk_pins;  // Установите RECOVERED_CLK на фальшивый вывод для детектора преамбулы высокой мощности. Будет
                                                                      // переопределено более высоким приоритетом (более низким индексом) SM.
        // GPIO 24-25 используются как выходы ШИМ для установки пороговых напряжений аналогового компаратора.
        uint16_t tl_pwm_pin = bsp.r1090_tl_pwm_pin;
        // GPIO 26-27 используются как входы АЦП для считывания пороговых напряжений аналогового компаратора после фильтра ВЧ.
        uint16_t tl_adc_pin = bsp.r1090_tl_adc_pin;
        uint16_t tl_adc_input = bsp.r1090_tl_adc_input;
        // GPIO 28 используется как вход АЦП для уровня мощности последнего декодированного пакета.
        uint16_t rssi_adc_pin = bsp.r1090_rssi_adc_pin;
        uint16_t rssi_adc_input = bsp.r1090_rssi_adc_input;
        // GPIO 2-3 используются для шины I2C EEPROM и цифрового потенциометра усиления RX через I2C1.
        i2c_inst_t* onboard_i2c = bsp.onboard_i2c;
        uint16_t onboard_i2c_sda_pin = bsp.onboard_i2c_sda_pin;
        uint16_t onboard_i2c_scl_pin = bsp.onboard_i2c_scl_pin;
        uint32_t onboard_i2c_clk_freq_hz = bsp.onboard_i2c_clk_freq_hz;  // 400kHz
        bool onboard_i2c_requires_init =
            bsp.onboard_i2c_requires_init;  // In case I2c is shared with something else that already initializes it.

        uint16_t bias_tee_enable_pin = 18;

        uint32_t aircraft_dictionary_update_interval_ms = 1000;
    };

    ADSBee(ADSBeeConfig config_in);
    bool Init();
    bool Update();

    /**
     * Inlne helper function that converts milliVolts at the AD8313 input to a corresponding value in dBm, using values
     * from the AD8313 datasheet.
     * @param[in] mv Voltage level, in milliVolts.
     * @retval Corresponding power level, in dBm.
     */
    static inline int AD8313MilliVoltsTodBm(int mv) 
    {
        static constexpr uint16_t kLNAGaindB = 44;    // Gain of 2x LNAs in front of the AD8313, from bench testing.
        return 60 * (mv - 1600) / 1000 - kLNAGaindB;  // AD8313 0dBm intercept at 1.6V, slope is 60dBm/V.
    }

    /**
     * Inline helper function that converts ADC counts on theRP2040 to milliVolts.
     * @param[in] adc_counts ADC counts, 0 to 4095.
     * @retval Voltage in milliVolts.
     */
    static inline int ADCCountsToMilliVolts(uint16_t adc_counts) { return 3300 * adc_counts / 0xFFF; }

    /**
     * Returns whether the bias tee is enabled.
     * @retval True if bias tee is enabled, false otherwise.
     */
    bool BiasTeeIsEnabled() { return bias_tee_enabled_; }

    /**
     * Convenience function for temporarily disabling the watchdog without changing its timeout.
     */
    void DisableWatchdog() { watchdog_disable(); }

    /**
     * Convenience function for re-enabling the watchdog with the last used timeout.
     */
    void EnableWatchdog() { SetWatchdogTimeoutSec(watchdog_timeout_sec_); }

    /**
     * Blinks the status LED for a given number of milliseconds. Non-blocking.
     * @param[in] led_on_ms Optional parameter specifying number of milliseconds to turn on for. Defaults to
     * kStatusLEDOnMs.
     */
    void FlashStatusLED(uint32_t led_on_ms = kStatusLEDOnMs);

    /**
     * Creates a composite timestamp using the current value of the SysTick timer (running at 125MHz) and the SysTick
     * wrap counter to simulate a timer running at 48MHz (which matches the frequency of the preamble detector PIO).
     * @param[in] num_bits Number of bits to mask the counter value to. Defaults to full resolution.
     * @retval 48MHz counter value.
     */
    uint64_t GetMLAT48MHzCounts(uint16_t num_bits = 64);

    /**
     * Creates a composite timestamp using the current value of the SysTick timer (running at 125MHz) and the SysTick
     * wrap counter to simulate a timer running at 12MHz, which matches existing decoders that use the Mode S Beast
     * protocol.
     * @param[in] num_bits Number of bits to mask the counter value to. Defaults to 48 bits (6 Bytes) to match Mode S
     * Beast protocol.
     * @retval 48MHz counter value.
     */
    uint64_t GetMLAT12MHzCounts(uint16_t num_bits = 48);

    /**
     * Returns the power level of the noise floor (signal strength sampled mostly during non-decode intervals and then
     * low-pass filtered).
     * @retval Power level of the noise floor, in dBm.
     */
    int GetNoiseFloordBm();

    /**
     * Get the current temperature used in learning trigger level (simulated annealing). A temperature of 0 means
     * learning has completed.
     * @retval Current temperature used for simulated annealing, in milliVolts.
     */
    uint16_t GetTLLearningTemperatureMV();

    /**
     * Return the value of the low Minimum Trigger Level threshold in milliVolts.
     * @retval TL in milliVolts.
     */
    int GetTLMilliVolts() { return tl_mv_; }

    inline uint32_t GetWatchdogTimeoutSec() { return watchdog_timeout_sec_; }

    /**
     * ISR for GPIO interrupts.
     */
    void OnDemodBegin(uint gpio);

    /**
     * ISR triggered by DECODE completing, via PIO0 IRQ0.
     */
    void OnDemodComplete();

    /**
     * ISR triggered by SysTick interrupt. Used to wrap the MLAT counter.
     */
    void OnSysTickWrap();

    /**
     * Resets the watchdog counter to the value set in SetWatchdogTimeoutSec().
     */
    inline void PokeWatchdog() { watchdog_update(); }

    /**
     * Returns the Receive Signal Strength Indicator (RSSI) of the signal currently provided by the RF power detector,
     * in mV.
     * @retval Voltage from the RF power detector, in mV.
     */
    inline int ReadSignalStrengthMilliVolts();

    /**
     * Returns the Receive Signal Strength Indicator (RSSI) of the message that is currently being provided by the RF
     * power detector, in dBm. makes use of ReadSignalStrengthMilliVolts().
     * @retval Voltage form the RF power detector converted to dBm using the chart in the AD8313 datasheet.
     */
    inline int ReadSignalStrengthdBm();

    /**
     * Read the low Minimum Trigger Level threshold via ADC.
     * @retval TL in milliVolts.
     */
    int ReadTLMilliVolts();

    /**
     * Reboots the RP2040 via the watchdog.
     * @param[in] delay_ms Milliseconds to wait befor rebooting. Defaults to 0 (immediate reboot).
     */
    inline void Reboot(uint16_t delay_ms = 0) { watchdog_reboot(0, 0, delay_ms); }

    /**
     * Returns whether ADS-B receiving is currently enabled.
     * @retval True if enabled, false otherwise.
     */
    bool Receiver1090IsEnabled() { return r1090_enabled_; }


    /**
     * Enable or disable the bias tee to inject 3.3V at the RF IN connector.
     * @param[in] is_enabled True to enable the bias tee, false otherwise.
     */
    inline void SetBiasTeeEnable(bool is_enabled) {
        bias_tee_enabled_ = is_enabled;
        gpio_put(config_.bias_tee_enable_pin, !bias_tee_enabled_);
    }

    /**
    * Включает или выключает приемник ADS-B, захватывая прерывание завершения демодуляции.
    * @param[in] is_enabled True, если приемник ADS-B должен быть включен, false в противном случае.
    */
    inline void SetReceiver1090Enable(bool is_enabled) 
    {
        r1090_enabled_ = is_enabled;
        irq_set_enabled(config_.preamble_detector_demod_complete_irq, r1090_enabled_);
    }
    /**
    * Устанавливает светодиод состояния в заданное состояние. Не записывает временные метки выключения светодиода.
    * @param[in] on True для включения светодиода, false для выключения.
    */
    inline void SetStatusLED(bool on) { gpio_put(config_.r1090_led_pin, on ? 1 : 0); }

    /**
   * Установите минимальный уровень срабатывания (TL) на выходе AD8314 в милливольтах.
   * @param[in] tl_mv Напряжение в милливольтах в верхней части подтяжки для сети LEVEL в слайсере данных. Подтяните выше
   * для размещения более высокого уровня шума без ложных срабатываний.
   * @retval True в случае успеха, False, если значение TL вышло за пределы диапазона.
   */
    bool SetTLMilliVolts(int tl_mv);

    /**
   * Устанавливает таймер сторожевого таймера и включает его.
   * @param[in] watchdog_timeout_sec Максимальный интервал между вызовами PokeWatchdog() до истечения времени ожидания сторожевого таймера и
   * запуска перезагрузки. 0 = watchodg отключен. Обратите внимание, что это значение указывается в секундах, чтобы, как можно надеяться, предотвратить
   * неустранимый цикл, когда таймер сторожевого таймера устанавливается слишком коротким и приводит к перезагрузке устройства до того, как настройка
   * может быть изменена.
   * @retval True, если установлено успешно, false, если недопустимое значение сторожевого таймера.
   */
    inline bool SetWatchdogTimeoutSec(uint32_t watchdog_timeout_sec) 
    {
        if (watchdog_timeout_sec > UINT32_MAX / kMsPerSec) 
        {
            return false;  // Watchdog timeout value too big.
        }
        watchdog_timeout_sec_ = watchdog_timeout_sec;
        if (watchdog_timeout_sec_ > 0) 
        {
            watchdog_enable(watchdog_timeout_sec_ * kMsPerSec, true);  // Pause the watchdog timer during debug.
        }
        else 
        {
            watchdog_disable();
        }
        return true;
    }

    /**
    * Запуск обучения уровня срабатывания через имитацию отжига. Начнется kTLLearningNumCycles циклов отжига
    * с интервалом отжига kTLLearningIntervalMs миллисекунд. Может быть предоставлен с максимальными и минимальными
    * границами уровня срабатывания для более узкого поиска.
    * @param[in] tl_learning_num_cycles Количество циклов, используемых при уровне срабатывания отжига (устанавливает величину, на которую
    * уменьшается температура отжига в каждом цикле). Необязательно, по умолчанию kTLLearningNumCycles.
    * @param[in] tl_learning_start_temperature_mv Температура отжига для начала, в мВ.
    * @param[in] tl_min_mv Минимальный уровень срабатывания, используемый при обучении, в милливольтах. Необязательно, по умолчанию полная шкала
    * (kTLMinMV).
    * @param[in] tl_max_mv Максимальный уровень срабатывания для использования при обучении, в милливольтах. Необязательно, по умолчанию — полная шкала
    * (kTLMaxMV).
    */
    void StartTLLearning(uint16_t tl_learning_num_cycles = kTLLearningNumCycles,
                         uint16_t tl_learning_start_temperature_mv = kTLLearningStartTemperatureMV,
                         uint16_t tl_min_mv = kTLMinMV, uint16_t tl_max_mv = kTLMaxMV);

    PFBQueue<Raw1090Packet> raw_1090_packet_queue = PFBQueue<Raw1090Packet>({.buf_len_num_elements = SettingsManager::Settings::kMaxNumTransponderPackets, .buffer = raw_1090_packet_queue_buffer_});

    AircraftDictionary aircraft_dictionary;
   
   private:
    ADSBeeConfig config_;
  
    uint32_t irq_wrapper_sm_ = 0;
    uint32_t preamble_detector_sm_[BSP::kMaxNumDemodStateMachines];
    uint32_t preamble_detector_offset_ = 0;

    uint32_t irq_wrapper_offset_ = 0;

    uint32_t message_demodulator_sm_[BSP::kMaxNumDemodStateMachines];
    uint32_t message_demodulator_offset_ = 0;

    uint32_t led_on_timestamp_ms_ = 0;

    uint16_t tl_pwm_slice_ = 0;
    uint16_t tl_pwm_chan_ = 0;

    uint16_t tl_mv_ = SettingsManager::Settings::kDefaultTLMV;
    uint16_t tl_pwm_count_ = 0;  // out of kTLMaxPWMCount

    uint16_t tl_adc_counts_ = 0;

    uint32_t tl_learning_cycle_start_timestamp_ms_ = 0;
    uint16_t tl_learning_temperature_mv_ = 0;  // Don't learn automatically.
    int16_t tl_learning_temperature_step_mv_ = 0;
    uint16_t tl_learning_max_mv_ = kTLMaxMV;
    uint16_t tl_learning_min_mv_ = kTLMinMV;
    int16_t tl_learning_num_valid_packets_ = 0;
    int16_t tl_learning_prev_num_valid_packets_ = 1;  // Set to 1 to avoid dividing by 0.
    uint16_t tl_learning_prev_tl_mv_ = tl_mv_;

    uint64_t mlat_counter_wraps_ = 0;

    Raw1090Packet rx_packet_[BSP::kMaxNumDemodStateMachines];
    Raw1090Packet raw_1090_packet_queue_buffer_[SettingsManager::Settings::kMaxNumTransponderPackets];

    uint32_t last_aircraft_dictionary_update_timestamp_ms_ = 0;

    bool r1090_enabled_ = true;
    bool bias_tee_enabled_ = false;
    uint32_t watchdog_timeout_sec_ = SettingsManager::Settings::kDefaultWatchdogTimeoutSec * kMsPerSec;

    int32_t noise_floor_mv_;
    uint32_t noise_floor_last_sample_timestamp_ms_ = 0;

};

extern ADSBee adsbee;

#endif /* _ADS_BEE_HH_ */