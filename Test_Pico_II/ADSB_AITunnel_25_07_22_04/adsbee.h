#ifndef _ADS_BEE_H_
#define _ADS_BEE_H_

//#include "aircraft_dictionary.h"
#include "bsp.h"
#include "data_structures.h"  // For PFBQueue.
//#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"
//#include "settings.h"
#include "stdint.h"
#include "transponder_packet.h"
//#include "comms.h"

class ADSBee
{
public:
    static constexpr uint16_t kTLMaxPWMCount = 5000;  // �������� ������� 125 ���, ������������� ������� ��� 25 ���.
    static constexpr int kVDDMV = 3300;               // [mV] Voltage of positive supply rail.
    static constexpr int kTLMaxMV = 3300;             // [mV]
    static constexpr int kTLMinMV = 0;                // [mV]
    static constexpr uint32_t kStatusLEDOnMs = 10;

    static constexpr uint32_t kTLLearningIntervalMs = 10000;  // [ms] Length of Simulated Annealing interval for learning trigger level.
    static constexpr uint16_t kTLLearningNumCycles = 100;    // Number of simulated annealing cycles for learning trigger level.
    static constexpr uint16_t kTLLearningStartTemperatureMV = 1000;   // [mV] Starting value for simulated annealing temperature when learning triger level. This corresponds
               // to the maximum value that the trigger level could be moved (up or down) when exploring a neighbor
               // state.

    static constexpr int32_t kNoiseFloorExpoFilterPercent = 50;  // [%] Weight to use for low pass expo filter of noise floor ADC counts. 0 = no filter, 100 = hold value.
    static constexpr uint32_t kNoiseFloorADCSampleIntervalMs = 1;   // [ms] Interval between ADC samples to approximate noise floor value.

    struct ADSBeeConfig
    {
        PIO preamble_detector_pio = pio0;
        uint preamble_detector_demod_pin_irq = IO_IRQ_BANK0;
        PIO message_demodulator_pio = pio1;
        uint preamble_detector_demod_complete_irq = PIO0_IRQ_0;

        uint16_t r1090_led_pin = 15;
        // ������ ADS-B �� GPIO19. ����� ������ DE// ������ ADS-B �� GPIO19. ����� ������ ������ DEMOD �� GPIO20.������ MOD �� GPIO20.
        uint16_t* pulses_pins = bsp.r1090_pulses_pins;
        uint16_t* demod_pins = bsp.r1090_demod_pins;
        // ����������� GPIO22 ��� ������������� ��������� PIO, ����� ������� �� ��������������� ���� (������ ��� �������).
        uint16_t* recovered_clk_pins = bsp.r1090_recovered_clk_pins;  // ���������� RECOVERED_CLK �� ��������� ����� ��� ��������� ��������� ������� ��������. �����
                                                                      // �������������� ����� ������� ����������� (����� ������ ��������) SM.

        uint16_t tl_pwm_pin = 9;      // GPIO 9 ������������ ��� ������ ��� ��� ��������� ��������� ���������� ����������� �����������.
        uint16_t tl_adc_pin = 27;           // GPIO 26-27 ������������ ��� ����� ��� ��� ���������� ��������� ���������� ����������� ����������� ����� ������� ��.
        uint16_t tl_adc_input = 1;
        uint16_t rssi_adc_pin = 28;         // GPIO 28 ������������ ��� ���� ��� ��� ������ �������� ���������� ��������������� ������.
        uint16_t rssi_adc_input = 2;
        uint16_t bias_tee_enable_pin = 18;
        uint32_t aircraft_dictionary_update_interval_ms = 1000;
    };

    ADSBee(ADSBeeConfig config_in);
    bool Init();
    // bool Update();

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
    * �������� ��� ��������� �������� ADS-B, ���������� ���������� ���������� �����������.
    * @param[in] is_enabled True, ���� �������� ADS-B ������ ���� �������, false � ��������� ������.
    */
    inline void SetReceiver1090Enable(bool is_enabled)
    {
        r1090_enabled_ = is_enabled;
        irq_set_enabled(config_.preamble_detector_demod_complete_irq, r1090_enabled_);
    }
    /**
    * ������������� ��������� ��������� � �������� ���������. �� ���������� ��������� ����� ���������� ����������.
    * @param[in] on True ��� ��������� ����������, false ��� ����������.
    */
    inline void SetStatusLED(bool on) { gpio_put(config_.r1090_led_pin, on ? 1 : 0); }

    /**
   * ���������� ����������� ������� ������������ (TL) �� ������ AD8314 � ������������.
   * @param[in] tl_mv ���������� � ������������ � ������� ����� �������� ��� ���� LEVEL � �������� ������. ��������� ����
   * ��� ���������� ����� �������� ������ ���� ��� ������ ������������.
   * @retval True � ������ ������, False, ���� �������� TL ����� �� ������� ���������.
   */
    bool SetTLMilliVolts(int tl_mv);

    /**
   * ������������� ������ ����������� ������� � �������� ���.
   * @param[in] watchdog_timeout_sec ������������ �������� ����� �������� PokeWatchdog() �� ��������� ������� �������� ����������� ������� �
   * ������� ������������. 0 = watchodg ��������. �������� ��������, ��� ��� �������� ����������� � ��������, �����, ��� ����� ���������, �������������
   * ������������ ����, ����� ������ ����������� ������� ��������������� ������� �������� � �������� � ������������ ���������� �� ����, ��� ���������
   * ����� ���� ��������.
   * @retval True, ���� ����������� �������, false, ���� ������������ �������� ����������� �������.
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
    * ������ �������� ������ ������������ ����� �������� ������. �������� kTLLearningNumCycles ������ ������
    * � ���������� ������ kTLLearningIntervalMs �����������. ����� ���� ������������ � ������������� � ������������
    * ��������� ������ ������������ ��� ����� ������ ������.
    * @param[in] tl_learning_num_cycles ���������� ������, ������������ ��� ������ ������������ ������ (������������� ��������, �� �������
    * ����������� ����������� ������ � ������ �����). �������������, �� ��������� kTLLearningNumCycles.
    * @param[in] tl_learning_start_temperature_mv ����������� ������ ��� ������, � ��.
    * @param[in] tl_min_mv ����������� ������� ������������, ������������ ��� ��������, � ������������. �������������, �� ��������� ������ �����
    * (kTLMinMV).
    * @param[in] tl_max_mv ������������ ������� ������������ ��� ������������� ��� ��������, � ������������. �������������, �� ��������� � ������ �����
    * (kTLMaxMV).
    */
    void StartTLLearning(uint16_t tl_learning_num_cycles = kTLLearningNumCycles,
        uint16_t tl_learning_start_temperature_mv = kTLLearningStartTemperatureMV,
        uint16_t tl_min_mv = kTLMinMV, uint16_t tl_max_mv = kTLMaxMV);

    PFBQueue<Raw1090Packet> raw_1090_packet_queue = PFBQueue<Raw1090Packet>({ .buf_len_num_elements = 100, .buffer = raw_1090_packet_queue_buffer_ });

   //!! AircraftDictionary aircraft_dictionary;

private:

    ADSBeeConfig config_;
    static const uint16_t kMaxNumDemodStateMachines = 4;
    uint32_t irq_wrapper_sm_ = 0;
    uint32_t preamble_detector_sm_[/*BSP::*/kMaxNumDemodStateMachines];
    uint32_t preamble_detector_offset_ = 0;

    uint32_t irq_wrapper_offset_ = 0;

    uint32_t message_demodulator_sm_[/*BSP::*/kMaxNumDemodStateMachines];
    uint32_t message_demodulator_offset_ = 0;

    uint32_t led_on_timestamp_ms_ = 0;

    uint16_t tl_pwm_slice_ = 0;
    uint16_t tl_pwm_chan_ = 0;

    uint16_t tl_mv_ = 1300;
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

    Raw1090Packet rx_packet_[/*BSP::*/kMaxNumDemodStateMachines];
    Raw1090Packet raw_1090_packet_queue_buffer_[100];

    uint32_t last_aircraft_dictionary_update_timestamp_ms_ = 0;

    bool r1090_enabled_ = true;
    bool bias_tee_enabled_ = false;
    uint32_t watchdog_timeout_sec_ = 1000;

    int32_t noise_floor_mv_;
    uint32_t noise_floor_last_sample_timestamp_ms_ = 0;

};

extern ADSBee adsbee;

#endif /* _ADS_BEE_HH_ */
