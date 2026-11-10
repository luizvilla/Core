// SPDX-License-Identifier: LGPL-2.1
#ifndef GRID_OBSERVER_H
#define GRID_OBSERVER_H

#include <stdint.h>

/** @brief Application-supplied protection settings; voltages are RMS, times seconds.
 * No grid-code settings are assumed. Validate these settings for the installation.
 */
struct GridObserverConfig {
    float sample_period_s;
    float voltage_min_rms;
    float voltage_max_rms;
    float frequency_min_hz;
    float frequency_max_hz;
    float voltage_trip_delay_s;
    float frequency_trip_delay_s;
    float qualification_time_s;
};

enum class GridTripReason {
    None,
    InvalidMeasurement,
    UnderVoltage,
    OverVoltage,
    UnderFrequency,
    OverFrequency
};

/** @brief Passive grid voltage/frequency supervision for a FOLLOWING VSI.
 *
 * Consume singlePhaseInverterFollowing::getVab() and getw() after calculateDuty(), then
 * pass canOperate() to setGridQuality() before applying PWM/relay requests.
 * SOGI alpha/beta are instantaneous quadrature signals, not RMS values:
 * Vrms = hypot(alpha, beta) / sqrt(2), for a sinusoidal grid after SOGI settling.
 * Frequency is the PLL estimate converted from rad/s to Hz. Filter/PLL latency
 * is additional to the configured trip delays. FORMING frequency is a reference
 * and must not be used as a grid-frequency measurement.
 *
 * Startup requires consecutive healthy samples. Once qualified, each voltage
 * and frequency limit has its own consecutive-violation timer. Brief violations
 * retain permission until their delay expires. Invalid/stale measurements trip
 * immediately. A trip retains its first cause until explicitly cleared with the
 * connection open and a newly qualified healthy grid. All calls belong in the
 * control task. The caller must update once per configured period and signal
 * stale measurements using measurements_valid=false; this class has no clock.
 *
 * This is a passive protection building block, not complete anti-islanding:
 * an island can maintain acceptable voltage/frequency. It neither commands
 * hardware nor implements active islanding detection or grid-code certification.
 */
class GridObserver {
public:
    /** @brief Configure and reset. Invalid settings leave operation inhibited. */
    bool init(const GridObserverConfig &config);

    /** @brief Advance one sample using SOGI volts and PLL angular frequency.
     * @return Current permission, including qualification and latched-trip state.
     */
    bool update(float voltage_alpha, float voltage_beta, float angular_frequency,
                bool measurements_valid = true);

    /** @brief Advance one sample from RMS voltage and frequency estimates computed by the
     * caller (for example one-period averages, which a real grid's harmonics do not ripple).
     * @param voltage_rms Fundamental RMS voltage, in V.
     * @param frequency_hz Grid frequency, in Hz.
     * @return Current permission, as update().
     */
    bool updateEstimates(float voltage_rms, float frequency_hz, bool measurements_valid = true);

    bool canOperate() const { return _configured && _qualified && !isTripped(); }
    bool isTripped() const { return _trip_reason != GridTripReason::None; }
    GridTripReason getTripReason() const { return _trip_reason; }
    float getVoltageRms() const { return _voltage_rms; }
    float getFrequencyHz() const { return _frequency_hz; }

    /** @brief Clear a trip only when disconnected and healthy for the full dwell.
     * The VSI fault must also be cleared explicitly by the application.
     */
    bool clearTrip(bool grid_connected);

private:
    void trip(GridTripReason reason);
    GridObserverConfig _config{};
    bool _configured = false;
    bool _qualified = false;
    /* Elapsed times as integer sample counts (exact; no software double arithmetic in the 10 kHz task). */
    uint32_t _healthy_ticks = 0U;
    uint32_t _violation_ticks[4]{};
    uint32_t _qualification_ticks = 0U;   ///< Samples of healthy grid needed to qualify (set by init).
    uint32_t _trip_ticks[2]{};            ///< Samples of violation before a voltage (0) or frequency (1) trip.
    GridTripReason _trip_reason = GridTripReason::None;
    float _voltage_rms = 0.0F;
    float _frequency_hz = 0.0F;
};

#endif // GRID_OBSERVER_H
