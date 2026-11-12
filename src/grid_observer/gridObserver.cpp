// SPDX-License-Identifier: LGPL-2.1
#include "gridObserver.h"

#include <math.h>
#include "fast_code.h"

bool GridObserver::init(const GridObserverConfig &config) {
    *this = GridObserver{};
    const float values[] = {
        config.sample_period_s, config.voltage_min_rms, config.voltage_max_rms,
        config.frequency_min_hz, config.frequency_max_hz,
        config.voltage_trip_delay_s, config.frequency_trip_delay_s,
        config.qualification_time_s
    };
    for (float value : values) {
        if (!isfinite(value)) return false;
    }
    if (config.sample_period_s <= 0.0F || config.voltage_min_rms <= 0.0F ||
        config.voltage_max_rms <= config.voltage_min_rms ||
        config.frequency_min_hz <= 0.0F ||
        config.frequency_max_hz <= config.frequency_min_hz ||
        config.voltage_trip_delay_s < 0.0F || config.frequency_trip_delay_s < 0.0F ||
        config.qualification_time_s < 0.0F) return false;
    _config = config;
    /* Durations to sample counts once (double here only, at init): elapsed >= delay after n samples of
     * sample_period, as the former double accumulation, rounded against float noise. */
    const auto ticks = [&](float duration) {
        return static_cast<uint32_t>(ceil(static_cast<double>(duration) / static_cast<double>(config.sample_period_s) - 1e-6));
    };
    _qualification_ticks = ticks(config.qualification_time_s);
    _trip_ticks[0] = ticks(config.voltage_trip_delay_s);
    _trip_ticks[1] = ticks(config.frequency_trip_delay_s);
    _configured = true;
    return true;
}

FAST_CODE bool GridObserver::update(float voltage_alpha, float voltage_beta,
                          float angular_frequency, bool measurements_valid) {
    if (!_configured) return false;
    constexpr float inverse_sqrt_two = 0.7071067811865475F;
    constexpr float inverse_two_pi = 0.1591549430918953F;
    const bool finite = isfinite(voltage_alpha) && isfinite(voltage_beta) && isfinite(angular_frequency);
    return updateEstimates(hypotf(voltage_alpha * inverse_sqrt_two, voltage_beta * inverse_sqrt_two),
                           angular_frequency * inverse_two_pi, measurements_valid && finite);
}

FAST_CODE bool GridObserver::updateEstimates(float voltage_rms, float frequency_hz, bool measurements_valid) {
    if (!_configured) return false;
    _voltage_rms = voltage_rms;
    _frequency_hz = frequency_hz;
    if (!measurements_valid || !isfinite(_voltage_rms) || !isfinite(_frequency_hz)) {
        _voltage_rms = _frequency_hz = NAN;
        _healthy_ticks = 0U;
        _qualified = false;
        for (uint32_t &n : _violation_ticks) n = 0U;
        trip(GridTripReason::InvalidMeasurement);
        return false;
    }

    const bool violations[] = {
        _voltage_rms < _config.voltage_min_rms,
        _voltage_rms > _config.voltage_max_rms,
        _frequency_hz < _config.frequency_min_hz,
        _frequency_hz > _config.frequency_max_hz
    };
    const GridTripReason reasons[] = {
        GridTripReason::UnderVoltage, GridTripReason::OverVoltage,
        GridTripReason::UnderFrequency, GridTripReason::OverFrequency
    };
    const bool healthy = !violations[0] && !violations[1] &&
                         !violations[2] && !violations[3];
    // Elapsed times are bounded sample counts (exact, cheap at the 10 kHz control rate).
    if (healthy) {
        if (_healthy_ticks < _qualification_ticks) ++_healthy_ticks;
        if (_healthy_ticks >= _qualification_ticks) _qualified = true;
    } else {
        _healthy_ticks = 0U;
        // While tripped, recovery must be consecutive; during normal operation
        // preserve qualification until the configured violation delay expires.
        if (isTripped()) _qualified = false;
    }
    for (unsigned int i = 0; i < 4; ++i) {
        if (!violations[i]) {
            _violation_ticks[i] = 0U;
            continue;
        }
        const uint32_t delay = _trip_ticks[i < 2 ? 0 : 1];
        if (_violation_ticks[i] < delay) ++_violation_ticks[i];
        // An unacceptable grid at startup inhibits connection without latching
        // a trip. Protection becomes armed after the first healthy qualification.
        if (_qualified && _violation_ticks[i] >= delay) trip(reasons[i]);
    }
    return canOperate();
}

void GridObserver::trip(GridTripReason reason) {
    if (isTripped()) return;
    _trip_reason = reason;
    _qualified = false;
    _healthy_ticks = 0U;
}

bool GridObserver::clearTrip(bool grid_connected) {
    if (!_configured || grid_connected || !_qualified) return false;
    _trip_reason = GridTripReason::None;
    return true;
}
