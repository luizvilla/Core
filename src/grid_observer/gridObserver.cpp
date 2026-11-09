// SPDX-License-Identifier: LGPL-2.1
#include "gridObserver.h"

#include <math.h>

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
    _configured = true;
    return true;
}

bool GridObserver::update(float voltage_alpha, float voltage_beta,
                          float angular_frequency, bool measurements_valid) {
    if (!_configured) return false;
    constexpr float inverse_sqrt_two = 0.7071067811865475F;
    constexpr float inverse_two_pi = 0.1591549430918953F;
    const bool finite = isfinite(voltage_alpha) && isfinite(voltage_beta) && isfinite(angular_frequency);
    return updateEstimates(hypotf(voltage_alpha * inverse_sqrt_two, voltage_beta * inverse_sqrt_two),
                           angular_frequency * inverse_two_pi, measurements_valid && finite);
}

bool GridObserver::updateEstimates(float voltage_rms, float frequency_hz, bool measurements_valid) {
    if (!_configured) return false;
    _voltage_rms = voltage_rms;
    _frequency_hz = frequency_hz;
    if (!measurements_valid || !isfinite(_voltage_rms) || !isfinite(_frequency_hz)) {
        _voltage_rms = _frequency_hz = NAN;
        _healthy_elapsed = 0.0;
        _qualified = false;
        for (double &elapsed : _violation_elapsed) elapsed = 0.0;
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
    const double sample_period = static_cast<double>(_config.sample_period_s);
    const double qualification_time = static_cast<double>(_config.qualification_time_s);
    // Keep elapsed times bounded and use double accumulation to avoid float
    // accumulation drift at the 10 kHz control rate.
    if (healthy) {
        if (_healthy_elapsed < qualification_time)
            _healthy_elapsed += sample_period;
        if (_healthy_elapsed >= qualification_time) _qualified = true;
    } else {
        _healthy_elapsed = 0.0;
        // While tripped, recovery must be consecutive; during normal operation
        // preserve qualification until the configured violation delay expires.
        if (isTripped()) _qualified = false;
    }
    for (unsigned int i = 0; i < 4; ++i) {
        if (!violations[i]) {
            _violation_elapsed[i] = 0.0;
            continue;
        }
        const double delay = static_cast<double>(i < 2 ? _config.voltage_trip_delay_s
                                                      : _config.frequency_trip_delay_s);
        if (_violation_elapsed[i] < delay)
            _violation_elapsed[i] += sample_period;
        // An unacceptable grid at startup inhibits connection without latching
        // a trip. Protection becomes armed after the first healthy qualification.
        if (_qualified && _violation_elapsed[i] >= delay) trip(reasons[i]);
    }
    return canOperate();
}

void GridObserver::trip(GridTripReason reason) {
    if (isTripped()) return;
    _trip_reason = reason;
    _qualified = false;
    _healthy_elapsed = 0.0;
}

bool GridObserver::clearTrip(bool grid_connected) {
    if (!_configured || grid_connected || !_qualified) return false;
    _trip_reason = GridTripReason::None;
    return true;
}
