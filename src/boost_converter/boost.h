/*
 * Copyright (c) 2021-2026 OWNTECH TECHNOLOGIES
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file boost.h
 * @brief Cascaded controller for two parallel unidirectional boost legs, with
 *        current-driven burst on leg 1 (docs/adr/0003-boost-adopts-current-driven-burst.md).
 */
#ifndef BOOST_H
#define BOOST_H

#include <stdint.h>
#include "pid.h"
#include "filters.h"
#include "pr.h"

/** @brief Operating modes of the boost controller. */
typedef enum {
    IDLE = 0,    ///< Disable control and clear every state.
    STARTUP = 1, ///< Current-limited bus charge on leg 1; automatically enters BUSREG at 95% of target.
    BUSREG = 2,  ///< Regulate bus voltage with the voltage/current cascade.
    MPPT = 3     ///< Track maximum input power by adjusting the total input current reference.
} boost_mode;

/** @brief Duty commands for the two parallel boost legs. */
struct BoostDutyCycles {
    float32_t leg1 = 0.0F; ///< Leg 1 duty cycle, or zero when disabled.
    float32_t leg2 = 0.0F; ///< Leg 2 duty cycle, or zero when disabled.
};

/**
 * @class BoostConverter
 * @brief Cascaded voltage and current control for two parallel unidirectional boost legs.
 * @details Update measurements through the setters before calling the loops.
 * Call coldLoop() at the configured cold-loop period and hotLoop() at the
 * configured hot-loop period. STARTUP/BUSREG use a voltage PID to supply the total
 * current reference; MPPT supplies it directly. Leg 1 receives up to leg1CurrentLimit,
 * leg 2 the excess; STARTUP uses leg 1 only, clamped to startupCurrentMax.
 *
 * **Burst (current-driven pulse skipping, leg 1).** Once configured with setBurst(),
 * light load is handled by skipping PWM periods on leg 1, out of a burst period of N
 * PWM periods. While the skip count is zero, leg 1's current PID runs; when its duty
 * sits at the lower bound and the current is still above its reference, the skip
 * count goes up by one per hot-loop call. While skipping, the PID is held (state kept,
 * duty held) and the skip count goes down by one per call while the current is below
 * its reference and up by one while above, within [0, N]. The PIDs are never reset by
 * burst. Entering an active mode from IDLE starts fully skipped (N). Leg 1's burst
 * follows its own current error independently of leg 2's reference. The caller
 * applies getBurstSkip() and getBurstPeriod() with
 * the hardware burst generator on leg 1 only.
 * The voltage PI uses symmetric internal bounds (+/- the mode's current ceiling).
 * Negative demand reduces leg 1 duty and increases skipping even at zero measured
 * current; delivered current references remain nonnegative. Active burst starts
 * with minimum duty, ready for the first unskipped pulse.
 *
 * **Protection.** Once configured with setProtection(): above fullBurstV the skip count
 * is forced to N; above tripV, or with either leg current above tripA, the controller
 * trips: legs disabled, outputs cleared, latched until setMode(IDLE).
 *
 * **Mode changes.** IDLE clears every state. From IDLE to an active mode, every state is
 * cleared and burst starts at N. Between active modes nothing is reset: PID states, the
 * total reference and the burst state carry over; only the bounds of the new mode apply.
 * STARTUP automatically enters BUSREG in coldLoop() when raw VBus reaches at least
 * 95% of a positive VBusRef. The transition is one-way and preserves control state.
 */
class BoostConverter {
public:
    /** @brief Construct an idle controller; call init() before enabling control. */
    BoostConverter() = default;
    ~BoostConverter();
    // Library filters own FIR buffers and must not be shallow-copied.
    BoostConverter(const BoostConverter&) = delete;
    BoostConverter& operator=(const BoostConverter&) = delete;

    /**
     * @brief Initialize the voltage PID and both current PIDs and reset control and MPPT history.
     * @param mode Initial operating mode.
     * @param V_bus Initial bus measurement and bus voltage reference, in V; must be positive.
     * @param voltageParams Voltage PID tuning; Ts is the cold-loop period in s.
     * Output bounds define the allowed nonnegative total current reference in A,
     * including the MPPT search limits. Internally the voltage PI uses symmetric
     * bounds around zero, with the mode's current ceiling as their magnitude.
     * @param currentParams Current PID tuning shared by both legs; Ts is the hot-loop period in s.
     * Output bounds define the duty-cycle range and must lie within [0, 1].
     * @param leg1CurrentLimit Positive current threshold in A. Leg 1 receives
     * min(total reference, threshold); leg 2 receives the remaining reference.
     * Initialization sets startupCurrentMax to min(leg1CurrentLimit, voltageParams.upper_bound).
     * @return 0 on success, or -EINVAL for invalid arguments or PID parameters.
     * @note Values must be finite. Failure leaves the controller disabled.
     * Both sampling periods must be positive, with voltageParams.Ts >= currentParams.Ts.
     * Burst, protection and gain-schedule settings are kept across init().
     * Ripple rejection must be configured again after init().
     */
    int8_t init(boost_mode mode, float32_t V_bus,
                const PidParams& voltageParams, const PidParams& currentParams,
                float32_t leg1CurrentLimit);

    /**
     * @brief Clear PID states, total and per-leg current references, both duties,
     * MPPT history and the trip; start burst fully skipped.
     * @details Preserve the mode, measurements, target voltage reference and settings.
     * With burst configured in an initialized active mode, prepare leg 1 at its
     * minimum duty for the first unskipped pulse; otherwise duties remain zero.
     */
    void reset();

    /**
     * @brief Update and distribute the total input current reference.
     * @details STARTUP and BUSREG regulate VBus toward VBusRef with the voltage PID
     * (its Kp first updated by the gain schedule when configured); MPPT calls Mppt().
     * Before regulation, STARTUP enters BUSREG when raw VBus >= 0.95 * VBusRef
     * and VBusRef > 0; BUSREG current bounds apply on that same call.
     * @return Total current reference in A within the voltage PID bounds, STARTUP
     * additionally clamped to startupCurrentMax. Zero if idle, uninitialized or tripped.
     */
    float32_t coldLoop();

    /**
     * @brief Check protection, update burst and run the current PIDs.
     * @return Per-leg duty cycles within currentParams output bounds when enabled.
     * Leg 2 is cleared (output and PID) when it has no demand. While burst skips
     * periods, leg 1's duty and PID are held.
     */
    BoostDutyCycles hotLoop();

    /**
     * @brief Update total input current demand using perturb-and-observe MPPT.
     * @details Compare VLow * (ILow1 + ILow2) with the previous power sample and reverse
     * the current perturbation direction when power decreases. Perturb by 0.1 A per call,
     * within the voltage PID current bounds; reverse direction at either bound.
     * Has no effect outside MPPT mode, before initialization, or when VLow <= 0.
     * @note coldLoop() calls this method automatically in MPPT mode.
     */
    void Mppt();

    /** @brief Get leg 1 input current. @return Measured current in A. */
    float32_t getILow1() const;
    /** @brief Get leg 2 input current. @return Measured current in A. */
    float32_t getILow2() const;
    /** @brief Get total input current. @return ILow1 + ILow2 in A. */
    float32_t getILowTotal() const;
    /** @brief Get the stored input voltage measurement. @return Low-side voltage in V. */
    float32_t getVLow() const;
    /** @brief Get the stored bus voltage measurement. @return Bus voltage in V. */
    float32_t getVBus() const;
    /** @brief Update leg 1 input current. @param ILow1 Current in A; non-finite values are ignored. */
    void setILow1(float32_t ILow1);
    /** @brief Update leg 2 input current. @param ILow2 Current in A; non-finite values are ignored. */
    void setILow2(float32_t ILow2);
    /** @brief Update the input voltage. @param VLow Voltage in V; negative or non-finite values are ignored. */
    void setVLow(float32_t VLow);
    /** @brief Update the raw bus voltage and hot-rate notch once per sample.
     * @param VBus Voltage in V; negative or non-finite values are ignored.
     */
    void setVBus(float32_t VBus);

    /**
     * @brief Configure and enable current-driven burst on leg 1.
     * @param period Burst period, in PWM periods (positive).
     * @return 0 on success, -EINVAL for a zero period (burst stays as it was).
     * @details Kept across init() and mode changes. Takes effect at the next hot loop;
     * an active mode entered from IDLE starts with all periods skipped.
     */
    int8_t setBurst(uint16_t period);
    /** @brief Check whether burst is configured. @return true when configured. */
    bool isBurstEnabled() const;
    /** @brief Get the burst period. @return PWM periods per burst period (0 when not configured). */
    uint16_t getBurstPeriod() const;
    /** @brief Get the skipped PWM periods per burst period. @return 0 (continuous) to the period. */
    uint16_t getBurstSkip() const;
    /** @brief Check whether leg 1 is skipping periods. @return true when the skip count is positive. */
    bool isBurstActive() const;

    /**
     * @brief Configure over-voltage and over-current protection.
     * @param fullBurstV Bus voltage above which every period is skipped, in V.
     * @param tripV Bus voltage above which the controller trips, in V (> fullBurstV).
     * @param tripA Leg current above which the controller trips, in A (positive).
     * @return 0 on success, -EINVAL for non-finite or inconsistent values.
     */
    int8_t setProtection(float32_t fullBurstV, float32_t tripV, float32_t tripA);
    /** @brief Check whether protection tripped. @return true until setMode(IDLE). */
    bool isTripped() const;

    /**
     * @brief Schedule the voltage PID's Kp with the operating point.
     * @details At each STARTUP/BUSREG cold loop, Kp = gain · clamp(VBusRef, vbusMin, vbusMax) /
     * clamp(VLow, vinMin, vinMax), applied without resetting the PID.
     * @return 0 on success, -EINVAL for non-positive or inconsistent values.
     */
    int8_t setVoltageGainSchedule(float32_t gain, float32_t vinMin, float32_t vinMax,
                                  float32_t vbusMin, float32_t vbusMax);
    /** @brief Get the voltage PID's current Kp. @return Kp in A/V. */
    float32_t getVoltageKp() const;

    /** @brief Configure ripple rejection after init(), at the hot-loop sample rate.
     * @return 0 on success, -EINVAL for invalid arguments or before init().
     * Both paths start disabled. init() disables them; reset()/IDLE clear histories
     * while preserving the enable flags. Frequency/bandwidth are in Hz, phase in rad.
     */
    int8_t configureRippleControl(float32_t frequency, float32_t bandwidth,
                                  float32_t resonantGain, float32_t phase);
    /** @brief Switch the voltage-feedback notch, resetting its history on a change. */
    void enableVoltageNotch(bool enabled);
    /** @brief Switch both current resonators, resetting histories on a change. */
    void enableCurrentResonant(bool enabled);
    bool isVoltageNotchEnabled() const { return _notch_enabled; }
    bool isCurrentResonantEnabled() const { return _resonant_enabled; }
    /** @brief Voltage used by the outer loop; protection always uses getVBus(). */
    float32_t getFilteredVBus() const { return _filtered_vbus; }

    /** @brief Get the total current reference. @return Total input current reference in A. */
    float32_t getILowRef() const;
    /** @brief Get leg 1 current reference. @return Reference in A. */
    float32_t getILow1Ref() const;
    /** @brief Get leg 2 current reference. @return Reference in A. */
    float32_t getILow2Ref() const;
    /** @brief Get the leg 1 current threshold. @return Threshold in A. */
    float32_t getLeg1CurrentLimit() const;
    /** @brief Check whether leg 1 PWM must run. @return true in an active mode, not tripped (burst gates its periods). */
    bool isLeg1Enabled() const;
    /** @brief Check whether leg 2 PWM must run. @return true when total demand exceeds the leg 1 threshold, not tripped. */
    bool isLeg2Enabled() const;

    /** @brief Get the target bus voltage. @return Bus voltage reference in V. */
    float32_t getVBusRef() const;

    /**
     * @brief Set the maximum total current reference during STARTUP (converter 1 only).
     * @param currentMax Nonnegative finite limit in A; invalid values are ignored.
     * @details Call after init(). Clamped to leg1CurrentLimit and the voltage PID upper
     * bound. Preserved across reset() and mode changes; init() restores its default.
     */
    void setStartupCurrentMax(float32_t currentMax);
    /** @brief Get the startup current limit. @return Effective maximum in A. */
    float32_t getStartupCurrentMax() const;

    /**
     * @brief Set and distribute the total current reference directly until the next cold-loop update.
     * @param ILowRef Total current reference in A, clamped to the voltage PID bounds
     * (STARTUP: also to startupCurrentMax). Non-finite values are ignored.
     */
    void setILowRef(float32_t ILowRef);

    /** @brief Set the target bus voltage. @param VBusRef Voltage in V; negative or non-finite values are ignored. */
    void setVBusRef(float32_t VBusRef);

    /** @brief Get leg 1 duty. @return Last duty command; zero when idle/tripped, minimum on active burst reset. */
    float32_t getDutyLeg1() const;
    /** @brief Get leg 2 duty. @return Last duty command, or zero when disabled/reset. */
    float32_t getDutyLeg2() const;
    /** @brief Compute input power. @return VLow * (ILow1 + ILow2) in W. */
    float32_t getPower() const;
    /** @brief Get the selected operating mode. @return Current boost controller mode. */
    boost_mode getMode() const;

    /**
     * @brief Select a mode.
     * @param mode Requested operating mode; invalid values select IDLE.
     * @details IDLE, or an active mode entered from IDLE, resets every state (reset()).
     * Between active modes nothing is reset; a trip is cleared only by IDLE, and while
     * tripped an active mode stays disabled.
     */
    void setMode(boost_mode mode);

private:
    /** @brief Clear all PID states, current references and duties. */
    void clearControlOutputs();
    /** @brief Trip on over-voltage or over-current. @return true when tripped. */
    bool checkProtection();
    /** @brief Run leg 1: burst bookkeeping or its current PID. */
    void runLeg1();
    /** @brief Run the voltage PID with symmetric mode-specific bounds. @return Signed internal demand in A. */
    float32_t calculateVoltageCurrent(float32_t reference, float32_t measurement);
    /** @brief Use converter 1 only in STARTUP; otherwise allocate excess demand to leg 2. */
    void splitCurrentReference();
    /** @brief Apply the voltage gain schedule to the voltage PID. */
    void scheduleVoltageGain();
    float32_t calculateCurrentDuty(Pid& pid, Pr* resonator,
                                   float32_t reference, float32_t measurement);
    void resetVoltageNotch();
    void releaseRippleControl();

    boost_mode _mode = IDLE;
    bool _initialized = false;
    float32_t _ILow1 = 0.0F;
    float32_t _ILow2 = 0.0F;
    float32_t _VLow = 0.0F;
    float32_t _VBus = 0.0F;
    float32_t _ILow_ref = 0.0F;
    float32_t _voltage_current_demand = 0.0F; ///< Signed voltage PI output; negative requests suppress pulses.
    float32_t _VBus_ref = 0.0F;
    float32_t _ILow1_ref = 0.0F;
    float32_t _ILow2_ref = 0.0F;
    float32_t _leg1_current_limit = 0.0F;
    float32_t _startup_current_max = 0.0F; ///< Total current reference ceiling in STARTUP, in A.
    float32_t _duty_min = 0.0F;            ///< Current PID lower output bound (duty).
    float32_t _duty_max = 1.0F;
    float32_t _hot_period = 0.0F;
    bool _ripple_configured = false;
    bool _notch_enabled = false, _resonant_enabled = false;
    float32_t _filtered_vbus = 0.0F;
    // Allocate only at configuration: library FIR buffers are unusable before
    // init(). Pr has no virtual destructor; the final type permits safe deletion.
    struct OwnedPr final : Pr {};
    NotchFilter* _voltage_notch = nullptr;
    OwnedPr* _current1_resonator = nullptr;
    OwnedPr* _current2_resonator = nullptr;
    bool _notch_primed = false;
    float32_t _notch_offset = 0.0F;
    BoostDutyCycles _duty;

    uint16_t _burst_period = 0U;  ///< PWM periods per burst period; 0 = burst not configured.
    uint16_t _burst_skip = 0U;    ///< Skipped PWM periods per burst period.

    bool _protection_enabled = false;
    float32_t _full_burst_V = 0.0F;
    float32_t _trip_V = 0.0F;
    float32_t _trip_A = 0.0F;
    bool _tripped = false;

    bool _gain_schedule_enabled = false;
    float32_t _gain = 0.0F;
    float32_t _vin_min = 0.0F, _vin_max = 0.0F, _vbus_min = 0.0F, _vbus_max = 0.0F;

    Pid _voltage_pid;
    Pid _current1_pid;
    Pid _current2_pid;
    float32_t _current_ref_min = 0.0F; ///< Voltage PID lower output bound, in A.
    float32_t _current_ref_max = 0.0F; ///< Voltage PID upper output bound, in A.
    static constexpr float32_t MPPT_CURRENT_STEP = 0.1F; ///< Total current step in A per MPPT call.
    float32_t _mppt_previous_power = 0.0F;
    float32_t _mppt_direction = 1.0F;
    bool _mppt_has_previous_power = false;
};

#endif // BOOST_H
