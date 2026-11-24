#ifndef BATTERY_CALIBRATION_H
#define BATTERY_CALIBRATION_H

/*
 * BATTERY-DEPENDENT CONSTANTS — CHECKLIST
 * =========================================
 * Every constant listed here was measured against a SPECIFIC battery's
 * real voltage. If the battery changes (new cells, different chemistry,
 * or even just significantly aged/degraded cells), these values may no
 * longer be accurate — as discovered the hard way on 2026-09-16, where
 * a battery swap silently caused severe long-distance drift for hours
 * before being traced back to stale feedforward + an outdated voltage
 * ceiling.
 *
 * WHENEVER THE BATTERY CHANGES, RE-RUN THESE TESTS AND UPDATE THE
 * CORRESPONDING CONSTANTS BELOW, IN THIS ORDER:
 *
 *   1. TEST_VOLTAGE_SWEEP  (single-motor, left then right)
 *        -> gives fresh SPEED_FF, BIAS_FF (motion_controller.c)
 *        -> also gives real breakaway/sustain speeds for forwardProfile's
 *           SF/KF (minMoveSpeed / kineticMinSpeed, in motion_state.c)
 *
 *   2. TEST_ACC_FF  (step response, both motors)
 *        -> gives fresh ACC_FF (motion_controller.c)
 *        -> use a sample interval wide enough to avoid tick-quantization
 *           artifacts (widen ACC_FF_LOG_INTERVAL_MS if data looks like
 *           a staircase instead of a smooth curve)
 *
 *   3. TEST_ROTATION_SWEEP  (right then left)
 *        -> gives real breakaway/sustain speeds for rotationProfile's
 *           SF/KF (minMoveSpeed / kineticMinSpeed, in motion_state.c)
 *        -> rotation does NOT need its own SPEED_FF/BIAS_FF — it reuses
 *           the shared forward feedforward via the tangent-speed
 *           conversion in MotionController_Update()
 *
 *   4. Sanity-check MAX_MOTOR_VOLTS (motion_controller.c) against the
 *      new battery's real voltage (Battery_ReadVoltage() at rest) —
 *      set it a bit below that, not left over from a previous battery.
 *
 * -----------------------------------------------------------------------
 * CURRENT VALUES, and WHERE THEY ACTUALLY LIVE (this file is a checklist
 * and reference only — the real #defines stay in their normal files):
 *
 *   motion_controller.c:
 *     SPEED_FF          — volts per mm/s
 *     BIAS_FF           — volts, breakaway/static bias
 *     ACC_FF            — volts per mm/s^2, acceleration feedforward
 *     MAX_MOTOR_VOLTS   — real voltage ceiling, must reflect current
 *                         battery's actual capability
 *
 *   motion_state.c (RampProfile_Init calls):
 *     forwardProfile.minMoveSpeed     (SF, mm/s)
 *     forwardProfile.kineticMinSpeed  (KF, mm/s)
 *     rotationProfile.minMoveSpeed    (SF, deg/s)
 *     rotationProfile.kineticMinSpeed (KF, deg/s)
 *
 * NOT battery-dependent (safe to leave alone on a battery swap):
 *   - FWD_KP, FWD_KD, ROT_KP, ROT_KD           (control gains)
 *   - STEERING_KP, STEERING_KD                  (control gains)
 *   - MOUSE_RADIUS_MM, DEG_TO_RAD                (geometry)
 *   - RampProfile accel/decel/brakeMargin/etc.  (trajectory shape)
 *   - MM_PER_TICK_LEFT/RIGHT                     (encoder geometry)
 *   - IR calibration tables (cal_left/cal_front/cal_right in sensors.c)
 *        — these depend on the SENSORS, not the battery. Recalibrate
 *          only if a sensor is physically replaced/repaired, per the
 *          IR calibration procedure (TEST_IR_CALIB), not on battery swap.
 * -----------------------------------------------------------------------
 */

#endif /* BATTERY_CALIBRATION_H */
