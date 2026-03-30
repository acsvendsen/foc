#include "outer_loop.h"

#include <stddef.h>

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void outer_loop_init(outer_loop_state_t *state, outer_loop_gains_t *gains) {
    if (state != NULL) {
        state->setpoint    = 0.0f;
        state->position    = 0.0f;
        state->velocity    = 0.0f;
        state->error       = 0.0f;
        state->vel_command = 0.0f;
        state->tick_count  = 0u;
        state->enabled     = false;
    }
    if (gains != NULL) {
        /* Conservative defaults: gentle P, moderate D, low speed. */
        gains->kp        = 2.0f;    /* [1/s]  — 1 output-turn error → 2 output-turns/s cmd */
        gains->kd        = 0.3f;    /* damping ratio                                       */
        gains->vel_limit = 1.0f;    /* motor turns/s  — safe for harmonic drive             */
        gains->gear_ratio = 25.0f;
    }
}

void outer_loop_set_setpoint(outer_loop_state_t *state, float setpoint_output_turns) {
    if (state != NULL) {
        state->setpoint = setpoint_output_turns;
    }
}

void outer_loop_set_enabled(outer_loop_state_t *state, bool enabled) {
    if (state == NULL) {
        return;
    }
    if (enabled && !state->enabled) {
        /* Entering closed-loop: reset state to avoid step transient.
         * Set the setpoint to wherever the output shaft currently is,
         * so enabling the loop doesn't cause a lurch.  The host should
         * then ramp the setpoint to the desired target. */
        state->setpoint    = state->position;
        state->error       = 0.0f;
        state->vel_command = 0.0f;
        state->tick_count  = 0u;
    }
    state->enabled = enabled;
}

void outer_loop_set_gains(outer_loop_gains_t *gains, float kp, float kd, float vel_limit) {
    if (gains == NULL) {
        return;
    }
    /* Clamp to sane ranges — these are safety limits, not tuning advice. */
    gains->kp        = clampf(kp,        0.0f, 50.0f);
    gains->kd        = clampf(kd,        0.0f, 10.0f);
    gains->vel_limit = clampf(vel_limit,  0.0f, 10.0f);
}

float outer_loop_update(
    outer_loop_state_t *state,
    const outer_loop_gains_t *gains,
    float output_pos_turns,
    float output_vel_turns_s
) {
    float error, vel_output, vel_motor;

    if (state == NULL || gains == NULL) {
        return 0.0f;
    }

    /* Latch sensor readings into state for telemetry. */
    state->position = output_pos_turns;
    state->velocity = output_vel_turns_s;

    if (!state->enabled) {
        state->error       = 0.0f;
        state->vel_command = 0.0f;
        return 0.0f;
    }

    /* PD control in output-shaft space. */
    error      = state->setpoint - output_pos_turns;
    vel_output = (gains->kp * error) - (gains->kd * output_vel_turns_s);

    /* Convert output-shaft velocity to motor-shaft velocity. */
    vel_motor = vel_output * gains->gear_ratio;

    /* Symmetric velocity clamp for safety. */
    vel_motor = clampf(vel_motor, -gains->vel_limit, gains->vel_limit);

    /* Latch for telemetry. */
    state->error       = error;
    state->vel_command = vel_motor;
    state->tick_count++;

    return vel_motor;
}
