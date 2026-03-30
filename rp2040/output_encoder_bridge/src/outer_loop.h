#ifndef OUTPUT_ENCODER_BRIDGE_OUTER_LOOP_H
#define OUTPUT_ENCODER_BRIDGE_OUTER_LOOP_H

/*
 * Cascade outer position loop.
 *
 * This runs on the RP2040 at the sensor-read rate (1-4 kHz) and outputs a
 * velocity command for the ODrive's inner velocity/current loops.
 *
 * Signal flow:
 *
 *   Host  --setpoint-->  RP2040 outer loop  --vel_cmd-->  ODrive inner PID
 *           (output turns)    (MT6835 fb)      (motor turns/s)   (8 kHz FOC)
 *
 * The controller is a PD on output-shaft position:
 *
 *   error     = setpoint - measured_output_pos    [output turns]
 *   vel_out   = Kp * error  -  Kd * measured_vel  [output turns/s]
 *   vel_motor = vel_out * gear_ratio               [motor turns/s]
 *   vel_motor = clamp(vel_motor, -vel_limit, +vel_limit)
 *
 * Kp has units [1/s]:  output_turns error  -->  output_turns/s command
 * Kd has units [1]:    output_turns/s      -->  output_turns/s damping
 * gear_ratio converts output velocity to motor velocity for the ODrive.
 */

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float kp;            /* Proportional gain  [1/s]                      */
    float kd;            /* Derivative gain    [dimensionless]            */
    float vel_limit;     /* Max motor velocity command [motor turns/s]    */
    float gear_ratio;    /* Motor-to-output gear ratio (e.g. 25.0)       */
} outer_loop_gains_t;

typedef struct {
    float    setpoint;        /* Target output position [output turns]    */
    float    position;        /* Last measured output position [turns]    */
    float    velocity;        /* Last measured output velocity [turns/s]  */
    float    error;           /* setpoint - position [turns]              */
    float    vel_command;     /* Last motor vel command [motor turns/s]   */
    uint32_t tick_count;      /* Loop iterations since enable             */
    bool     enabled;         /* Loop is active                           */
} outer_loop_state_t;

/* Initialise with default gains (safe, conservative). */
void  outer_loop_init(outer_loop_state_t *state, outer_loop_gains_t *gains);

/* Set the target output-shaft position [output turns]. */
void  outer_loop_set_setpoint(outer_loop_state_t *state, float setpoint_output_turns);

/* Enable/disable the loop.  When disabled, vel_command is always 0. */
void  outer_loop_set_enabled(outer_loop_state_t *state, bool enabled);

/* Update the gains at runtime (from host command). */
void  outer_loop_set_gains(outer_loop_gains_t *gains, float kp, float kd, float vel_limit);

/*
 * Run one iteration of the controller.
 *
 * Call this once per sensor read.  Returns the motor velocity command
 * [motor turns/s] that should be sent to the ODrive.
 *
 *   output_pos_turns  — MT6835 measured output position [turns]
 *   output_vel_turns_s — MT6835 computed output velocity [turns/s]
 */
float outer_loop_update(
    outer_loop_state_t *state,
    const outer_loop_gains_t *gains,
    float output_pos_turns,
    float output_vel_turns_s
);

#endif
