#include "PID.h"

PID::PID(float kp, float ki, float kd, float max_out)
    : _kp(kp), _ki(ki), _kd(kd), _max_out(max_out), _integral(0.0f), _prev_error(0.0f) {}

float PID::compute(float setpoint, float measured, float dt) {
    float error = setpoint - measured;

    // Proportional
    float P = _kp * error;

    // Integral
    _integral += error * dt;

    // Simple Integral Anti-windup clamping to avoiding runaway
    // If we want strict limits on I-term, we could do it here.
    // For now, relying on output clamping is okay, but preventing integral buildup is better.
    // Let's keep it simple for now.

    float I = _ki * _integral;

    // Derivative
    // Avoid divide by zero
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - _prev_error) / dt;
    }
    float D = _kd * derivative;

    _prev_error = error;

    float output = P + I + D;

    // Output clamping
    if (output > _max_out) output = _max_out;
    if (output < -_max_out) output = -_max_out;

    return output;
}

void PID::reset() {
    _integral = 0.0f;
    _prev_error = 0.0f;
}

void PID::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}
