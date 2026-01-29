#ifndef PID_H
#define PID_H

class PID {
public:
    PID(float kp, float ki, float kd, float max_out);
    float compute(float setpoint, float measured, float dt);
    void reset();
    void setTunings(float kp, float ki, float kd);

private:
    float _kp, _ki, _kd;
    float _max_out;
    float _integral;
    float _prev_error;
};

#endif
