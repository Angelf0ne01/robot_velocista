#include "MotorHardware.h"

MotorHardware::~MotorHardware()
{
}
MotorHardware::MotorHardware(int p_d, int p_pwm)
{
    pin_dir = p_d;
    pin_pwm = p_pwm;
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_pwm, OUTPUT);
}

void MotorHardware::setPwm(int pwm)
{
    if (pwm < 0)
        pwm = 0;
    if (pwm > 255)
        pwm = 255;
    pwm = pwm;
}

void MotorHardware::stop()
{
    analogWrite(pin_pwm, 0);
}

void MotorHardware::run()
{
    digitalWrite(pin_dir, LOW);
    analogWrite(pin_pwm, abs(pwm + pwm_correction));
}

void MotorHardware::setPwmCorrection(int val)
{
    pwm_correction = val;
}