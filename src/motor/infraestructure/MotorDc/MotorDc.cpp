#include "MotorDc.h"

MotorDc::MotorDc(int pin_enable, int pin_pwm)
{
    this->pin_enable = pin_enable;
    this->pin_pwm = pin_pwm;
}

void MotorDc::setPwm(int pwm)
{
    this->pwm = pwm;
}

void MotorDc::setPwmCorrection(int pwm_correction)
{
    this->pwm = this->pwm - pwm_correction;
}

void MotorDc::stop()
{
    analogWrite(this->pin_enable, 0);
}

void MotorDc::run()
{
    analogWrite(this->pin_enable, this->pwm);
}