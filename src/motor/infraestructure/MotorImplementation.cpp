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

void MotorHardware::setSpeed(int speed)
{   
    if(speed<0)speed=0;
    if(speed>255)speed=255;

    digitalWrite(pin_dir, LOW);
    analogWrite(pin_pwm, abs(speed+pwm_correction));
}

void MotorHardware::stop()
{    
    analogWrite(pin_pwm, 0);
}

void MotorHardware::setPwmCorrection(int val)
{
    pwm_correction = val;
}