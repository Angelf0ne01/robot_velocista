#include "Motor.h"

Motor::~Motor() {}
Motor::Motor(MotorHardware *motorHardware)
{
    this->motorHardware = motorHardware;
}

void  Motor::setSpeed(int speed)
{
    this->motorHardware->setSpeed(speed);
}

void  Motor::stop()
{
    this->motorHardware->stop();
}

void Motor::setPwmCorrection(int pwmCorrection)
{
    this->motorHardware->setPwmCorrection(pwmCorrection);
}