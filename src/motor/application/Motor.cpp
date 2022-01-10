#include "Motor.h"

Motor::~Motor() {}
Motor::Motor(IMotor *iMotor)
{
    this->iMotor = iMotor;
}

void Motor::setPwm(int pwm)
{
    this->iMotor->setPwm(pwm);
}

void Motor::setPwmCorrection(int pwmCorrection)
{
    this->iMotor->setPwmCorrection(pwmCorrection);
}
void Motor::run()
{
    this->iMotor->run();
}

void Motor::stop()
{
    this->iMotor->stop();
}
