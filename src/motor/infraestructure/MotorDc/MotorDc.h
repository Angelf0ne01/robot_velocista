#include "motor/infraestructure/IMotor.h"

class MotorDc : public IMotor
{
private:
    int pin_enable, pin_pwm;
    int pwm = 0;

public:
    virtual ~MotorDc();
    MotorDc(int pin_enable, int pin_pwm);
    virtual void setPwm(int pwm) = 0;
    virtual void setPwmCorrection(int pwmCorrectionValue) = 0;
    virtual void run() = 0;
    virtual void stop() = 0;
};