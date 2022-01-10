#include "Arduino.h"

class IMotor
{
public:
    virtual ~IMotor() {}
    virtual void setPwm(int pwm) = 0;
    virtual void setPwmCorrection(int pwmCorrectionValue) = 0;
    virtual void run() = 0;
    virtual void stop() = 0;
};