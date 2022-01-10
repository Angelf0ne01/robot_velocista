#include "IMotor.h"
class MotorHardware : public IMotor
{
private:
  int pin_dir;
  int pin_pwm;
  int pwm;
  int pwm_correction = 0;

public:
  virtual ~MotorHardware();
  MotorHardware(int pin_dir, int pin_pwm);
  virtual void setPwm(int pwm);
  virtual void setPwmCorrection(int pwmCorrectionValue);
  virtual void run();
  virtual void stop();
};