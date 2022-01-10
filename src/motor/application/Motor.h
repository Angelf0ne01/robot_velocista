#include "./../infraestructure/IMotor.h"

class Motor
{
private:
  IMotor *iMotor;

public:
  ~Motor();
  Motor(IMotor *iMotor);
  virtual void setPwm(int pwm);
  virtual void setPwmCorrection(int pwmCorrectionValue);
  virtual void run();
  virtual void stop();
};