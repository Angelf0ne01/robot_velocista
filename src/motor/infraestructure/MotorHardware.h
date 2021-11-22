#include "Arduino.h"

class MotorHardware {
    private:
    int pin_dir;    
    int pin_pwm;
    int pwm_correction=0;
  public:
   ~MotorHardware(); 
    MotorHardware(int pin_dir, int pin_pwm);
    void setSpeed(int speed);    
    void stop();
    void setPwmCorrection(int correction);
};