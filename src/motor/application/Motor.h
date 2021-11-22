#include "./../infraestructure/MotorHardware.h"


class Motor {
    private:
    MotorHardware *motorHardware;
  public:
   ~Motor(); 
    Motor(MotorHardware *motorHardware);
    void setSpeed(int speed);    
    void stop();
    void setPwmCorrection(int pwmCorrection);
};