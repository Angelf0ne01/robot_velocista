#include "Arduino.h"

class ButtonHardware 
{
private:
    int pin;

public:
    ButtonHardware(int p);
    virtual ~ButtonHardware();
    bool isPressed();
};

