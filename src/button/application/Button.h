#include "./../infraestructure/ButtonHardware.h"
class Button 
{
private:
    ButtonHardware *buttonHardware;

public:
    Button(ButtonHardware *buttonHardware);
    virtual ~Button();
    bool isPressed();
};
