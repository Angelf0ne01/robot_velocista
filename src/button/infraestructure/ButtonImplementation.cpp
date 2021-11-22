#include "ButtonHardware.h"

ButtonHardware::~ButtonHardware()
{
}	

ButtonHardware::ButtonHardware(int p){
    pin=p;
    pinMode(p, INPUT);
};

bool ButtonHardware::isPressed(){
    return !digitalRead(pin);
};