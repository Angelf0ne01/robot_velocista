#include "button.h"

Button::~Button(){}
Button::Button(ButtonHardware *btn){
    this->buttonHardware = btn;
}

bool Button::isPressed(){
    return this->buttonHardware->isPressed();
}