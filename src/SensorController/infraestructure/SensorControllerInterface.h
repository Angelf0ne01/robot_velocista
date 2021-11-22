#include "Arduino.h"
#include "./../lib/qtr-sensors-arduino-master/QTRSensors.h"

class SensorControllerInterface{
    private:
    uint16_t sensorValues[6];
    QTRSensors qtr;
    const uint8_t SensorCount =6;
    public:
    SensorControllerInterface();
    virtual ~SensorControllerInterface();
    float getSumAllSensors();
    float getPromedioPonderado();
    private:
    void getSensorValues();

};