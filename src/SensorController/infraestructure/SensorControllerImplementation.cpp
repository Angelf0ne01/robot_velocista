#include "SensorControllerInterface.h"



SensorControllerInterface::SensorControllerInterface()
{
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){5, 6, 7, 8, 9, 10}, SensorCount);
}

SensorControllerInterface::~SensorControllerInterface()
{
}

float SensorControllerInterface::getPromedioPonderado()
{
    getSensorValues();
    float promedioPonderado = 0;
    for(int x=0;x<SensorCount;x++)
    {
        promedioPonderado += sensorValues[x]*(x+1);
    }
    return promedioPonderado;
}

float SensorControllerInterface::getSumAllSensors()
{
    getSensorValues();
    float sumAllSensors = 0;
    for(int x=0;x<SensorCount;x++)
    {
        sumAllSensors += sensorValues[x];
    }
    return sumAllSensors;
}

void SensorControllerInterface::getSensorValues()
{
    qtr.read(sensorValues);
}