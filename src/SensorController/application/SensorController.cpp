#include "SensorController.h"



SensorController::~SensorController()
{
 
}
SensorController::SensorController(SensorControllerInterface *sensorControllerInterface)
{
    this->sensorControllerInterface = sensorControllerInterface;
}


float SensorController::getSumAllSensors(){
    return this->sensorControllerInterface->getSumAllSensors();
}

float SensorController::getPromedioPonderado(){
    return this->sensorControllerInterface->getPromedioPonderado();
}

