#include "./../infraestructure/SensorControllerInterface.h"


class SensorController{
    private:
    SensorControllerInterface *sensorControllerInterface;

    public:
    SensorController( SensorControllerInterface *sensorControllerInterface );
    virtual ~SensorController();
    float getSumAllSensors();
    float getPromedioPonderado();


};