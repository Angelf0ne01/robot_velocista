#include <Arduino.h>
#include "button/application/Button.h"
#include "motor/application/Motor.h"
#include "SensorController/application/SensorController.h"

#define BUTTON_PIN A5
//Motor LEFT
#define MOTOR_IZQ_PIN_DIR 12
#define MOTOR_IZQ_PIN_PWM 11
//Motor RIGHT
#define MOTOR_DER_PIN_DIR 4
#define MOTOR_DER_PIN_PWM 3

//button
ButtonHardware *btn_hardware = new ButtonHardware(BUTTON_PIN);
Button *btn = new Button(btn_hardware); //Inyeccion de dependencia
//motors
MotorHardware *motor_izq_hardware = new MotorHardware(MOTOR_IZQ_PIN_DIR, MOTOR_IZQ_PIN_PWM);
MotorHardware *motor_der_hardware = new MotorHardware(MOTOR_DER_PIN_DIR, MOTOR_DER_PIN_PWM);
Motor *motor_izq = new Motor(motor_izq_hardware); //Inyeccion de dependencia
Motor *motor_der = new Motor(motor_der_hardware); //Inyeccion de dependencia
//SensorController
SensorControllerInterface *sensor_controller_interface = new SensorControllerInterface();
SensorController *sensor_controller = new SensorController(sensor_controller_interface); //Inyeccion de dependencia

float error;
float error_anterior;
float integral;
float setpoint = 3.50;
float kp = 80;
float ki = 0;
float kd = 20;
float pid;
float vel_base = 200; //50 funca bien

void ControlMotores()
{
  if (pid > 0)
  {

    float pwm_izq = vel_base - pid;
    float pwm_der = vel_base + pid;
    motor_izq->setSpeed(pwm_izq);
    motor_der->setSpeed(pwm_der);
  }
  else
  {

    float pwm_izq = vel_base - pid;
    float pwm_der = vel_base + pid;
    motor_izq->setSpeed(pwm_izq);
    motor_der->setSpeed(pwm_der);
  }
}

void setup()
{
  Serial.begin(9600);

  while (!btn->isPressed())
  {
    Serial.println("Pulsa el boton para iniciar");
  }
  while (btn->isPressed())
  {
    Serial.println("Pulsado");
  }
}

void loop()
{
  float d = sensor_controller->getSumAllSensors();
  float n = sensor_controller->getPromedioPonderado();

  //contorl pid
  float prom = n / d;
  float error = setpoint - prom;
  float derivativo = error - error_anterior;
  integral += error_anterior;
  error_anterior = error;

  float up = kp * error;
  float dp = kd * derivativo;
  float ip = ki * integral;
  pid = up + dp + ip;
  ControlMotores();
}
