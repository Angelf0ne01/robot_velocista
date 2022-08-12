#include <Arduino.h>
#include <SoftwareSerial.h>

class Button
{
  int pin;
  bool last_state;
  bool flanco = LOW;

public:
  Button(int pin)
  {
    this->pin = pin;
    this->last_state = last_state;
    pinMode(pin, INPUT);
  }

  void setFlagDesc()
  {
    this->flanco = LOW;
  }

  bool isPress()
  {
    bool pin_state = digitalRead(this->pin);

    bool current_state = last_state != pin_state && pin_state == flanco;
    last_state = pin_state;
    return current_state;
  }
};

class Rango
{
  int min = 1024;
  int max = 0;

public:
  void setMax(int max)
  {
    this->max = max;
  }
  void setMin(int min)
  {
    this->min = min;
  }

  int getMax()
  {
    return this->max;
  }
  int getMin()
  {
    return this->min;
  }
};
class Motor
{
  int pin_dir;
  int pin_pwm;
  int pwm_value = 0;

  Rango *pwmRango = new Rango();
  int pwm_correction = 0;

public:
  Motor(int pin_dir, int pin_pwm)
  {
    this->pin_dir = pin_dir;
    this->pin_pwm = pin_pwm;

    pinMode(pin_dir, OUTPUT);
    pinMode(pin_pwm, OUTPUT);
  }

  void setPwm(int pwm)
  {
    this->pwm_value = pwm;
  }
  void moveUp()
  {
    digitalWrite(this->pin_pwm, LOW);
    analogWrite(this->pin_dir, this->getPwm() + this->pwm_correction);
  }

  void stop()
  {
    digitalWrite(this->pin_dir, LOW);
    analogWrite(this->pin_dir, 0);
  }

  void setMinPwm(int pwm)
  {
    this->pwmRango->setMin(pwm);
  }

  void setMaxPwm(int pwm)
  {
    this->pwmRango->setMax(pwm);
  }

  void setPwmCorrection(int pwm)
  {
    this->pwm_correction = pwm;
  }

  int getPwm()
  {
    // set min pwm
    if (this->pwm_value <= this->pwmRango->getMin())
      this->pwm_value = this->pwmRango->getMin();
    // set max pwm
    if (this->pwm_value >= this->pwmRango->getMax())
      this->pwm_value = this->pwmRango->getMax();
    return this->pwm_value;
  }
};

class Sensor
{
  int pin;
  int value;
  Rango *rango;

public:
  Sensor(int pin, Rango *rango)
  {
    this->pin = pin;
    // No se puede crear variables del tipo static
    // Creo una instancia unica de "Rango" y se las inyecto a las diferentes instancias de los sensores
    // de esta forma, hay 1 sola instancia de rango, para multiples sensores.
    this->rango = rango; // DI -> dependency injection
    pinMode(this->pin, INPUT);
  }

  int readValue()
  {
    return analogRead(this->pin);
  }

  void calibrate()
  {
    int value = this->readValue();
    if (value >= this->rango->getMax())
      this->rango->setMax(value);
    else if (value <= this->rango->getMin())
      this->rango->setMin(value);
  }

  int readValueCalibrate()
  {
    const int MIN_QRE = 0;
    const int MAX_QRE = 1000;

    int value = this->readValue();
    if (value >= this->rango->getMax())
      value = this->rango->getMax();
    else if (value <= this->rango->getMin())
      value = this->rango->getMin();

    value = map(value, this->rango->getMin(), this->rango->getMax(), MIN_QRE, MAX_QRE); // MIN_QRE = 0 & MAX_QRE=1000

    return value;
  }
};

/********* BUTTONS *********/
#define BUTTON_PIN 4
/********* MOTORS *********/
// Motor LEFT
#define MOTOR_IZQ_PIN_DIR 6
#define MOTOR_IZQ_PIN_PWM 9
// Motor RIGHT
#define MOTOR_DER_PIN_DIR 10
#define MOTOR_DER_PIN_PWM 11
// pwm
#define MIN_PWM 25
#define PWM_BASE 40
#define MAX_PWM 50
/********* SENSORS *********/
#define PIN_SENSOR_0 A0
#define PIN_SENSOR_1 A1
#define PIN_SENSOR_2 A2
#define PIN_SENSOR_3 A3
#define PIN_SENSOR_4 A4
#define PIN_SENSOR_5 A5
#define PIN_SENSOR_6 A6
#define PIN_SENSOR_7 A7

/********* SENSORS *********/
#define SET_POINT 4.5
#define KP 10
#define KD 2
#define KI 1

// Motores
Motor *motor_left = new Motor(MOTOR_IZQ_PIN_DIR, MOTOR_IZQ_PIN_PWM);
Motor *motor_right = new Motor(MOTOR_DER_PIN_DIR, MOTOR_DER_PIN_PWM);
Button *btn = new Button(BUTTON_PIN);

// Sensores
int pin_sensor[] = {
    PIN_SENSOR_7,
    PIN_SENSOR_6,
    PIN_SENSOR_5,
    PIN_SENSOR_4,
    PIN_SENSOR_3,
    PIN_SENSOR_2,
    PIN_SENSOR_1,
    PIN_SENSOR_0,
};
const byte sensoresLength = sizeof(pin_sensor) / sizeof(int);
Rango *rango = new Rango();
Sensor *sensores[sensoresLength];

void sensorInitInstance()
{
  // instancio los sensores en una vector
  for (int idx = 0; idx < sensoresLength; idx++)
  {
    // obtengo el pin
    int pin = pin_sensor[idx];
    // instancio el sensor
    sensores[idx] = new Sensor(pin, rango);
  }
}

void calibrarSensores()
{
  for (int idx = 0; idx < sensoresLength; idx++)
  {
    Sensor *sensor = sensores[idx];
    sensor->calibrate();
    Serial.print("calibrando->");
    Serial.print("min:");
    Serial.print(rango->getMin());
    Serial.print(" - max:");
    Serial.print(rango->getMax());
    Serial.println("");
  }
}

float promedioPonderado()
{
  float numerador = 0, denominador = 0;
  for (int idx = 0; idx < sensoresLength; idx++)
  {
    Sensor *sensor = sensores[idx];
    int value = sensor->readValueCalibrate();

    numerador += value * idx + 1;
    denominador += value;
  }

  if (denominador == 0)
  {
    return 0;
  }

  return numerador / denominador;
}

void printSensor()
{
  // imprimo los sensores
  for (int idx = 0; idx < sensoresLength; idx++)
  {

    Serial.print("s");
    Serial.print(idx);
    Serial.print(":");
    Sensor *sensor = sensores[idx];
    Serial.print(sensor->readValueCalibrate());
    Serial.print(" - ");
  }
}

#define BT_RX 3
#define BT_TX 2

SoftwareSerial bt(BT_RX, BT_TX);

void setup()
{
  Serial.begin(9600);
  bt.begin(9600);

  // configuro los maximos y minimos (default max =  255 and min = 0 );
  motor_left->setMinPwm(MIN_PWM);
  motor_right->setMinPwm(MIN_PWM);
  motor_left->setMaxPwm(MAX_PWM);
  motor_right->setMaxPwm(MAX_PWM);
  // correccion de rpm entre un motor y  el otro
  motor_left->setPwmCorrection(15);

  sensorInitInstance();

  // mientras
  while (!btn->isPress())
  {
    calibrarSensores();
  }
}

unsigned long tiempo_actual = 0;
// pid
float integral, error_anterior, promedio_anterior;
float kp = 12;
float kd = 0;
float ki = 2;

int ganancia = 1;
int duty = 10;

void loop()
{

  float promedio = promedioPonderado();

  float error = (promedio - SET_POINT);
  integral += error_anterior;
  float derivativo = (error - error_anterior);
  float error_anterior = error;
  float kp_aux = kp * error;
  float kd_aux = kd * derivativo;
  float ki_aux = ki * integral;
  promedio_anterior = promedio;
  float pid = kp_aux;

  printSensor();

  int pwm_izq = 0;
  int pwm_der = 0;

  pwm_der = PWM_BASE - pid;
  pwm_izq = PWM_BASE + pid;
  // disminuyo la velocidad del motor izq
  motor_right->setPwm(pwm_der);
  motor_left->setPwm(pwm_izq);

  Serial.print("| M:<--");
  Serial.print(pwm_izq);
  Serial.print(" - M:-->");
  Serial.print(pwm_der);

  motor_left->moveUp();
  motor_right->moveUp();
  Serial.println("");
}
