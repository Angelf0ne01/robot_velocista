#include <Arduino.h>

class Pid
{
private:
  //PID
  float set_point = 0;
  float kp, kd, ki;
  //aux print
  float integral;
  float derivativo;
  float error_anterior;

  //aux
  float promedio;
  float error;
  float kp_aux, ki_aux, kd_aux;

public:
  Pid() {}
  Pid(float p, float d, float i)
  {
    kd = d;
    kp = p;
    ki = i;
  }

  float getPid(float num, float den)
  {
    promedio = num / den;

    error = (promedio - set_point);
    integral += error_anterior;
    derivativo = (error - error_anterior);
    error_anterior = error;
    kp_aux = kp * error;
    kd_aux = kd * derivativo;
    ki_aux = ki * integral;
    return kp_aux;
  }
  void setProporcional(float p)
  {
    kp = p;
  }

  void setIntegral(float i)
  {
    ki = i;
  }

  float setDerivativo(float d)
  {
    kd = d;
  }

  void setSetPoint(float sp)
  {
    set_point = sp;
  }

  void PrintAllValue()
  {
    /*  Serial.print("kp:");
    Serial.print(kp);
    Serial.print(" - ");

    Serial.print("kd:");
    Serial.print(kd);
    Serial.print(" - ");

    Serial.print("kp:");
    Serial.print(kp);
    Serial.print(" - ");
*/
    Serial.print("kp:");
    Serial.print(kp);
    Serial.print(" - ");

    Serial.print("kp_aux:");
    Serial.print(kp_aux);
    Serial.print(" - ");

    Serial.print("prom-setpoint:");
    Serial.print(promedio);
    Serial.println("  ");
  }
};

class Sensor
{
private:
  int pin;
  int max = 0;
  int min = 1024;
  int value;
  bool calibrateIsEnable = false;

public:
  Sensor(){};
  Sensor(int p)
  {
    pin = p;
    pinMode(pin, INPUT);
  }

  int read()
  {
    return getValue();
  }

  void enableCalibrate()
  {
    calibrateIsEnable = true;
  }
  void calibrate()
  {
    int val = read();
    if (val >= max)
    {
      max = val;
    }
    if (val <= min)
    {
      val = min;
    }
  }

private:
  int getValue()
  {
    value = analogRead(pin);
    if (calibrateIsEnable)
    {
      if (value >= max)
      {
        value = max;
      }

      if (value <= min)
      {
        value = min;
      }
    }
    return value;
  }
};

class Motor
{
private:
  int pin_a;
  int pin_b;
  int pwm_max = 100;
  int pwm_min = 0;
  int pwm;
  bool invert = false;
  int pwm_correction = 0;

public:
  Motor() {}
  Motor(int pa, int pb)
  {
    pin_a = pa;
    pin_b = pb;
    pinMode(pa, OUTPUT);
    pinMode(pb, OUTPUT);
  }

  void setPwmMax(int pwm)
  {
    pwm_max = pwm;
  }

  void setPwmMin(int pwm)
  {
    pwm_min = pwm;
  }

  void invertDir()
  {
    invert = true;
  }

  void pwmCorrection(int pwm)
  {
    pwm_correction = pwm;
  }

  void writePwm(int p)
  {
    pwm = p;
    if (pwm >= pwm_max)
    {
      pwm = pwm_max;
    }

    if (pwm <= pwm_min)
    {
      pwm = pwm_min;
    }
    //int pwm_out = (pwm * 255) / 100u
    int pwm_out = pwm;
    if (invert)
    {
      analogWrite(pin_b, pwm_out - pwm_correction);
      analogWrite(pin_a, LOW);
    }
    else
    {
      analogWrite(pin_a, pwm_out - pwm_correction);
      analogWrite(pin_b, LOW);
    }
  }

  void printPwm()
  {
    Serial.print(pwm);
  }
};

class Velocista
{
private:
  int pines[8] = {
      A0,
      A1,
      A2,
      A3,
      A4,
      A5,
      A6,
      A7};
  const int sensor_quantity = sizeof(pines) / sizeof(int);
  Sensor sensores[8];
  Pid pid = Pid();

  float ganancia;

public:
  Velocista()
  {

    //inicio los sensores
    for (int x = 0; x < sensor_quantity; x++)
    {
      sensores[x] = Sensor(pines[x]);
    }
  }

  float getGanancia()
  {
    return ganancia;
  }

  float promedioPonderado()
  {
    int sum = 0;
    for (int x = 0; x < sensor_quantity; x++)
    {
      Sensor sensor = sensores[x];
      sum += sensor.read() * x + 1;
    }

    return sum;
  }

  float sumAllSensor()
  {
    int sum = 0;
    for (int x = 0; x < sensor_quantity; x++)
    {
      Sensor sensor = sensores[x];
      sum += sensor.read();
    }

    return sum;
  }

  float getPid()
  {
    float promedio_ponderado_numerador = promedioPonderado();
    float denominador = sumAllSensor();
    return pid.getPid(promedio_ponderado_numerador, denominador);
  }

  void calibrateSensor()
  {
    for (int x = 0; x < sensor_quantity; x++)
    {
      sensores[x].calibrate();
    }
  }

  void setSetPoint(float sp)
  {
    pid.setSetPoint(sp);
  }

  void setPwmGanancia(float g)
  {
    ganancia = g;
  }
  //pid
  void pidSetProporcional(float p)
  {
    pid.setProporcional(p);
  }

  void sensorPrintAll()
  {
    for (int x = 0; x < sensor_quantity; x++)
    {
      Sensor sensor = sensores[x];
      Serial.print(x);
      Serial.print("-");
      Serial.print(sensor.read());
      Serial.print("  | ");
    }

    Serial.println();
  }

  void printPid()
  {
    pid.PrintAllValue();
  }
};

#define PIN_ML_A 6
#define PIN_ML_B 9
#define PIN_MR_A 10
#define PIN_MR_B 11

Velocista velocista;
Motor ml = Motor(PIN_ML_A, PIN_ML_B);
Motor mr = Motor(PIN_MR_A, PIN_MR_B);

int duty = 100;

void ActualizarMotores(float pid)
{
  float ganancia = velocista.getGanancia();

  if (pid < 0)
  {
    float pwm = pid * ganancia;
    float pwm_izq = (duty - pwm);

    ml.writePwm(pwm_izq);
    mr.writePwm(duty);
  }
  else
  {
    pid = ((-1) * pid);
    float pwm = pid * ganancia;
    float pwm_der = (duty - pwm);

    ml.writePwm(duty);
    mr.writePwm(pwm_der);
  }
}

void calibrarSensor()
{
  float t_start = millis();
  int TIME = 5000;
  bool calibrate_mode = true;
  while (calibrate_mode)
  {

    velocista.calibrateSensor();
    float t_current = millis();
    if (t_current - t_start >= TIME)
    {
      //salgo del bucle
      calibrate_mode = false;
    };
  }
}
void setup()
{
  Serial.begin(9600);
  ml.setPwmMax(255);
  mr.setPwmMax(255);
  //  calibrarSensor();
  //configuro PID
  velocista.setSetPoint(4.25);
  velocista.setPwmGanancia(50);
  velocista.pidSetProporcional(20);
}

void loop()
{

  int pid = velocista.getPid();
  Serial.print("PID:");
  Serial.print(pid);
  Serial.print("   |    ");
  ActualizarMotores(pid);
  Serial.print("ML:");
  ml.printPwm();
  Serial.print("-  MR:");
  mr.printPwm();
  Serial.print("   |    ");
  velocista.printPid();
}
