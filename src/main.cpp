#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include <PID_v1.h>

// MPU 6050 sensor
MPU6050 mpu(Wire);

// Escs
Servo yellow_esc;
Servo pink_esc;

// Controller variables
double Setpoint, Input, Output;
double Kp = .15, Ki = 1, Kd = 0.15;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
float angle = 0.0;

// Input
float ref = 0;
String stin = "";
int ii = 0;

// Filter variables
float buff[5], filtered_angle = 0;

void setup()
{
  Serial.begin(9600);

  // Escs setup
  yellow_esc.writeMicroseconds(1000);
  pink_esc.attach(9);
  pink_esc.writeMicroseconds(1000);
  yellow_esc.attach(8);
  delay(2500);

  // MPU 6050 setup
  Wire.begin();
  mpu.begin();
  byte status = mpu.begin(1, 0);
  while (status != 0)
  {
  }
  // mpu.calcOffsets(true,true);
  mpu.setFilterGyroCoef(0.98);

  // Controler
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100, 100);
}
void loop()
{
  // Measures new value of angle
  mpu.update();
  angle = mpu.getAngleX();

  filtered_angle = filteredAngle(angle);
  Serial.println(filtered_angle);

  // Controler
  Setpoint = getReference();
  Input = filtered_angle;
  myPID.Compute();

  // Signal to motors
  yellow_esc.writeMicroseconds(int(1140 - Output));
  pink_esc.writeMicroseconds(int((1140 + Output) * 1.135));
}

float filteredAngle(float angle)
{
  // This function returns the mean from the previous 5 values of angle measured
  buff[0] = buff[1];
  buff[1] = buff[2];
  buff[2] = buff[3];
  buff[3] = buff[4];
  buff[4] = angle;

  return (buff[0] + buff[1] + buff[2] + buff[3] + buff[4]) / 5;
}

float s2n(int des)
{
  char atual;
  float number = 0;
  int ponto = stin.indexOf('.', des);
  int pot = -1;
  ii = ponto + pot;
  atual = stin.charAt(ii);
  while (((int)atual < 58) && ((int)atual > 47))
  {
    number = number + ((int)atual - 48) * pow(10, -pot - 1);
    pot--;
    ii = ponto + pot;
    atual = stin.charAt(ii);
  }
  pot = 1;
  ii = ponto + pot;
  atual = stin.charAt(ii);
  while (((int)atual < 58) && ((int)atual > 47))
  {
    number = number + ((int)atual - 48) / pow(10, pot);
    pot++;
    ii = ponto + pot;
    atual = stin.charAt(ii);
  }
  return number;
}

float getReference()
{
  if (Serial.available())
  {
    while (Serial.available())
    {
      char inChar = (char)Serial.read();
      if (true)
      {
        stin = "";
        while (Serial.available())
        {
          stin = stin + inChar;
          inChar = (char)Serial.read();
        }
        ii = 0;
        ref = s2n(ii);
        ref = ref * 100;
      }
    }
  }
  return ref;
}