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

  Serial.println("Starting...");

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
  // mpu.calcOffsets(true,true); // Calculate offsets for gyro and acc
  mpu.setFilterGyroCoef(0.98);

  // Controler
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-90, 90);
}

float filteredAngle(float angle);
float getReference();

void loop()
{
  // Measures new value of angle
  mpu.update();
  angle = mpu.getAngleX();

  filtered_angle = filteredAngle(angle);
  Serial.print("ref.: ");
  Serial.print(ref);
  Serial.print(" filtered angle: ");
  Serial.print(filtered_angle);
  Serial.print(" PID Output: ");
  Serial.println(Output);

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

float getReference()
{
    if (Serial.available() > 0) {
      ref = Serial.parseFloat();
    }

    return ref;
}
