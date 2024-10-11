#include <Wire.h> // For I2C communication
#include <MPU6050_light.h>
#include <Servo.h>
#include <PID_v1.h>

#define MIN_PULSE_LENGTH 1000
#define MAX_PULSE_LENGTH 2000

// MPU 6050 sensor
MPU6050 mpu(Wire);

// Escs
Servo yellow_esc;
Servo pink_esc;

// Controller variables
double Setpoint, Input, Output;
double Kp = .5, Ki = 0.4, Kd = 0.3;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
float angle = 0.0;

// Input
float ref = 0;
String stin = "";
int ii = 0;

// Filter variables
float buff[5], filtered_angle = 0;
void calibrateSensor();
void calibrateEscs();

void setup()
{
  Serial.begin(9600);

  Serial.println("Starting...");

  calibrateEscs();

  // MPU 6050 setup
  Wire.begin();
  mpu.begin();
  byte status = mpu.begin(0, 0); // sensibility of the gyro and acc
  while (status != 0)
  {
  }

  calibrateSensor();
  mpu.setFilterGyroCoef(0.98);

  // Controler
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100, 100);
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
  yellow_esc.writeMicroseconds(int(1200 - Output));
  pink_esc.writeMicroseconds(int((1200 + Output)));

  //yellow_esc.writeMicroseconds(1300);
  //pink_esc.writeMicroseconds(1300);

}

void calibrateEscs()
{
  yellow_esc.attach(8);
  pink_esc.attach(9);
  Serial.println("Calibrating ESCs...");
  delay(1000);

  Serial.println("Writting maximum pulse length...");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  yellow_esc.writeMicroseconds(MAX_PULSE_LENGTH);
  pink_esc.writeMicroseconds(MAX_PULSE_LENGTH);
  while (!Serial.available())
  {
  }
  delay(1000);

  Serial.println("Writting minimum pulse length...");
  yellow_esc.writeMicroseconds(MIN_PULSE_LENGTH);
  pink_esc.writeMicroseconds(MIN_PULSE_LENGTH);
  delay(1000);
  Serial.println("ESCs calibration done.");
  return;
}

void calibrateSensor()
{
  Serial.println("Calibrating MPU sensor...");  
  delay(3000);
  mpu.calcOffsets(true, true); // Calculate offsets for gyro and acc
  delay(1000);
  Serial.println("MPU calibration done");
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
  if (Serial.available() > 0)
  {
    ref = Serial.parseFloat();
  }

  return ref;
}
