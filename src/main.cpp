#include <Wire.h> // For I2C communication
#include <MPU6050_light.h>
#include <Servo.h>
#include <PID_v1.h>
#include <ArduinoJson.h>

#define MIN_PULSE_LENGTH 1000 // TODO: entender melhor esses valores
#define MAX_PULSE_LENGTH 1400

// MPU 6050 sensor
MPU6050 mpu(Wire);

// Escs
Servo yellowEsc;
Servo pinkEsc;

// Filter
float getFilteredAngle();
float buff[5], filteredAngle = 0;

// Calibration
void calibrateSensor();
void calibrateEscs();

// input
float ref = 0, gyr = 0, disturbanceState = 0, automaticState = 0;
int m1 = MIN_PULSE_LENGTH, m2 = MIN_PULSE_LENGTH;
float measuredAngle;
void getRemoteControlParameters();
void setMotors();
String incomingMessage;
JsonDocument doc;

// PID
double kp = 1.5, ki = 0.05, kd = 0, P, I, D;
float initialTime, deltaT, error, previousError, pidOutput;
void calculatePid();

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting...");

  Serial1.begin(115200);
  Serial1.println("Starting...");

  // MPU 6050 setup
  Wire.begin();
  mpu.begin();
  calibrateSensor();
  mpu.setFilterGyroCoef(0.98);

  // Escs setup
  yellowEsc.attach(8);
  pinkEsc.attach(9);
  calibrateEscs();
}

void loop()
{
  initialTime = millis();

  // angle
  mpu.update();
  measuredAngle = mpu.getAngleX();
  filteredAngle = getFilteredAngle();

  getRemoteControlParameters();

  setMotors();

}

void calculatePid()
{
  deltaT = (millis() - initialTime) / 1000.0;

  error = ref - filteredAngle;

  P = kp * error;
  I += ki * error * deltaT;
  D = kd * (error - previousError) / deltaT;

  previousError = error;

  pidOutput = P + I + D;
}

void calibrateEscs()
{
  Serial.println("Calibrating ESCs...");
  delay(1000);

  Serial.println("Writting maximum pulse length...");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  yellowEsc.writeMicroseconds(MAX_PULSE_LENGTH);
  pinkEsc.writeMicroseconds(MAX_PULSE_LENGTH);
  while (!Serial.available())
  {
  }
  delay(1000);

  Serial.println("Writting minimum pulse length...");
  yellowEsc.writeMicroseconds(MIN_PULSE_LENGTH);
  pinkEsc.writeMicroseconds(MIN_PULSE_LENGTH);
  delay(1000);
  Serial.println("ESCs calibration done.");
  return;
}

void calibrateSensor()
{
  Serial.println("Calibrating MPU sensor...");
  mpu.calcOffsets(true, true); // Calculate offsets for gyro and acc
  delay(100);
  Serial.println("MPU calibration done");
}

float getFilteredAngle()
{
  // This function returns the mean from the previous 5 values of angle measured
  buff[0] = buff[1];
  buff[1] = buff[2];
  buff[2] = buff[3];
  buff[3] = buff[4];
  buff[4] = measuredAngle;

  return (buff[0] + buff[1] + buff[2] + buff[3] + buff[4]) / 5;
}

void setMotors()
{
  if (automaticState)
  {
    calculatePid();
    yellowEsc.writeMicroseconds(int(1200 - pidOutput));
    pinkEsc.writeMicroseconds(int((1200 + pidOutput)));
  }
  else
  {
    yellowEsc.writeMicroseconds(m1);
    pinkEsc.writeMicroseconds(m2);
  }
}

void getRemoteControlParameters()
{
  if (Serial1.available())
  {
    incomingMessage = Serial1.readString();
    deserializeJson(doc, incomingMessage);
    Serial.println(incomingMessage);
    kp = doc["kp"];
    kd = doc["kd"];
    ki = doc["ki"];
    m1 = doc["m1"];
    m2 = doc["m2"];
    ref = doc["reference"];
    gyr = doc["gyroscope"];
    disturbanceState = doc["disturbance"];
    automaticState = doc["automatic"];
  }
}
