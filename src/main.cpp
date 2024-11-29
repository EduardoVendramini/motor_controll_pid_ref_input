#include <Wire.h> // For I2C communication
#include <MPU6050_light.h>
#include <Servo.h>
#include <PID_v1.h>

#define MIN_PULSE_LENGTH 1000 // https://howtomechatronics.com/tutorials/arduino/arduino-brushless-motor-control-tutorial-esc-bldc/
#define MAX_PULSE_LENGTH 2000

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
float ref = 0, gyr = 0;
int m1 = MIN_PULSE_LENGTH, m2 = MIN_PULSE_LENGTH, disturbanceState = 0, automaticState = 1;
int index;
float measuredAngle;
void getRemoteControlParameters();
void printRemoteControlParameters();
void setMotors();
String incomingMessage;

// PID
double kp = 1.5, ki = 0.05, kd = 0, P, I, D;
float deltaT, error, previousError, pidOutput;
void calculatePid();

unsigned long initialTime = 0, finalTime = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting...");

  Serial1.begin(115200);
  // Serial1.begin(9600); // From home

  // MPU 6050 setup
  Wire.begin();
  mpu.begin();
  calibrateSensor();
  mpu.setFilterGyroCoef(0.98);

  // Escs setup
  yellowEsc.attach(8);
  pinkEsc.attach(9);
  calibrateEscs();
  Serial1.setTimeout(0);
}

void loop()
{
  initialTime = millis();

  // angle
  mpu.update();
  measuredAngle = mpu.getAngleX();
  filteredAngle = getFilteredAngle();

  getRemoteControlParameters();

  // Serial.print(">measuredAngle:");
  // Serial.println(measuredAngle);
  printRemoteControlParameters();

  setMotors();

  finalTime = millis();
  if (finalTime - initialTime < 50)
  {
    delay(50 - (finalTime - initialTime));
  }
}

void printRemoteControlParameters()
{
  Serial.print("measuredAngle: ");
  Serial.print(filteredAngle);
  Serial.print("kp: ");
  Serial.print(kp);
  Serial.print(" kd: ");
  Serial.print(kd);
  Serial.print(" ki: ");
  Serial.print(ki);
  Serial.print(" m1: ");
  Serial.print(m1);
  Serial.print(" m2: ");
  Serial.print(m2);
  Serial.print(" gyr: ");
  Serial.print(gyr);
  Serial.print(" ref: ");
  Serial.print(ref);
  Serial.print(" automaticState: ");
  Serial.print(automaticState);
  Serial.print(" disturbanceState: ");
  Serial.println(disturbanceState);
}

void getRemoteControlParameters()
{
  if (Serial1.available())
  {
    incomingMessage = Serial1.readString();

    index = 0;

    kp = incomingMessage.substring(0, incomingMessage.indexOf(',')).toFloat();
    index = incomingMessage.indexOf(',') + 1;
    ki = incomingMessage.substring(index, incomingMessage.indexOf(',', index)).toFloat();
    index = incomingMessage.indexOf(',', index) + 1;
    kd = incomingMessage.substring(index, incomingMessage.indexOf(',', index)).toFloat();
    index = incomingMessage.indexOf(',', index) + 1;
    m1 = incomingMessage.substring(index, incomingMessage.indexOf(',', index)).toInt();
    index = incomingMessage.indexOf(',', index) + 1;
    m2 = incomingMessage.substring(index, incomingMessage.indexOf(',', index)).toInt();
    index = incomingMessage.indexOf(',', index) + 1;
    gyr = incomingMessage.substring(index, incomingMessage.indexOf(',', index)).toFloat();
    index = incomingMessage.indexOf(',', index) + 1;
    ref = incomingMessage.substring(index, incomingMessage.indexOf(',', index)).toFloat();
    index = incomingMessage.indexOf(',', index) + 1;
    automaticState = incomingMessage.substring(index, incomingMessage.indexOf(',', index)).toInt();
    index = incomingMessage.indexOf(',', index) + 1;
    disturbanceState = incomingMessage.substring(index).toInt();
  }
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
    yellowEsc.writeMicroseconds(int(1300 - pidOutput));
    pinkEsc.writeMicroseconds(int((1300 + pidOutput)));
  }
  else
  {
    yellowEsc.writeMicroseconds(m1);
    pinkEsc.writeMicroseconds(m2);
  }
}
