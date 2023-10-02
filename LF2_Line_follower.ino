/*
 Sample Line Following Code for the Robojunkies LF-2 robot
*/

#include <SparkFun_TB6612.h>

#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200;

float Kp = 0.0006;  // Adjust Kp value as needed
float Kd = 10 * Kp;
float Ki = 0.0001;  // Adjust Ki value as needed

int minValues[6], maxValues[6], threshold[6];

void setup()
{
  Serial.begin(9600);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
}


void loop()
{
  // while (digitalRead(11)) {}
  // delay(1000);
  // calibrate();
  // while (digitalRead(12)) {}
  // delay(1000);

  while (1)
  {
    if (analogRead(A1) > threshold[1] && analogRead(A5) < threshold[5] )
    {
      lsp = 0; rsp = lfspeed;
      motor1.drive(0);
      motor2.drive(lfspeed);
    }

    else if (analogRead(A5) > threshold[5] && analogRead(A1) < threshold[1])
    { lsp = lfspeed; rsp = 0;
      motor1.drive(lfspeed);
      motor2.drive(0);
    }
    else if (analogRead(A3) > threshold[3])
    {
      Kp = 0.0006 * (1000 - analogRead(A3));  // Adjust Kp based on sensor readings
      Kd = 10 * Kp;
      // Ki = 0.0001;  // Uncomment and adjust Ki if needed
      linefollow();
    }
    else
    {
      // All sensors detect black, stop the robot
      motor1.drive(0);
      motor2.drive(0);
    }
  }
}

void linefollow()
{
  int error = (analogRead(A2) - analogRead(A4));

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  motor1.drive(lsp);
  motor2.drive(rsp);

  int sensorValue1 = analogRead(A1);
  int sensorValue2 = analogRead(A2);
  int sensorValue3 = analogRead(A3);
  int sensorValue4 = analogRead(A4);
  int sensorValue5 = analogRead(A5);
  
  // print out the value you read:
  
  Serial.print(sensorValue1);
  Serial.print("   ");
  Serial.print(sensorValue2);
  Serial.print("   ");
  Serial.print(sensorValue3);
  Serial.print("   ");
  Serial.print(sensorValue4);
  Serial.print("   ");
  Serial.print(sensorValue5);
  Serial.println(" ");
  delay(50);
}

void calibrate()
{
  for ( int i = 1; i < 6; i++)
  {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  
  for (int i = 0; i < 3000; i++)
  {
    motor1.drive(50);
    motor2.drive(-50);

    for ( int i = 1; i < 6; i++)
    {
      if (analogRead(i) < minValues[i])
      {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i])
      {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for ( int i = 1; i < 6; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
  motor1.drive(0);
  motor2.drive(0);
}
