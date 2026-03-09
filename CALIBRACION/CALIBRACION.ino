#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 150
#define SERVOMAX 600

void setup()
{

  pca.begin();
  pca.setPWMFreq(50);
  delay(10);

  // poner los 12 servos en 90 grados
  for(int i=0;i<12;i++)
  {
    int pulso = map(90,0,180,SERVOMIN,SERVOMAX);
    pca.setPWM(i,0,pulso);
    delay(20);
  }

}

void loop()
{

}