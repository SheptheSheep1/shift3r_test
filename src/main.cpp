
//A delay of 1000 Microseconds is Full Reverse
//A delay of 1000 to 1460 Microseconds is Proportional Reverse
//A delay of 1460 to 1540 Microseconds is neutral
//A delay of 1540 to 2000 Microseconds is Proportional Forward
//A delay of 2000 Microseconds is Full Forward

#include <Servo.h>

int backLeftPin = ;      //Back Left Motor pin
volatile int backLeftSpeed = 1460; //Back Left Motor starting speed
Servo backLeftMotor;           //Back Left Motor Servo Object

int Speed = 1460;           //Starting speed for Serial communication
bool change = false;

void setup()
{
  // Tells each of the servo objects which pin it should output to
  backLeftMotor.attach(backLeftPin);

  Serial.begin(115200);
}

void loop()
{
  // This code is used when controlling using serial

  // FORMAT Example: fr 2000
  // FORMAT Example: a 1500

  // If you send the character "a" then all the motors will run at the provided speed
  // fr -> front right
  // fl -> front left
  // br -> back right
  // bl -> back left
  if (Serial.available() > 0) {
    char fb = Serial.read();
    if (fb == 'a') {
		Serial.println("got a");
		Serial.println(backLeftSpeed, backLeftPin);
    }
    else {
      char lr = Serial.read();
      Speed = Serial.parseInt();
      if (fb == 'b') {
        if (lr == 'l') {
          backLeftSpeed = Speed;
		  //change = true;
		  Serial.printf("wrote: %d to pin : %d", backLeftSpeed, backLeftPin);
		}
	  }
	}
    Serial.println(Speed);
  }
  
  //This code creates the PWM signal on each pin based on the speed provided
  encoder
  backLeftMotor.writeMicroseconds(backLeftSpeed);           //Back left motor driver code
  if(change == true){Serial.printf("wrote: %d to pin : %d", backLeftSpeed, backLeftPin);}
}
