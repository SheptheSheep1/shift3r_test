//A delay of 1000 Microseconds is Full Reverse
//A delay of 1000 to 1460 Microseconds is Proportional Reverse
//A delay of 1460 to 1540 Microseconds is neutral
//A delay of 1540 to 2000 Microseconds is Proportional Forward
//A delay of 2000 Microseconds is Full Forward
//
//
#include "Arduino.h"
#include "pico/stdlib.h"
#include <stdbool.h>
#include <Servo.h>
//#define readA bitRead(PIND,2)//faster than digitalRead()
//#define readB bitRead(PIND,3)//faster than digitalRead()
#define GPIO_A 19
#define GPIO_B 20

#define readA gpio_get(GPIO_A)
#define readB gpio_get(GPIO_B)
							 //
//const byte encoderPinA = 2;//outputA digital pin
//const byte encoderPinB = 3;//outputB digital pin
volatile int count = 0;
int protectedCount = 0;
int previousCount = 0;



int backLeftPin = 9;      //Back Left Motor pin
volatile int backLeftSpeed = 1460; //Back Left Motor starting speed
Servo backLeftMotor;           //Back Left Motor Servo Object

int Speed = 1460;           //Starting speed for Serial communication
bool change = false;

void isrA(void);
void isrB(void);

void setup()
{
	// Initialize the standard library
    //stdio_init_all();
    
    // Set up GPIO 2 and 3 as inputs
    gpio_init(GPIO_A);
    gpio_set_dir(GPIO_A, GPIO_IN);
    gpio_pull_up(GPIO_A); // Optional: Enable pull-up, often needed for inputs

    gpio_init(GPIO_B);
    gpio_set_dir(GPIO_B, GPIO_IN);
    gpio_pull_up(GPIO_B); // Optional: Enable pull-up
  // Tells each of the servo objects which pin it should output to
	//backLeftMotor.attach(backLeftPin);
	//pinMode(encoderPinA, INPUT_PULLUP);
	//pinMode(encoderPinB, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(GPIO_A), isrA, CHANGE);
	attachInterrupt(digitalPinToInterrupt(GPIO_B), isrB, CHANGE);
	Serial.begin(115200);
	delay(1000);
	Serial.println("hello");
	Serial.println(digitalRead(GPIO_A));
	Serial.println(digitalRead(GPIO_B));
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
//  if (Serial.available() > 0) {
//    char fb = Serial.read();
//    if (fb == 'a') {
//		Serial.println("got a");
//		Serial.println(backLeftSpeed, backLeftPin);
//    }
//    else {
//      char lr = Serial.read();
//      Speed = Serial.parseInt();
//      if (fb == 'b') {
//        if (lr == 'l') {
//          backLeftSpeed = Speed;
//		  //change = true;
//		  Serial.printf("wrote: %d to pin : %d", backLeftSpeed, backLeftPin);
//		}
//	  }
//	}
//	Serial.println(Speed);

	noInterrupts();
	protectedCount = count;
	interrupts();

	if(protectedCount != previousCount) {
		Serial.println(protectedCount);
	}
	previousCount = protectedCount;
  
  //This code creates the PWM signal on each pin based on the speed provided
  //backLeftMotor.writeMicroseconds(backLeftSpeed);           //Back left motor driver code
  //if(change == true){Serial.printf("wrote: %d to pin : %d", backLeftSpeed, backLeftPin);}
}
void isrA() {
	if(digitalRead(GPIO_B) != digitalRead(GPIO_A)) {
		count ++;
	} else {
		count --;
	}
}
void isrB() {
	if (digitalRead(GPIO_A) == digitalRead(GPIO_B)) {
		count ++;
	} else {
		count --;
	}
}
