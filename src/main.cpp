#include <Arduino.h>
#include <Servo.h>

// --- Configuration ---
// Define the GPIO pin numbers. These are the same as the Pico's physical GP pins.
#define encoderPinA 14
#define encoderPinB 15
#define LED1 16
#define LED2 17
int backLeftPin = 9; // Pin for the ESC/Motor

// --- Macros for fast reading ---
// The Arduino core handles the fast digitalRead() for RP2040 interrupts.
#define readA digitalRead(encoderPinA)
#define readB digitalRead(encoderPinB)
#define FULL_SHIFT_COUNT 1700
#define HALF_SHIFT_COUNT 1000

// --- Variables ---
volatile int count = 0;
int protectedCount = 0;
int previousCount = 0;
volatile int backLeftSpeed = 1460; 
Servo backLeftMotor;
volatile bool shiftUpRequested = false;
volatile bool shiftDownRequested = true;
unsigned long startMillis;
unsigned long currentMillis;

void isrA();
void isrB();

void setup()
{
    // Initialize motor
    backLeftMotor.attach(backLeftPin); 

    // Use Arduino pinMode() for input setup
    // This replaces gpio_init(), gpio_set_dir(), and gpio_pull_up()
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	digitalWrite(LED1, LOW);
	digitalWrite(LED2, LOW);
	digitalReadFast(1);

    // Attach interrupts to the pins on both CHANGE (RISING and FALLING)
    attachInterrupt(digitalPinToInterrupt(encoderPinA), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), isrB, CHANGE);
    
    Serial.begin(115200);
    delay(1000); // Give time for serial to initialize
    Serial.println("Encoder and Servo system active.");
	shiftDownRequested = true;
	startMillis = millis();
}

void requestShiftUp(){
	shiftUpRequested = true;
}

void loop()
{
    // Safely read the volatile count variable
    noInterrupts();
    protectedCount = count;
    interrupts();
	if(shiftUpRequested){
		// attempt shift up
		// if in first gear shift blah blah blah


		shiftUpRequested = false;
	}
	if(shiftDownRequested){
		Serial.println("Shift Down Requested...");
		currentMillis = millis();
		Serial.println(currentMillis-startMillis);
		if (currentMillis - startMillis < 1000)  //test whether the period has elapsed
		{
			//digitalWrite(ledPin, !digitalRead(ledPin));  //if so, change the state of the LED.  Uses a neat trick to change the state
			//startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
			Serial.print("\nencoder, ");
			Serial.print(protectedCount);
			backLeftMotor.writeMicroseconds(1400);
			if (protectedCount >= FULL_SHIFT_COUNT){
				Serial.println("shift down successful");
				backLeftMotor.writeMicroseconds(1500);
				shiftDownRequested = false;
			}
		}else {
			Serial.println("timeout");
			shiftDownRequested = false;
		}
		

		//shiftDownRequested = false;
	}

    // Only print when the count changes
    if(protectedCount != previousCount) {
        //Serial.print("Encoder Count: ");
        //Serial.println(protectedCount);
    }
    previousCount = protectedCount;
	
	//if(protectedCount >= 1500){
	//	digitalWrite(LED1, HIGH);
	//	digitalWrite(LED2, LOW);
	//	noInterrupts();
	//	backLeftMotor.writeMicroseconds(1500);
	//	delay(500);
	//	count = 0;
	//	protectedCount = 0;
	//	interrupts();
	//	backLeftMotor.writeMicroseconds(1600);
	//	Serial.println("fwd");
	//}
	//else if(protectedCount < 1500 && protectedCount > -1500){
	//	backLeftMotor.writeMicroseconds(1500);
	//	Serial.println("neutral");
	//}
	//else if(protectedCount <= -1500){
	//	digitalWrite(LED2, HIGH);
	//	digitalWrite(LED1, LOW);
	//	backLeftMotor.writeMicroseconds(1600);
	//	Serial.println("fwd");
	//}
    // Write the speed to the ESC/motor (PWM microseconds)
    // Uncomment this when you are ready to control the motor:
    // backLeftMotor.writeMicroseconds(backLeftSpeed);
    
    // Simple delay to keep loop from running too fast (optional)
    // delay(10); 
	//Serial.println(digitalRead(encoderPinA) + digitalRead(encoderPinB));
	//Serial.println(digitalRead(11));
}

// --- Interrupt Service Routines (ISRs) ---
// Note: This logic assumes a standard quadrature encoder setup.
void isrA() { 
    // When A changes, check B to determine direction.
    // Assuming A leads B for one direction (e.g., CW)
    if (readA != readB) {
        count++; 
    } else {
        count--; 
    }
}

void isrB() { 
    // When B changes, check A to determine direction.
    // Ensures counts are captured regardless of which pin changes first.
    if (readB == readA) {
        count++;
    } else {
        count--;
    }
}
