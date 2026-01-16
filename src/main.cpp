#include <Arduino.h>
#include <Servo.h>
extern "C" {
	#include <hardware/flash.h>
	#include <hardware/sync.h>
};

// --- Defines ---
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
constexpr uint8_t ENCODER_PIN_A = 14;
constexpr uint8_t ENCODER_PIN_B = 15;
constexpr uint8_t LED_1 = 16;
constexpr uint8_t LED_2 = 17;
constexpr uint8_t ESC_PIN_PWM = 9;
constexpr uint8_t SHIFT_DOWN_PIN = 18;
constexpr uint8_t SHIFT_UP_PIN = 19;
constexpr uint8_t UP = 1;
constexpr uint8_t DOWN = 0;

// --- Macros for fast reading ---
// The Arduino core handles the fast digitalRead() for RP2040 interrupts.
#define readA digitalRead(encoderPinA)
#define readB digitalRead(encoderPinB)
#define FULL_SHIFT_COUNT 1700
#define HALF_SHIFT_COUNT 1000

// --- Variables ---
volatile int count = 0;
volatile int upCount = 0;
volatile int downCount = 0;
int protectedCount = 0;
int previousCount = 0;
volatile int backLeftSpeed = 1460; 
Servo esc;
volatile bool shiftUpRequested = false;
volatile bool shiftDownRequested = false;
volatile bool isFirstShiftUpRun = true;
volatile bool isFirstShiftDownRun = true;
volatile bool upError = false;
volatile bool downError = false;
unsigned long startMillis;
unsigned long currentMillis;

// --- Pointers for Flash Writes ---
int8_t buf[FLASH_PAGE_SIZE/sizeof(int8_t)];  // One page buffer of ints
uint8_t page; // prevent comparison of unsigned and signed int
int8_t gearCount = 1;

void saveGearProtected(int8_t value){
	// implements wear-leveling
	// from https://makermatrix.com
	//
	int8_t *p;
	uint32_t addr;
	int first_empty_page = -1;

	Serial.print("\nvalue: ");
	Serial.print(value);
	Serial.print("\n");
	*buf = value;

	for(page = 0; page < FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE; page++){
		addr = XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE);
		p = (int8_t *)addr;
		Serial.print("First four bytes of page " + String(page, DEC) );
		Serial.print("( at 0x" + (String(int(p), HEX)) + ") = ");
		Serial.println(*p);
		if( *p == -1 && first_empty_page < 0){
			first_empty_page = page;
			Serial.println("First empty page is #" + String(first_empty_page, DEC));
		}
	}
	if (first_empty_page < 0){
		Serial.println("Full sector, erasing...");
		uint32_t ints = save_and_disable_interrupts();
		flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
		first_empty_page = 0;
		restore_interrupts (ints);
	}
	Serial.println("Writing" + String(*buf)+ "to page #" + String(first_empty_page, DEC));
	uint32_t ints = save_and_disable_interrupts();
	flash_range_program(FLASH_TARGET_OFFSET + (first_empty_page*FLASH_PAGE_SIZE), (uint8_t *)buf, FLASH_PAGE_SIZE);
	restore_interrupts (ints);
}

int8_t* readGearFlash(){
	int8_t *p;
	uint32_t addr;
	int first_empty_page = -1;
	for(page = 0; page < FLASH_SECTOR_SIZE/FLASH_PAGE_SIZE; page++){
		addr = XIP_BASE + FLASH_TARGET_OFFSET + (page * FLASH_PAGE_SIZE);
		p = (int8_t*)addr;
		Serial.print("First four bytes of page " + String(page, DEC) );
		Serial.print("( at 0x" + (String(int(p), HEX)) + ") = ");
		Serial.println(*p);
		if( *p == -1 && first_empty_page < 0){
			first_empty_page = page;
			Serial.println("First empty page is #" + String(first_empty_page, DEC));
		}
	}
	return p;
}

void isrA();
void isrB();
void requestShiftDown();
void requestShiftUp();
int8_t calcNextGear(uint8_t direction);


/***
 * @brief Calculates next gear to shift to based on direction if available or returns -1.
 * @details Returns -1 if no valid next gear for direction, return nextGear number 1-6
 **/
int8_t calcNextGear(uint8_t currentGear, uint8_t direction){
	if (direction == UP){
		if (currentGear < 6 && currentGear >= 1){
			Serial.println("shifting up from :"+String(currentGear));
			currentGear++;
			return currentGear;
		}//else
	}
	if(direction == DOWN){
		if(currentGear >= 1 && currentGear <= 6){
			Serial.println("shifting down from :"+String(currentGear));
			currentGear--;
			return currentGear;
		} //else
	}
	Serial.println("next gear error");
	return -1;
}

void setup()
{
    // Initialize motor
    esc.attach(ESC_PIN_PWM); 

    // Use Arduino pinMode() for input setup
    // This replaces gpio_init(), gpio_set_dir(), and gpio_pull_up()
	pinMode(SHIFT_DOWN_PIN, INPUT_PULLUP);
	pinMode(SHIFT_UP_PIN, INPUT_PULLUP);
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
	//gpio_pull_up(encoderPinA);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
	pinMode(LED_1, OUTPUT);
	pinMode(LED_2, OUTPUT);
	digitalWrite(LED_1, LOW);
	digitalWrite(LED_2, LOW);
	//digitalReadFast(1);

    // Attach interrupts to the pins on both CHANGE (RISING and FALLING)
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), isrB, CHANGE);
	attachInterrupt(digitalPinToInterrupt(SHIFT_DOWN_PIN), requestShiftDown, FALLING);
	attachInterrupt(digitalPinToInterrupt(SHIFT_UP_PIN), requestShiftUp, FALLING);
    
    Serial.begin(115200);
	// restore gear from flash
	//gearCount = *readGearFlash();
	//saveGearProtected(1);

    delay(1000); // Give time for serial to initialize
    Serial.println("Encoder and Servo system active.");
	Serial.print("\ngear count:");
	Serial.print(*readGearFlash());
	Serial.print("\n");
	//shiftDownRequested = true;
}

void requestShiftUp(void){
	shiftUpRequested = true;
}

void requestShiftDown(void){
	shiftDownRequested = true;
}

void loop()
{
    // Safely read the volatile count variable
    noInterrupts();
    protectedCount = count;
    interrupts();
	if(shiftUpRequested && shiftDownRequested){Serial.println("conflict!!!");}//do something
	if(shiftUpRequested){
		bool error = (gearCount >= 6);
		//Serial.println("errorUp: "+String(error));
		// attempt shift up
		// if in first gear shift blah blah blah
		if(gearCount >= 6){Serial.println("ignoring...");
		shiftUpRequested = false;}
		if(isFirstShiftUpRun){
			startMillis = millis();
			Serial.println("Shift Up Requested...");
			Serial.println("first shift, start millis count and first shift set false");
			isFirstShiftUpRun = false;
			noInterrupts();
			count = 0;
			interrupts();
		}

		//Serial.println("Shift Up Requested...");

		if(shiftDownRequested){
			Serial.println("Canceling Shift Down...");
			noInterrupts();
			esc.writeMicroseconds(1500);
			shiftDownRequested = false;
			interrupts();
		}

		currentMillis = millis();
		//Serial.println(currentMillis-startMillis);
		if ((currentMillis - startMillis < 1000)&&!error)  //test whether the period has elapsed
		{
			digitalWriteFast(LED_1, HIGH);
			//Serial.print("\nencoder, ");
			//Serial.print(count);
			esc.writeMicroseconds(1300);
			if (count < -1*FULL_SHIFT_COUNT){
				Serial.println("shift up successful");
				digitalWriteFast(LED_1, LOW);
				esc.writeMicroseconds(1500);
				shiftUpRequested = false;
				//gearCount++;
				//Serial.println(gearCount);
				Serial.println("shift took "+String(currentMillis - startMillis)+" ms");
				isFirstShiftUpRun = true;
				gearCount = calcNextGear(gearCount, UP);
				if(gearCount == -1){Serial.println("Too high");}
				saveGearProtected(gearCount);
			}
		}else {
			Serial.println("up timeout");
			esc.writeMicroseconds(1500);
			shiftUpRequested = false;
			isFirstShiftUpRun = true;
		}

		//shiftUpRequested = false;
	}
	if(shiftDownRequested){
		//Serial.println("Shift Down Requested...");

		bool error = (gearCount <= 1);
		//Serial.println("errorDown: "+String(error));
		if(gearCount <= 1){Serial.println("ignoring...");
		shiftDownRequested = false;}
		if(isFirstShiftDownRun){
			startMillis = millis();
			Serial.println("Shift Down Requested...");
			Serial.println("first shift, start millis count and first shift set false");
			isFirstShiftDownRun = false;
			noInterrupts();
			count = 0;
			interrupts();
		}

		if(shiftUpRequested){
			Serial.println("Canceling Shift Up...");
			noInterrupts();
			esc.writeMicroseconds(1500);
			shiftUpRequested = false;
			//gearCount--;
			interrupts();
		}

		currentMillis = millis();
		//Serial.println("time delta:" + String(currentMillis-startMillis));
		//Serial.print("\ndown encoder, ");
		//Serial.print(count);
		if ((currentMillis - startMillis < 1000) && !error)  //test whether the period has elapsed
		{
			digitalWriteFast(LED_1, HIGH);
			//Serial.print("\nencoder, ");
			//Serial.print(count);
			esc.writeMicroseconds(1700);
			if (count >= FULL_SHIFT_COUNT){
				Serial.println("shift down successful");
				digitalWriteFast(LED_1, LOW);
				esc.writeMicroseconds(1500);
				shiftDownRequested = false;
				isFirstShiftDownRun = true;
				Serial.println("shift took "+String(currentMillis - startMillis)+" ms");
				gearCount = calcNextGear(gearCount, DOWN);
				saveGearProtected(gearCount);
				//TODO: sort out another shift request happening while gear is being saved to memory
			}
		}else {
			Serial.println("down timeout");
			esc.writeMicroseconds(1500);
			shiftDownRequested = false;
			isFirstShiftDownRun = true;
		}
		

		//shiftDownRequested = false;
	}

    // Only print when the count changes
    if(protectedCount != previousCount) {
        //Serial.print("Encoder Count: ");
        //Serial.println(protectedCount);
    }
    previousCount = protectedCount;
}

// --- Interrupt Service Routines ---
void isrA() { 
    // Assuming A leads B for one direction (e.g., CW)
    if (digitalReadFast(ENCODER_PIN_A) != digitalReadFast(ENCODER_PIN_B)) {
        count++; 
    } else {
        count--; 
    }
}

void isrB() { 
    // Ensures counts are captured regardless of which pin changes first.
    if (digitalReadFast(ENCODER_PIN_A) == digitalReadFast(ENCODER_PIN_B)) {
        count++;
    } else {
        count--;
    }
}
