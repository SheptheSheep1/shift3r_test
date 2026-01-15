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
constexpr uint8_t UP = 1;
constexpr uint8_t DOWN = 0

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
Servo esc;
volatile bool shiftUpRequested = false;
volatile bool shiftDownRequested = false;
unsigned long startMillis;
unsigned long currentMillis;

// --- Pointers for Flash Writes ---
int8_t buf[FLASH_PAGE_SIZE/sizeof(int8_t)];  // One page buffer of ints
int8_t *p;
uint32_t addr;
uint8_t page; // prevent comparison of unsigned and signed int
int first_empty_page = -1;
int8_t gearCount = 0;

void saveGearProtected(int8_t value){
	// implements wear-leveling
	// from https://makermatrix.com
	
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
	Serial.println("Writing to page #" + String(first_empty_page, DEC));
	uint32_t ints = save_and_disable_interrupts();
	flash_range_program(FLASH_TARGET_OFFSET + (first_empty_page*FLASH_PAGE_SIZE), (uint8_t *)buf, FLASH_PAGE_SIZE);
	restore_interrupts (ints);
}

int8_t* readGearFlash(){
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
int8_t calcNextGear(uint8_t direction);


/***
 * @brief Calculates next gear to shift to based on direction if available or returns -1.
 * @details Returns -1 if no valid next gear for direction, return nextGear number 1-6
 **/
int8_t calcNextGear(uint8_t currentGear, uint8_t direction){
	if (direction == UP){
		if (currentGear < 6 && currentGear >= 1){
			return currentGear++;
		}//else
	}
	if(direction == DOWN){
		if(currentGear >= 1 && currentGear <= 6){
			return currentGear--;
		} //else
	}
}

void setup()
{
    // Initialize motor
    esc.attach(ESC_PIN_PWM); 

    // Use Arduino pinMode() for input setup
    // This replaces gpio_init(), gpio_set_dir(), and gpio_pull_up()
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
    
    Serial.begin(115200);
	// restore gear from flash
	gearCount = *readGearFlash();

    delay(1000); // Give time for serial to initialize
    Serial.println("Encoder and Servo system active.");
	shiftDownRequested = true;
	startMillis = millis();
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
	if(shiftUpRequested){
		// attempt shift up
		// if in first gear shift blah blah blah

		Serial.println("Shift Up Requested...");

		if(shiftDownRequested){
			Serial.println("Canceling Shift Down...");
			noInterrupts();
			esc.writeMicroseconds(1500);
			shiftDownRequested = false;
			interrupts();
		}

		currentMillis = millis();
		Serial.println(currentMillis-startMillis);
		if (currentMillis - startMillis < 1000)  //test whether the period has elapsed
		{
			Serial.print("\nencoder, ");
			Serial.print(protectedCount);
			esc.writeMicroseconds(1400);
			if (protectedCount >= FULL_SHIFT_COUNT){
				Serial.println("shift up successful");
				esc.writeMicroseconds(1500);
				shiftUpRequested = false;
				gearCount++;
				saveGearProtected(gearCount);
			}
		}else {
			Serial.println("timeout");
			shiftUpRequested = false;
		}

		//shiftUpRequested = false;
	}
	if(shiftDownRequested){
		Serial.println("Shift Down Requested...");

		if(shiftUpRequested){
			Serial.println("Canceling Shift Up...");
			noInterrupts();
			esc.writeMicroseconds(1500);
			shiftUpRequested = false;
			//gearCount--;
			interrupts();
		}

		currentMillis = millis();
		Serial.println(currentMillis-startMillis);
		if (currentMillis - startMillis < 1000)  //test whether the period has elapsed
		{
			Serial.print("\nencoder, ");
			Serial.print(protectedCount);
			esc.writeMicroseconds(1400);
			if (protectedCount >= FULL_SHIFT_COUNT){
				Serial.println("shift down successful");
				esc.writeMicroseconds(1500);
				shiftDownRequested = false;
				saveGearProtected(gearCount);
				//TODO: sort out another shift request happening while gear is being saved to memory
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
