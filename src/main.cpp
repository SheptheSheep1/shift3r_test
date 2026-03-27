#include <Arduino.h>
#include <Servo.h>
extern "C" {
	#include <hardware/flash.h>
	#include <hardware/sync.h>
};

// --- Defines ---
#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
//constexpr uint8_t ENCODER_PIN_A = 3;
//constexpr uint8_t ENCODER_PIN_B = 4;
//constexpr uint8_t LED_1 = 5;
//constexpr uint8_t LED_2 = 6;
constexpr uint8_t SHIFT_DOWN_PIN = 6;
constexpr uint8_t ESC_PIN_PWM =26;
constexpr uint8_t SHIFT_UP_PIN = 7;
constexpr uint8_t SHIFT_NEUTRAL_PIN = 0; // swapped with down
constexpr uint8_t UP = 1;
constexpr uint8_t DOWN = 0;
constexpr uint8_t ECU_PWM_PIN = 2;
constexpr uint8_t ECU_RANGE = 255;
//constexpr uint8_t TIME_UP_PIN = 1;
constexpr uint8_t SHIFT_TIMEOUT_MS = 200;

constexpr uint16_t SHIFT_UP_SPEED = 1900;
constexpr uint16_t SHIFT_DOWN_SPEED = 1100;
constexpr uint16_t SHIFT_NEUTRAL_SPEED = 1500;

// --- Macros for fast reading ---
// The Arduino core handles the fast digitalRead() for RP2040 interrupts.
#define FULL_SHIFT_COUNT 30
#define HALF_SHIFT_COUNT 100

// --- Variables ---
volatile int upCount = 0;
volatile int downCount = 0;
int protectedCount = 0;
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


// voltage for different gears: 1-0.0V, 2-1.0V, 3-2.0V, 4-3.0V, 5-4.0V, 6-5.0V
// actual voltage for different gears: 1-0.55V, 2-1.10V, 3-1.65V, 4-2.20V, 5-2.75V, 6-3.30V
// NOTE: ECU requires 5V logic, pico produces 3.3V
constexpr uint8_t DUTY_GEAR_1 = 0; // duty cycle for different gears 1-
constexpr uint8_t DUTY_GEAR_2 = 51;
constexpr uint8_t DUTY_GEAR_3 = 102;
constexpr uint8_t DUTY_GEAR_4 = 153;
constexpr uint8_t DUTY_GEAR_5 = 204;
constexpr uint8_t DUTY_GEAR_6 = 255;

// --- Pointers for Flash Writes ---
int8_t buf[FLASH_PAGE_SIZE/sizeof(int8_t)];  // One page buffer of ints
uint8_t page; // prevent comparison of unsigned and signed int
int8_t gearCount = 1;

void saveGearProtected(int8_t value){
	// implements wear-leveling
	// from https://makermatrix.com
	
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

int8_t readGearFlash() {
    int8_t lastValidGear = 1; // Default fallback
    bool foundData = false;

    for (int p = 0; p < FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE; p++) {
        uint32_t addr = XIP_BASE + FLASH_TARGET_OFFSET + (p * FLASH_PAGE_SIZE);
        int8_t *ptr = (int8_t *)addr;
        int8_t val = *ptr;

        // Flash is erased to -1 (0xFF). 
        // If it's NOT -1, it's a gear we saved.
        if (val != -1) {
            lastValidGear = val;
            foundData = true;
        } else {
            // We hit an empty page, so the previous page was the most recent data.
            break; 
        }
    }

    if (!foundData) {
        Serial.println("No gear found in flash, using default: 1");
    } else {
        Serial.print("Recovered gear from flash: ");
        Serial.println(lastValidGear);
    }

    return lastValidGear;
}

void isrA();
void isrB();
void requestShiftDown();
void requestShiftUp();
int8_t calcNextGear(int8_t gearCount, uint8_t direction);
uint8_t writeGearPos(int8_t gearPosition);


/***
 * @brief Calculates next gear to shift to based on direction if available or returns -1.
 * @details Returns -1 if no valid next gear for direction, return nextGear number 1-6
 **/
int8_t calcNextGear(int8_t gearCount, uint8_t direction){
	int8_t currentGear = gearCount;
	switch (direction) {
		case UP:
			if (currentGear < 6 && currentGear >= 1){
				Serial.println("shifting up from :"+String(currentGear));
				currentGear++;
				return currentGear;
			}//else
			break;
		case DOWN:
			if(currentGear == 2){
				Serial.println("going to neutral");
			}
			if(currentGear >= 1 && currentGear <= 6){
				Serial.println("shifting down from :"+String(currentGear));
				currentGear--;
				return currentGear;
			} //else
			break;
		default:
			Serial.println("error; unexpected direction");
			break;
	}
	Serial.println("next gear error");
	return -1;
}

/***
 * @brief Writes to pwm gear position (inverted), intended for use with NPN transistor to go from 3.3 logic to 5
 * @param gearPosition Current Gear Position (1-6)
 * @return -1 if error, 0 if success
 **/
uint8_t writeGearPos(int8_t gearPosition){
	switch (gearPosition) {
		case 1:
			analogWrite(ECU_PWM_PIN, (ECU_RANGE - DUTY_GEAR_1));
			break;
		case 2:
			analogWrite(ECU_PWM_PIN, (ECU_RANGE - DUTY_GEAR_2));
			break;
		case 3:
			analogWrite(ECU_PWM_PIN, (ECU_RANGE - DUTY_GEAR_3));
			break;
		case 4:
			analogWrite(ECU_PWM_PIN, (ECU_RANGE - DUTY_GEAR_4));
			break;
		case 5:
			analogWrite(ECU_PWM_PIN, (ECU_RANGE - DUTY_GEAR_5));
			break;
		case 6:
			analogWrite(ECU_PWM_PIN, (ECU_RANGE - DUTY_GEAR_6));
			break;
		default:
			Serial.println("error!!! invalid gear pos: "+String(gearPosition));
			return -1; // error
	}
	return 0; // no error
}

void setup()
{
    // Initialize motor
    esc.attach(ESC_PIN_PWM); 

    // Use Arduino pinMode() for input setup
	//pinMode(SHIFT_DOWN_PIN, INPUT_PULLDOWN);
	//pinMode(SHIFT_UP_PIN, INPUT_PULLDOWN);
    // This replaces gpio_init(), gpio_set_dir(), and gpio_pull_up()
	pinMode(SHIFT_DOWN_PIN, INPUT);
	pinMode(SHIFT_UP_PIN, INPUT);
	pinMode(SHIFT_NEUTRAL_PIN, INPUT);
	//pinMode(LED_1, OUTPUT);
	//pinMode(LED_2, OUTPUT);
	//digitalWrite(LED_1, LOW);
	//digitalWrite(LED_2, LOW);
	pinMode(ECU_PWM_PIN, OUTPUT);
	//pinMode(TIME_UP_PIN, OUTPUT);
	//digitalWrite(TIME_UP_PIN, LOW);
	analogWriteFreq(100000);
	analogWriteRange(255);
	//digitalReadFast(1);

	attachInterrupt(digitalPinToInterrupt(SHIFT_DOWN_PIN), requestShiftDown, RISING);
	attachInterrupt(digitalPinToInterrupt(SHIFT_UP_PIN), requestShiftUp, RISING);
    
    Serial.begin(115200);
	// restore gear from flash
	// gearCount = readGearFlash();
	//gearCount = 1;
	//saveGearProtected(1);

    delay(1000); // Give time for serial to initialize
    Serial.println("Servo system active.");
	Serial.print("\ngear count:");
	Serial.print(gearCount);
	//Serial.print(*readGearFlash(), HEX);
	Serial.print("\n");
	//shiftDownRequested = true;
}

void requestShiftUp(void){
	//Serial.println("UP HIGH");
	shiftUpRequested = true;
}

void requestShiftDown(void){
	//Serial.println("DOWN HIGH");
	shiftDownRequested = true;
}

void loop()
{
	if(Serial.available()){

	}
    // Safely read the volatile count variable
	if(shiftUpRequested && shiftDownRequested){Serial.println("conflict!!!");
		shiftUpRequested = false;
		shiftDownRequested = false;
	}//do something
	if(shiftUpRequested){
		bool error = (gearCount >= 6);
		//Serial.println("errorUp: "+String(error));
		// attempt shift up
		// if in first gear shift blah blah blah
		if(gearCount >= 6){Serial.println("ignoring...");
			Serial.println("current gear: "+String(gearCount));
		shiftUpRequested = false;}
		if(isFirstShiftUpRun && shiftUpRequested && !error){
			startMillis = millis();
			Serial.println("Shift Up Requested...");
			Serial.println("first shift, start millis count and first shift set false");
			isFirstShiftUpRun = false;
			//noInterrupts();
			//digitalWriteFast(TIME_UP_PIN, HIGH);
			//interrupts();
		}

		//Serial.println("Shift Up Requested...");

		if(shiftDownRequested){
			Serial.println("Canceling Shift Down...");
			noInterrupts();
			esc.writeMicroseconds(SHIFT_NEUTRAL_SPEED);
			shiftDownRequested = false;
			interrupts();
		}

		currentMillis = millis();
		//Serial.println(currentMillis-startMillis);
		if ((currentMillis - startMillis < SHIFT_TIMEOUT_MS)&&!error)  //test whether the period has elapsed
		{
			//digitalWriteFast(LED_1, HIGH);
			//Serial.print("\nencoder, ");
			//Serial.print(count);
			esc.writeMicroseconds(SHIFT_DOWN_SPEED);
		}else {
			gearCount = calcNextGear(gearCount, UP);
			Serial.println("up timeout");
			//digitalWriteFast(TIME_UP_PIN, LOW);
			esc.writeMicroseconds(SHIFT_NEUTRAL_SPEED);
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
		Serial.println("current gear: "+String(gearCount));
		shiftDownRequested = false;}
		if(isFirstShiftDownRun && shiftDownRequested && !error){
			startMillis = millis();
			Serial.println("Shift Down Requested...");
			Serial.println("first shift, start millis count and first shift set false");
			isFirstShiftDownRun = false;
			//noInterrupts();
			//digitalWriteFast(TIME_UP_PIN, HIGH);
			//interrupts();
		}

		if(shiftUpRequested){
			Serial.println("Canceling Shift Up...");
			noInterrupts();
			esc.writeMicroseconds(SHIFT_NEUTRAL_SPEED);
			shiftUpRequested = false;
			//gearCount--;
			interrupts();
		}

		currentMillis = millis();
		//Serial.println("time delta:" + String(currentMillis-startMillis));
		//Serial.print("\ndown encoder, ");
		if ((currentMillis - startMillis < SHIFT_TIMEOUT_MS) && !error)  //test whether the period has elapsed
		{
			//digitalWriteFast(LED_1, HIGH);
			//Serial.print("\nencoder, ");
			esc.writeMicroseconds(SHIFT_UP_SPEED);
		}else {
			//digitalWriteFast(TIME_UP_PIN, LOW);
			Serial.println("down timeout");
			esc.writeMicroseconds(SHIFT_NEUTRAL_SPEED);
			shiftDownRequested = false;
			isFirstShiftDownRun = true;
			gearCount = calcNextGear(gearCount, DOWN);
			saveGearProtected(gearCount);
		}
		

		//shiftDownRequested = false;
	}
	//if(digitalRead(SHIFT_DOWN_PIN)){
		//Serial.println("DOWN HIGH"); 
	//}
	//if(digitalRead(SHIFT_UP_PIN)){
		//Serial.println("UP HIGH");
	//}
	//if(digitalRead(SHIFT_NEUTRAL_PIN)){
		//Serial.println("NEUTRAL HIGH");
	//}
	//Serial.println(String(digitalRead(SHIFT_UP_PIN)) + String(digitalRead(SHIFT_DOWN_PIN)) + String(digitalRead(SHIFT_NEUTRAL_PIN)));
	//interrupts();
	//gearCount = 6;
	writeGearPos(gearCount);
}
