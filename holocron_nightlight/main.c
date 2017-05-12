/*
 * holocron_nightlight.c
 *
 * Created: 4/20/2017 7:31:38 AM
 * Author : jday01
 */ 

#define F_CPU 8000000UL
// enable this define to save power/program state to internal eeprom with wear leveling
//#define USE_EEPROM

#define RED_LEVEL OCR1B
#define GREEN_LEVEL OCR0A
#define BLUE_LEVEL OCR0B

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h>

void setup_blip();
void setup_white();
void setup_rainbow();
void setup_pulse();
void setup_flash();
void tick_rainbow();
void tick_pulse();
void tick_blip();
void tick_flash();

typedef void (*eventHandler)();

#define PROGRAM_COUNT 5

int map(int x, int in_min, int in_max, int out_min, int out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

const eventHandler setupHandlers[] = {
	setup_blip,
	setup_white,
	setup_rainbow,
	setup_pulse,
	setup_flash,
};

const eventHandler tickHandlers[] = {
	tick_blip,
	NULL,
	tick_rainbow,
	tick_pulse,
	tick_flash,
};

struct rainbowdata {
	uint8_t stage;
};

struct pulsedata {
	uint8_t color;
	uint8_t direction;
};

struct blipdata {
	uint8_t delayCount;
	uint8_t targetRed;
	uint8_t targetBlue;
	uint8_t targetGreen;
	uint8_t stage;
	uint8_t delayTime;
};

struct flashdata {
	uint8_t targetRed, targetBlue, targetGreen;
	uint8_t stage;
	uint8_t delayTime;
	uint8_t delayCount;
};

union programdata {
	struct rainbowdata rainbow;
	struct pulsedata pulse;
	struct blipdata blip;
	struct flashdata flash;
} progdata;

// program execution time, increments at ~62Hz
volatile uint32_t programTicks = 0;
volatile uint32_t pinStableAt = 0;
uint32_t updateEepromAt = 0;
volatile uint8_t bouncingPins = 0;
volatile uint8_t stablePinState = 0b00001100; // assume both buttons are open at reset
uint32_t lastTick = 0;
uint8_t writeLocation = 1;
struct eepromData {
	uint8_t writeCount;
	uint8_t powerOn;
	uint8_t program;
} eepromData;

void updateEeprom() {
	#ifdef USE_EEPROM
	eepromData.writeCount++;
	if(eepromData.writeCount == 0) {
		writeLocation += sizeof(eepromData);
		if(writeLocation == 0 || writeLocation > 256 - sizeof(eepromData)) {
			writeLocation = 1;
			eeprom_busy_wait();
			eeprom_write_byte((uint8_t *)0, writeLocation);
		}
	}
	eeprom_busy_wait();
	eeprom_update_block(&eepromData, (uint8_t *)(uint16_t) writeLocation, sizeof(eepromData));
	#endif
}

void loadEeprom() {
	// always initialize eepromData, even if we aren't storing information in the eeprom
	eepromData.writeCount = 0;
	eepromData.powerOn = 1;
	eepromData.program = 0;
	#ifdef USE_EEPROM
	writeLocation = eeprom_read_byte((uint8_t *) 0);
	if(writeLocation == 0 || writeLocation == 0xff) {
		// uninitialized eeprom
		writeLocation = 1;
		eeprom_busy_wait();
		eeprom_write_byte((uint8_t *)0, writeLocation);
		updateEeprom();
	} else {
		eeprom_read_block(&eepromData, (uint8_t *)(uint16_t) writeLocation, sizeof(eepromData));
	}
	// sanity check
	if(eepromData.program > PROGRAM_COUNT) eepromData.program = 0;
	#endif
}

void togglePower() {
	if(eepromData.powerOn) {
		eepromData.powerOn = 0;
		// make LED pins inputs, current limiter is set up to power down LEDs when these pins float
		DDRB &= ~((1<<PB0) + (1<<PB1) + (1<<PB4));
	} else {
		eepromData.powerOn = 1;
		// make LED pins outputs again
		DDRB |= (1<<PB0) + (1<<PB1) + (1<<PB4);
	}
	// only save the state to the eeprom if there hasn't been any activity for 5 seconds
	updateEepromAt = programTicks + (5 * 62);
}

void incrementProgram() {
	eepromData.program++;
	if(eepromData.program >= PROGRAM_COUNT) eepromData.program = 0;
	if(setupHandlers[eepromData.program]) setupHandlers[eepromData.program]();
	// only save the state to the eeprom if there hasn't been any activity for 5 seconds
	updateEepromAt = programTicks + (5 * 62);
}

void setup_flash() {
	srand(programTicks);
	progdata.flash.delayCount = 0;
	progdata.flash.stage = 0;
	progdata.flash.delayTime = (rand() & 0x7f) + 1;
	RED_LEVEL = 0;
	BLUE_LEVEL = 0;
	GREEN_LEVEL = 0;
}

void tick_flash() {
	progdata.flash.delayCount++;
	switch(progdata.flash.stage) {
		case 0:
			// hold off for a random delay
			if(progdata.flash.delayCount == progdata.flash.delayTime) {
				progdata.flash.stage = 1;
				progdata.flash.delayTime = (rand() & 0x3f) + 64;
				progdata.flash.delayCount = 0;
				progdata.flash.targetBlue = rand();
				progdata.flash.targetGreen = rand();
				progdata.flash.targetRed = rand();
			}
			break;
		case 1:
			// head towards color
			if(progdata.flash.delayCount == progdata.flash.delayTime) {
				progdata.flash.stage = 2;
				progdata.flash.delayCount = 0;
				RED_LEVEL = progdata.flash.targetRed;
				BLUE_LEVEL = progdata.flash.targetBlue;
				GREEN_LEVEL = progdata.flash.targetGreen;
				break;
			}
			RED_LEVEL = map(progdata.flash.delayCount, 0, progdata.flash.delayTime, 0, progdata.flash.targetRed);
			GREEN_LEVEL = map(progdata.flash.delayCount, 0, progdata.flash.delayTime, 0, progdata.flash.targetGreen);
			BLUE_LEVEL = map(progdata.flash.delayCount, 0, progdata.flash.delayTime, 0, progdata.flash.targetBlue);
			break;
		case 2:
			// hold color
			if(progdata.flash.delayCount == progdata.flash.delayTime) {
				progdata.flash.stage = 3;
				progdata.flash.delayCount = 0;
			}
			break;
		case 3:
			// go to black
			if(progdata.flash.delayCount == progdata.flash.delayTime) {
				progdata.flash.stage = 0;
				progdata.flash.delayTime = (rand() % 254) + 1;
				progdata.flash.delayCount = 0;
				RED_LEVEL = 0;
				BLUE_LEVEL = 0;
				GREEN_LEVEL = 0;
				break;
			}
			RED_LEVEL = map(progdata.flash.delayCount, 0, progdata.flash.delayTime, progdata.flash.targetRed, 0);
			GREEN_LEVEL = map(progdata.flash.delayCount, 0, progdata.flash.delayTime, progdata.flash.targetGreen, 0);
			BLUE_LEVEL = map(progdata.flash.delayCount, 0, progdata.flash.delayTime, progdata.flash.targetBlue, 0);
			break;			
	}
}

void setup_blip() {
	srand(programTicks);
	progdata.blip.delayCount = (rand() & 0x7f) + 127;
	progdata.blip.stage = 0;
	RED_LEVEL = 255;
	BLUE_LEVEL = 255;
	GREEN_LEVEL = 255;
}

void tick_blip() {
	switch(progdata.blip.stage) {
		case 0:
			// hold white for a random delay
			progdata.blip.delayCount--;
			if(progdata.blip.delayCount == 0) {
				progdata.blip.stage = 1;
				progdata.blip.delayCount = (rand() & 0x3f) + 64;
				progdata.blip.delayTime = progdata.blip.delayCount;
				progdata.blip.targetBlue = rand();
				progdata.blip.targetRed = rand();
				progdata.blip.targetGreen = rand();
			}
			break;
		case 1:
			// head towards color
			progdata.blip.delayCount--;
			RED_LEVEL = map(progdata.blip.delayCount, 0, progdata.blip.delayTime, progdata.blip.targetRed, 255);
			GREEN_LEVEL = map(progdata.blip.delayCount, 0, progdata.blip.delayTime, progdata.blip.targetGreen, 255);
			BLUE_LEVEL = map(progdata.blip.delayCount, 0, progdata.blip.delayTime, progdata.blip.targetBlue, 255);
			if(progdata.blip.delayCount == 0) {
				progdata.blip.stage = 2;
				progdata.blip.delayCount = (rand() & 0x3f) + 1; // max delay 64 ticks, ~1 second, min 1
			}
			break;
		case 2:
			// hold color
			progdata.blip.delayCount--;
			if(progdata.blip.delayCount == 0) {
				progdata.blip.stage = 3;
				progdata.blip.delayCount = (rand() & 0x3f) + 64;
				progdata.blip.delayTime = progdata.blip.delayCount;
			}
			break;
		case 3:
			// return to white
			progdata.blip.delayCount--;
			RED_LEVEL = map(progdata.blip.delayCount, 0, progdata.blip.delayTime, 255, progdata.blip.targetRed);
			BLUE_LEVEL = map(progdata.blip.delayCount, 0, progdata.blip.delayTime, 255, progdata.blip.targetBlue);
			GREEN_LEVEL = map(progdata.blip.delayCount, 0, progdata.blip.delayTime, 255, progdata.blip.targetGreen);
			if(progdata.blip.delayCount == 0) {
				progdata.blip.stage = 0;
				progdata.blip.delayCount = (rand() & 0x7f) + 127;
			}
			break;
	}
}

void setup_white() {
	RED_LEVEL = 255;
	BLUE_LEVEL = 255;
	GREEN_LEVEL = 255;
}

void setup_rainbow() {
	RED_LEVEL = 255;
	BLUE_LEVEL = 0;
	GREEN_LEVEL = 0;
	progdata.rainbow.stage = 0;
}

void tick_rainbow() {
	switch(progdata.rainbow.stage) {
		case 0:
		RED_LEVEL--;
		BLUE_LEVEL++;
		if(RED_LEVEL == 0) progdata.rainbow.stage = 1;
		break;
		case 1:
		BLUE_LEVEL--;
		GREEN_LEVEL++;
		if(BLUE_LEVEL == 0) progdata.rainbow.stage = 2;
		break;
		case 2:
		GREEN_LEVEL--;
		RED_LEVEL++;
		if(GREEN_LEVEL == 0) progdata.rainbow.stage = 0;
		break;
		default:
		progdata.rainbow.stage = 0;
		break;
	}
}

void setup_pulse() {
	RED_LEVEL = 0;
	BLUE_LEVEL = 0;
	GREEN_LEVEL = 0;
	progdata.pulse.color = 0;
	progdata.pulse.direction = 1;
}

void tick_pulse() {
	if(progdata.pulse.direction) {
		switch(progdata.pulse.color) {
			case 0:
				RED_LEVEL++;
				if(RED_LEVEL == 255) progdata.pulse.direction = 0;
				break;
			case 1:
				BLUE_LEVEL++;
				if(BLUE_LEVEL == 255) progdata.pulse.direction = 0;
				break;
			case 2:
				GREEN_LEVEL++;
				if(GREEN_LEVEL == 255) progdata.pulse.direction = 0;
				break;
		}
	} else {
		switch(progdata.pulse.color) {
			case 0:
				RED_LEVEL--;
				if(RED_LEVEL == 0) {
					progdata.pulse.direction = 1;
					progdata.pulse.color = 1;
				}
				break;
			case 1:
				BLUE_LEVEL--;
				if(BLUE_LEVEL == 0) {
					progdata.pulse.direction = 1;
					progdata.pulse.color = 2;
				}
				break;
			case 2:
				GREEN_LEVEL--;
				if(GREEN_LEVEL == 0) {
					progdata.pulse.direction = 1;
					progdata.pulse.color = 0;	
				}
				break;
		}
	}
}

void setup_low() {
	RED_LEVEL = 16;
	BLUE_LEVEL = 16;
	GREEN_LEVEL = 16;
}

int main(void) {
	wdt_reset();
	// clear WDRF in MCUSR
	MCUSR = 0x00;
	// enable watchdog change
	WDTCR |= (1<<WDCE) + (1<<WDE);
	// setup watchdog prescalar to 16ms timeout
	// interrupt on watchdog overflow
	WDTCR = (1<<WDIE);
	
	// enable interrupts
	sei();
	
	// set PB0, 1 and 4 pins as outputs, 2 and 3 as inputs
	DDRB = 0b00010011;
	
	// enable pullups on PB2 and 3, pull 0, 1 and 4 high to turn off LEDs
	PORTB = 0xff;
	
	// interrupt on falling edge for external interrupts
	MCUCR |= (1<<ISC01); // ISC00 should be 0 after a reset

	// enable pin change interrupts
	GIMSK |= (1<<PCIE);
	
	// enable interrupts for changes on PB2 and 3
	PCMSK |= (1<<PCINT2) + (1<<PCINT3);
	
	// stop timer0, enable OC1B PWM output
	GTCCR = 0b11110001;
	
	// enable PWM on OC0A and OC0B
	TCCR0A = 0b11110011; // inverted PWM, Fast PWM mode
	TCCR0B = 0b00000011; // TOP is 0xFF, CLK is io/64 (should be 125khz, 488Hz PWM frequency?)
	
	// enable PWM on OC1B
	//PLLCSR &= ~_BV(PCKE); //Set Synchronous mode (disable PLL)
	OCR1C = 255; // full 8-bit resolution on timer1 pwm
	TCCR1 = 0b11110111; // reset on OCR1C match, OCR1A enabled (OCR1B doesn't work without OCR1A, OC0B seems to be a higher priority), ck/64 prescalar
	
	loadEeprom();
	if(eepromData.powerOn == 0) DDRB &= ~((1<<PB0) + (1<<PB1) + (1<<PB4));
	
	if(setupHandlers[eepromData.program]) setupHandlers[eepromData.program]();
		
	// start timer0, leave OC1B in PWM mode
	GTCCR = 0b01110000;
	
	// set all port b pins high (which turns off LEDs)
    while (1) {
		// debounce button presses, this should check to see that the button is still held low 2 ticks later, which should be between 32 and 64ms
		if(pinStableAt != 0 && pinStableAt <= programTicks) {
			if(bouncingPins & (1<<PB2)) {
				if(!(PINB & (1<<PB2))) {
					togglePower();
				}
			}
			if(bouncingPins & (1<<PB3)) {
				if(!(PINB & (1<<PB3))) {
					incrementProgram();
				}
			}
			pinStableAt = 0;
			stablePinState = PINB & 0b00001100;
		}
		if((lastTick != programTicks) && tickHandlers[eepromData.program]) {
			tickHandlers[eepromData.program]();
			lastTick = programTicks;
		}
		if(updateEepromAt != 0 && updateEepromAt <= programTicks) {
			updateEeprom();
			updateEepromAt = 0;
		}
		// FIXME - if eepromData.powerOn is 0 and bouncingPins is 0, we might as well sleep here
    }
}

// interrupt handler for watchdog timeout
ISR(WDT_vect) {
	programTicks++;
}

// interrupt handler for PB2 & 3 level changes
ISR(PCINT0_vect) {
	// find which button we're responding to, this sets bits in changedPins that correlate to new values
	//BLUE_LEVEL += 1;
	bouncingPins = (PINB & 0b00001100) ^ stablePinState;
	pinStableAt = programTicks + 2;
}