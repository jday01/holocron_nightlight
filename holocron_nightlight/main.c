/*
 * holocron_nightlight.c
 *
 * Created: 4/20/2017 7:31:38 AM
 * Author : jday01
 */ 

#define F_CPU 8000000UL

#define RED_LEVEL OCR1B
#define GREEN_LEVEL OCR0A
#define BLUE_LEVEL OCR0B

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

void setup_white();
void setup_rainbow();
void setup_pulse();
void tick_rainbow();
void tick_pulse();

typedef void (*eventHandler)();

#define PROGRAM_COUNT 3

const eventHandler setupHandlers[] = {
	setup_white,
	setup_rainbow,
	setup_pulse
};

const eventHandler tickHandlers[] = {
	NULL,
	tick_rainbow,
	tick_pulse
};

struct rainbowdata {
	uint8_t stage;
};

struct pulsedata {
	uint8_t color;
	uint8_t direction;
};

union programdata {
	struct rainbowdata rainbow;
	struct pulsedata pulse;
} progdata;

// program execution time, increments at ~33Hz
volatile uint32_t programTicks = 0;
volatile uint32_t pinStableAt = 0;
volatile uint8_t bouncingPins = 0;
volatile uint8_t stablePinState = 0b00001100; // assume both buttons are open at reset
uint32_t lastTick = 0;
uint8_t powerOn = 1;
uint8_t program = 0;

void togglePower() {
	if(powerOn) {
		powerOn = 0;
		// make LED pins inputs, current limiter is set up to power down LEDs when these pins float
		DDRB &= ~((1<<PB0) + (1<<PB1) + (1<<PB4));
	} else {
		powerOn = 1;
		// make LED pins outputs again
		DDRB |= (1<<PB0) + (1<<PB1) + (1<<PB4);
	}
}

void incrementProgram() {
	program++;
	if(program >= PROGRAM_COUNT) program = 0;
	if(setupHandlers[program]) setupHandlers[program]();
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
	TCCR0B = 0b00000100; // TOP is 0xFF, CLK is io/256 (should be 31.25khz)
	
	// enable PWM on OC1B
	//PLLCSR &= ~_BV(PCKE); //Set Synchronous mode (disable PLL)
	OCR1C = 255; // full 8-bit resolution on timer1 pwm
	TCCR1 = 0b11111001; // reset on OCR1C match, OCR1A enabled (OCR1B doesn't work without OCR1A, OC0B seems to be a higher priority), ck/256 prescalar
	
	if(setupHandlers[program]) setupHandlers[program]();
		
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
		if((lastTick != programTicks) && tickHandlers[program]) {
			tickHandlers[program]();
			lastTick = programTicks;
		}
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