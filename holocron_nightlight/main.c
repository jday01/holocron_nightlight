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

// program execution time, increments at ~33Hz
volatile uint32_t programTicks = 0;
volatile uint32_t pinStableAt = 0;
volatile uint8_t bouncingPins = 0;
volatile uint8_t stablePinState = 0b00001100; // assume both buttons are open at reset
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
	if(program > 6) program = 0;
	RED_LEVEL = 0;
	BLUE_LEVEL = 0;
	GREEN_LEVEL = 0;
	switch(program) {
		case 0:
			RED_LEVEL = 255;
			break;
		case 1:
			RED_LEVEL = 255;
			GREEN_LEVEL = 255;
			break;
		case 2:
			GREEN_LEVEL = 255;
			break;
		case 3:
			GREEN_LEVEL = 255;
			BLUE_LEVEL = 255;
			break;
		case 4:
			BLUE_LEVEL = 255;
			break;
		case 5:
			BLUE_LEVEL = 255;
			RED_LEVEL = 255;
			break;
		default:
			RED_LEVEL = 255;
			BLUE_LEVEL = 255;
			GREEN_LEVEL = 255;
			break;
	}
}

int main(void) {
	wdt_reset();
	// clear WDRF in MCUSR
	MCUSR = 0x00;
	// enable watchdog change
	WDTCR |= (1<<WDCE) + (1<<WDE);
	// interrupt on watchdog overflow, 32ms timeout
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
	
	// turn off LEDs
	GREEN_LEVEL = 0;
	BLUE_LEVEL = 0;
	RED_LEVEL = 255;
	
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
    }
}

// interrupt handler for watchdog timeout
ISR(WDT_vect) {
	sleep_disable();
	power_all_enable();
	programTicks++;
}

// interrupt handler for PB2 & 3 level changes
ISR(PCINT0_vect) {
	// find which button we're responding to, this sets bits in changedPins that correlate to new values
	//BLUE_LEVEL += 1;
	bouncingPins = (PINB & 0b00001100) ^ stablePinState;
	pinStableAt = programTicks + 2;
}