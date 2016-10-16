/* -*- mode: jde; c-basic-offset: 2; indent-tabs-mode: nil -*- */

/*
  Part of the Wiring project - http://wiring.org.co

  Copyright (c) 2004-10 Hernando Barragan

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

/*************************************************************
 * Includes
 *************************************************************/

#include <inttypes.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "WProgram.h"

/*************************************************************
 * Delay & Timer
 *************************************************************/

#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)


//volatile unsigned long mscount;
//volatile unsigned long timecount;

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
static unsigned char timer0_fract = 0;

extern uint8_t pwmpins[];

// Previuosly called initTimer, initializes main clock used for delay functions
void initWiring(void)
{
////  TIFR  |= _BV(OCIE0);
////  TCCR0  = _BV(WGM01)|_BV(CS02)|_BV(CS00); /* CTC, prescale = 128 */
////  TCNT0  = 0;
////  TIMSK |= _BV(OCIE0);    /* enable output compare interrupt */
////  OCR0   = 125;          /* match in 1 ms */

//  TIFR  |= _BV(OCIE0)|_BV(TOIE0);
//  TIMSK |= _BV(OCIE0);    /* enable output compare interrupt */
//  TIMSK &= ~_BV(TOIE0);   /* disable overflow interrupt */
//  ASSR  |= _BV(AS0);      /* use asynchronous clock source */
//  TCNT0  = 0;
//  OCR0   = 32;            /* match in 0.9765625 ms */
//  TCCR0  = _BV(WGM01) | _BV(CS00); /* CTC, no prescale */
//  while (ASSR & 0x07)
//    ;
//  TIFR  |= _BV(OCIE0)|_BV(TOIE0);
  timer0Init();
}

/*
ISR(SIG_OUTPUT_COMPARE0) {
  mscount++;
  timecount++;
}
*/


ISR(TIMER0_OVF_vect) {
  // copy these to local variables so they can be stored in registers
  // (volatile variables must be read from memory on every access)
  unsigned long m = timer0_millis;
  unsigned char f = timer0_fract;

  m += MILLIS_INC;
  f += FRACT_INC;
  if(f >= FRACT_MAX) {
    f -= FRACT_MAX;
    m += 1;
  }

  timer0_fract = f;
  timer0_millis = m;
  timer0_overflow_count++;  
}


unsigned long millis() {
  unsigned long m;
  uint8_t oldSREG = SREG;

  // disable interrupts while we read timer0_millis or we might get an
  // inconsistent value (e.g. in the middle of a write to timer0_millis)
  cli();
  m = timer0_millis;
  SREG = oldSREG;

  return m;
  //return timecount;
}


unsigned long micros() {
  unsigned long m, t;
  uint8_t oldSREG = SREG;
        
  cli();  
  m = timer0_overflow_count;
  t = TCNT0;
  
#ifdef TIFR0
  if ((TIFR0 & _BV(TOV0)) && (t < 255))
    m++;
#else
  if ((TIFR & _BV(TOV0)) && (t < 255))
    m++;
#endif

  SREG = oldSREG;
       
  return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}


void delay(unsigned long ms) {
  uint16_t start = (uint16_t)micros();

  while (ms > 0) {
    if (((uint16_t)micros() - start) >= 1000) {
      ms--;
      start += 1000;
    }
  }
//  unsigned long start = millis();
        
//  while (millis() - start < ms)
//    ;
}

/* Delay for the given number of microseconds.
 * From D.Mellis for Arduino
 * Assumes a 16 MHz clock. 
 * Disables interrupts, disrupts millis() if used frequently
 * note: digitalWrite() executes in 2.5 microseconds
 */

void delayMicroseconds(unsigned int us)
{
  // calling avrlib's delay_us() function with low values (e.g. 1 or
  // 2 microseconds) gives delays longer than desired.
  //delay_us(us);

#if F_CPU >= 16000000L
  // for a one-microsecond delay, simply return.  the overhead
  // of the function call yields a delay of approximately 1 1/8 us.
  if (--us == 0)
    return;

  // the following loop takes a quarter of a microsecond (4 cycles)
  // per iteration, so execute it four times for each microsecond of
  // delay requested.
  us <<= 2;

  // account for the time taken in the preceeding commands.
  us -= 2;
#else
  if (--us == 0)
    return;
  if (--us == 0)
    return;
  us <<= 1;
  us--;
#endif
  // disable interrupts, otherwise the timer 0 overflow interrupt that
  // tracks milliseconds will make us delay longer than we want.
  //cli();

  // busy wait
  __asm__ __volatile__ (
    "1: sbiw %0,1" "\n\t" // 2 cycles
    "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
  );

  // reenable interrupts.
  //sei();
}


/*************************************************************
 * Digital I/O
 *************************************************************/

const uint8_t pins[8] = {
  WPIN0, WPIN1, WPIN2, WPIN3, WPIN4, WPIN5, WPIN6, WPIN7
};

volatile uint8_t * ports[7] = {
  &PORTD, &PORTC, &PORTA, &PORTB, &PORTE, &PORTF, &PORTG
};

volatile uint8_t * portsddr[7] = {
  &DDRD, &DDRC, &DDRA, &DDRB, &DDRE, &DDRF, &DDRG
};

volatile uint8_t * portspin[7] = {
  &PIND, &PINC, &PINA, &PINB, &PINE, &PINF, &PING
};

void pinMode(uint8_t pin, uint8_t mode) {
  if(mode == OUTPUT) {
    uint8_t oldSREG = SREG;
    cli();
    *portsddr[pin/8] |= pins[pin%8];
    SREG = oldSREG;
  } else {
    uint8_t oldSREG = SREG;
    cli();
    *portsddr[pin/8] &= ~pins[pin%8];
    SREG = oldSREG;
  }
}

inline uint8_t digitalRead(uint8_t pin) {
  if(*portspin[pin/8] & pins[pin%8])
    return HIGH;
  return LOW;
}

/*
 * Michael Margolis
 * this version is based on Paul Stoffregen's Nov13 2009 submission to the
 * Arduino developers list
 */
//inline void digitalWrite(uint8_t, uint8_t) __attribute__((always_inline, unused));
/*inline void digitalWrite(uint8_t P, uint8_t V)
{
       (__builtin_constant_p(P) && __builtin_constant_p(V))
       ? bitWrite(*((volatile unsigned char *) digitalPinToPortReg(P)), digitalPinToBit(P), V)
       : _digitalWrite(P, V);
} 
*/

void _digitalWrite(uint8_t pin, uint8_t val) {
  if(val == HIGH) {
    uint8_t oldSREG = SREG;
    cli();
    *ports[pin/8] |= pins[pin%8];
    SREG = oldSREG;
  } else {
    uint8_t oldSREG = SREG;
    cli();
    *ports[pin/8] &= ~pins[pin%8];
    SREG = oldSREG;
  }
}


void portMode(uint8_t port, uint8_t mode) {
  if(mode == OUTPUT) {
    *portsddr[port] = 0xff;
  } else {
    *portsddr[port] = 0x00;
  }
}

uint8_t portRead(uint8_t port) {
  return *portspin[port];
}

void portWrite(uint8_t port, uint8_t val) {
  *ports[port] = val;
}


/*************************************************************
 * Analog Input
 *************************************************************/

uint8_t adcFirstTime = true;
uint8_t adcLastPin = 0xFF; //save last pin

void adcInit() {
  DDRF = 0x00;
  PORTF = 0x00;
  ADMUX = _BV(REFS0);
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADATE)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0);
  ADCSRB &= ~(_BV(ADTS2)|_BV(ADTS1)|_BV(ADTS0)); 
#else
  ADCSR = _BV(ADEN)|_BV(ADSC)|_BV(ADFR)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0);
#endif
}


int analogRead(uint8_t pin) {
  if(adcFirstTime == true) {
    adcInit();
    adcFirstTime = false;
  }

// fixes analogRead bug when reading multiple analog inputs
// thanks to Bjoern Hartman bjoern@stanford.edu

  if(pin == adcLastPin) {
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
    ADCSRA |= _BV(ADIF); //"start": set ADIF to interrupt flag
    while((ADCSRA & _BV(ADIF)) == 0) ;  //wait until conversion completes (ADIF set to 0)
#else
    ADCSR |= _BV(ADIF); //"start": set ADIF to interrupt flag
    while((ADCSR & _BV(ADIF)) == 0) ;  //wait until conversion completes (ADIF set to 0)
#endif
  } else {
    adcLastPin=pin; //save new channel
    ADMUX = (ADMUX & 0xe0) | (pin & 0x07);  //set new conversion channel
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
    ADCSRA |= _BV(ADIF);  //"start": set ADIF to interrupt flag
    while((ADCSRA & _BV(ADIF)) == 0) ;  //wait until conversion completes (ADIF set to 0)
    //now do it again since we don't get correct channel until 2nd conversion
    ADCSRA |= _BV(ADIF);  //"start" again: set ADIF to interrupt flag
    while((ADCSRA & _BV(ADIF)) == 0) ;  //wait until 2nd conversion completes
#else
    ADCSR |= _BV(ADIF);  //"start": set ADIF to interrupt flag
    while((ADCSR & _BV(ADIF)) == 0) ;  //wait until conversion completes (ADIF set to 0)
    //now do it again since we don't get correct channel until 2nd conversion
    ADCSR |= _BV(ADIF);  //"start" again: set ADIF to interrupt flag
    while((ADCSR & _BV(ADIF)) == 0) ;  //wait until 2nd conversion completes
#endif
  }
  return ADC;
}

/*
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, byte val) {
  int i;

  for (i = 0; i < 8; i++)  {
    if (bitOrder == LSBFIRST)
      digitalWrite(dataPin, !!(val & (1 << i)));
    else	
      digitalWrite(dataPin, !!(val & (1 << (7 - i))));
			
    digitalWrite(clockPin, HIGH);
    digitalWrite(clockPin, LOW);		
  }
}
*/

/*************************************************************
 * Main Program
 *************************************************************/
/*
extern void setup(void);
extern void loop(void);

int main(void) {
  //initWiring();
  portMode(3, OUTPUT);
  portMode(4, OUTPUT); 
  sei();
  setup();
  for(;;) {
    loop();
  }
} 
*/
