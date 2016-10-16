/* -*- mode: jde; c-basic-offset: 2; indent-tabs-mode: nil -*- */

/*
  Part of the Wiring project - http://wiring.org.co

  Copyright (c) 2004-10 Hernando Barragan
  Based on pulse.c by Pascal Stang - Copyright (C) 2000-2002

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

// The pulseOut command will work on PWM output pins 3, 4, 5

#include "WProgram.h"

void pulseT1Init(void);
void pulseT1Off(void);
void pulseT1ASetFreq(uint16_t);
//void pulseT1BSetFreq(uint16_t);
//void pulseT1CSetFreq(uint16_t);
void pulseT1ARun(uint16_t);
//void pulseT1BRun(uint16_t);
//void pulseT1CRun(uint16_t);
void pulseT1AStop(void);
//void pulseT1BStop(void);
//void pulseT1CStop(void);
uint16_t pulseT1ARemaining(void);
//uint16_t pulseT1BRemaining(void);
//uint16_t pulseT1CRemaining(void);
void pulseT1AService(void);
//void pulseT1BService(void);
//void pulseT1CService(void);
void beginPulse(void);
void pulseFrequency(uint8_t, uint16_t);
void pulseRun(uint8_t, uint16_t);
void pulseOut(uint8_t pin, uint16_t freq);
void pulseOut(uint8_t, uint16_t, uint16_t);

// pulse generation registers
volatile static uint8_t  pulseT1AMode;
volatile static int pulseT1ACount;
volatile static uint16_t pulseT1APeriodTics;
//volatile static uint8_t  pulseT1BMode;
//volatile static int pulseT1BCount;
//volatile static uint16_t pulseT1BPeriodTics;
//volatile static uint8_t  pulseT1CMode;
//volatile static int pulseT1CCount;
//volatile static uint16_t pulseT1CPeriodTics;

// pulse mode bit definitions
// PULSE_MODE_COUNTED
// if true, the requested number of pulses are output, then output is turned off
// if false, pulses are output continuously
#define PULSE_MODE_CONTINUOUS		0x00
#define PULSE_MODE_COUNTED		0x01


/*************************************************************
 * Pulse Input
 *************************************************************/

/* Measures the length (in microseconds) of a pulse on the pin
 * by Mellis from Arduino for core API compatibility
 * State is HIGH or LOW, the type of pulse to measure
 * Works on pulses from 10 microseconds to 3 minutes in length
 * Must be called at least N microseconds before the start of the pulse
 */
unsigned long pulseIn(uint8_t pin, uint8_t state) {
  pulseIn(pin, state, 1000000UL);
}


unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
{
  // cache the port and bit of the pin in order to speed up the
  // pulse width measuring loop and achieve finer resolution.  calling
  // digitalWrite() instead yields much coarser resolution.
  //int r = port_to_input[digitalPinToPort(pin)];
  //int bit = digitalPinToBit(pin);
  int r = _SFR_IO_ADDR(*portspin[pin/8]);
  int bit = pin % 8;
  int mask = 1 << bit;
  unsigned long width = 0;

  unsigned long numloops = 0;
  unsigned long maxloops = microsecondsToClockCycles(timeout) / 16;


  // compute the desired bit pattern for the port reading (e.g. set or
  // clear the bit corresponding to the pin being read).  the !!state
  // ensures that the function treats any non-zero value of state as HIGH.
  state = (!!state) << bit;

  // wait for any previous pulse to end
  while ((_SFR_IO8(r) & mask) == state)
    if (numloops++ == maxloops)
      return 0;

  // wait for the pulse to start
  while ((_SFR_IO8(r) & mask) != state)
    if (numloops++ == maxloops)
      return 0;

  // wait for the pulse to stop
  while ((_SFR_IO8(r) & mask) == state)
    width++;

  // convert the reading to microseconds.  the slower the CPU speed, the
  // proportionally fewer iterations of the loop will occur (e.g. a
  // 4 MHz clock will yield a width that is one-fourth of that read with
  // a 16 MHz clock).  each loop was empirically determined to take
  // approximately 23/20 of a microsecond with a 16 MHz clock.
  return clockCyclesToMicroseconds(width * 10 + 16); //width * (16000000UL / F_CPU) * 20 / 23;
}


/*************************************************************
 * Pulse Output
 *************************************************************/


void pulseOut(uint8_t pin, uint16_t freq) {
  pulseOut(pin, freq, 0);
}


void pulseT1Init(void) {
  
  timer1Init();

  // try to make sure that timer1 is in "normal" mode
  // most importantly, turn off PWM mode
  timer1PWMOff();

  TCCR1B |= _BV(WGM12);
  
  pulseT1AMode = 0;
//  pulseT1BMode = 0;
//  pulseT1CMode = 0;
  
  pulseT1ACount = 0;
//  pulseT1BCount = 0;
//  pulseT1CCount = 0;
  
  pulseT1APeriodTics = 0x8000;
//  pulseT1BPeriodTics = 0x8000;
//  pulseT1CPeriodTics = 0x8000;
  
  timerAttach(TIMER1OUTCOMPAREA_INT, pulseT1AService);
//  timerAttach(TIMER1OUTCOMPAREB_INT, pulseT1BService);
//  timerAttach(TIMER1OUTCOMPAREC_INT, pulseT1CService);
}


void pulseOut(uint8_t pin, uint16_t freq, uint16_t nPulses=0) {
  beginPulse();
  pinMode(pin, OUTPUT);
  pulseFrequency(pin, freq);
  pulseRun(pin, nPulses);  
}


inline void beginPulse() {
  pulseT1Init();
}


inline void endPulse() {
  pulseT1Off();
}


void pulseFrequency(uint8_t channel, uint16_t freq) {
  switch (channel) {
    case 5:
    case 29:
      //if(channel == 5) 
      pulseT1ASetFreq(freq);
      break;
/*
    case 4:
    case 30:
      //if(channel == 4)
      pulseT1BSetFreq(freq);
      break;
    case 3:
    case 31:
      //if(channel == 3)
      pulseT1CSetFreq(freq);
      break;
*/
    default:
      break;
  }
}


void pulseRun(uint8_t channel, uint16_t nPulses) {
  switch (channel) {
    case 5:
    case 29:
      pulseT1ARun(nPulses);
      break;
/*
    case 4:
    case 30:
      pulseT1BRun(nPulses);
      break;
    case 3:
    case 31:
      pulseT1CRun(nPulses);
      break;
*/
    default:
      break;
  }
}


void pulseStop(uint8_t channel) {
  switch (channel) {
    case 5:
    case 29:
      pulseT1AStop();
      break;
/*    case 4:
    case 30: 
      pulseT1BStop();
      break;
    case 3:
    case 31:
      pulseT1CStop();
      break;
*/
    default:
      break;
  }
}


uint16_t pulseRunning(uint8_t channel) {
  switch (channel) {
    case 5:
    case 29:
      return pulseT1ARemaining();
      break;
/*    case 4:
    case 30:
      return pulseT1BRemaining();
      break;
    case 3:
    case 31:
      return pulseT1CRemaining();
      break;
*/
    default:
      break;
  }
  return 0;
}


void pulseT1Off(void) {
  // turns pulse outputs off immediately
	
  pulseT1ACount = 0;
//  pulseT1BCount = 0;
//  pulseT1CCount = 0;
  
  // disconnect OutputCompare action from OC1A pin
  TCCR1A &= ~_BV(COM1A1);
  TCCR1A &= ~_BV(COM1A0);
  
  // disconnect OutputCompare action from OC1B pin
//  TCCR1A &= ~_BV(COM1B1);
//  TCCR1A &= ~_BV(COM1B0);
  
  // disconnect OutputCompare action from OC1C pin
//  TCCR1A &= ~_BV(COM1C1);
//  TCCR1A &= ~_BV(COM1C0);

  timerDetach(TIMER1OUTCOMPAREA_INT);
//  timerDetach(TIMER1OUTCOMPAREB_INT);
//  timerDetach(TIMER1OUTCOMPAREC_INT);
}


void pulseT1ASetFreq(uint16_t freqHz) {
  // get the current prescaler setting
  // calculate how many tics in period/2
  // this is the (timer tic rate)/(2*requested freq)
  pulseT1APeriodTics = ((uint32_t)CPU_FREQ/((uint32_t)timer1GetPrescaler()*2*freqHz));
}


/*
void pulseT1BSetFreq(uint16_t freqHz) {
  pulseT1BPeriodTics = ((uint32_t)CPU_FREQ/((uint32_t)timer1GetPrescaler()*2*freqHz));
}

void pulseT1CSetFreq(uint16_t freqHz) {
  pulseT1CPeriodTics = ((uint32_t)CPU_FREQ/((uint32_t)timer1GetPrescaler()*2*freqHz));
}
*/


void pulseT1ARun(uint16_t nPulses) {
  if(nPulses > 0) {
    // if the nPulses is non-zero, use "counted" mode
	  pulseT1AMode = PULSE_MODE_COUNTED;
    pulseT1ACount = nPulses<<1;
  }
  else {
    // if nPulses is zero, run forever
    pulseT1AMode = PULSE_MODE_CONTINUOUS;
    pulseT1ACount = 1<<1;
  }
  
  // set OutputCompare action to toggle OC1A pin
  TCCR1A &= ~_BV(COM1A1); // toggle pin
  TCCR1A |= _BV(COM1A0);
  
  //OCR1A = TCNT1 + pulseT1APeriodTics;
  
  //OCR1A += pulseT1APeriodTics;
  OCR1A = pulseT1APeriodTics;  

  // enable OutputCompare interrupt
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  TIMSK1 |= _BV(OCIE1A);
#else
  TIMSK |= _BV(OCIE1A);
#endif
}


/*
void pulseT1BRun(uint16_t nPulses) {
  if(nPulses > 0) {
    pulseT1BMode = PULSE_MODE_COUNTED;
    pulseT1BCount = nPulses<<1;
  }
  else {
    pulseT1BMode = PULSE_MODE_CONTINUOUS;
    pulseT1BCount = 1<<1;
  }
  TCCR1A &= ~_BV(COM1B1);
  TCCR1A |= _BV(COM1B0);
  //OCR1B = TCNT1 + pulseT1BPeriodTics;

  //OCR1B += pulseT1BPeriodTics;
  OCR1B = pulseT1BPeriodTics;
  
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  TIMSK1 |= _BV(OCIE1B);
#else
  TIMSK |= _BV(OCIE1B);
#endif
}


void pulseT1CRun(uint16_t nPulses) {
  if(nPulses > 0) {
    pulseT1CMode = PULSE_MODE_COUNTED;
    pulseT1CCount = nPulses<<1;
  }
  else {
    pulseT1CMode = PULSE_MODE_CONTINUOUS;
    pulseT1CCount = 1<<1;
  }
  TCCR1A &= ~_BV(COM1C1);
  TCCR1A |= _BV(COM1C0);


  //OCR1C += pulseT1CPeriodTics;
  OCR1C = pulseT1CPeriodTics;
  
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  TIMSK1 |= _BV(OCIE1C);
#else
  ETIMSK |= _BV(OCIE1C);
#endif
}
*/


void pulseT1AStop(void) {
  // stop output regardless of remaining pulses or mode
  // go to "counted" mode
  pulseT1AMode = PULSE_MODE_COUNTED;
  pulseT1ACount = 0;
}


/*
void pulseT1BStop(void) {
  pulseT1BMode = PULSE_MODE_COUNTED;
  pulseT1BCount = 0;
}


void pulseT1CStop(void) {
  pulseT1CMode = PULSE_MODE_COUNTED;
  pulseT1CCount = 0;
}
*/


uint16_t pulseT1ARemaining(void) {
  return pulseT1ACount >> 1;
}


/*
uint16_t pulseT1BRemaining(void) {
  return pulseT1BCount >> 1;
}


uint16_t pulseT1CRemaining(void) {
  return pulseT1CCount >> 1;
}
*/


void pulseT1AService(void) {
  if(pulseT1ACount>0) {
    //OCR1A += pulseT1APeriodTics;
				
    if(pulseT1AMode == PULSE_MODE_COUNTED)
      pulseT1ACount--;
  }
  else {
    // pulse count has reached zero
    // disable the output compare's action on OC1A pin
    TCCR1A &= ~_BV(COM1A1);
    TCCR1A &= ~_BV(COM1A0);
    
    // disable the output compare's interrupt to stop pulsing
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
    TIMSK1 &= ~_BV(OCIE1A);
#else
    TIMSK &= ~_BV(OCIE1A);
#endif
  }
}


/*
void pulseT1BService(void) {
  if(pulseT1BCount > 0) {
    //OCR1B += pulseT1BPeriodTics;

    if(pulseT1BMode == PULSE_MODE_COUNTED)
      pulseT1BCount--;
  }
  else {
    TCCR1A &= ~_BV(COM1B1);
    TCCR1A &= ~_BV(COM1B0);
    
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
    TIMSK1 &= ~_BV(OCIE1B);
#else
    TIMSK &= ~_BV(OCIE1B);
#endif
  }
}


void pulseT1CService(void) {
  if(pulseT1CCount > 0) {
    //OCR1C += pulseT1CPeriodTics;

    if(pulseT1CMode == PULSE_MODE_COUNTED)
      pulseT1CCount--;
  }
  else {
    pulseT1CCount-=10;
    TCCR1A &= ~_BV(COM1C1);
    TCCR1A &= ~_BV(COM1C0);
    
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
#else
    ETIMSK &= ~_BV(OCIE1C);
#endif
  }
}
*/
