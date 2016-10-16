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

#ifndef WPROGRAM_H
#define WPROGRAM_H

#ifdef __cplusplus
extern "C" {
#endif

/*************************************************************
 * C Includes
 *************************************************************/

#include <math.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include "WConstants.h"
#include "Binary.h"

/*************************************************************
 * C Core API Functions
 *************************************************************/

// timing prototypes
void initWiring(void);
void delay(unsigned long);
void delayMicroseconds(unsigned int);
unsigned long millis(void);
unsigned long micros(void);

// pin prototypes
void pinMode(uint8_t, uint8_t);
uint8_t digitalRead(uint8_t);
void _digitalWrite(uint8_t, uint8_t);

/*
 * Michael Margolis
 * this version is based on Paul Stoffregen's Nov13 2009 submission to the
 * Arduino developers list
 */
static inline void digitalWrite(uint8_t, uint8_t) __attribute__((always_inline, unused));
static inline void digitalWrite(uint8_t P, uint8_t V)
{
   if(__builtin_constant_p(P) && __builtin_constant_p(V)) {
       if(V) {
           uint8_t oldSREG = SREG;
           cli();
           *digitalPinToPortReg(P) |= _BV(digitalPinToBit(P));
	   SREG = oldSREG;
       } else {
	   uint8_t oldSREG = SREG;
	   cli();
           *digitalPinToPortReg(P)&= ~ _BV(digitalPinToBit(P));  
           SREG = oldSREG;
       }
   }
   else
       _digitalWrite(P, V);
}


/* 
 * Power management
 */
static inline void powerEnable(uint8_t) __attribute__((always_inline, unused));
static inline void powerEnable(uint8_t P)
{
#if !defined(__AVR_ATmega128__)
  switch(P) {
    case POWER_ADC:
      power_adc_enable();
      break;
    case POWER_SPI:
      power_spi_enable();
      break;
    case POWER_WIRE:
      power_twi_enable();
      break;
    case POWER_TIMER0:
      power_timer0_enable();
      break;
    case POWER_TIMER1:
      power_timer1_enable();
      break;
    case POWER_TIMER2:
      power_timer2_enable();
      break;
    case POWER_TIMER3:
      power_timer3_enable();
      break;
    case POWER_TIMER4:
      power_timer4_enable();
      break;
    case POWER_TIMER5:
      power_timer5_enable();
      break;
    case POWER_SERIAL0:
      power_usart0_enable();
      break;
    case POWER_SERIAL1:
      power_usart1_enable();
      break;
    case POWER_SERIAL2:
      power_usart2_enable();
      break;
    case POWER_SERIAL3:
      power_usart3_enable();
      break;
    case POWER_ALL:
      power_all_enable();
    default:
      break;
  }
#endif
}

static inline void powerDisable(uint8_t) __attribute__((always_inline, unused));
static inline void powerDisable(uint8_t P)
{
#if !defined(__AVR_ATmega128__)
  switch(P) { 
    case POWER_ADC:
      power_adc_disable();
      break;
    case POWER_SPI:
      power_spi_disable();
      break;
    case POWER_WIRE:
      power_twi_disable();
      break;
    case POWER_TIMER0:
      power_timer0_disable();
      break;
    case POWER_TIMER1:
      power_timer1_disable();
      break;
    case POWER_TIMER2:
      power_timer2_disable();
      break;
    case POWER_TIMER3:
      power_timer3_disable();
      break;
    case POWER_TIMER4:
      power_timer4_disable();
      break;
    case POWER_TIMER5:
      power_timer5_disable();
      break;
    case POWER_SERIAL0:
      power_usart0_disable();
      break;
    case POWER_SERIAL1:
      power_usart1_disable();
      break;
    case POWER_SERIAL2:
      power_usart2_disable();
      break;
    case POWER_SERIAL3:
      power_usart3_disable();
      break;
    case POWER_ALL:
      power_all_disable();
    default:
      break;
  }
#endif
}


void portMode(uint8_t, uint8_t);
uint8_t portRead(uint8_t);
void portWrite(uint8_t, uint8_t);
int analogRead(uint8_t);
void analogWrite(uint8_t, uint16_t);

// pulse prototypes
// unsigned long pulseIn(uint8_t, uint8_t);
// void shiftOut(uint8_t, uint8_t, uint8_t, uint8_t);

// interrupt management prototypes
void attachInterrupt(uint8_t, void (*)(void), uint8_t mode);
void detachInterrupt(uint8_t);
void interruptMode(uint8_t, uint8_t);
void attachInterruptSPI(void (*)(void));
void detachInterruptSPI(void);

// timer management prototypes
void timerAttach(uint8_t, void(*userFunc)(void));
void timerDetach(uint8_t);
void timer0Init(void) __attribute__ ((naked)) __attribute__ ((section (".init3")));
void timerInit(void);
void timer1Init(void);
void timer2Init(void);
void timer3Init(void);
void timer0SetPrescaler(uint8_t);
void timer1SetPrescaler(uint8_t);
void timer2SetPrescaler(uint8_t);
void timer3SetPrescaler(uint8_t);
uint16_t timer0GetPrescaler(void);
uint16_t timer1GetPrescaler(void);
uint16_t timer2GetPrescaler(void);
uint16_t timer3GetPrescaler(void);

  
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
void timer4Init(void);
void timer5Init(void);
void timer4SetPrescaler(uint8_t prescale);
void timer5SetPrescaler(uint8_t prescale);
uint16_t timer4GetPrescaler(void);
uint16_t timer5GetPrescaler(void);
#endif

void timerPause(unsigned short );
void timer1PWMInit(uint8_t);
void timer1PWMOff(void);
void timer1PWMAOn(void);
void timer1PWMBOn(void);
void timer1PWMCOn(void);
void timer1PWMAOff(void);
void timer1PWMBOff(void);
void timer1PWMCOff(void);
void timer1PWMASet(uint16_t);
void timer1PWMBSet(uint16_t);
void timer1PWMCSet(uint16_t);
void timer3PWMInit(uint8_t);
void timer3PWMOff(void);
void timer3PWMAOn(void);
void timer3PWMBOn(void);
void timer3PWMCOn(void);
void timer3PWMAOff(void);
void timer3PWMBOff(void);
void timer3PWMCOff(void);
void timer3PWMASet(uint16_t);
void timer3PWMBSet(uint16_t);
void timer3PWMCSet(uint16_t);
void pinPWMOn(uint8_t pin);
  
extern volatile uint8_t * ports[];
extern volatile uint8_t * portsddr[];
extern volatile uint8_t * portspin[];

#ifdef __cplusplus
}
#endif

/*************************************************************
 * C++ Core API Functions
 *************************************************************/

#ifdef __cplusplus
#include "WVector.h"
#include "WString.h"
#include "Print.h"
#include "WCharacter.h"
// WMath.cpp prototypes
float random(float);
float random(float, float);
float map(float, float, float, float, float);
void randomSeed(unsigned int);
unsigned int makeWord(unsigned char, unsigned char);
unsigned int makeWord(unsigned int);
// Shift.cpp
uint16_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t count = 8, uint8_t delayTime = 5);
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint16_t value, uint8_t count = 8, uint8_t delayTime = 5);
// WPulse.cpp prototypes
// Pulse input
unsigned long pulseIn(uint8_t, uint8_t);
unsigned long pulseIn(uint8_t, uint8_t, unsigned long);
// Pulse output 
void pulseOut(uint8_t, uint16_t, uint16_t);	
void pulseOut(uint8_t, uint16_t);	 
uint16_t pulseRunning(uint8_t);	 
void pulseStop(uint8_t);
// WMemory.cpp prototypes
void * operator new(size_t size);
void operator delete(void * ptr); 
void * operator new[](size_t size);
void operator delete[](void * ptr);
int splitString( const String &, int ,  Vector<int> & );
int splitString( const String &, int,  Vector<long> & );
void tone(uint8_t _pin, unsigned int frequency, long duration = 0);
void noTone(uint8_t _pin);
__extension__ typedef int __guard __attribute__((mode (__DI__)));
extern "C" int __cxa_guard_acquire(__guard *);
extern "C" void __cxa_guard_release (__guard *);
extern "C" void __cxa_guard_abort (__guard *);
extern "C" void __cxa_pure_virtual(void);

// definitions for analog pins, not a problem in Wiring since
// pins are properly labeled 
const static uint8_t A0 = 0;
const static uint8_t A1 = 1;
const static uint8_t A2 = 2;
const static uint8_t A3 = 3;
const static uint8_t A4 = 4;
const static uint8_t A5 = 5;
const static uint8_t A6 = 6;
const static uint8_t A7 = 7;

// main program prototypes
void setup(void);
void loop(void);
int main(void) __attribute__ ((noreturn));

#endif

#endif

