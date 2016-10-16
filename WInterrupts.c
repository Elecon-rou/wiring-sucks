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

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#include "WProgram.h"

static volatile voidFuncPtr intFunc[EXTERNAL_NUM_INTERRUPTS];
static volatile voidFuncPtr spiIntFunc;
// volatile static voidFuncPtr twiIntFunc;

/*
modes can be 
LOW Trigger on LOW level
FALLING Trigger on FALLING edge
RISING Trigger on RISING edge
*/


void interruptMode(uint8_t n, uint8_t mode) {
  switch(n) {
    case 0:
      EICRA = (EICRA & ~(_BV(ISC00) | _BV(ISC01))) | (mode << ISC00);
      EIMSK |= _BV(INT0);
      break;
    case 1:
      EICRA = (EICRA & ~(_BV(ISC10) | _BV(ISC11))) | (mode << ISC10);
      EIMSK |= _BV(INT1);
      break;
    case 2:
      EICRA = (EICRA & ~(_BV(ISC20) | _BV(ISC21))) | (mode << ISC20);
      EIMSK |= _BV(INT2);
      break;
    case 3:
      EICRA = (EICRA & ~(_BV(ISC30) | _BV(ISC31))) | (mode << ISC30);
      EIMSK |= _BV(INT3);
      break;
    case 4:
      EICRB = (EICRB & ~(_BV(ISC40) | _BV(ISC41))) | (mode << ISC40);
      EIMSK |= _BV(INT4);
      break;
    case 5:
      EICRB = (EICRB & ~(_BV(ISC50) | _BV(ISC51))) | (mode << ISC50);
      EIMSK |= _BV(INT5);
      break;
    case 6:
      EICRB = (EICRB & ~(_BV(ISC60) | _BV(ISC61))) | (mode << ISC60);
      EIMSK |= _BV(INT6);
      break;
    case 7:
      EICRB = (EICRB & ~(_BV(ISC70) | _BV(ISC71))) | (mode << ISC70);
      EIMSK |= _BV(INT7);
      break; 
    default:
      break;
  }
}


void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), uint8_t mode) {
  if(interruptNum < EXTERNAL_NUM_INTERRUPTS) {
    intFunc[interruptNum] = userFunc;
    interruptMode(interruptNum, mode);
  }
}

void detachInterrupt(uint8_t interruptNum) {
  if(interruptNum < EXTERNAL_NUM_INTERRUPTS) {
    EIMSK &= ~_BV(interruptNum);
    intFunc[interruptNum] = 0;
  }
}

void attachInterruptSPI(void (*userFunc)(void) ) {
  spiIntFunc = userFunc;
}

void detachInterruptSPI(void) {
  spiIntFunc = 0;
}

/*
void attachInterruptTwi(void (*userFunc)(void) ) {
  twiIntFunc = userFunc;
}
*/

ISR(INT0_vect) {
  if(intFunc[EXTERNAL_INT_0])
    intFunc[EXTERNAL_INT_0]();
}

ISR(INT1_vect) {
  if(intFunc[EXTERNAL_INT_1])
    intFunc[EXTERNAL_INT_1]();
}

ISR(INT2_vect) {
  if(intFunc[EXTERNAL_INT_2])
    intFunc[EXTERNAL_INT_2]();
}

ISR(INT3_vect) {
  if(intFunc[EXTERNAL_INT_3])
    intFunc[EXTERNAL_INT_3]();
}

ISR(INT4_vect) {
  if(intFunc[EXTERNAL_INT_4])
    intFunc[EXTERNAL_INT_4]();
}

ISR(INT5_vect) {
  if(intFunc[EXTERNAL_INT_5])
    intFunc[EXTERNAL_INT_5]();
}

ISR(INT6_vect) {
  if(intFunc[EXTERNAL_INT_6])
    intFunc[EXTERNAL_INT_6]();
}

ISR(INT7_vect) {
  if(intFunc[EXTERNAL_INT_7])
    intFunc[EXTERNAL_INT_7]();
}

ISR(SPI_STC_vect) {
  if(spiIntFunc)
    spiIntFunc();
}

/*
ISR(SIG_2WIRE_SERIAL) {
  if(twiIntFunc)
    twiIntFunc();
}
*/

