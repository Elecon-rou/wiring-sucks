/* -*- mode: jde; c-basic-offset: 2; indent-tabs-mode: nil -*- */

/*
  Part of the Wiring project - http://wiring.org.co

  Copyright (c) 2004-10 Hernando Barragan
  Based on timer128.c by Pascal Stang - Copyright (C) 2000-2003

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


// Program ROM constants
// the prescale division values stored in 2^n format
// STOP, CLK, CLK/8, CLK/64, CLK/256, CLK/1024
unsigned const short __attribute__ ((progmem)) timerPrescaleFactor[] = {0,1,8,64,256,1024};
// the prescale division values stored in 2^n format
// STOP, CLK, CLK/8, CLK/32, CLK/64, CLK/128, CLK/256, CLK/1024
unsigned const short __attribute__ ((progmem)) timerRTCPrescaleFactor[] = {0,1,8,32,64,128,256,1024};

//volatile unsigned long timerPauseReg;
//volatile unsigned long timer0Reg0;
//volatile unsigned long timer0Reg1;
//volatile unsigned long timer2Reg0;
//volatile unsigned long timer2Reg1;
volatile uint8_t pwmFlags = 0x00;

static volatile voidFuncPtr timerIntFunc[TIMER_NUM_INTERRUPTS];
const uint8_t pwmpins[6] = {WPWMPIN0, WPWMPIN1, WPWMPIN2, WPWMPIN3,
                              WPWMPIN4, WPWMPIN5 };

void analogWrite(uint8_t pin, uint16_t value) {
  // add support for regular digital I/O pin numbering
  // needed in WiringMini since it doesn't have a separate section
  // por PWM outputs
  if(pin >= 29) {
    if(pin == 31) {
      pin = 3;
    } else if(pin == 30) {
      pin = 4;
    } else if(pin == 29) {
      pin = 5;
    } else if(pin == 37) {
      pin = 0;
    } else if(pin == 36) {
      pin = 1;
    } else if(pin == 35) {
      pin = 2;
    }
  }

  if(pin >= 3) {
    if(bit_is_clear(pwmFlags, 6)) {
      timer1Init();
      sei();
      timer1PWMInit(10);
      pwmFlags |= _BV(6);
    }
    if(bit_is_clear(pwmFlags, pin)) {
        DDRB |= pwmpins[pin];
        pinPWMOn(pin);
        pwmFlags |= _BV(pin);
    }
  } else {
    if(bit_is_clear(pwmFlags, 7)) {
      timer3Init();
      sei();
      timer3PWMInit(10);
      pwmFlags |= _BV(7);
    }
    if(bit_is_clear(pwmFlags, pin)) {
      DDRE |= pwmpins[pin];
      pinPWMOn(pin);
      pwmFlags |= _BV(pin);
    }
  }
  if(pin == 5)
    timer1PWMASet(value);
  if(pin == 4)
    timer1PWMBSet(value);
  if(pin == 3)
    timer1PWMCSet(value);
  if(pin == 2)
    timer3PWMASet(value);
  if(pin == 1)
    timer3PWMBSet(value);
  if(pin == 0)
    timer3PWMCSet(value);
}

void pinPWMOn(uint8_t pin) {
  if(pin == 5) 
    timer1PWMAOn();
  if(pin == 4) 
    timer1PWMBOn();
  if(pin == 3)
    timer1PWMCOn();
  if(pin == 2)
    timer3PWMAOn(); 
  if(pin == 1)
    timer3PWMBOn();
  if(pin == 0)
    timer3PWMCOn();
}  


void timerInit(void)
{
  uint8_t intNum;
  // detach all user functions from interrupts
  for(intNum=0; intNum<TIMER_NUM_INTERRUPTS; intNum++)
    timerDetach(intNum);

  // initialize all timers
  // timer0Init();
  timer1Init();
  timer2Init();
  timer3Init();
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  timer4Init();
  timer5Init();
#endif
  // enable interrupts
  sei();
}


// Now defined in .init3 section for automatic startup
void timer0Init() {
  // initialize timer 0
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  TCCR0A = _BV(WGM00) | _BV(WGM01);
#else
  TCCR0 = _BV(WGM00) | _BV(WGM01);
#endif
  timer0SetPrescaler( TIMER0PRESCALE );	// set prescaler

  TCNT0 = 0;		// reset TCNT0
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  TIMSK0 |= _BV(TOIE0);  // enable TCNT0 overflow interrupt
#else
  TIMSK |= _BV(TOIE0);	// enable TCNT0 overflow interrupt
#endif
  //timer0ClearOverflowCount();	// initialize time registers
  sei();  // enable interruptions in case any class constructor use timing functions
}


void timer1Init(void) {
  timer1SetPrescaler( TIMER1PRESCALE );	 // default to 64, defined in WConstants.h
  TCNT1H = 0;			
  TCNT1L = 0;
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  TIMSK1 |= _BV(TOIE1);  // enable TCNT1 overflow
#else 
  TIMSK |= _BV(TOIE1);  // enable TCNT1 overflow
#endif
}

void timer2Init(void) {
  timer2SetPrescaler( TIMER2PRESCALE );	 // default to 8
  TCNT2 = 0;	
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  TIMSK2 |= _BV(TOIE2);  // enable TCNT2 overflow
#else		
  TIMSK |= _BV(TOIE2);	// enable TCNT2 overflow
#endif
  //timer2ClearOverflowCount();	
}

void timer3Init(void) {
  timer3SetPrescaler( TIMER3PRESCALE );  // default to 64	
  TCNT3H = 0;
  TCNT3L = 0;
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  TIMSK3 |= _BV(TOIE3); // enable TCNT3 overflow
#else
  ETIMSK |= _BV(TOIE3);	// enable TCNT3 overflow
#endif
}


#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
void timer4Init(void) {
  timer4SetPrescaler( TIMER4PRESCALE );  // default to 64	
  TCNT4H = 0;
  TCNT4L = 0;
  TIMSK4 |= _BV(TOIE4); // enable TCNT4 overflow
}


void timer5Init(void) {
  timer5SetPrescaler( TIMER5PRESCALE );  // default to 64	
  TCNT3H = 0;
  TCNT3L = 0;
  TIMSK5 |= _BV(TOIE5); // enable TCNT3 overflow
}
#endif


void timer0SetPrescaler(uint8_t prescale) {
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  TCCR0B = (TCCR0B & ~TIMER_PRESCALE_MASK) | prescale;
#else
  TCCR0 = (TCCR0 & ~TIMER_PRESCALE_MASK) | prescale;
#endif
}


void timer1SetPrescaler(uint8_t prescale) {
  TCCR1B = (TCCR1B & ~TIMER_PRESCALE_MASK) | prescale;
}


void timer2SetPrescaler(uint8_t prescale) {
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  TCCR2B = (TCCR2B & ~TIMER_PRESCALE_MASK) | prescale;
#else
  TCCR2 = (TCCR2 & ~TIMER_PRESCALE_MASK) | prescale;
#endif
}


void timer3SetPrescaler(uint8_t prescale) {
  TCCR3B = (TCCR3B & ~TIMER_PRESCALE_MASK) | prescale;
}


#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
void timer4SetPrescaler(uint8_t prescale) {
  TCCR4B = (TCCR4B & ~TIMER_PRESCALE_MASK) | prescale;
}


void timer5SetPrescaler(uint8_t prescale) {
  TCCR5B = (TCCR5B & ~TIMER_PRESCALE_MASK) | prescale;
}
#endif


void timerAttach(uint8_t interruptNum, void (*userFunc)(void) ) {
  if(interruptNum < TIMER_NUM_INTERRUPTS) {
    timerIntFunc[interruptNum] = userFunc;
  }
}


uint16_t timer0GetPrescaler(void)
{
  // get the current prescaler setting
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  return (pgm_read_word(timerPrescaleFactor+(TCCR0A & TIMER_PRESCALE_MASK)));
#else
  return (pgm_read_word(timerPrescaleFactor+(TCCR0 & TIMER_PRESCALE_MASK)));
#endif
}


uint16_t timer1GetPrescaler(void)
{
  // get the current prescaler setting
  return (pgm_read_word(timerPrescaleFactor+(TCCR1B & TIMER_PRESCALE_MASK)));
}


uint16_t timer2GetPrescaler(void)
{
  // get the current prescaler setting
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
  return (pgm_read_word(timerPrescaleFactor+(TCCR2B & TIMER_PRESCALE_MASK)));
#else
  return (pgm_read_word(timerPrescaleFactor+(TCCR2 & TIMER_PRESCALE_MASK)));
#endif
}


uint16_t timer3GetPrescaler(void)
{
  // get the current prescaler setting
  return (pgm_read_word(timerPrescaleFactor+(TCCR3B & TIMER_PRESCALE_MASK)));
}


#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
uint16_t timer4GetPrescaler(void)
{
	// get the current prescaler setting
	return (pgm_read_word(timerPrescaleFactor+(TCCR4B & TIMER_PRESCALE_MASK)));
}


uint16_t timer5GetPrescaler(void)
{
	// get the current prescaler setting
	return (pgm_read_word(timerPrescaleFactor+(TCCR5B & TIMER_PRESCALE_MASK)));
}
#endif


void timerDetach(uint8_t interruptNum) {
  if(interruptNum < TIMER_NUM_INTERRUPTS) {
    timerIntFunc[interruptNum] = 0;
  }
}


/*
void timerPause(unsigned short pause_ms) {
  uint8_t timerThres;
  uint32_t ticRateHz;
  uint32_t pause;

  // capture current pause timer value
  timerThres = TCNT2;
  // reset pause timer overflow count
  timerPauseReg = 0;
  // calculate delay for [pause_ms] milliseconds
  // prescaler division = 1<<(PRG_RDB(TimerPrescaleFactor + TCCR2))
  ticRateHz = CPU_FREQ/timer2GetPrescaler();
  // precision management
  if( ((ticRateHz < 429497) && (pause_ms <= 10000)) )
    pause = (pause_ms*ticRateHz)/1000;
  else
    pause = pause_ms*(ticRateHz/1000);
  // loop until time expires
  while( ( (timerPauseReg<<8) | TCNT2 ) < (pause+timerThres) );
}


void timer0ClearOverflowCount(void) {
  timer0Reg0 = 0;
  timer0Reg1 = 0;
}


long timer0GetOverflowCount(void) {
  // return the current timer overflow count
  // (this is since the last timer0ClearOverflowCount() command was called)
  return timer0Reg0;
}
*/

// Timer2 will be managed through Timer2 library
/*
void timer2ClearOverflowCount(void) {
  timer2Reg0 = 0;
  timer2Reg1 = 0;
}

long timer2GetOverflowCount(void) {
  return timer2Reg0;
}
*/


void timer1PWMInit(uint8_t bitRes) {
  // configures timer1 for use with PWM output
  // on pins OC1A, OC1B, and OC1C

  // enable Timer1 as 8,9,10bit PWM
  if(bitRes == 9) {
    TCCR1A |= _BV(WGM11);
    TCCR1A &= ~_BV(WGM10);
  }
  else if( bitRes == 10 ) {  // default
    TCCR1A |= _BV(WGM11);
    TCCR1A |= _BV(WGM10);
  }
  else {
    // 8bit mode
    TCCR1A &= ~_BV(WGM11);
    TCCR1A |= _BV(WGM10);
  }
  // set clear-timer-on-compare-match
  //cbi(TCCR1B,CTC1);
  OCR1AH = 0;
  OCR1AL = 0;
  OCR1BH = 0;
  OCR1BL = 0;
  OCR1CH = 0;
  OCR1CL = 0;
}

void timer1PWMOff(void) {
  TCCR1A &= ~_BV(WGM11);
  TCCR1A &= ~_BV(WGM10);
  // clear (disable) clear-timer-on-compare-match
  //cbi(TCCR1B,CTC1);
  // set PWM1A/B/C (OutputCompare action) to none
  timer1PWMAOff();
  timer1PWMBOff();
  timer1PWMCOff();
}

void timer1PWMAOn(void) {
  // turn on channel A OC1A PWM output
  // set OC1A as non-inverted PWM
  TCCR1A |= _BV(COM1A1);
  TCCR1A &= ~_BV(COM1A0);
}

void timer1PWMBOn(void) {
  // turn on channel B OC1B PWM output
  // set OC1B as non-inverted PWM
  TCCR1A |= _BV(COM1B1);
  TCCR1A &= ~_BV(COM1B0);
}

void timer1PWMCOn(void) {
  // turn on channel C OC1C PWM output
  // set OC1C as non-inverted PWM
  TCCR1A |= _BV(COM1C1);
  TCCR1A &= ~_BV(COM1C0);
}

void timer1PWMAOff(void) {
  // turn off channel A (OC1A) PWM output
  // set OC1A (OutputCompare action) to none
  TCCR1A &= ~_BV(COM1A1);
  TCCR1A &= ~_BV(COM1A0);
}

void timer1PWMBOff(void) {
  // turn off channel B (OC1B) PWM output
  // set OC1B (OutputCompare action) to none
  TCCR1A &= ~_BV(COM1B1);
  TCCR1A &= ~_BV(COM1B0);
}

void timer1PWMCOff(void) {
  // turn off channel C (OC1C) PWM output
  // set OC1C (OutputCompare action) to none
  TCCR1A &= ~_BV(COM1C1);
  TCCR1A &= ~_BV(COM1C0);
}

void timer1PWMASet(uint16_t pwmDuty) {
  // set PWM (output compare) duty for channel A
  // this PWM output is generated on OC1A pin
  // NOTE:	pwmDuty should be in the range 0-255 for 8bit PWM
  //			pwmDuty should be in the range 0-511 for 9bit PWM
  //			pwmDuty should be in the range 0-1023 for 10bit PWM
  OCR1AH = pwmDuty>>8;		// set the high 8bits of OCR1A
  OCR1AL = pwmDuty & 0x00ff;	// set the low 8bits of OCR1A
}

void timer1PWMBSet(uint16_t pwmDuty) {
  // set PWM (output compare) duty for channel B
  // this PWM output is generated on OC1B pin
  OCR1BH = pwmDuty>>8;		
  OCR1BL = pwmDuty & 0x00ff;	
}

void timer1PWMCSet(uint16_t pwmDuty) {
  // set PWM (output compare) duty for channel C
  // this PWM output is generated on OC1C pin
  OCR1CH = pwmDuty>>8;		
  OCR1CL = pwmDuty & 0x00ff;	
}


void timer3PWMInit(uint8_t bitRes) {
  // configures timer1 for use with PWM output
  // on pins OC3A, OC3B, and OC3C

  // enable Timer3 as 8,9,10bit PWM
  if(bitRes == 9) {
    // 9bit mode
    TCCR3A |= _BV(WGM31);
    TCCR3A &= ~_BV(WGM30);
  }
  else if( bitRes == 10 ) {
    // 10bit mode
    TCCR3A |= _BV(WGM31);
    TCCR3A |= _BV(WGM30);
  }
  else {
    // default 8bit mode
    TCCR3A &= ~_BV(WGM31);
    TCCR3A |= _BV(WGM30);
  }

  // set clear-timer-on-compare-match
  //cbi(TCCR3B,CTC1);
  // clear output compare value A
  OCR3AH = 0;
  OCR3AL = 0;
  // clear output compare value B
  OCR3BH = 0;
  OCR3BL = 0;
  // clear output compare value C
  OCR3CH = 0;
  OCR3CL = 0;
}

void timer3PWMOff(void) {
  // turn off PWM mode on Timer3
  TCCR3A &= ~_BV(WGM31);
  TCCR3A &= ~_BV(WGM30);
  timer3PWMAOff();
  timer3PWMBOff();
  timer3PWMCOff();
}

void timer3PWMAOn(void) {
  // turn on channel A (OC3A) PWM output
  // set OC3A as non-inverted PWM
  TCCR3A |= _BV(COM3A1);
  TCCR3A &= ~_BV(COM3A0);
}

void timer3PWMBOn(void) {
  // turn on channel B (OC3B) PWM output
  // set OC3B as non-inverted PWM
  TCCR3A |= _BV(COM3B1);
  TCCR3A &= ~_BV(COM3B0);
}

void timer3PWMCOn(void) {
  // turn on channel C (OC3C) PWM output
  // set OC3C as non-inverted PWM
  TCCR3A |= _BV(COM3C1);
  TCCR3A &= ~_BV(COM3C0);
}

void timer3PWMAOff(void) {
  // turn off channel A (OC3A) PWM output
  // set OC3A (OutputCompare action) to none
  TCCR3A &= ~_BV(COM3A1);
  TCCR3A &= ~_BV(COM3A0);
}

void timer3PWMBOff(void) {
  // turn off channel B (OC3B) PWM output
  // set OC3B (OutputCompare action) to none
  TCCR3A &= ~_BV(COM3B1);
  TCCR3A &= ~_BV(COM3B0);
}

void timer3PWMCOff(void) {
  // turn off channel C (OC3C) PWM output
  // set OC3C (OutputCompare action) to none
  TCCR3A &= ~_BV(COM3C1);
  TCCR3A &= ~_BV(COM3C0);
}

void timer3PWMASet(uint16_t pwmDuty) {
  // set PWM (output compare) duty for channel A
  // this PWM output is generated on OC3A pin
  // NOTE: pwmDuty should be in the range 0-255 for 8bit PWM
  //	   pwmDuty should be in the range 0-511 for 9bit PWM
  //	   pwmDuty should be in the range 0-1023 for 10bit PWM
  OCR3AH = pwmDuty>>8;		// set the high 8bits of OCR3A
  OCR3AL = pwmDuty & 0x00ff;	// set the low 8bits of OCR3A
}

void timer3PWMBSet(uint16_t pwmDuty) {
  // set PWM (output compare) duty for channel B
  // this PWM output is generated on OC3B pin
  OCR3BH = pwmDuty>>8;		
  OCR3BL = pwmDuty & 0x00ff;	
}

void timer3PWMCSet(uint16_t pwmDuty) {
  // set PWM (output compare) duty for channel B
  // this PWM output is generated on OC3C pin
  OCR3CH = pwmDuty>>8;		
  OCR3CL = pwmDuty & 0x00ff;
}


// Interrupt handler for tcnt0 overflow interrupt / this is handled in WApplet.c
// for timer0 since it is the main timer for millis() etc.
/*
ISR(SIG_OVERFLOW0) {
  timer0Reg0++;		// increment low-order counter
  if(!timer0Reg0)	// if low-order counter rollover
    timer0Reg1++;	// increment high-order counter	

  if(timerIntFunc[TIMER0OVERFLOW_INT])
    timerIntFunc[TIMER0OVERFLOW_INT]();
}
*/


// Interrupt handler for Timer1 overflow interrupt
ISR(TIMER1_OVF_vect) {
  // if a user function is defined, execute it
  if(timerIntFunc[TIMER1OVERFLOW_INT])
    timerIntFunc[TIMER1OVERFLOW_INT]();
}


// Interrupt handler for Timer2 overflow interrupt
ISR(TIMER2_OVF_vect) {
  if(timerIntFunc[TIMER2OVERFLOW_INT])
    timerIntFunc[TIMER2OVERFLOW_INT]();
}


// Interrupt handler for Timer3 overflow interrupt
ISR(TIMER3_OVF_vect) {
  if(timerIntFunc[TIMER3OVERFLOW_INT])
    timerIntFunc[TIMER3OVERFLOW_INT]();
}

#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
// Interrupt handler for Timer4 overflow interrupt
ISR(TIMER4_OVF_vect) {
	if(timerIntFunc[TIMER4OVERFLOW_INT])
		timerIntFunc[TIMER4OVERFLOW_INT]();
}

// Interrupt handler for Timer5 overflow interrupt
ISR(TIMER5_OVF_vect) {
	if(timerIntFunc[TIMER5OVERFLOW_INT])
		timerIntFunc[TIMER5OVERFLOW_INT]();
}
#endif



#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
ISR(TIMER0_COMPA_vect) {
  if(timerIntFunc[TIMER0OUTCOMPAREA_INT])
    timerIntFunc[TIMER0OUTCOMPAREA_INT]();
}

ISR(TIMER0_COMPB_vect) {
  if(timerIntFunc[TIMER0OUTCOMPAREB_INT])
    timerIntFunc[TIMER0OUTCOMPAREB_INT]();
}
#elif defined (__AVR_ATmega128__)
// Interrupt handler for OutputCompare0 OC0 interrupt
ISR(TIMER0_COMP_vect) {
  if(timerIntFunc[TIMER0OUTCOMPARE_INT])
    timerIntFunc[TIMER0OUTCOMPARE_INT]();
}
#endif


// Interrupt handler for OutputCompare1A OC1A interrupt
ISR(TIMER1_COMPA_vect) {
  if(timerIntFunc[TIMER1OUTCOMPAREA_INT])
    timerIntFunc[TIMER1OUTCOMPAREA_INT]();
}

// Interrupt handler for OutputCompare1B OC1B interrupt
ISR(TIMER1_COMPB_vect) {
  if(timerIntFunc[TIMER1OUTCOMPAREB_INT])
    timerIntFunc[TIMER1OUTCOMPAREB_INT]();
}

// Interrupt handler for OutputCompare1C OC1C interrupt
ISR(TIMER1_COMPC_vect) {
  if(timerIntFunc[TIMER1OUTCOMPAREC_INT])
    timerIntFunc[TIMER1OUTCOMPAREC_INT]();
}

// Interrupt handler for InputCapture1 IC1 interrupt
ISR(TIMER1_CAPT_vect) {
  if(timerIntFunc[TIMER1INPUTCAPTURE_INT])
    timerIntFunc[TIMER1INPUTCAPTURE_INT]();
}

// Interrupt handler for OutputCompare2 OC2 interrupt
#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
ISR(TIMER2_COMPA_vect) {
  if(timerIntFunc[TIMER2OUTCOMPAREA_INT])
    timerIntFunc[TIMER2OUTCOMPAREA_INT]();
}
ISR(TIMER2_COMPB_vect) {
  if(timerIntFunc[TIMER2OUTCOMPAREB_INT])
    timerIntFunc[TIMER2OUTCOMPAREB_INT]();
}

#elif defined (__AVR_ATmega128__)
ISR(TIMER2_COMP_vect) {
  if(timerIntFunc[TIMER2OUTCOMPARE_INT])
    timerIntFunc[TIMER2OUTCOMPARE_INT]();
}
#endif

// Interrupt handler for OutputCompare3A OC3A interrupt
ISR(TIMER3_COMPA_vect) {
  if(timerIntFunc[TIMER3OUTCOMPAREA_INT])
    timerIntFunc[TIMER3OUTCOMPAREA_INT]();
}

// Interrupt handler for OutputCompare3B OC3B interrupt
ISR(TIMER3_COMPB_vect) {
  if(timerIntFunc[TIMER3OUTCOMPAREB_INT])
    timerIntFunc[TIMER3OUTCOMPAREB_INT]();
}

// Interrupt handler for OutputCompare3C OC3C interrupt
ISR(TIMER3_COMPC_vect) {
  if(timerIntFunc[TIMER3OUTCOMPAREC_INT])
    timerIntFunc[TIMER3OUTCOMPAREC_INT]();
}

// Interrupt handler for InputCapture3 IC3 interrupt
ISR(TIMER3_CAPT_vect) {
  if(timerIntFunc[TIMER3INPUTCAPTURE_INT])
    timerIntFunc[TIMER3INPUTCAPTURE_INT]();
}


#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
// Interrupt handler for OutputCompare4A OC4A interrupt
ISR(TIMER4_COMPA_vect) {
	if(timerIntFunc[TIMER4OUTCOMPAREA_INT])
		timerIntFunc[TIMER4OUTCOMPAREA_INT]();
}

// Interrupt handler for OutputCompare4B OC4B interrupt
ISR(TIMER4_COMPB_vect) {
	if(timerIntFunc[TIMER4OUTCOMPAREB_INT])
		timerIntFunc[TIMER4OUTCOMPAREB_INT]();
}

// Interrupt handler for OutputCompare4C OC4C interrupt
ISR(TIMER4_COMPC_vect) {
	if(timerIntFunc[TIMER4OUTCOMPAREC_INT])
		timerIntFunc[TIMER4OUTCOMPAREC_INT]();
}

// Interrupt handler for OutputCompare5A OC5A interrupt
ISR(TIMER5_COMPA_vect) {
	if(timerIntFunc[TIMER5OUTCOMPAREA_INT])
		timerIntFunc[TIMER5OUTCOMPAREA_INT]();
}

// Interrupt handler for OutputCompare5B OC5B interrupt
ISR(TIMER5_COMPB_vect) {
	if(timerIntFunc[TIMER5OUTCOMPAREB_INT])
		timerIntFunc[TIMER5OUTCOMPAREB_INT]();
}

// Interrupt handler for OutputCompare5C OC5C interrupt
ISR(TIMER5_COMPC_vect) {
	if(timerIntFunc[TIMER5OUTCOMPAREC_INT])
		timerIntFunc[TIMER5OUTCOMPAREC_INT]();
}

// There is no InputCapture4 IC4 and InputCapture5 IC5 interrupt on atmega1281/2561
#endif
