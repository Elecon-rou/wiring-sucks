/* Tone.cpp

  A Tone Generator Library

  Written by Brett Hagman

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

Version Modified By Date     Comments
------- ----------- -------- --------
0001    B Hagman    09/08/02 Initial coding
0002    B Hagman    09/08/18 Multiple pins
0003    B Hagman    09/08/18 Moved initialization from constructor to begin()
0004    B Hagman    09/09/26 Fixed problems with ATmega8
0005    B Hagman    09/11/23 Scanned prescalars for best fit on 8 bit timers
                    09/11/25 Changed pin toggle method to XOR
                    09/11/25 Fixed timer0 from being excluded
0006    D Mellis    09/12/29 Replaced objects with functions
0007    D Barragan  10/01/26 Support for Wiring
0008    B Hagman    10/09/15 Fixed ck/32 prescalar divisor

*************************************************/
#include <avr/pgmspace.h>
#include "WProgram.h"

//  > 0 - duration specified
//  = 0 - stopped
//  < 0 - infinitely (until stop() method called, or new play() called)


volatile long timer1_toggle_count;
volatile uint8_t *timer1_pin_port;
volatile uint8_t timer1_pin_mask;
volatile long timer2_toggle_count;
volatile uint8_t *timer2_pin_port;
volatile uint8_t timer2_pin_mask;
volatile long timer3_toggle_count;
volatile uint8_t *timer3_pin_port;
volatile uint8_t timer3_pin_mask;
void ToneTimer1Service();
void ToneTimer2Service();
void ToneTimer3Service();
#if defined(__AVR_ATmega1281__) || (__AVR_ATmega2561__)
volatile long timer4_toggle_count;
volatile uint8_t *timer4_pin_port;
volatile uint8_t timer4_pin_mask;
volatile long timer5_toggle_count;
volatile uint8_t *timer5_pin_port;
volatile uint8_t timer5_pin_mask;
void ToneTimer4Service();
void ToneTimer5Service();
#endif


#if defined(__AVR_ATmega128__)

#define AVAILABLE_TONE_PINS 1

const uint8_t PROGMEM tone_pin_to_timer_PGM[] = { 2 /*, 1 */ };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255 /*, 255 */ };

#elif defined(__AVR_ATmega1281__) || (__AVR_ATmega2561__)

#define AVAILABLE_TONE_PINS 1

const uint8_t PROGMEM tone_pin_to_timer_PGM[] = { 2 /*, 3, 4, 5, 1, 0 */ };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255 /*, 255, 255, 255, 255, 255 */ };

#else

#define AVAILABLE_TONE_PINS 1

// Leave timer 0 to last.
const uint8_t PROGMEM tone_pin_to_timer_PGM[] = { 2 /*, 1, 0 */ };
static uint8_t tone_pins[AVAILABLE_TONE_PINS] = { 255 /*, 255, 255 */ };

#endif



static int8_t toneBegin(uint8_t _pin)
{
  int8_t _timer = -1;

  // if we're already using the pin, the timer should be configured.  
  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
    if (tone_pins[i] == _pin) {
      return pgm_read_byte(tone_pin_to_timer_PGM + i);
    }
  }
  
  // search for an unused timer.
  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
    if (tone_pins[i] == 255) {
      tone_pins[i] = _pin;
      _timer = pgm_read_byte(tone_pin_to_timer_PGM + i);
      break;
    }
  }
  
  if (_timer != -1)
  {
    // Set timer specific stuff
    // All timers in CTC mode
    // 8 bit timers will require changing prescalar values,
    // whereas 16 bit timers are set to either ck/1 or ck/64 prescalar
    switch (_timer)
    {
      case 1:
        // 16 bit timer
        TCCR1A = 0;
        TCCR1B = 0;
        TCCR1B |= _BV(WGM12) | _BV(CS10);
        //bitWrite(TCCR1B, WGM12, 1);
        //bitWrite(TCCR1B, CS10, 1);
        timer1_pin_port = digitalPinToPortReg(_pin); //portOutputRegister(digitalPinToPort(_pin));
        timer1_pin_mask = digitalPinToBit(_pin); //digitalPinToBitMask(_pin);
        break;
      case 2:
        // 8 bit timer
#if defined(__AVR_ATmega128__)
        TCCR2 = 0;
        TCCR2  |= _BV(WGM21) || _BV(CS20);
#else
        TCCR2A = 0;
        TCCR2B = 0;
        TCCR2A |= _BV(WGM21);
        TCCR2B |= _BV(CS20);
        //bitWrite(TCCR2A, WGM21, 1);
        //bitWrite(TCCR2B, CS20, 1);
#endif
        timer2_pin_port = digitalPinToPortReg(_pin); //portOutputRegister(digitalPinToPort(_pin));
        timer2_pin_mask = digitalPinToBit(_pin); //digitalPinToBitMask(_pin);
        break;
      case 3:
        // 16 bit timer
        TCCR3A = 0;
        TCCR3B = 0;
        TCCR3B |= _BV(WGM32) | _BV(CS30);
        //bitWrite(TCCR3B, WGM32, 1);
        //bitWrite(TCCR3B, CS30, 1);
        timer3_pin_port = digitalPinToPortReg(_pin); //portOutputRegister(digitalPinToPort(_pin));
        timer3_pin_mask = digitalPinToBit(_pin); //digitalPinToBitMask(_pin);
        break;
#if defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)
      case 4:
        // 16 bit timer
        TCCR4A = 0;
        TCCR4B = 0;
        TCCR4B |= _BV(WGM42) | _BV(CS40);
        //bitWrite(TCCR4B, WGM42, 1);
        //bitWrite(TCCR4B, CS40, 1);
        timer4_pin_port = digitalPinToPortReg(_pin); //portOutputRegister(digitalPinToPort(_pin));
        timer4_pin_mask = digitalPinToBit(_pin); //digitalPinToBitMask(_pin);
        break;
      case 5:
        // 16 bit timer
        TCCR5A = 0;
        TCCR5B = 0;
        TCCR5B |= _BV(WGM52) | _BV(CS50);
        //bitWrite(TCCR5B, WGM52, 1);
        //bitWrite(TCCR5B, CS50, 1);
        timer5_pin_port = digitalPinToPortReg(_pin); //portOutputRegister(digitalPinToPort(_pin));
        timer5_pin_mask = digitalPinToBit(_pin); //digitalPinToBitMask(_pin);
        break;
#endif
      default:
        break;
    }
  }

  return _timer;
}



// frequency (in hertz) and duration (in milliseconds).

void tone(uint8_t _pin, unsigned int frequency, long duration)
{
  uint8_t prescalar = TIMER_CLK_DIV1;
  long toggle_count = 0;
  uint32_t ocr = 0;
  int8_t _timer;

  _timer = toneBegin(_pin);

  if (_timer >= 1)
  {
    // Set the pinMode as OUTPUT
    pinMode(_pin, OUTPUT);
    
    // if we are using an 8 bit timer, scan through prescalars to find the best fit
    if (_timer == 2)
    {
      ocr = F_CPU / frequency / 2 - 1;
      prescalar = TIMER_CLK_DIV1;
      if (ocr > 255)
      {
        ocr = F_CPU / frequency / 2 / 8 - 1;
        prescalar = TIMER_CLK_DIV8;

#if !defined(__AVR_ATmega128__)
        if (ocr > 255)
        {
          ocr = F_CPU / frequency / 2 / 32 - 1;
          prescalar = TIMERRTC_CLK_DIV32;
        }
#endif

        if (ocr > 255)
        {
          ocr = F_CPU / frequency / 2 / 64 - 1;
#if defined(__AVR_ATmega128__)
          prescalar = TIMER_CLK_DIV64;
#else
          prescalar = TIMERRTC_CLK_DIV64;
#endif

          if (_timer == 2 && ocr > 255)
          {
            ocr = F_CPU / frequency / 2 / 128 - 1;
#if defined(__AVR_ATmega128__)
            prescalar = TIMER_CLK_DIV1024;
#else
            prescalar = TIMERRTC_CLK_DIV1024;
#endif
          }

          if (ocr > 255)
          {
            ocr = F_CPU / frequency / 2 / 256 - 1;
#if defined(__AVR_ATmega128__)
            prescalar = TIMER_CLK_DIV256; 
#else
            prescalar = TIMERRTC_CLK_DIV256;
#endif
            if (ocr > 255)
            {
              // can't do any better than /1024
              ocr = F_CPU / frequency / 2 / 1024 - 1;
#if defined(__AVR_ATmega128__)
              prescalar = TIMER_CLK_DIV1024; 
#else
              prescalar = TIMERRTC_CLK_DIV1024;
#endif
            }
          }
        }
      }

      timer2SetPrescaler(prescalar);
    }
    else
    {
      // two choices for the 16 bit timers: ck/1 or ck/64
      ocr = F_CPU / frequency / 2 - 1;
      prescalar = TIMER_CLK_DIV1;
      if (ocr > 0xffff)
      {
        ocr = F_CPU / frequency / 2 / 64 - 1;
        prescalar = TIMER_CLK_DIV64;
      }

      if (_timer == 1)
        timer1SetPrescaler(prescalar);
      else if (_timer == 3)
        timer3SetPrescaler(prescalar);
#if defined(__AVR_ATmega1281__)||(__AVR_ATmega2561__)
      else if (_timer == 4)
        timer4SetPrescaler(prescalar);
      else if (_timer == 5)
        timer5SetPrescaler(prescalar);
#endif

    }
    

    // Calculate the toggle count
    if (duration > 0)
      toggle_count = 2 * frequency * duration / 1000;
    else
      toggle_count = -1;

    // Set the OCR for the given timer,
    // set the toggle count,
    // then turn on the interrupts
    switch (_timer)
    {
      case 1:
        OCR1A = ocr;
        timer1_toggle_count = toggle_count;
        timerAttach(TIMER1OUTCOMPAREA_INT, ToneTimer1Service);
#if defined(__AVR_ATmega128__)
        TIMSK |= _BV(OCIE1A);
#else
        TIMSK1 |= _BV(OCIE1A);
#endif
        //bitWrite(TIMSK1, OCIE1A, 1);
        break;
      case 2:
#if defined(__AVR_ATmega128__)
        OCR2 = ocr;
#else
        OCR2A = ocr;
#endif
        timer2_toggle_count = toggle_count;
#if defined(__AVR_ATmega128__)
        timerAttach(TIMER2OUTCOMPARE_INT, ToneTimer2Service);
        TIMSK |= _BV(OCIE2);
#else
        timerAttach(TIMER2OUTCOMPAREA_INT, ToneTimer2Service);
        TIMSK2 |= _BV(OCIE2A);
#endif

        //bitWrite(TIMSK2, OCIE2A, 1);
        break;
      case 3:
        OCR3A = ocr;
        timer3_toggle_count = toggle_count;
        timerAttach(TIMER3OUTCOMPAREA_INT, ToneTimer2Service);
#if defined(__AVR_ATmega128__)
        ETIMSK |= _BV(OCIE3A);
#else
        TIMSK3 |= _BV(OCIE3A);
#endif
        //bitWrite(TIMSK3, OCIE3A, 1);
        break;
#if defined(__AVR_ATmega1281__)||(__AVR_ATmega2561__)
      case 4:
        OCR4A = ocr;
        timer4_toggle_count = toggle_count;
        timerAttach(TIMER4OUTCOMPAREA_INT, ToneTimer4Service);
        TIMSK4 |= _BV(OCIE4A);
        //bitWrite(TIMSK4, OCIE4A, 1);
        break;
      case 5:
        OCR5A = ocr;
        timer5_toggle_count = toggle_count;
        timerAttach(TIMER5OUTCOMPAREA_INT, ToneTimer5Service);
        TIMSK5 |= _BV(OCIE5A);
        //bitWrite(TIMSK5, OCIE5A, 1);
        break;
#endif

    }
  }
}


void noTone(uint8_t _pin)
{
  int8_t _timer = -1;
  
  for (int i = 0; i < AVAILABLE_TONE_PINS; i++) {
    if (tone_pins[i] == _pin) {
      _timer = pgm_read_byte(tone_pin_to_timer_PGM + i);
      tone_pins[i] = 255;
    }
  }
  
  switch (_timer)
  {
#if defined(__AVR_ATmega128__)
    case 1:
      TIMSK &= ~_BV(OCIE1A);
      timerDetach(TIMER1OUTCOMPAREA_INT);
      //bitWrite(TIMSK1, OCIE1A, 0);
      break;
    case 2:
      TIMSK &= ~_BV(OCIE2);
      timerDetach(TIMER2OUTCOMPARE_INT);
      //bitWrite(TIMSK2, OCIE2A, 0);
      break;
    case 3:
      ETIMSK &= ~_BV(OCIE3A);
      timerDetach(TIMER3OUTCOMPAREA_INT);
      break;
#else
    case 1:
      TIMSK1 = 0;
      timerDetach(TIMER1OUTCOMPAREA_INT);
      break;
    case 2:
      TIMSK2 = 0;
      timerDetach(TIMER2OUTCOMPAREA_INT);
      break;
    case 3:
      TIMSK3 = 0;
      timerDetach(TIMER3OUTCOMPAREA_INT);
      break;
    case 4:
      TIMSK4 = 0;
      timerDetach(TIMER4OUTCOMPAREA_INT);
      break;
    case 5:
      TIMSK5 = 0;
      timerDetach(TIMER5OUTCOMPAREA_INT);
      break;
#endif
    default:
      break;
  }

  digitalWrite(_pin, LOW);
}


void ToneTimer1Service() //ISR(TIMER1_COMPA_vect)
{
  if (timer1_toggle_count != 0)
  {
    // toggle the pin
    *timer1_pin_port ^= _BV(timer1_pin_mask);

    if (timer1_toggle_count > 0)
      timer1_toggle_count--;
  }
  else
  {
#if defined(__AVR_ATmega128__)
    TIMSK &= ~_BV(OCIE1A);
#else 
    TIMSK1 = 0;   // disable the interrupt
#endif
    *timer1_pin_port &= ~_BV(timer1_pin_mask);  // keep pin low after stop
  }
}


void ToneTimer2Service() //ISR(TIMER2_COMPA_vect)
{

  if (timer2_toggle_count != 0)
  {
    // toggle the pin
    *timer2_pin_port ^= _BV(timer2_pin_mask);

    if (timer2_toggle_count > 0)
      timer2_toggle_count--;
  }
  else
  {
#if defined(__AVR_ATmega128__)
    TIMSK &= ~_BV(OCIE2);
#else
    TIMSK2 = 0;   // disable the interrupt
#endif
    *timer2_pin_port &= ~_BV(timer2_pin_mask);  // keep pin low after stop
  }
}



void ToneTimer3Service() //ISR(TIMER3_COMPA_vect)
{
  if (timer3_toggle_count != 0)
  {
    // toggle the pin
    *timer3_pin_port ^= _BV(timer3_pin_mask);

    if (timer3_toggle_count > 0)
      timer3_toggle_count--;
  }
  else
  {
#if defined(__AVR_ATmega128__)
    ETIMSK &= ~_BV(OCIE3A);
#else
    TIMSK3 = 0;   // disable the interrupt
#endif
    *timer3_pin_port &= ~_BV(timer3_pin_mask);  // keep pin low after stop
  }
}

#if defined(__AVR_ATmega1281__)||(__AVR_ATmega2561__)
void ToneTimer4Service() //ISR(TIMER4_COMPA_vect)
{
  if (timer4_toggle_count != 0)
  {
    // toggle the pin
    *timer4_pin_port ^= _BV(timer4_pin_mask);

    if (timer4_toggle_count > 0)
      timer4_toggle_count--;
  }
  else
  {
    TIMSK4 = 0;   // disable the interrupt
    *timer4_pin_port &= ~_BV(timer4_pin_mask);  // keep pin low after stop
  }
}

void ToneTimer5Service() //ISR(TIMER5_COMPA_vect)
{
  if (timer5_toggle_count != 0)
  {
    // toggle the pin
    *timer5_pin_port ^= _BV(timer5_pin_mask);

    if (timer5_toggle_count > 0)
      timer5_toggle_count--;
  }
  else
  {
    TIMSK5 = 0;   // disable the interrupt
    *timer5_pin_port &= ~_BV(timer5_pin_mask);  // keep pin low after stop
  }
}

#endif
