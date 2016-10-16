/*
  WConstants.h - Main definitions file for Wiring 
  Part of the Wiring project - http://wiring.org.co

  Copyright (c) 2004-2010 Hernando Barragan

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef WConstants_h
#define WConstants_h

// Wiring API version for libraries
// this is also defined at compile-time
#ifndef WIRING
#define WIRING 27 
#endif

// passed in at compile-time
#ifndef F_CPU
#define F_CPU 16000000L
#endif

// passed in at compile-time
#ifndef CPU_FREQ
#define CPU_FREQ 16000000L
#endif

#define LOW  0x0
#define HIGH 0x1

#define INPUT  0x0
#define OUTPUT 0x1

#define CHANGE 1
#define FALLING 2
#define RISING 3

#define LSBFIRST 0x0
#define MSBFIRST 0x1

#define true  0x1
#define false 0x0
#define null  NULL

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0

#define PI         (3.1415927)
#define TWO_PI     (6.2831854)
#define HALF_PI    (1.57079)
#define EPSILON    (0.0001)
#define DEG_TO_RAD (0.01745329)
#define RAD_TO_DEG (57.2957786)

#define int(x)     ((int)(x))
#define char(x)    ((char)(x))
#define long(x)    ((long)(x))
#define byte(x)    ((uint8_t)(x))
#define float(x)   ((float)(x))
#define boolean(x) ((uint8_t)((x)==0?0:1))

#define interrupts() sei()
#define noInterrupts() cli()

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif

#define sq(x)        ((x)*(x))
#define abs(x)       ((x)>0?(x):-(x))
#define min(a,b)     ((a)<(b)?(a):(b))
#define max(a,b)     ((a)>(b)?(a):(b))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define bit(x) (1<<(x))
#define setBits(x, y) ((x)|=(y))
#define clearBits(x, y) ((x)&=(~(y)))
#define setBit(x, y) setBits((x), (bit((y))))
#define clearBit(x, y) clearBits((x), (bit((y))))

#define bitsSet(x,y) (((x) & (y)) == (y))
#define bitsClear(x,y) (((x) & (y)) == 0)

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define lowByte(x) ((x) & 0x00ff)
#define highByte(x) ((x)>>8)


#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

/*
 * Michael Margolis
 * this version is based on Paul Stoffregen's Nov13 2009 submission to the
 * Arduino developers list
 */
#define digitalPinToPortReg(P) \
        (((P) >= 0 && (P) <= 7)   ? &PORTD : \
		(((P) >= 8 && (P) <= 15)  ? &PORTC : \
 		(((P) >= 16 && (P) <= 23) ? &PORTA : \
		(((P) >= 24 && (P) <= 31) ? &PORTB : \
		(((P) >= 32 && (P) <= 39) ? &PORTE : \
		(((P) >= 40 && (P) <= 47) ? &PORTF : &PORTG))))))
#define digitalPinToBit(P)  ((P) & 7)

// macros added for compatibility
#define digitalPinToBitMask(P)  ((P) & 7)
#define digitalPinToPort(P) \
        (((P) >= 0 && (P) <= 7)   ? 0 : \
                (((P) >= 8 && (P) <= 15)  ? 1 : \
                (((P) >= 16 && (P) <= 23) ? 2 : \
                (((P) >= 24 && (P) <= 31) ? 3 : \
                (((P) >= 32 && (P) <= 39) ? 4 : \
                (((P) >= 40 && (P) <= 47) ? 5 : 6))))))
#define portOutputRegister(P) \
        (((P) == 0 )   ? &PORTD : \
                (((P) == 1 )  ? &PORTC : \
                (((P) == 2 ) ? &PORTA : \
                (((P) == 3 ) ? &PORTB : \
                (((P) == 4 ) ? &PORTE : \
                (((P) == 5 ) ? &PORTF : &PORTG))))))
#define portInputRegister(P) \
        (((P) == 0 )   ? &PIND : \
                (((P) == 1 )  ? &PINC : \
                (((P) == 2 ) ? &PINA : \
                (((P) == 3 ) ? &PINB : \
                (((P) == 4 ) ? &PINE : \
                (((P) == 5 ) ? &PINF : &PING))))))
#define portModeRegister(P) \
        (((P) == 0 )   ? &DDRD : \
                (((P) == 1 )  ? &DDRC : \
                (((P) == 2 ) ? &DDRA : \
                (((P) == 3 ) ? &DDRB : \
                (((P) == 4 ) ? &DDRE : \
                (((P) == 5 ) ? &DDRF : &DDRG))))))


#define WPIN0 (1<<0)
#define WPIN1 (1<<1)
#define WPIN2 (1<<2)
#define WPIN3 (1<<3)
#define WPIN4 (1<<4)
#define WPIN5 (1<<5)
#define WPIN6 (1<<6)
#define WPIN7 (1<<7)

#define WPWMPIN5 (1<<5) // PINB5
#define WPWMPIN4 (1<<6) // PINB6
#define WPWMPIN3 (1<<7) // PINB7
#define WPWMPIN2 (1<<3) // PINE3
#define WPWMPIN1 (1<<4) // PINE4
#define WPWMPIN0 (1<<5) // PINE5

#define WPORTA PORTA
#define WPORTB PORTB
#define WPORTC PORTC
#define WPORTD PORTD
#define WPORTE PORTE
#define WPORTF PORTF
#define WPORTG PORTG

#define WPINA PINA
#define WPINB PINB
#define WPINC PINC
#define WPIND PIND
#define WPINE PINE
#define WPINF PINF
#define WPING PING

#define WDDRA DDRA
#define WDDRB DDRB
#define WDDRC DDRC
#define WDDRD DDRD
#define WDDRE DDRE
#define WDDRF DDRF
#define WDDRG DDRG


/*
 * Internal constants for timer management
 */
#define TIMER0OVERFLOW_INT      0
#define TIMER0OUTCOMPARE_INT    1
#define TIMER0OUTCOMPAREA_INT    1
#define TIMER0OUTCOMPAREB_INT    2
#define TIMER1OVERFLOW_INT      3
#define TIMER1OUTCOMPAREA_INT   4
#define TIMER1OUTCOMPAREB_INT   5
#define TIMER1OUTCOMPAREC_INT   6
#define TIMER1INPUTCAPTURE_INT  7
#define TIMER2OVERFLOW_INT      8
#define TIMER2OUTCOMPARE_INT    9
#define TIMER2OUTCOMPAREA_INT   9
#define TIMER2OUTCOMPAREB_INT   10
#define TIMER3OVERFLOW_INT      11 
#define TIMER3OUTCOMPAREA_INT   12
#define TIMER3OUTCOMPAREB_INT   13
#define TIMER3OUTCOMPAREC_INT   14
#define TIMER3INPUTCAPTURE_INT  15

#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
#define TIMER4OVERFLOW_INT      16 
#define TIMER4OUTCOMPAREA_INT   17
#define TIMER4OUTCOMPAREB_INT   18
#define TIMER4OUTCOMPAREC_INT   19
#define TIMER5OVERFLOW_INT      20 
#define TIMER5OUTCOMPAREA_INT   21
#define TIMER5OUTCOMPAREB_INT   22
#define TIMER5OUTCOMPAREC_INT   23
#endif

#if defined (__AVR_ATmega1281__)||(__AVR_ATmega2561__)
#define TIMER_NUM_INTERRUPTS    24
#else
#define TIMER_NUM_INTERRUPTS    16
#endif


/*
 * Timer prescaler constants for normal timers
 * on atmega128 Timer2, Timer1, Timer3
 * on atmega1281 and atmega2561 Timer1, Timer2, Timer3, Timer4 and Timer5
 */
#define TIMER_CLK_STOP      0x00
#define TIMER_CLK_DIV1      0x01
#define TIMER_CLK_DIV8      0x02
#define TIMER_CLK_DIV64     0x03
#define TIMER_CLK_DIV256    0x04
#define TIMER_CLK_DIV1024   0x05
#define TIMER_CLK_T_FALL    0x06
#define TIMER_CLK_T_RISE    0x07
#define TIMER_PRESCALE_MASK 0x07

/*
 * Timer prescaler constants for RTC type timers 
 * on atmega128 Timer0
 * on atmega1281 and atmega2561 Timer2
 */
#define TIMERRTC_CLK_STOP      0x00
#define TIMERRTC_CLK_DIV1      0x01
#define TIMERRTC_CLK_DIV8      0x02
#define TIMERRTC_CLK_DIV32     0x03
#define TIMERRTC_CLK_DIV64     0x04
#define TIMERRTC_CLK_DIV128    0x05
#define TIMERRTC_CLK_DIV256    0x06
#define TIMERRTC_CLK_DIV1024   0x07
#define TIMERRTC_PRESCALE_MASK 0x07


/*
 * Predefined prescalers for timers Timer0, Timer1, Timer2, Timer3, Timer4 and Timer5
 */
#if defined(__AVR_ATmega1281__)||(__AVR_ATmega2561__)
#define TIMER0PRESCALE TIMER_CLK_DIV64
#else
#define TIMER0PRESCALE TIMERRTC_CLK_DIV64
#endif

#define TIMER1PRESCALE TIMER_CLK_DIV64

#if defined(__AVR_ATmega1281__)||(__AVR_ATmega2561__)
#define TIMER2PRESCALE TIMERRTC_CLK_DIV8
#else
#define TIMER2PRESCALE TIMER_CLK_DIV8
#endif

#define TIMER3PRESCALE TIMER_CLK_DIV64

#if defined(__AVR_ATmega1281__)||(__AVR_ATmega2561__)
#define TIMER4PRESCALE TIMER_CLK_DIV64
#define TIMER5PRESCALE TIMER_CLK_DIV64
#endif


/*
 * External interrupts constants
 */
#define EXTERNAL_INT_0 0
#define EXTERNAL_INT_1 1
#define EXTERNAL_INT_2 2
#define EXTERNAL_INT_3 3
#define EXTERNAL_INT_4 4
#define EXTERNAL_INT_5 5
#define EXTERNAL_INT_6 6
#define EXTERNAL_INT_7 7

#define EXTERNAL_NUM_INTERRUPTS 8

/*
 * Power management constants
 */
#define POWER_ADC 0
#define POWER_SPI 1
#define POWER_WIRE 2
#define POWER_TIMER0 3
#define POWER_TIMER1 4
#define POWER_TIMER2 5
#define POWER_TIMER3 6
#define POWER_TIMER4 7
#define POWER_TIMER5 8
#define POWER_SERIAL0 9
#define POWER_SERIAL1 10
#define POWER_SERIAL2 11
#define POWER_SERIAL3 12
#define POWER_ALL 13


/*
 * typedefs for extra types
 */
typedef uint8_t byte;
typedef uint8_t boolean;
typedef void (*voidFuncPtr)(void);

#endif

