/*
  WPrint.h - Print library for Wiring & Arduino
  Class code factored out from Serial and LiquidCrystal libraries
  Copyright (c) 2007-08 Hernando Barragan.  All right reserved.

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
*/

#ifndef Print_h
#define Print_h

#include "WProgram.h"
#include "WConstants.h"
#include <string.h>
#include <ctype.h>
#include <inttypes.h>

class Print
{
  public:
    virtual void write(uint8_t);
    void print(char);
    void print(const char[]);
    void print(const String &);
    void print(uint8_t);
    void print(int);
    void print(double);
    void print(double, int);
    void print(unsigned int);
    void print(long);
    void print(unsigned long);
    void print(int, int);
    void print(unsigned int, int);
    void print(long, int);
    void print(unsigned long, int);
    void println(void);
    void println(char);
    void println(const char[]);
    void println(const String &);
    void println(uint8_t);
    void println(int);
    void println(double);
    void println(double, int);
    void println(unsigned int);
    void println(long);
    void println(unsigned long);
    void println(int, int);
    void println(unsigned int, int);
    void println(long, int);
    void println(unsigned long, int);
  private:
    void printNumber(unsigned long, uint8_t);
    void printFloat(double, uint8_t);
};

#endif
