/* -*- mode: jde; c-basic-offset: 2; indent-tabs-mode: nil -*- */

/*
  Part of the Wiring project - http://wiring.org.co

  Copyright (c) 2009 Hernando Barragan

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

#include "WProgram.h"

void * operator new(size_t size)
{
  return malloc(size);
}

void operator delete(void * ptr)
{
  if (ptr)
    free(ptr);
}

void * operator new[](size_t size)
{
    return malloc(size);
}

void operator delete[](void * ptr)
{
  if(ptr)
    free(ptr);
}


int __cxa_guard_acquire(__guard *g) {return !*(char *)(g);};
void __cxa_guard_release (__guard *g) {*(char *)g = 1;};
void __cxa_guard_abort (__guard *) {};
void __cxa_pure_virtual(void) {};


int splitString( const String &what, int delim,  Vector<long> &splits )
{
  what.trim();
  splits.removeAllElements();
  const char *chars = what._buffer;
  int splitCount = 0; //1;
  for (int i = 0; i < what.length(); i++) {
    if (chars[i] == delim) splitCount++;
  }
  if (splitCount == 0) {
    splits.addElement(atol(what._buffer));
    return 1;
  }

  int pieceCount = splitCount + 1;

  int splitIndex = 0;
  int startIndex = 0;
  for (int i = 0; i < what.length(); i++) {
    if (chars[i] == delim) {
      splits.addElement(atol(what.substring(startIndex, i)._buffer));
      splitIndex++;
      startIndex = i + 1;
    }
  }
  splits.addElement(atol(what.substring(startIndex, what.length())._buffer));

  return pieceCount;
}

int splitString( const String &what, int delim,  Vector<int> &splits )
{
  what.trim();
  splits.removeAllElements();
  const char *chars = what._buffer;
  int splitCount = 0; //1;
  for (int i = 0; i < what.length(); i++) {
    if (chars[i] == delim) splitCount++;
  }
  if (splitCount == 0) {
    splits.addElement(atoi(what._buffer));
    return(1);
  }

  int pieceCount = splitCount + 1;

  int splitIndex = 0;
  int startIndex = 0;
  for (int i = 0; i < what.length(); i++) {
    if (chars[i] == delim) {
      splits.addElement(atoi(what.substring(startIndex, i)._buffer));
      splitIndex++;
      startIndex = i + 1;
    }
  }
  splits.addElement(atoi(what.substring(startIndex, what.length())._buffer));

  return pieceCount;
}

 
