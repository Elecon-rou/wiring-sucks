/* -*- mode: jde; c-basic-offset: 2; indent-tabs-mode: nil -*- */

/*
  WMath.cpp: Part of the Wiring project - http://wiring.org.co

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

#include "WProgram.h"


void randomSeed(unsigned int seed)
{
  if (seed != 0) {
    srandom(seed);
  }
}


float random(float howbig)
{
  float value;
  if (howbig == 0){
    return 0;
  }
  value = rand();
  value = value / RAND_MAX;
  return value * howbig;
}

float random(float howsmall, float howbig)
{
  if(howsmall >= howbig){
    return howsmall;
  }
  float diff = howbig - howsmall;
  return random(diff) + howsmall;
}

float map(float value, float istart, float istop, float ostart, float ostop) {
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

unsigned int makeWord(unsigned int w) { return w; }
unsigned int makeWord(unsigned char highByte, unsigned char lowByte) { return (highByte << 8) | lowByte; }


