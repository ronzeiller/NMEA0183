/*
  AISUnitsConverts.h

  Often needed converts vor several tasks

  Copyright (c) 2017 Thomas Sarlandie thomas@sarlandie.net, (c) Ronnie Zeiller 2019

  The MIT License

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#pragma once

#include <math.h>

#define aKnotToMs(x) (x) * 1852 / 3600
#define aStatuteMphToMs(x) (x) * 1609.344 / 3600
#define aKmphToMs(x) (x) * 1000 / 3600

#define aDegToRad(x)   (x) * 2 * M_PI / 360
#define aRadToDeg(x)   (x) * 360.0 / (2 * M_PI)
#define aMsToKnot(x)   (x) * 3600.0 / 1852.0
#define aMsToKmh(x)    (x) * 3.60
#define aRadSecondsToDegreeMinutes(x)   (x) * 60 * 360.0 / (2 * M_PI)
#define aKelvinToC(x)   (x) - 273.15

inline double aPascalToBar(double x) {
  return x / 1e5;
}

/**
 * Normalizes any angle in radians to the range [0,2*M_PI)
 */
inline double aNormalizeDirection(double x) {
  x = fmod(x, 2 * M_PI);
  if (x < 0) {
    x += 2 * M_PI;
  }
  return x;
}

/**
 * Normalizes any angle in radians to the range [-M_PI,M_PI)
 */
inline double aNormalizeAngle(double x) {
  x = fmod(x + M_PI, 2 * M_PI);
  if (x < 0) {
    x += 2 * M_PI;
  }
  return x - M_PI;
}

const double aDoubleNAN = -1e9;

/**
 * Rounds double to int
 */
inline int aRoundToInt(double x) {
  return x >= 0
      ? (int) floor(x + 0.5)
      : (int) ceil(x - 0.5);
}
