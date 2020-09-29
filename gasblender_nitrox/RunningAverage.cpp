//
//    FILE: RunningAverage.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.3.1
//    DATE: 2015-July-10
// PURPOSE: Arduino library to calculate the running average by means of a circular buffer
//     URL: https://github.com/RobTillaart/RunningAverage
//
// The library stores N individual values in a circular buffer,
// to calculate the running average.
//
// HISTORY:
// 0.1.00 - 2011-01-30 initial version
// 0.1.01 - 2011-02-28 fixed missing destructor in .h
// 0.2.00 - 2012-??-?? Yuval Naveh added trimValue (found on web)
//          http://stromputer.googlecode.com/svn-history/r74/trunk/Arduino/Libraries/RunningAverage/RunningAverage.cpp
// 0.2.01 - 2012-11-21 refactored
// 0.2.02 - 2012-12-30 refactored trimValue -> fillValue
// 0.2.03 - 2013-11-31 getElement
// 0.2.04 - 2014-07-03 added memory protection
// 0.2.05 - 2014-12-16 changed float -> double
// 0.2.06 - 2015-03-07 all size uint8_t
// 0.2.07 - 2015-03-16 added getMin() and getMax() functions (Eric Mulder)
// 0.2.08 - 2015-04-10 refactored getMin() and getMax() implementation
// 0.2.09 - 2015-07-12 refactor const + constructor
// 0.2.10 - 2015-09-01 added getFastAverage() and refactored getAverage()
//                     http://forum.arduino.cc/index.php?topic=50473
// 0.2.11 - 2015-09-04 added getMaxInBuffer() getMinInBuffer() request (Antoon)
// 0.2.12 - 2016-12-01 added GetStandardDeviation() GetStandardError() BufferIsFull()  (V0v1kkk)
// 0.2.13 - 2017-07-26 revert double to float - issue #33;
//                     refactored a bit; marked some TODO's; all function names to camelCase
// 0.2.14 - 2020-01-15 added getValue(n) to retrieve elements in order of addition - see issue #132
// 0.2.15 - 2020-01-17 fix overflow in getValue - see issue #139
// 0.2.16   2020-04-16 improve _sum - see issue #149 (bourkemcrobbo)
// 0.3.0    2020-04-16 main refactor
// 0.3.1    2020-06-19 fix library.json; minor refactor

#include "RunningAverage.h"

RunningAverage::RunningAverage(const uint8_t size)
{
  _size = size;
  _ar = (float*) malloc(_size * sizeof(float));
  if (_ar == NULL) _size = 0;
  clear();
}

RunningAverage::~RunningAverage()
{
  if (_ar != NULL) free(_ar);
}

// resets all counters
void RunningAverage::clear()
{
  _cnt = 0;
  _idx = 0;
  _sum = 0.0;
  _min = NAN;
  _max = NAN;
  for (uint8_t i = _size; i > 0; )
  {
    _ar[--i] = 0.0; // keeps addValue simpler
  }
}

// adds a new value to the data-set
void RunningAverage::addValue(const float value)
{
  if (_ar == NULL) return;  // allocation error

  _sum -= _ar[_idx];
  _ar[_idx] = value;
  _sum += _ar[_idx];
  _idx++;

  if (_idx == _size) _idx = 0;  // faster than %

  // handle min max
  if (_cnt == 0) _min = _max = value;
  else if (value < _min) _min = value;
  else if (value > _max) _max = value;

  // update count as last otherwise if ( _cnt == 0) above will fail
  if (_cnt < _size) _cnt++;
}

// returns the average of the data-set added sofar
float RunningAverage::getAverage()
{
  if (_cnt == 0) return NAN;

  _sum = 0;
  for (uint8_t i = 0; i < _cnt; i++)
  {
    _sum += _ar[i];
  }
  return _sum / _cnt;   // multiplication is faster ==> extra admin
}

// the larger the size of the internal buffer the greater the gain wrt getAverage()
float RunningAverage::getFastAverage() const
{
  if (_cnt == 0) return NAN;
  
  return _sum / _cnt;   // multiplication is faster ==> extra admin
}

// returns the minimum value in the buffer
float RunningAverage::getMinInBuffer() const
{
  if (_cnt == 0) return NAN;

  float min = _ar[0];
  for (uint8_t i = 1; i < _cnt; i++)
  {
    if (_ar[i] < min) min = _ar[i];
  }
  return min;
}

// returns the maximum value in the buffer
float RunningAverage::getMaxInBuffer() const
{
  if (_cnt == 0) return NAN;

  float max = _ar[0];
  for (uint8_t i = 1; i < _cnt; i++)
  {
    if (_ar[i] > max) max = _ar[i];
  }
  return max;
}


// returns the value of an element if exist, NAN otherwise
float RunningAverage::getElement(uint8_t idx) const
{
  if (idx >=_cnt ) return NAN;

  return _ar[idx];
}

// Return standard deviation of running average. If buffer is empty, return NAN.
float RunningAverage::getStandardDeviation() const
{
  if (_cnt <= 1) return NAN;

  float temp = 0;
  float average = getFastAverage();
  for (uint8_t i = 0; i < _cnt; i++)
  {
    temp += pow((_ar[i] - average), 2);
  }
  temp = sqrt(temp/(_cnt - 1));     // TODO possible divide by zero ....

  return temp;
}

// Return standard error of running average. If buffer is empty, return NAN.
float RunningAverage::getStandardError() const //++
{
  float temp = getStandardDeviation();

  if (temp == NAN) return NAN;
  if (_cnt <= 1) return NAN;

  float n;
  if (_cnt >= 30) n = _cnt;
  else n = _cnt - 1;
  temp = temp/sqrt(n);

  return temp;
}

// fill the average with the same value number times. (weight)
// This is maximized to size times. no need to fill the internal buffer over 100%
void RunningAverage::fillValue(const float value, const uint8_t number)
{
  clear();
  uint8_t s = number;
  if (s > _size) s = _size;
  for (uint8_t i = s; i > 0; i--)
  {
    addValue(value);
  }
  // NOTE: the clear iterates over the buffer, 
  //       so merging the clear loop could gain some performance.
}

float RunningAverage::getValue(const uint8_t idx)
{
  if (_cnt == 0) return NAN;
  if (idx >= _cnt) return NAN;  // cannot ask more than is added
  uint16_t pos = idx + _idx;
  if (pos >= _cnt) pos -= _cnt;
  return _ar[pos];
}

// -- END OF FILE --
