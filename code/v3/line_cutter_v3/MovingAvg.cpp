// Arduino Moving Average Library
// https://github.com/JChristensen/movingAvg
// Copyright (C) 2018 by Jack Christensen and licensed under
// GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html

#include "MovingAvg.h"

// initialize - allocate the interval array
void MovingAvg::begin() {
  m_readings = new double[m_interval];
}

// add a new reading and return the new moving average
double MovingAvg::reading(double newReading) {
  // add each new data point to the sum until the m_readings array is filled
  if (m_nbrReadings < m_interval) {
    ++m_nbrReadings;
    m_sum = m_sum + newReading;
  } else {
  // once the array is filled, subtract the oldest data point and add the new one
    m_sum = m_sum - m_readings[m_next] + newReading;
  }

  m_readings[m_next] = newReading;
  if (++m_next >= m_interval) m_next = 0;
  return this->getAvg();
}

// just return the current moving average
double MovingAvg::getAvg() {
  return m_sum / m_nbrReadings;
}

// start the moving average over again
void MovingAvg::reset() {
  m_nbrReadings = 0;
  m_sum = 0;
  m_next = 0;
}
