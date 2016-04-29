/*
 * Author: Jon Trulson <jtrulson@ics.com>
 * Copyright (c) 2015 Intel Corporation.
 *
 * Adapted from the seeedstudio example
 * https://github.com/Seeed-Studio/Accelerometer_ADXL377
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <iostream>
#include <string>
#include <stdexcept>
#include <chrono>
#include <thread>
#include "mraa/adxl377.h"

using namespace std;
using namespace upm;

ADXL377::ADXL377(upm::ADS1015 *adc, float aref, float vpg) {
	m_aref = aref;
	m_zeroX = aref / 2;
	m_zeroY = aref / 2;
	m_zeroZ = aref / 2;
	m_vpg = vpg;
	m_adc = adc;

	// because we are using mode 1, the FS is +/- 4.096, but we are single
	// ended, so we only get half of that range (4.096) and 11 bits, not 12.
	// therefore, 4096mV/2^11 = 2mV/bit.
	std::cout << "ADXL377 0g = " << aref / 2 << " 2 mV/b " << "V/g = " << m_vpg
			<< std::endl;
}

ADXL377::~ADXL377() {
}

void ADXL377::values(float *xVal, float *yVal, float *zVal) {
	*xVal = m_adc->getSample(upm::ADS1X15::SINGLE_1); // 0 is Z, 1 is Y, 2 is X
	*yVal = m_adc->getSample(upm::ADS1X15::SINGLE_2);
	*zVal = m_adc->getSample(upm::ADS1X15::SINGLE_0);
}

void ADXL377::acceleration(float *xAccel, float *yAccel, float *zAccel) {
	float xVolts, yVolts, zVolts;

	values(&xVolts, &yVolts, &zVolts);
	*xAccel = ((xVolts - m_zeroX) / m_vpg) * 9.81;
	*yAccel = ((yVolts - m_zeroY) / m_vpg) * 9.81;
	*zAccel = ((zVolts - m_zeroZ) / m_vpg) * 9.81;
}

void ADXL377::accelerationX(float *xAccel) {
	*xAccel = m_adc->getSample(upm::ADS1X15::SINGLE_1);
	*xAccel = ((*xAccel - m_zeroX) / m_vpg) * 9.81;
}

void ADXL377::accelerationY(float *yAccel) {
	*yAccel = m_adc->getSample(upm::ADS1X15::SINGLE_2);
	*yAccel = ((*yAccel - m_zeroY) / m_vpg) * 9.81;
}

void ADXL377::accelerationZ(float *zAccel) {
	*zAccel = m_adc->getSample(upm::ADS1X15::SINGLE_0);
	*zAccel = ((*zAccel - m_zeroZ) / m_vpg) * 9.81;
}

void ADXL377::calibrate() {
	// make sure the sensor is still before running calibration.

	float xSum = 0.0, ySum = 0.0, zSum = 0.0;
	float x, y, z;

	std::chrono::high_resolution_clock::time_point time =
			std::chrono::high_resolution_clock::now()
					+ std::chrono::microseconds(10000);
	std::this_thread::sleep_until(time);

	// Read a bunch of values and take the last one
	for (int i = 0; i < 1000; i++) {
		values(&x, &y, &z);
		xSum += x;
		ySum += y;
		zSum += z;
	}
	x = xSum/1000.0;
	y = ySum/1000.0;
	z = zSum/1000.0;
	std::cout << "ADXL377 x = " << x << " y = " << y << " z " << z << std::endl;

	setZeroX(x);
	setZeroY(y);
	setZeroZ(z);  // Adjust for orientation with Z up.

	std::cout << "ADXL377 zeroX = " << m_zeroX << " zeroY = " << m_zeroY
			<< " zeroZ " << m_zeroZ << std::endl;

}
