/*

The MIT License (MIT)

Copyright (c) 2016 rklinkhammer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "rrcs/rrcs_accelerometer.h"

namespace rrcs {

RRCSAccelerometer::RRCSAccelerometer(
        std::function<bool(RRCSSensorMeasurement& measurement)> update,
        std::function<bool()> abort) :
        RRCSSensor("ADXL377", update, abort) {
}

RRCSAccelerometer::~RRCSAccelerometer() {
}

void RRCSAccelerometer::Init() {
    gpio115_st_ = new mraa::Gpio(MRAA_INTEL_EDISON_GP115);
    if (gpio115_st_ == NULL) {
        std::cout << "count not create GPIO(115 mraa == 11)" << std::endl;
        mraa::printError(mraa::ERROR_UNSPECIFIED);
    }
    mraa::Result response = gpio115_st_->dir(mraa::DIR_OUT);
    if (response != mraa::SUCCESS) {
        mraa::printError(response);
    }
    response = gpio115_st_->mode(mraa::MODE_STRONG);
    if (response != mraa::SUCCESS) {
        mraa::printError(response);
    }
    response = gpio115_st_->write(0);
    if (response != mraa::SUCCESS) {
        mraa::printError(response);
    }

    adc_ = new upm::ADS1015(1, 0x48);
    adc_->setContinuous(false);
    adc_->setSPS(upm::ADS1015::SPS_3300);
    adc_->setGain(upm::ADS1X15::GAIN_ONE);
    acc_ = new upm::ADXL377(adc_);
    acc_->calibrate();

    SetRate(std::chrono::milliseconds(RRCS_ACCELERATION_PERIOD_MS));
}

void RRCSAccelerometer::ReadSensor(RRCSSensorMeasurement& measurement,
        std::chrono::high_resolution_clock::time_point time) {
    float accX, accY, accZ;
    acc_->acceleration(&accX, &accY, &accZ);
    measurement.SetAccXYZ(time, accX, accY, accZ);
}

} /* namespace rrcs */
