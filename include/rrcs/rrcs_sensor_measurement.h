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

/*
 * sensor_measurement.h
 *
 *  Created on: Feb 8, 2016
 *      Author: dev
 */

#ifndef RRCS_SENSOR_MEASUREMENT_H_
#define RRCS_SENSOR_MEASUREMENT_H_

namespace rrcs {

class RRCSSensorMeasurement {
public:
    RRCSSensorMeasurement() {
    }

    virtual ~RRCSSensorMeasurement() {
    }

    void SetPressure(std::chrono::high_resolution_clock::time_point timestamp,
            float pressure) {
        timestamp_ = timestamp;
        pressure_ = pressure;
        has_pressure_ = true;
        has_accX_ = false;
        has_accYZ_ = false;
    }

    void SetAccX(std::chrono::high_resolution_clock::time_point timestamp,
            float accX) {
        timestamp_ = timestamp;
        accX_ = accX;
        has_pressure_ = false;
        has_accX_ = true;
        has_accYZ_ = false;
    }

    void SetAccXYZ(std::chrono::high_resolution_clock::time_point timestamp,
            float accX, float accY, float accZ) {
        timestamp_ = timestamp;
        accX_ = accX;
        has_pressure_ = false;
        has_accX_ = true;
        has_accYZ_ = true;
        accY_ = accY;
        accZ_ = accZ;
    }

    void SetAccYZ(std::chrono::high_resolution_clock::time_point timestamp,
            float accY, float accZ) {
        timestamp_ = timestamp;
        accY_ = accY;
        accZ_ = accZ;
        has_pressure_ = false;
        has_accX_ = false;
        has_accYZ_ = true;
    }

    bool IsPressure() const {
        return has_pressure_;
    }

    float GetPressure() const {
        return pressure_;
    }

    float GetAccX() const {
        return accX_;
    }

    float GetAccY() const {
        return accY_;
    }

    float GetAccZ() const {
        return accZ_;
    }

    bool IsAccX() const {
        return has_accX_;
    }

    bool IsAccYZ() const {
        return has_accYZ_;
    }

    const std::chrono::high_resolution_clock::time_point& GetTimestamp() const {
        return timestamp_;
    }

private:
    std::chrono::high_resolution_clock::time_point timestamp_;
    bool has_pressure_ { false };
    float pressure_ { 0.0 };
    bool has_accX_ { false };
    float accX_ { 0.0 };
    bool has_accYZ_ { false };
    float accY_ { 0.0 };
    float accZ_ { 0.0 };
};

} /* namespace rrcs */

#endif /* RRCS_SENSOR_MEASUREMENT_H_ */
