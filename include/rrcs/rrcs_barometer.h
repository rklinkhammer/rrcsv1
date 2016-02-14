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

#ifndef RRCS_BAROMETER_H_
#define RRCS_BAROMETER_H_

#include "mraa/bmpx8x.h"
#include "rrcs/rrcs_sensor.h"

namespace rrcs {

class RRCSBarometer: public RRCSSensor {
public:
    RRCSBarometer(
            std::function<bool(RRCSSensorMeasurement& measurement)> update,
            std::function<bool()> abort);
    virtual ~RRCSBarometer();

    virtual void Init();
    virtual void ReadSensor(RRCSSensorMeasurement& measurement,
            std::chrono::high_resolution_clock::time_point time);

private:
    upm::BMPX8X *baro_;
    float pressure_;
};

} /* namespace rrcs */

#endif /* RRCS_BAROMETER_H_ */
