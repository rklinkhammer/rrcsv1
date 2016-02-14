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

#ifndef RRCS_SENSOR_H_
#define RRCS_SENSOR_H_

#include <chrono>
#include <thread>
#include <mutex>
#include "rrcs/rrcs_sensor_measurement.h"

namespace rrcs {

class RRCSSensor {
public:
    RRCSSensor(std::string name, std::function<bool(RRCSSensorMeasurement& measurement)> update,
            std::function<bool()> abort);
    virtual ~RRCSSensor();

    virtual void Init() = 0;
    virtual void ReadSensor(RRCSSensorMeasurement& measurement,
            std::chrono::high_resolution_clock::time_point time) = 0;
    virtual void Run();

    const std::chrono::milliseconds& GetRate() const {
        return rate_;
    }

    void SetRate(const std::chrono::milliseconds& rate) {
        rate_ = rate;
    }

    const std::chrono::high_resolution_clock::time_point& GetTime() const {
        return time_;
    }

    void SetTime(const std::chrono::high_resolution_clock::time_point& time) {
        time_ = time;
    }

    void Join() {
        if (thread_.joinable()) {
            thread_.join();
        }
    }

protected:
    std::function<bool(RRCSSensorMeasurement& measurement)> update_;
    std::function<bool()> abort_;

private:

    std::string name_;
    std::chrono::high_resolution_clock::time_point time_;
    std::chrono::milliseconds rate_ { 40 };
    std::thread thread_;
}
;

} /* namespace rrcs */

#endif /* RRCS_SENSOR_H_ */
