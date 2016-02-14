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

#include <iostream>
#include "rrcs/rrcs_sensor.h"

namespace rrcs {

RRCSSensor::RRCSSensor(std::string name,
        std::function<bool(RRCSSensorMeasurement& measurement)> update,
        std::function<bool()> abort) :
        update_(update), abort_(abort), name_(name) {
}

RRCSSensor::~RRCSSensor() {
}

void RRCSSensor::Run() {
    auto thread_lambda = [this]() -> void {
    	std::cout << "Starting " << name_ << std::endl;
        time_ = std::chrono::high_resolution_clock::now();
        time_ += std::chrono::milliseconds(10000);
        bool queue_status = true;
        RRCSSensorMeasurement d;
        while(!abort_() && queue_status) {
            std::this_thread::sleep_until(time_);
            ReadSensor(d, time_);
            time_ += rate_;
            queue_status = update_(d);
        }
    };

    thread_ = std::thread(thread_lambda);
}

} /* namespace rrcs */
