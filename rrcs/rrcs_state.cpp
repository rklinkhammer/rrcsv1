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

#include "rrcs/rrcs_state.h"

namespace rrcs {

RRCSState RRCSState::instance_;

const char *RRCSState::RRCS_STATE_STR_[] = { "Init", "Calibrating", "Ready",
        "Boost", "Coast", "Main", "Drogue" };

RRCSState::RRCSState() {
    state_ = RRCS_STATE_INIT;
    for (int i = 0; i < 10; i++) {
        state_vector_.push_back(0.0);
    }
    for (int i = 0; i < 4; i++) {
        measurement_vector_.push_back(0.0);
    }
}

RRCSState::~RRCSState() {
    // TODO Auto-generated destructor stub
}

bool rrcs::RRCSState::IsCalibrated() {
    if ((boost::accumulators::variance(Xcal_) < ACC_EPISLON)
            && (boost::accumulators::variance(Ycal_) < ACC_EPISLON)
            && (boost::accumulators::variance(Zcal_) < ACC_EPISLON)
            && (boost::accumulators::variance(Pcal_) < ALTITUDE_EPISLON) &&
			boost::accumulators::count(Pcal_) >= 250) {
        return true;
    } else {
        return false;
    }
}

} /* namespace rrcs */

