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

#ifndef RRCS_DYNAMICS_H_
#define RRCS_DYNAMICS_H_

#include <thread>
#include <mraa/pwm.hpp>
#include "util/threaded_queue.h"
#include "rrcs/rrcs_event.h"
#include "rrcs/rrcs_state.h"

namespace rrcs {

class RRCSDynamics {
public:

	static const int MRAA_PWM2_PIN = 0;
	static const int MRAA_PWM1_PIN = 14;

	RRCSDynamics(std::function<bool()> abort);
	virtual ~RRCSDynamics();

	void Init();
	void Run();

	void Join() {
        if (thread_.joinable()) {
            thread_.join();
        }
    }

protected:
    std::function<bool()> abort_;

private:
	void DynamicsStateCallback();
	void DynamicsConfigCallback();
	void HandleRRCSStateChange();
	void HandleRRCSStateReset();

	mraa::Pwm	*pwm_main_; // GP182 (MRAA pin 0)
	mraa::Pwm	*pwm_drogue_;  // GP13 (MRRA pin 14)

    ThreadedQueue<RRCSEvent> events_;
    RRCSState::RRCS_STATE state_ {RRCSState::RRCS_STATE_INIT};

    std::thread thread_;
    bool toggle_ {false};
};

} /* namespace rrcs */

#endif /* RRCS_DYNAMICS_H_ */
