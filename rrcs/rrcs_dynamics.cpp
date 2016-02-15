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
#include "rrcs/rrcs_dynamics.h"
#include "rrcs/rrcs_state.h"
#include "rrcs/rrcs_config.h"

namespace rrcs {

RRCSDynamics::RRCSDynamics(std::function<bool()> abort) :
		abort_(abort) {

}

RRCSDynamics::~RRCSDynamics() {
}

void RRCSDynamics::Init() {
	mraa::Result ret;
//	RRCSState::GetInstance().AddCallback([this]() {
//		DynamicsStateCallback();
//	});
////
//	RRCSConfig::GetInstance().AddCallback([this]() {
//		DynamicsConfigCallback();
//	});
//
//	pwm_main_ = new mraa::Pwm(MRAA_PWM2_PIN, true, -1);
//
//	ret = pwm_main_->enable(false);
//	if(ret != mraa::SUCCESS) {
//		std::cout << " error on main enable:" << ret << std::endl;
//	}
//	ret = pwm_main_->config_percent(50,1);
//	if(ret !=  mraa::SUCCESS) {
//		std::cout << " error on main:" << ret << std::endl;
//	}
//	pwm_drogue_ = new mraa::Pwm(MRAA_PWM1_PIN, true, -1);
//	ret = pwm_drogue_->enable(false);
//	if(ret !=  mraa::SUCCESS) {
//		std::cout << " error on drogue enable:" << ret << std::endl;
//	}
//	ret = pwm_drogue_->config_percent(218,1);
//	if(ret !=  mraa::SUCCESS) {
//		std::cout << " error on drogue:" << ret << std::endl;
//	}
}

void RRCSDynamics::DynamicsConfigCallback() {
}

void RRCSDynamics::DynamicsStateCallback() {
	RRCSEvent event;
	event.SetCommand(RRCSEvent::RRCS_STATE_CHANGE);
	events_.Enqueue(event);
//	switch (RRCSState::GetInstance().GetState()) {
//	case RRCSState::RRCS_STATE_INIT:
//	case RRCSState::RRCS_STATE_CALIBRATING:
//	case RRCSState::RRCS_STATE_READY:
////    	if(toggle_) {
////        	std::cout << "dynamics(on)" << std::endl;
////        	pwm_drogue_->enable(true);
////        	pwm_main_->enable(true);
////    	} else {
////    		std::cout << "dynamics(off)" << std::endl;
////    		pwm_drogue_->enable(false);
////    		pwm_main_->enable(false);
////    	}
////    	toggle_ = !toggle_;
//		break;
//	case RRCSState::RRCS_STATE_BOOST:
//		break;
//	case RRCSState::RRCS_STATE_COAST:
//		break;
//	case RRCSState::RRCS_STATE_MAIN:
//		break;
//	case RRCSState::RRCS_STATE_DROGUE:
//		break;
//	}
}

void RRCSDynamics::HandleRRCSStateChange() {
	std::cout << "State Change" << std::endl;
}

void RRCSDynamics::HandleRRCSStateReset() {
	state_ = RRCSState::RRCS_STATE_INIT;
	std::cout << "Reset" << std::endl;
}

void RRCSDynamics::Run() {
	auto thread_lambda = [this]() -> void {
		bool queue_status = true;
		RRCSEvent event;
		while(!abort_() && queue_status) {
			queue_status = events_.Dequeue(event);
			if(event.GetCommand() == RRCSEvent::RRCS_STATE_CHANGE) {
				// If the new state is the next state, then we have
				// not been reset.  Otherwise, we have been reset and
				// set start over.
			if(event.GetState() > state_) {
				HandleRRCSStateChange();
			} else if (event.GetState() == RRCSState::RRCS_STATE_INIT) {
				HandleRRCSStateReset();
			}
		}
	}
}	;

	thread_ = std::thread(thread_lambda);
}

} /* namespace rrcs */
