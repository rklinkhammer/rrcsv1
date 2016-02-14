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

#ifndef RRCS_EVENT_H_
#define RRCS_EVENT_H_

#include "rrcs/rrcs_state.h"
#include "rrcs/rrcs_config.h"

namespace rrcs {

class RRCSEvent {
	enum RRCSEventType {
		RRCS_STATE_CHANGE
	};

public:
	RRCSEvent();
	virtual ~RRCSEvent();

	RRCSEventType GetCommand() const {
		return command_;
	}

	void SetCommand(RRCSEventType command) {
		command_ = command;
	}

	RRCSState::RRCS_STATE GetState() const {
		return state_;
	}

	void SetState(RRCSState::RRCS_STATE state) {
		state_ = state;
	}

private:

	RRCSEventType command_;
	RRCSState::RRCS_STATE	state_;

};

} /* namespace rrcs */

#endif /* RRCS_EVENT_H_ */
