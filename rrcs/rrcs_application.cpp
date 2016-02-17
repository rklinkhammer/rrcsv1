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
#include "rrcs/rrcs.h"
#include "rrcs/rrcs_application.h"
#include "rrcs/rrcs_dashboard.h"
#include "rrcs/rrcs_state.h"

namespace rrcs {

std::function<bool()> RRCSApplication::abort_ {nullptr};
std::thread update;

Wt::WApplication *RRCSApplication::createApplication(
        const Wt::WEnvironment& env) {
    Wt::WApplication *app = new Wt::WApplication(env);
    app->setTitle("RRCS");
    app->enableUpdates(true);
    new RRCSDashboard(app->root());
    update = std::thread([app]() { RRCSApplication::UpdateThread(app, RRCSApplication::abort_); });
    return app;
}

void RRCSApplication::UpdateThread(Wt::WApplication *app, std::function<bool()> abort) {
    std::chrono::high_resolution_clock::time_point time;
    time = std::chrono::high_resolution_clock::now();
    time += std::chrono::milliseconds(5000);
    std::chrono::milliseconds rate {1000};
    while(!abort()) {
        std::this_thread::sleep_until(time);
        Wt::WApplication::UpdateLock uiLock(app);
        if (uiLock) {
            RRCSState::GetInstance().Notify();
            app->triggerUpdate();
        }
        time += rate;
    }
}
} /* namespace rrcs */
