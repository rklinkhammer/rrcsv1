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
#include "rrcs/rrcs_application.h"
#include "rrcs/rrcs_accelerometer.h"
#include "rrcs/rrcs_barometer.h"
#include "rrcs/rrcs_kalman_filter.h"
#include "rrcs/rrcs_dynamics.h"

int main(int argc, char **argv) {
    bool abort = false;
    try {
        std::function<bool()> f = [abort]() -> bool {return abort;};
        rrcs::RRCSKalmanFilter kf(f);
        rrcs::RRCSBarometer baro(kf.GetUpdateFunction(), f);
        rrcs::RRCSAccelerometer acc(kf.GetUpdateFunction(), f);
        rrcs::RRCSApplication::abort_ = f;
        rrcs::RRCSDynamics	dynamics(f);

        kf.Init();
        baro.Init();
        acc.Init();
        dynamics.Init();

        kf.Run();
        baro.Run();
        acc.Run();
        dynamics.Run();

        Wt::WServer server(argv[0]);
        server.setServerConfiguration(argc, argv, WTHTTP_CONFIGURATION);
        server.addEntryPoint(Wt::Application,
                rrcs::RRCSApplication::createApplication);

        if (server.start()) {
            Wt::WServer::waitForShutdown();
            server.stop();
            abort = true;
        }
        kf.Join();
        baro.Join();
        acc.Join();
        dynamics.Join();

    } catch (Wt::WServer::Exception& e) {
        std::cerr << e.what() << std::endl;
    } catch (std::exception &e) {
        std::cerr << "exception: " << e.what() << std::endl;
    }

}
