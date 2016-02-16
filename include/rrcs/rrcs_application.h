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

#ifndef RRCS_APPLICATION_H_
#define RRCS_APPLICATION_H_

#include <thread>
#include <functional>
#include <Wt/WApplication>
#include <Wt/WServer>

namespace rrcs {

class RRCSApplication {
public:
    static Wt::WApplication *createApplication(const Wt::WEnvironment& env);
    static void UpdateThread(Wt::WApplication *app, std::function<bool()> abort);
    static std::function<bool()> abort_;
};

} /* namespace rrcs */

#endif /* RRCS_APPLICATION_H_ */
