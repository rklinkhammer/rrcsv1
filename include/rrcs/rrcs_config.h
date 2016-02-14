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

#ifndef RRCS_CONFIG_H_
#define RRCS_CONFIG_H_

#include <string>
#include <vector>
#include <mutex>

namespace rrcs {

class RRCSConfig {
public:
    static RRCSConfig& GetInstance() { return instance_; }
    void ReadConfig(std::string& mode, std::string& altitude);
    void WriteConfig(const std::string mode, const std::string altitude);
    void AddCallback(std::function<void()> f) { callbacks_.push_back(f); }
    std::string GetNextFileName();

private:
    RRCSConfig();
    virtual ~RRCSConfig();
    void Notify() {
        std::unique_lock<std::mutex> lock(lock_);
        for(auto f : callbacks_) {
            f();
        }
    }
    std::vector<std::function<void()>> callbacks_;
    std::mutex  lock_;

    static RRCSConfig instance_;
};

} /* namespace rrcs */

#endif /* RRCS_CONFIG_H_ */
