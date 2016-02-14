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

#ifndef RRCS_STATE_H_
#define RRCS_STATE_H_

#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/density.hpp>
#include <boost/accumulators/statistics/variance.hpp>

using namespace boost::accumulators;

namespace rrcs {

class RRCSState {
public:
    static const char *RRCS_STATE_STR_[];

    enum RRCS_STATE {
        RRCS_STATE_INIT,
        RRCS_STATE_CALIBRATING,
        RRCS_STATE_READY,
        RRCS_STATE_BOOST,
        RRCS_STATE_COAST,
        RRCS_STATE_MAIN,
        RRCS_STATE_DROGUE
    };

    static constexpr const float ALTITUDE_EPISLON = 0.5;
    static constexpr const float ACC_EPISLON = (9.81 * 0.25); // accuracy of ADC

    static RRCSState& GetInstance() {
        return instance_;
    }

    bool IsCalibrated();

    void AddCallback(std::function<void()> f) {
        callbacks_.push_back(f);
    }

    RRCS_STATE GetState() const {
        return state_;
    }

    void SetState(enum RRCS_STATE state) {
        state_ = state;
    }

    std::string GetRrcsStateStr() {
        return RRCS_STATE_STR_[state_];
    }

    const std::vector<double>& GetMeasurementVector() const {
        return measurement_vector_;
    }

    void SetMeasurementVector(const std::vector<double>& measurement_vector) {
        measurement_vector_ = measurement_vector;
    }

    const std::vector<double> GetStateVector() const {
        return state_vector_;
    }

    void SetStateVector(const std::vector<double>& state_vector) {
        state_vector_ = state_vector;
    }

    void UpdateXJitter(double delta) {
        Xjitter_(delta);
    }

    void UpdateYZJitter(double delta) {
        YZjitter_(delta);
    }

    void UpdatePJitter(double delta) {
        Pjitter_(delta);
    }

    void UpdateXCal(double x, double y, double z) {
        Xcal_(x);
        Ycal_(y);
        Zcal_(z);
    }

    void UpdatePCal(double p) {
         Pcal_(p);
     }

    void Notify() {
        std::unique_lock<std::mutex> lock(lock_);
        for (auto f : callbacks_) {
            f();
        }
    }

    const std::vector<double> GetXCal() {
        std::vector<double> v;
        v.push_back(boost::accumulators::mean(Xcal_));
        v.push_back(boost::accumulators::variance(Xcal_));
        v.push_back(boost::accumulators::count(Xcal_));
        return v;
    }

    const std::vector<double> GetYCal() {
        std::vector<double> v;
        v.push_back(boost::accumulators::mean(Ycal_));
        v.push_back(boost::accumulators::variance(Ycal_));
        v.push_back(boost::accumulators::count(Ycal_));
        return v;
    }

    const std::vector<double> GetZCal() {
        std::vector<double> v;
        v.push_back(boost::accumulators::mean(Zcal_));
        v.push_back(boost::accumulators::variance(Zcal_));
        v.push_back(boost::accumulators::count(Zcal_));
        return v;
    }

    const std::vector<double> GetPCal() {
        std::vector<double> v;
        v.push_back(boost::accumulators::mean(Pcal_));
        v.push_back(boost::accumulators::variance(Pcal_));
        v.push_back(boost::accumulators::count(Pcal_));
        return v;
    }

    const std::vector<double> GetXJitter() {
        std::vector<double> v;
        v.push_back(boost::accumulators::mean(Xjitter_));
        v.push_back(boost::accumulators::variance(Xjitter_));
        v.push_back(boost::accumulators::count(Xjitter_));
        return v;
    }

    const std::vector<double> GetPJitter() {
        std::vector<double> v;
        v.push_back(boost::accumulators::mean(Pjitter_));
        v.push_back(boost::accumulators::variance(Pjitter_));
        v.push_back(boost::accumulators::count(Pjitter_));
        return v;
    }

private:
    RRCSState();
    virtual ~RRCSState();

    accumulator_set<double,
            stats<tag::count, tag::mean, tag::variance, tag::density> > Xjitter_ {
            tag::density::cache_size = 100, tag::density::num_bins = 10 };

    accumulator_set<double,
            stats<tag::count, tag::mean, tag::variance, tag::density> > YZjitter_ {
            tag::density::cache_size = 100, tag::density::num_bins = 10 };

    accumulator_set<double,
            stats<tag::count, tag::mean, tag::variance, tag::density> > Pjitter_ {
            tag::density::cache_size = 100, tag::density::num_bins = 10 };

    accumulator_set<double,
            stats<tag::count, tag::mean, tag::variance> > Xcal_;

    accumulator_set<double,
            stats<tag::count, tag::mean, tag::variance> > Ycal_;

    accumulator_set<double,
             stats<tag::count, tag::mean, tag::variance> > Zcal_;

    accumulator_set<double,
            stats<tag::count, tag::mean, tag::variance> > Pcal_;

    std::atomic<RRCS_STATE> state_;
    std::vector<double> state_vector_;
    std::vector<double> measurement_vector_;
    std::vector<std::function<void()>> callbacks_;
    std::mutex lock_;

    static RRCSState instance_;
};

} /* namespace rrcs */

#endif /* RRCS_STATE_H_ */
