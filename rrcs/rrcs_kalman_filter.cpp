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
#include <algorithm>
#include <Wt/WApplication>
#include "rrcs/rrcs_state.h"
#include "rrcs/rrcs_kalman_filter.h"

namespace rrcs {

RRCSKalmanFilter::RRCSKalmanFilter(std::function<bool()> abort) :
        abort_(abort) {
    for(int i = 0; i < RRCS_ACCELERATION_SAMPLES; i++) {
        acc_A_[i] = 0.0;
        acc_B_[i] = 0.0;
    }
    memset(ip_, 0, sizeof(ip_));
    memset(w_, 0, sizeof(w_));

}

RRCSKalmanFilter::~RRCSKalmanFilter() {
}

void RRCSKalmanFilter::Init() {
    InitKalmanFilter();
}

void RRCSKalmanFilter::Run() {
    auto thread_lambda = [this]() -> void {
        RRCSSensorMeasurement d;
        while(!abort_() && measurements_.Dequeue(d)) {
            ProcessData(d);
        }
    };

    thread_ = std::thread(thread_lambda);
}

void RRCSKalmanFilter::JitterStatistics(const RRCSSensorMeasurement& d) {
    auto current = std::chrono::high_resolution_clock::now();
    if (d.IsAccX()) {
        if (acc_observations_) {
            RRCSState::GetInstance().UpdateXJitter((current - last_acc_).count());
            RRCSState::GetInstance().UpdateXLatency((d.GetTimestamp() - last_acc_).count());
        }
        last_acc_ = current;
        acc_observations_++;
    }
    if (d.IsPressure()) {
        if (baro_observations_) {
            RRCSState::GetInstance().UpdatePJitter((current - last_baro_).count());
            RRCSState::GetInstance().UpdatePLatency((d.GetTimestamp() - last_baro_).count());
        }
        last_baro_ = current;
        baro_observations_++;
    }
}

bool RRCSKalmanFilter::AccelerationDoubleBuffer(const RRCSSensorMeasurement& d, double A[], double B[]) {
    A[acc_count_] = d.GetAccX();
    if (acc_count_ >= RRCS_ACCELERATION_SAMPLES / 2) {
        B[acc_count_ - RRCS_ACCELERATION_SAMPLES / 2] = d.GetAccX();
    }
    acc_count_++;
    if (acc_count_ == RRCS_ACCELERATION_SAMPLES) {
        is_acc_A_ = !is_acc_A_;
        acc_count_ = RRCS_ACCELERATION_SAMPLES / 2;
        return true;
    }
    return false;
}

void RRCSKalmanFilter::AccelerationFFT(double A[]) {
    for (int i = 0; i < RRCS_ACCELERATION_SAMPLES; i++) {
        a_[i] = A[i];
    }
    for (int i = RRCS_ACCELERATION_SAMPLES; i < ACC_FFT_SIZE; i++) {
        a_[i] = 0.0;
    }
    rdft(ACC_FFT_SIZE, 1, a_, ip_, w_);
    std::vector<int> result;
    for (int i = 0; i < RRCS_ACCELERATION_SAMPLES/2; i++) {
    	if(a_[i] > 75.0) {
    		result.push_back(i);
    	}
    }
    std::sort (result.begin(), result.end());
    RRCSState::GetInstance().SetVibrationVector(result);
}

void RRCSKalmanFilter::AccelerationVibrationAnalysis(const RRCSSensorMeasurement& d) {
    if(is_acc_A_) {
        if(AccelerationDoubleBuffer(d, acc_A_, acc_B_)) {
            AccelerationFFT(acc_A_);
        }
    } else {
        if(AccelerationDoubleBuffer(d, acc_B_, acc_A_)) {
            AccelerationFFT(acc_B_);
      }
    }
}

void RRCSKalmanFilter::InitKalmanFilter() {
    Xe_ = StateVector::Zero(); // State Vector
    Xf_ = StateVector::Zero(); // State Vector
    Xp_ = StateVector::Zero(); // State Vector
    A_ = TransitionMatrix::Identity(); // Transition Next State Matrix
    SetNextStateDeltaT(0.04);

    U_ = StateVector::Zero(); // Noise Vector

    M_ = MeasurementMatrix::Zero();
    M_(0, 2) = 1;
    M_(1, 5) = 1;
    M_(2, 8) = 1;
    M_(3, 9) = 1;

    H_ = GainMatrix::Zero();
    H_(2, 0) = 0.018; // AccX
    H_(5, 1) = 0.018; // AccY
    H_(8, 2) = 0.018; // AccZ
    H_(9, 3) = 0.020; // Baro

    Hp_ = GainMatrix::Zero();
    Hp_(9, 3) = 0.020; // Baro

    Hx_ = GainMatrix::Zero();
    Hx_(2, 0) = 0.018; // AccX

    Hyz_ = GainMatrix::Zero();
    Hyz_(5, 1) = 0.018; // AccY
    Hyz_(8, 2) = 0.018; // AccZ

    Hxyz_ = GainMatrix::Zero();
    Hxyz_(2, 0) = 0.018; // AccX
    Hxyz_(5, 1) = 0.018; // AccY
    Hxyz_(8, 2) = 0.018; // AccZ
}

void RRCSKalmanFilter::PrecalibrationState(const RRCSSensorMeasurement& d) {
    // collect statistics
    if (d.IsPressure()) {
        RRCSState::GetInstance().UpdatePCal(d.GetPressure());
    }
    if (d.IsAccX()) {
        RRCSState::GetInstance().UpdateXCal(d.GetAccX(),d.GetAccY(),d.GetAccZ());
    }
    if (RRCSState::GetInstance().IsCalibrated()) {
        RRCSState::GetInstance().SetState(RRCSState::RRCS_STATE_CALIBRATING);
        std::vector<double>  x = RRCSState::GetInstance().GetXCal();
        std::vector<double>  y = RRCSState::GetInstance().GetYCal();
        std::vector<double>  z = RRCSState::GetInstance().GetZCal();
        std::vector<double>  p = RRCSState::GetInstance().GetPCal();
        Basep_ = p[0];
        Xf_(9) = p[0];
        U_(0) = x[1];
        U_(3) = y[1];
        U_(6) = z[1];
        U_(9) = p[1];

        std::cout << "KalmanFilter:: CALIBRATED" << std::endl;
    }
}

void RRCSKalmanFilter::KalmanGainState(const RRCSSensorMeasurement& d) {
    RRCSState::GetInstance().SetState(RRCSState::RRCS_STATE_READY);
    std::cout << "KalmanFilter:: READY" << std::endl;
}

void RRCSKalmanFilter::EstimationStep() {
    /* State Estimation */
    // X*n+1,n = AX*n,n + Un,n
    Xp_ = A_ * Xf_; //+ U_;
}

void RRCSKalmanFilter::CorrectionStep(const RRCSSensorMeasurement& d) {
    // X*n,n= X*n,n-1 + H(Y-MX*n,n-1)
    Y_ = MeasurementVector::Zero();
    if (d.IsPressure()) {
        Y_(3) = d.GetPressure() - Basep_;
        Xf_ = Xp_ + Hp_ * (Y_ - M_ * Xp_);
    }
    if (RRCSState::GetInstance().GetState() > RRCSState::RRCS_STATE_READY) {
        if (d.IsAccX() && d.IsAccYZ()) {
            Y_(0) = d.GetAccX();
            Y_(1) = d.GetAccY();
            Y_(2) = d.GetAccZ();
            Xf_ = Xp_ + Hxyz_ * (Y_ - M_ * Xp_);
        } else if (d.IsAccX()) {
            Y_(0) = d.GetAccX();
            Xf_ = Xp_ + Hx_ * (Y_ - M_ * Xp_);
        } else if (d.IsAccYZ()) {
            Y_(1) = d.GetAccY();
            Y_(2) = d.GetAccZ();
            Xf_ = Xp_ + Hyz_ * (Y_ - M_ * Xp_);
        }
    } else {
        if (d.IsAccX() && d.IsAccYZ()) {
        	 Xf_(2) = d.GetAccX();
        	 Xf_(5) = d.GetAccY();
        	 Xf_(8) = d.GetAccZ();
        }
        if (d.IsPressure()) {
        	Xf_(9) = d.GetPressure();
        }
        Xf_(0) = 0.0;
        Xf_(1) = 0.0;
        Xf_(3) = 0.0;
        Xf_(4) = 0.0;
        Xf_(6) = 0.0;
        Xf_(7) = 0.0;
    }
    if ((RRCSState::GetInstance().GetState() == RRCSState::RRCS_STATE_READY) && d.IsAccX()
            && (fabs(d.GetAccX()) > ACCX_LAUNCH_VALUE)) {
        std::cout << "BOOST!" << std::endl;
        RRCSState::GetInstance().SetState(RRCSState::RRCS_STATE_BOOST);
    }
    if ((RRCSState::GetInstance().GetState() == RRCSState::RRCS_STATE_BOOST) && (Xf_(2) < 0)) {
        std::cout << "COAST" << std::endl;
        RRCSState::GetInstance().SetState(RRCSState::RRCS_STATE_COAST);
    }
}

void RRCSKalmanFilter::KalmanFilterStep(const RRCSSensorMeasurement& d) {
    float dT = (float) std::chrono::duration_cast<std::chrono::microseconds>(
            d.GetTimestamp() - last_).count() / 1000000.0;
    SetNextStateDeltaT(dT);
    EstimationStep();
    CorrectionStep(d);
}

void RRCSKalmanFilter::UpdateKalmanFilter(const RRCSSensorMeasurement& d) {
    switch (RRCSState::GetInstance().GetState()) {
    case RRCSState::RRCS_STATE_INIT:
        PrecalibrationState(d);
        break;
    case RRCSState::RRCS_STATE_CALIBRATING:
        KalmanGainState(d);
        break;
    case RRCSState::RRCS_STATE_READY:
    case RRCSState::RRCS_STATE_BOOST:
    case RRCSState::RRCS_STATE_COAST:
    case RRCSState::RRCS_STATE_MAIN:
    case RRCSState::RRCS_STATE_DROGUE:
        KalmanFilterStep(d);
        break;
    }
}

void RRCSKalmanFilter::SetNextStateDeltaT(float deltaT) {
    for (int i = 0; i < NSTATES - 1; i += 3) {
        A_(i, i + 1) = deltaT;
        A_(i, i + 2) = deltaT * deltaT * 0.5;
        A_(i + 1, i + 2) = deltaT;
    }
}

void RRCSKalmanFilter::ProcessData(const RRCSSensorMeasurement& d) {
    JitterStatistics(d);
    if (d.IsAccX()) {
        AccelerationVibrationAnalysis(d);
    }
    UpdateKalmanFilter(d);
    last_ = d.GetTimestamp();
    if((++Nobs_ % 250) == 0) {
        std::vector<double> x;
        for(int i = 0; i < NSTATES; i++) {
            x.push_back(static_cast<double>(Xf_(i)));
        }
        RRCSState::GetInstance().SetStateVector(x);
    }
}

} /* namespace rrcs */
