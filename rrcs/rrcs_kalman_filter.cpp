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
#include "rrcs/rrcs_config.h"

namespace rrcs {

RRCSKalmanFilter::RRCSKalmanFilter(std::function<bool()> abort) :
        abort_(abort) {
    for (int i = 0; i < RRCS_ACCELERATION_SAMPLES; i++) {
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
    logfile_.open(RRCSConfig::GetInstance().GetFilename(), std::fstream::out | std::fstream::trunc);
    logfile_
            << "elapsed,time,state,Xa,Ya,Za,Xp,X*a,X*v,X*d,Y*a,Y*v,Y*d,Z*a,Z*v,Z*d,X*p,min(Xp),max(Xp),N(Xp)"
            << std::endl;
    elapsed_time_ = 0.0;
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

bool RRCSKalmanFilter::AccelerationDoubleBuffer(const RRCSSensorMeasurement& d, double A[],
        double B[]) {
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
    for (int i = 0; i < RRCS_ACCELERATION_SAMPLES / 2; i++) {
        if (a_[i] > 75.0) {
            result.push_back(i);
        }
    }
    std::sort(result.begin(), result.end());
    RRCSState::GetInstance().SetVibrationVector(result);
}

void RRCSKalmanFilter::AccelerationVibrationAnalysis(const RRCSSensorMeasurement& d) {
    if (is_acc_A_) {
        if (AccelerationDoubleBuffer(d, acc_A_, acc_B_)) {
            AccelerationFFT(acc_A_);
        }
    } else {
        if (AccelerationDoubleBuffer(d, acc_B_, acc_A_)) {
            AccelerationFFT(acc_B_);
        }
    }
}

void RRCSKalmanFilter::InitKalmanFilter() {
    Xe_ = StateVector::Zero(); // State Vector
    Xf_ = StateVector::Zero(); // State Vector
    A_ = StateMatrix::Identity(); // Transition Next State Matrix
    SetNextStateDeltaT(0.04);

    U_ = StateVector::Zero();
    R_ = MeasurementCovarianceMatrix::Zero();

    M_ = MeasurementMatrix::Zero();
    M_(0, 2) = 1;
    M_(1, 5) = 1;
    M_(2, 8) = 1;
    M_(3, 9) = 1;

    Hp_ = GainMatrix::Zero();
    Hp_(9, 3) = 0.020; // Baro

    Hxyz_ = GainMatrix::Zero();
    Hxyz_(2, 0) = 0.018; // AccX
    Hxyz_(5, 1) = 0.018; // AccY
    Hxyz_(8, 2) = 0.018; // AccZ

    Se_ = StateMatrix::Zero();
    Sf_ = StateMatrix::Zero(); // TODO -- init this

    // This is somewhat of a hack -- estimation of Q is based on the physics
    // of the motion.  For this, we guestimate the max covariance of the motion
    // in the next time interval.
    Q_ = StateMatrix::Zero();
    Q_(Xa, Xa) = (50.0 * 50.0);
    Q_(Ya, Ya) = (10.0 * 10.0);
    Q_(Za, Za) = (10.0 * 10.0);
    Q_(Xp, Xp) = (10.0 * 10.0);

}

void RRCSKalmanFilter::KalmanGainState(const RRCSSensorMeasurement& d) {
}

void RRCSKalmanFilter::EstimationStep() {
    /* State Estimation (Predictor)*/
    // X*n+1,n = AX*n,n
    Xe_ = A_ * Xf_;
    // S*n+1,n =  A Sk,k A^t + Q_
    Se_ = A_ * Sf_ * A_.transpose() + Q_;
    // H = (P*n+1,n M.transpose())(M * P*n+1,n * M.transpose() + R)^-1
    He_ = (Se_ * M_.transpose()) * ((M_ * Se_ * M_.transpose() + R_).inverse());
    Hp_(9, 3) = He_(9, 3);
    Hxyz_(2, 0) = He_(2, 0);
    Hxyz_(5, 1) = He_(5, 1);
    Hxyz_(8, 2) = He_(8, 2);
}

void RRCSKalmanFilter::CorrectionStep(const RRCSSensorMeasurement& d) {
    if (RRCSState::GetInstance().GetState() > RRCSState::RRCS_STATE_READY) {
        // X*n,n= X*n,n-1 + H(Y-MX*n,n-1)
        Y_ = MeasurementVector::Zero();
        if (d.IsPressure()) {
            Y_(3) = d.GetPressure() - Xp_min_;
            Xf_ = Xe_ + Hp_ * (Y_ - M_ * Xe_);
        }
        if (d.IsAccX() && d.IsAccYZ()) {
            Y_(0) = d.GetAccX();
            Y_(1) = d.GetAccY();
            Y_(2) = d.GetAccZ();
            Xf_ = Xe_ + Hxyz_ * (Y_ - M_ * Xe_);
        }
        Sf_ = (StateMatrix::Identity() - He_ * M_) * Se_;
    } else {
        if (d.IsAccX() && d.IsAccYZ()) {
            Xf_(Xa) = d.GetAccX();
            Xf_(Ya) = d.GetAccY();
            Xf_(Za) = d.GetAccZ();
        }
        if (d.IsPressure()) {
            Xf_(Xp) = d.GetPressure();
        }
        Xf_(Xd) = 0.0;
        Xf_(Xv) = 0.0;
        Xf_(Yd) = 0.0;
        Xf_(Yv) = 0.0;
        Xf_(Zd) = 0.0;
        Xf_(Zv) = 0.0;
    }
}

void RRCSKalmanFilter::KalmanFilterStep(const RRCSSensorMeasurement& d) {
    float dT = (float) std::chrono::duration_cast < std::chrono::microseconds
            > (d.GetTimestamp() - last_).count() / 1000000.0;
    SetNextStateDeltaT(dT);
    EstimationStep();
    CorrectionStep(d);
    if (Xf_(Xp) > Xp_max_) {
        Xp_max_ = Xf_(Xp);
    }
}

void RRCSKalmanFilter::SetRValues() {
    std::vector<double> c = RRCSState::GetInstance().GetXYZPCovariance();
    for (int i = 0; i < 4; i++) {
        if (R_(i, i) < c[i]) {
            R_(i, i) = c[i];
        }
    }
}

void RRCSKalmanFilter::SetCalibrationValues() {
    std::vector<double> p = RRCSState::GetInstance().GetPCal();
    Xp_min_ = p[0];
    Xf_(Xp) = p[0];
}

void RRCSKalmanFilter::SetNextState(RRCSState::RRCS_STATE state) {
    RRCSState::GetInstance().SetState(state);
    state_count_ = 0;
    std::cout << RRCSState::GetInstance().GetRrcsStateStr() << std::endl;
}

void RRCSKalmanFilter::UpdateKalmanFilter(const RRCSSensorMeasurement& d) {
    // collect statistics for covariances.
    if (d.IsPressure()) {
        RRCSState::GetInstance().UpdatePCal(d.GetPressure());
    }
    if (d.IsAccX()) {
        RRCSState::GetInstance().UpdateXCal(d.GetAccX(), d.GetAccY(), d.GetAccZ());
    }
    if (RRCSState::GetInstance().GetState() > RRCSState::RRCS_STATE_CALIBRATED) {
        SetRValues();
    }
    switch (RRCSState::GetInstance().GetState()) {
    case RRCSState::RRCS_STATE_INIT:
        if (RRCSState::GetInstance().IsCalibrated()) {
            SetNextState(RRCSState::RRCS_STATE_CALIBRATED);
            SetCalibrationValues();
        }
        break;

    case RRCSState::RRCS_STATE_CALIBRATED:
        KalmanGainState(d);
        if (IsReady()) {
            Log(d);
            SetNextState(RRCSState::RRCS_STATE_READY);
        }
        break;

    case RRCSState::RRCS_STATE_READY:
        KalmanFilterStep(d);
        if (IsBoost()) {
            Log(d);
            SetNextState(RRCSState::RRCS_STATE_BOOST);
        }
        break;

    case RRCSState::RRCS_STATE_BOOST:
        KalmanFilterStep(d);
        Log(d);
        if (IsApogee()) {
            SetNextState(RRCSState::RRCS_STATE_APOGEE);
        }
        break;

    case RRCSState::RRCS_STATE_APOGEE:
        KalmanFilterStep(d);
        Log(d);
        if (DualDeployDrogue()) {
            SetNextState(RRCSState::RRCS_STATE_DROGUE);
        } else if (DeployMain()) {
            SetNextState(RRCSState::RRCS_STATE_DONE);
        }
        break;

    case RRCSState::RRCS_STATE_DROGUE:
        KalmanFilterStep(d);
        Log(d);
        if (DualDeployMain()) {
            SetNextState(RRCSState::RRCS_STATE_DONE);
        }
        break;

    case RRCSState::RRCS_STATE_DONE:
        KalmanFilterStep(d);
        logfile_.close();
        SetNextState(RRCSState::RRCS_STATE_CLEANUP);
        std::cout << He_ << std::endl;
        break;

    case RRCSState::RRCS_STATE_CLEANUP:
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

bool RRCSKalmanFilter::IsReady() {
    return true;
}

bool RRCSKalmanFilter::IsBoost() {
    return (Xf_(Xa) > ACCX_LAUNCH_VALUE);
}

bool RRCSKalmanFilter::AccelerationSamples() {
    return (state_count_++ > RRCS_ACCELERATION_SAMPLES / 2);
}

bool RRCSKalmanFilter::IsApogee() {
    // we are at apogee at one of the following conditions:
    // 1) The Barometric pressure starts to increase.  For this, we need
    //    to monitor a sequence of values.

    if ((Xf_(Xp) < (Xp_min_ + 15.0))) {
        return false;
    }

    if ((Xf_(Xp) < Xp_max_) && (++Xp_max_observations_ == RRCS_DESCENT_OBSERVATIONS)) {
        return true;
    }

    return false;
}

bool RRCSKalmanFilter::DeployMain() {
    return (RRCSConfig::GetInstance().GetRrcsMode() == RRCS_MODE_MAIN);
}

bool RRCSKalmanFilter::DualDeployDrogue() {
    return (RRCSConfig::GetInstance().GetRrcsMode() == RRCS_MODE_DUAL);
}

bool RRCSKalmanFilter::DualDeployMain() {
    return (RRCSConfig::GetInstance().GetRrcsMode() == RRCS_MODE_DUAL)
            && (Xf_(Xp) < RRCSConfig::GetInstance().GetDualAltitude());
}

void RRCSKalmanFilter::ProcessData(const RRCSSensorMeasurement& d) {
    JitterStatistics(d);
    if (d.IsAccX()) {
        AccelerationVibrationAnalysis(d);
    }
    UpdateKalmanFilter(d);
    last_ = d.GetTimestamp();
    if ((++Nobs_ % 250) == 0) {
        std::vector<double> x;
        for (int i = 0; i < NSTATES; i++) {
            x.push_back(static_cast<double>(Xf_(i)));
        }
        RRCSState::GetInstance().SetStateVector(x);
    }
}

void RRCSKalmanFilter::Log(const RRCSSensorMeasurement& d) {
    float dT = (float) std::chrono::duration_cast < std::chrono::microseconds
            > (d.GetTimestamp() - last_).count() / 1000000.0;
    elapsed_time_ += dT;
    logfile_ << elapsed_time_ << "," << d.GetTimestamp().time_since_epoch().count() << ","
            << RRCSState::GetInstance().GetRrcsStateStr() << ",";
    for (int i = 0; i < NSENSORS; i++) {
        logfile_ << Y_(i) << ",";
    }
    for (int i = 0; i < NSTATES; i++) {
        logfile_ << Xf_(i) << ",";
    }
    logfile_ << Xp_min_ << Xp_max_ << "," << Xp_max_observations_ << std::endl;
}

} /* namespace rrcs */
