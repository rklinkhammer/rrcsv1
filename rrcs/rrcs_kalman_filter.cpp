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
	logfile_.open(RRCSConfig::GetInstance().GetFilename(),
			std::fstream::out | std::fstream::trunc);
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
			RRCSState::GetInstance().UpdateXJitter(
					(current - last_acc_).count());
			RRCSState::GetInstance().UpdateXLatency(
					(d.GetTimestamp() - last_acc_).count());
		}
		last_acc_ = current;
		acc_observations_++;
	}
	if (d.IsPressure()) {
		if (baro_observations_) {
			RRCSState::GetInstance().UpdatePJitter(
					(current - last_baro_).count());
			RRCSState::GetInstance().UpdatePLatency(
					(d.GetTimestamp() - last_baro_).count());
		}
		last_baro_ = current;
		baro_observations_++;
	}
}

bool RRCSKalmanFilter::AccelerationDoubleBuffer(const RRCSSensorMeasurement& d,
		double A[], double B[]) {
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

void RRCSKalmanFilter::AccelerationVibrationAnalysis(
		const RRCSSensorMeasurement& d) {
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
	Xp_ = StateVector::Zero(); // State Vector
	A_ = TransitionMatrix::Identity(); // Transition Next State Matrix
	SetNextStateDeltaT(0.04);

	U_ = StateVector::Zero(); 
	R_ = MeasurementCovarianceMatrix::Zero();

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

	Hxyz_ = GainMatrix::Zero();
	Hxyz_(2, 0) = 0.018; // AccX
	Hxyz_(5, 1) = 0.018; // AccY
	Hxyz_(8, 2) = 0.018; // AccZ

	Pe_ = GainMatrix::Zero();
	Pf_ = GainMatrix::Zero();

}

void RRCSKalmanFilter::PrecalibrationState(const RRCSSensorMeasurement& d) {
	// collect statistics
	if (d.IsPressure()) {
		RRCSState::GetInstance().UpdatePCal(d.GetPressure());
	}
	if (d.IsAccX()) {
		RRCSState::GetInstance().UpdateXCal(d.GetAccX(), d.GetAccY(),
				d.GetAccZ());
	}
}

void RRCSKalmanFilter::KalmanGainState(const RRCSSensorMeasurement& d) {
}

void RRCSKalmanFilter::EstimationStep() {
	/* State Estimation */
	// X*n+1,n = AX*n,n + Un,n
	Xe_ = A_ * Xf_ + U_;
	// P*n+1,n =  H Pk,k H^t + Q_
	Pe_ = H_ * Pf_ * H_.transpose() + Q_;
	// H = (P*n+1,n M.transpose())(M * P*n+1,n * M.transpose() + R)^-1
	GainMatrix tmp = M_ * Pe_ * M_.transpose() + R_;
	H_ = (Pe_ * M_.transpose()) * tmp.inverse(); 
	Hp_(9,3) = H_(9,3);
	Hxyz_(2,0) = H_(2,0);
	Hxyz_(5,1) = H_(5,1);
	Hxyz_(8,2) = H_(8,2);
}

void RRCSKalmanFilter::CorrectionStep(const RRCSSensorMeasurement& d) {
	// X*n,n= X*n,n-1 + H(Y-MX*n,n-1)
	Y_ = MeasurementVector::Zero();
	if (d.IsPressure()) {
		Y_(3) = d.GetPressure() - Xp_min_;
		Xf_ = Xe_ + Hp_ * (Y_ - M_ * Xe_);
	}
	if (RRCSState::GetInstance().GetState() > RRCSState::RRCS_STATE_READY) {
		if (d.IsAccX() && d.IsAccYZ()) {
			Y_(0) = d.GetAccX();
			Y_(1) = d.GetAccY();
			Y_(2) = d.GetAccZ();
			Xf_ = Xe_ + Hxyz_ * (Y_ - M_ * Xe_);
		}
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

void RRCSKalmanFilter::SetCalibrationValues() {
	std::vector<double> x = RRCSState::GetInstance().GetXCal();
	std::vector<double> y = RRCSState::GetInstance().GetYCal();
	std::vector<double> z = RRCSState::GetInstance().GetZCal();
	std::vector<double> p = RRCSState::GetInstance().GetPCal();
	Xp_min_ = p[0];
	Xf_(Xp) = p[0];
	Q_(Xa,Xa) = x[1];
	Q_(Ya,Ya) = y[1];
	Q_(Za, Za) = z[1];
	Q_(Xp, Xp) = p[1];
}

void RRCSKalmanFilter::UpdateKalmanFilter(const RRCSSensorMeasurement& d) {
	switch (RRCSState::GetInstance().GetState()) {
	case RRCSState::RRCS_STATE_INIT:
		PrecalibrationState(d);
		if (RRCSState::GetInstance().IsCalibrated()) {
			RRCSState::GetInstance().SetState(RRCSState::RRCS_STATE_CALIBRATED);
			std::cout << RRCSState::GetInstance().GetRrcsStateStr() << std::endl;
			SetCalibrationValues();
		}
		break;

	case RRCSState::RRCS_STATE_CALIBRATED:
		KalmanGainState(d);
		if (IsReady()) {
			RRCSState::GetInstance().SetState(RRCSState::RRCS_STATE_READY);
			std::cout << RRCSState::GetInstance().GetRrcsStateStr() << std::endl;
		}
		break;

	case RRCSState::RRCS_STATE_READY:
		KalmanFilterStep(d);
		Log(d);
		if (IsBoost()) {
			RRCSState::GetInstance().SetState(RRCSState::RRCS_STATE_BOOST);
			std::cout << RRCSState::GetInstance().GetRrcsStateStr() << std::endl;
		}
		break;

	case RRCSState::RRCS_STATE_BOOST:
		KalmanFilterStep(d);
		Log(d);
		if (IsCoast()) {
			RRCSState::GetInstance().SetState(RRCSState::RRCS_STATE_COAST);
			std::cout << RRCSState::GetInstance().GetRrcsStateStr() << std::endl;
		}
		break;

	case RRCSState::RRCS_STATE_COAST:
		KalmanFilterStep(d);
		Log(d);
		if (IsApogee()) {
			RRCSState::GetInstance().SetState(RRCSState::RRCS_STATE_APOGEE);
			std::cout << RRCSState::GetInstance().GetRrcsStateStr() << std::endl;
		}
		break;

	case RRCSState::RRCS_STATE_APOGEE:
		KalmanFilterStep(d);
		Log(d);
		if (DualDeployDrogue()) {
			RRCSState::GetInstance().SetState(RRCSState::RRCS_STATE_DROGUE);
			std::cout << RRCSState::GetInstance().GetRrcsStateStr() << std::endl;
		} else if (DeployMain()) {
			RRCSState::GetInstance().SetState(RRCSState::RRCS_STATE_DONE);
			std::cout << RRCSState::GetInstance().GetRrcsStateStr() << std::endl;
		}
		break;

	case RRCSState::RRCS_STATE_DROGUE:
		KalmanFilterStep(d);
		Log(d);
		if (DualDeployMain()) {
			RRCSState::GetInstance().SetState(RRCSState::RRCS_STATE_DONE);
			std::cout << RRCSState::GetInstance().GetRrcsStateStr() << std::endl;
		}
		break;

	case RRCSState::RRCS_STATE_DONE:
		logfile_.close();
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

bool RRCSKalmanFilter::IsCoast() {
	// we are coasting if the X acceleration goes negative.
	return (Xf_(Xa) < 0.0);
}

bool RRCSKalmanFilter::IsApogee() {
	// we are at apogee at one of the following conditions:
	// 1) the acceleration becomes positive, indicating that we
	//    have arced over.
	// 2) The velocity becomes close to zero (but, this is not a guarantee
	//    since we could have ballistic trajectory where this is not
	//    true
	// 3) The Barometric pressure starts to increase.  For this, we need
	//    to monitor a sequence of values.

	if (Xf_(Xa) > 0.0) {
		return true;
	}
	if (fabs(Xf_(Xv)) < VELOCITY_APOGEE_VALUE) {
		return true;
	}
	if ((Xf_(Xp) < Xp_min_)
			&& (++Xp_max_observations_ == RRCS_DESCENT_OBSERVATIONS)) {
		return true;
	} else {
		Xp_max_observations_ = 0;
	}
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
	logfile_ << d.GetTimestamp().time_since_epoch().count() << ","
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
