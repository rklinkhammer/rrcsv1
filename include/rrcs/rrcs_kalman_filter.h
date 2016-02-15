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

#ifndef RRCS_KALMAN_FILTER_H_
#define RRCS_KALMAN_FILTER_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "util/threaded_queue.h"
#include "rrcs/rrcs.h"
#include "rrcs/rrcs_sensor_measurement.h"

extern "C" {
void rdft(int n, int isgn, double *a, int *ip, double *w);
}

namespace rrcs {

class RRCSKalmanFilter {
public:

    static constexpr const int NSTATES = 10;
    static constexpr const int NSENSORS = 4;
    static constexpr const int CALIBRATION_SAMPLES = 250;
    static constexpr const float ACCX_LAUNCH_VALUE = (9.81 * 2);

    typedef Eigen::Matrix<float, NSTATES, 1> StateVector;
    typedef Eigen::Matrix<float, NSENSORS, 1> MeasurementVector;
    typedef Eigen::Matrix<float, NSTATES, NSTATES> TransitionMatrix;
    typedef Eigen::Matrix<float, NSENSORS, CALIBRATION_SAMPLES> MeasurementObservations;
    typedef Eigen::Matrix<float, NSENSORS, NSENSORS> MeasurementCovarianceMatrix;
    typedef Eigen::Matrix<float, NSENSORS, NSTATES> MeasurementMatrix;
    typedef Eigen::Matrix<float, NSTATES, NSENSORS> GainMatrix;

    RRCSKalmanFilter(std::function<bool()> abort);
    virtual ~RRCSKalmanFilter();
    virtual void Init();
    virtual void Run();

    std::function<bool(RRCSSensorMeasurement& measurement)> GetUpdateFunction() {
        return [this](RRCSSensorMeasurement& d) -> bool {return measurements_.Enqueue(d);};
    }

    void Join() {
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:

    void ProcessData(const RRCSSensorMeasurement& d);
    void JitterStatistics(const RRCSSensorMeasurement& d);
    void AccelerationVibrationAnalysis(const RRCSSensorMeasurement& d);
    bool AccelerationDoubleBuffer(
            const RRCSSensorMeasurement& d, double A[], double B[]);
    void AccelerationFFT(double A[]);

    void InitKalmanFilter();
    void UpdateKalmanFilter(const RRCSSensorMeasurement& d);
    void PrecalibrationState(const RRCSSensorMeasurement& data);
    void KalmanGainState(const RRCSSensorMeasurement& data);
    void EstimationStep();
    void CorrectionStep(const RRCSSensorMeasurement& d);
    void KalmanFilterStep(const RRCSSensorMeasurement& data);
    void SetNextStateDeltaT(float deltaT);

    std::chrono::high_resolution_clock::time_point last_acc_;
    uint32_t acc_observations_ { 0 };
    std::chrono::high_resolution_clock::time_point last_baro_;
    uint32_t baro_observations_ { 0 };

    static const int ACC_FFT_SIZE = 64; // next highest power of two.

    // Acceleration Vibration Analysis

    double acc_A_[RRCS_ACCELERATION_SAMPLES];
    double acc_B_[RRCS_ACCELERATION_SAMPLES];
    bool is_acc_A_ { true };
    int acc_count_ { 0 };
    int ip_[ACC_FFT_SIZE * 2];
    double w_[ACC_FFT_SIZE * 2]; //
    double a_[ACC_FFT_SIZE * 2];

    // Kalman State Vector
    float Basep_ {0.0} ; // Base Pressure Offset
    StateVector Xe_;    // X*n+1,n
    StateVector Xf_;    // X*n,n
    StateVector Xp_;    // X*n,n-1

    // Kalman Transition Matrix
    TransitionMatrix A_;
    // Kalman Transition Matrix
    StateVector U_;
    // Kalman Measurement Vector
    MeasurementVector Y_;
    MeasurementCovarianceMatrix R_;
    MeasurementVector Yavg_;
    MeasurementObservations Yobs_;
    int Ysamples_{0};

    // Kalman Measurement Mapping Matrix
    MeasurementMatrix M_;   // all

    // Kalman Gain
    GainMatrix H_;
    GainMatrix Hp_;
    GainMatrix Hx_;
    GainMatrix Hyz_;
    GainMatrix Hxyz_;

    // Delta T
    std::chrono::high_resolution_clock::time_point last_;

    ThreadedQueue<RRCSSensorMeasurement> measurements_;
    std::function<bool()> abort_;
    std::thread thread_;

    int Nobs_ {0} ;
};

} /* namespace rrcs */

#endif /* RRCS_KALMAN_FILTER_H_ */
