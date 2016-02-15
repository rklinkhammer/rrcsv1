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

/*
 * rrcs_widget.h
 *
 *  Created on: Feb 5, 2016
 *      Author: dev
 */

#ifndef RRCS_DASHBOARD_H_
#define RRCS_DASHBOARD_H_

#include <thread>
#include <Wt/WContainerWidget>
#include <Wt/WPanel>
#include <Wt/WGroupBox>
#include <Wt/WComboBox>
#include <Wt/WText>

namespace rrcs {

class RRCSDashboard : public Wt::WContainerWidget {
public:
    RRCSDashboard(Wt::WContainerWidget *parent = nullptr);
    virtual ~RRCSDashboard();

    const std::string& GetDeployAltitudeStr() const {
        return deploy_altitude_str_;
    }

    void SetDeployAltitudeStr(const std::string& deploy_altitude_str) {
        deploy_altitude_str_ = deploy_altitude_str;
    }

    const std::string& GetModeStr() const {
        return mode_str_;
    }

    void SetModeStr(const std::string& mode_str) {
        mode_str_ = mode_str;
    }


private:
    void ModeChange(const Wt::WString mode_str);
    void DeployAltitudeChange(const Wt::WString mode_str);
    void SetDeployAltitudeCbIndex();
    void SetModeCbIndex();
    void SetRRCSStateCb();
    void SetKalmanFilterCb();
    void SetSensorStatsCb();
    void SetVibrationCb();

    void SetupVersionPanel();
    void SetupOperationalParametersPanel();
    void SetupRRCSStatusPanel();
    void SetupKalmanFilterPanel();
    void SetupStatisticsPanel();
	void SetupAccelerationVibrationPanel();

    Wt::WGroupBox   *title_panel_;
    Wt::WGroupBox   *mode_panel_;
    Wt::WGroupBox   *status_panel_;
    Wt::WGroupBox   *kalman_panel_;
    Wt::WGroupBox   *stats_panel_;
    Wt::WGroupBox   *vibration_panel_;

    Wt::WComboBox   *mode_cb_;
    Wt::WComboBox   *deploy_altitude_cb_;
    Wt::WText       *apogee_string_tx_;
    Wt::WText       *rrcs_state_tx_;
    std::vector<Wt::WText *> status_vector_tx_;
    std::vector<Wt::WText *> stats_vector_tx_;
    std::vector<Wt::WText *> vibration_vector_tx_;

    std::string     mode_str_;
    std::string     deploy_altitude_str_;
};

} /* namespace rrcs */

#endif /* RRCS_DASHBOARD_H_ */
