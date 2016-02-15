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

#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <Wt/WColor>
#include <Wt/WFileUpload>
#include <Wt/WProgressBar>
#include <Wt/WPushButton>
#include <Wt/WGridLayout>
#include <Wt/WApplication>

#include "rrcs/rrcs.h"
#include "rrcs/rrcs_config.h"
#include "rrcs/rrcs_state.h"
#include "rrcs/rrcs_version.h"
#include "rrcs/rrcs_dashboard.h"

namespace rrcs {

void RRCSDashboard::SetupVersionPanel() {
	std::string title = "Version " + std::string(RRCS_VERSION) + " [Build: "
			+ std::string(__DATE__) + " " + std::string(__TIME__) + " UTC "
			+ "]";
	title_panel_ = new Wt::WGroupBox(this);
	title_panel_->setTitle("RRCS");
	title_panel_->addWidget(new Wt::WText(title));
}

void RRCSDashboard::SetupOperationalParametersPanel() {
	mode_panel_ = new Wt::WGroupBox(this);
	mode_panel_->setTitle("Operational Parameters");
	mode_cb_ = new Wt::WComboBox(this);
	mode_cb_->addItem(RRCS_MODE_MAIN_STR);
	mode_cb_->addItem(RRCS_MODE_DUAL_STR);
	mode_cb_->addItem(RRCS_MODE_STAGING_STR);
	Wt::WText* mode_tx = new Wt::WText("<b>Mode: </b>");
	mode_panel_->addWidget(mode_tx);
	mode_panel_->addWidget(mode_cb_);
	mode_cb_->changed().connect(std::bind([this]() {
		ModeChange(mode_cb_->currentText());
	}));
	Wt::WText* parameters = new Wt::WText("<b>  Parameters: </b>");
	mode_panel_->addWidget(parameters);
	apogee_string_tx_ = new Wt::WText(RRCS_APOGEE_MAIN_STR);
	mode_panel_->addWidget(apogee_string_tx_);
	//parameters->positionAt(mode_tx, Wt::Vertical);
	deploy_altitude_cb_ = new Wt::WComboBox(this);
	deploy_altitude_cb_->addItem(RRCS_MAIN_DEPLOY_400_STR);
	deploy_altitude_cb_->addItem(RRCS_MAIN_DEPLOY_800_STR);
	deploy_altitude_cb_->changed().connect(std::bind([this]() {
		DeployAltitudeChange(deploy_altitude_cb_->currentText());
	}));
	mode_panel_->addWidget(deploy_altitude_cb_);
}

void RRCSDashboard::SetupRRCSStatusPanel() {
	status_panel_ = new Wt::WGroupBox(this);
	status_panel_->setTitle("Status");
	status_panel_->addWidget(new Wt::WText("<b>  RRCS State: </b>"));
	rrcs_state_tx_ = new Wt::WText(RRCSState::GetInstance().GetRrcsStateStr());
	status_panel_->addWidget(rrcs_state_tx_);
	RRCSState::GetInstance().AddCallback([this]() {
		SetRRCSStateCb();
	});
}

void RRCSDashboard::SetupKalmanFilterPanel() {
	kalman_panel_ = new Wt::WGroupBox(this);
	kalman_panel_->setTitle("Kalman Filter");
	Wt::WGridLayout *grid = new Wt::WGridLayout();
	kalman_panel_->setLayout(grid);
	static const char *labels[] =
			{ "<b>d</b><sub>x</sub> (m)", "<b>v</b><sub>x</sub> (m/s)",
					"<b>a</b><sub>x</sub> (m/s<sup>2</sup>)",
					"<b>d</b><sub>y</sub> (m)", "<b>v</b><sub>y</sub> (m/s)",
					"<b>a</b><sub>y</sub> (m/s<sup>2</sup>)",
					"<b>d</b><sub>z</sub> (m)", "<b>v</b><sub>z</sub> (m/s)",
					"<b>a</b><sub>z</sub> (m/s<sup>2</sup>)",
					"<b>h</b><sub>p</sub> (m)" };

	for (int i = 0; i < 10; i++) {
		grid->addWidget(new Wt::WText(labels[i]), 0, i, Wt::AlignCenter);
		Wt::WText *value = new Wt::WText("0.0");
		grid->addWidget(value, 1, i, Wt::AlignCenter);
		status_vector_tx_.push_back(value);
	}
	RRCSState::GetInstance().AddCallback([this]() {
		SetKalmanFilterCb();
	});

}

void RRCSDashboard::SetupStatisticsPanel() {
	stats_panel_ = new Wt::WGroupBox(this);
	stats_panel_->setTitle("Statistics");
	Wt::WGridLayout *grid = new Wt::WGridLayout();
	stats_panel_->setLayout(grid);

	static const char *labels[] = { "<b>Sensor</b>", "<b>J</b><sub>avg</sub>",
			"<b>J</b><sub>std</sub>", "<b>L</b><sub>avg</sub>",
			"<b>L</b><sub>std</sub>", "<b>N</b>" };

	for (int i = 0; i < 6; i++) {
		grid->addWidget(new Wt::WText(labels[i]), 0, i, Wt::AlignCenter);
	}
	grid->addWidget(new Wt::WText("Acc"), 1, 0, Wt::AlignCenter);
	grid->addWidget(new Wt::WText("Baro"), 2, 0, Wt::AlignCenter);

	for (int i = 1; i < 6; i++) {
		Wt::WText *value = new Wt::WText("0.0");
		grid->addWidget(value, 1, i, Wt::AlignCenter);
		stats_vector_tx_.push_back(value);
	}
	for (int i = 1; i < 6; i++) {
		Wt::WText *value = new Wt::WText("-0.0");
		grid->addWidget(value, 2, i, Wt::AlignCenter);
		stats_vector_tx_.push_back(value);
	}
	RRCSState::GetInstance().AddCallback([this]() {
		SetSensorStatsCb();
	});

}

void RRCSDashboard::SetupAccelerationVibrationPanel() {
	vibration_panel_ = new Wt::WGroupBox(this);
	vibration_panel_->setTitle("Vibration");
	Wt::WGridLayout *grid = new Wt::WGridLayout();
	vibration_panel_->setLayout(grid);

	static const char *labels[] = { "<b>f</b><sub>0</sub> ",
			"<b>f</b><sub>1</sub> ", "<b>f</b><sub>2</sub> ",
			"<b>f</b><sub>3</sub>", "<b>f</b><sub>4</sub>" };

	for (int i = 0; i < 5; i++) {
		grid->addWidget(new Wt::WText(labels[i]), 0, i, Wt::AlignCenter);
	}
	for (int i = 0; i < 5; i++) {
		Wt::WText *value = new Wt::WText("--");
		grid->addWidget(value, 1, i, Wt::AlignCenter);
		vibration_vector_tx_.push_back(value);
	}
	RRCSState::GetInstance().AddCallback([this]() {
		SetVibrationCb();
	});

}

RRCSDashboard::RRCSDashboard(Wt::WContainerWidget *parent) :
		WContainerWidget(parent) {
	std::string mode;
	std::string altitude;

	RRCSConfig::GetInstance().ReadConfig(mode, altitude);
	SetModeStr(mode);
	SetDeployAltitudeStr(altitude);
	setContentAlignment(Wt::AlignLeft);

	SetupVersionPanel();
	SetupOperationalParametersPanel();
	SetupRRCSStatusPanel();
	SetupKalmanFilterPanel();
	SetupAccelerationVibrationPanel();
	SetupStatisticsPanel();

	SetDeployAltitudeCbIndex();
	SetModeCbIndex();
	SetRRCSStateCb();

	ModeChange(GetModeStr());

}
RRCSDashboard::~RRCSDashboard() {
// TODO Auto-generated destructor stub
}

void RRCSDashboard::ModeChange(const Wt::WString mode_str) {
	if (mode_str == RRCS_MODE_MAIN_STR) {
		deploy_altitude_cb_->hide();
		apogee_string_tx_->setText(RRCS_APOGEE_MAIN_STR);
	} else if (mode_str == RRCS_MODE_DUAL_STR) {
		deploy_altitude_cb_->show();
		apogee_string_tx_->setText(RRCS_APOGEE_DROGUE_STR);
	} else if (mode_str == RRCS_MODE_STAGING_STR) {
		deploy_altitude_cb_->hide();
		apogee_string_tx_->setText(RRCS_APOGEE_MAIN_STR);
	}
	SetModeStr(mode_str.narrow());
	RRCSConfig::GetInstance().WriteConfig(GetModeStr(), GetDeployAltitudeStr());
}

void RRCSDashboard::DeployAltitudeChange(const Wt::WString mode_str) {
	SetDeployAltitudeStr(mode_str.narrow());
	RRCSConfig::GetInstance().WriteConfig(GetModeStr(), GetDeployAltitudeStr());
}

void RRCSDashboard::SetModeCbIndex() {
	if (GetModeStr() == RRCS_MODE_MAIN_STR) {
		mode_cb_->setCurrentIndex(0);
	} else if (GetModeStr() == RRCS_MODE_DUAL_STR) {
		mode_cb_->setCurrentIndex(1);
	} else if (GetModeStr() == RRCS_MODE_STAGING_STR) {
		mode_cb_->setCurrentIndex(2);
	}
}

void RRCSDashboard::SetDeployAltitudeCbIndex() {
	if (GetDeployAltitudeStr() == RRCS_MAIN_DEPLOY_400_STR) {
		deploy_altitude_cb_->setCurrentIndex(0);
	} else if (GetDeployAltitudeStr() == RRCS_MAIN_DEPLOY_800_STR) {
		deploy_altitude_cb_->setCurrentIndex(1);
	}
}

void RRCSDashboard::SetKalmanFilterCb() {
	std::vector<double> state = RRCSState::GetInstance().GetStateVector();
	int i = 0;
	for (auto t : status_vector_tx_) {
		std::stringstream ss;
		ss << state[i];
		t->setText(ss.str());
		i++;
	}
}

void RRCSDashboard::SetRRCSStateCb() {
	rrcs_state_tx_->setText(RRCSState::GetInstance().GetRrcsStateStr());
}

void RRCSDashboard::SetSensorStatsCb() {
	std::vector<double> acc = RRCSState::GetInstance().GetXJitter();
	std::vector<double> baro = RRCSState::GetInstance().GetPJitter();
	std::vector<double> lacc = RRCSState::GetInstance().GetXLatency();
	std::vector<double> lbaro = RRCSState::GetInstance().GetPLatency();

	std::vector<double> data { acc[0], acc[1], lacc[0], lacc[1], acc[2],
			baro[0], baro[1], lbaro[0], lbaro[1], baro[2] };
	std::size_t i = 0;
	for (auto t : stats_vector_tx_) {
		t->setText(std::to_string(data[i]));
		i++;
	}
}

void RRCSDashboard::SetVibrationCb() {
	std::vector<int> vib = RRCSState::GetInstance().GetVibrationVector();
	std::size_t i = 0;
	for (auto t : vibration_vector_tx_) {
		if (i < vib.size()) {
			t->setText(std::to_string(vib[i]));
		} else {
			t->setText("--");
		}
		i++;
	}
}

} /* namespace rrcs */

