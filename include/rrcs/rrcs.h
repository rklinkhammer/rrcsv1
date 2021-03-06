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

#ifndef RRCS_H_
#define RRCS_H_

namespace rrcs {

static constexpr const char *RRCS_MODE_MAIN_STR = "Main";
static constexpr const char *RRCS_MODE_DUAL_STR = "Dual";
static constexpr const char *RRCS_MODE_STAGING_STR = "Staging";

static constexpr const char *RRCS_APOGEE_MAIN_STR = " main @ apogee ";
static constexpr const char *RRCS_APOGEE_DROGUE_STR = " drogue @ apogee ";

static constexpr const char *RRCS_MAIN_DEPLOY_150_STR = "+ main @ 150 m";
static constexpr const char *RRCS_MAIN_DEPLOY_300_STR = "+ main @ 300 m";

static constexpr const int RRCS_ACCELERATION_PERIOD_MS = 20;
static constexpr const int RRCS_BAROMETER_PERIOD_MS = 40;
static constexpr const int RRCS_DEFAULT_PERIOD_MS = RRCS_BAROMETER_PERIOD_MS;
static constexpr const int RRCS_ACCELERATION_SAMPLES = 1000/RRCS_ACCELERATION_PERIOD_MS;
static constexpr const int RRCS_BAROMETER_SAMPLES = 1000/RRCS_BAROMETER_PERIOD_MS;
static constexpr const int RRCS_DESCENT_OBSERVATIONS = RRCS_BAROMETER_SAMPLES;

enum RRCSMode {
	RRCS_MODE_MAIN,
	RRCS_MODE_DUAL,
	RRCS_MODE_STAGING
};

} /* namespace rrcs */


#endif /* RRCS_H_ */
