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

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include "rrcs/rrcs.h"
#include "rrcs/rrcs_config.h"

namespace rrcs {

RRCSConfig RRCSConfig::instance_;

RRCSConfig::RRCSConfig() {
    // TODO Auto-generated constructor stub

}

RRCSConfig::~RRCSConfig() {
    // TODO Auto-generated destructor stub
}

void RRCSConfig::ReadConfig(std::string& mode, std::string& altitude) {
    std::unique_lock<std::mutex> lock(lock_);
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini("config.ini", pt);
    mode = pt.get("operational.mode", RRCS_MODE_MAIN_STR);
    altitude = pt.get("operational.parameters", RRCS_MAIN_DEPLOY_800_STR);
}

void RRCSConfig::WriteConfig(const std::string mode,
        const std::string altitude) {
    std::unique_lock<std::mutex> lock(lock_);
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini("config.ini", pt);
    pt.put("operational.mode", mode);
    pt.put("operational.parameters", altitude);
    boost::property_tree::ini_parser::write_ini("config.ini", pt);
}

std::string RRCSConfig::GetNextFileName() {
    std::unique_lock<std::mutex> lock(lock_);
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini("config.ini", pt);
    int index = pt.get("logging.index", 0);
    pt.put("logging.index", index+1);
    std::string prefix = pt.get("logging.prefix", "rrcs");
    boost::property_tree::ini_parser::write_ini("config.ini", pt);

    return prefix + std::to_string(index) + ".csv";
}


} /* namespace rrcs */
