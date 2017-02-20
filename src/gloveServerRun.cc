/**

Copyright (c) 2016, Heller Florian, Meißner Pascal, Nguyen Trung, Yi Xie
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/* system includes */
#include <stdio.h>
#define BOOST_FILESYSTEM_VERSION 2
#include <boost/program_options.hpp>
#include <iostream>
#include <signal.h>

/* my includes */
#include "gloveServer_impl.h"

/* ros */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

/*
 \brief The program's main function.


 */

const static char* DEFAULT_CALIBRATION_LEFT = "cfg/GloveCalibrationRight.cal";
const static char* DEFAULT_CALIBRATION_RIGHT = "cfg/GloveCalibrationRight.cal";
const static char* DEFAULT_PUBLISH_NAME_RIGHT = "GloveRight";
const static char* DEFAULT_PUBLISH_NAME_LEFT = "GloveLeft";
const static char* DEFAULT_TTY_RIGHT = "/dev/ttyUSB0";
const static char* DEFAULT_TTY_LEFT = "/dev/ttyUSB1";

namespace po = boost::program_options;
using namespace std;
unsigned int debugLevel;

po::variables_map vm;
gloveServer_impl *gloveRight, *gloveLeft;
bool do_loop;

gloveServer_impl *
startGlove(gloveID id, std::string serialPort, std::string gloveName, std::string calibFileName) {
	// create glove
	gloveServer_impl * glove = new gloveServer_impl(id, gloveName);
	if (glove->init(serialPort)) {
		printf("glove %s started\n", (id == rightGlove) ? "right" : "left");
		if (id == rightGlove) {
			gloveRight = glove;
		} else {
			gloveLeft = glove;
		}
	}

	if (!glove->loadCalibFile(calibFileName.c_str())) {
		printf("Error while loading calibration. Bailing out!\n");
		exit(-1);
	} else
		printf("Calibration loaded successfully.\n");

	// start glove
//	glove->start();

//! start thread
	if (!glove->threadRunning) {
		glove->threadRunning = true;
		glove->boostWorkerThread = new boost::thread(&gloveServer_impl::workerThread, glove);
	} else
		printf("Thread started, but was already running.\n");
	printf("Thread running.\n");

	return glove;
}

int main(int argc, char **argv) {
	std::string ttyright;
	std::string ttyleft;
	std::string calibfileLeft;
	std::string calibfileRight;
	std::string pubNameRight = DEFAULT_PUBLISH_NAME_RIGHT;
	std::string pubNameLeft;

	po::options_description desc("Usage : cyberGloveServer [options]", 120);
	desc.add_options()("help,h", "show help screen")("glove-right,r", "use right glove")("glove-left,l",
			"use left glove")("calibration-file-left",
			po::value<std::string>(&calibfileLeft)->default_value(DEFAULT_CALIBRATION_LEFT),
			"left glove calibration file")("calibration-file-right",
			po::value<std::string>(&calibfileRight)->default_value(DEFAULT_CALIBRATION_RIGHT),
			"right glove calibration file")("tty-right",
			po::value<std::string>(&ttyright)->default_value(DEFAULT_TTY_RIGHT), "right glove tty (e.g. /dev/ttyS0")(
			"tty-left", po::value<std::string>(&ttyleft)->default_value(DEFAULT_TTY_LEFT),
			"left glove tty (e.g. /dev/ttyS1")("register-as-right",
			po::value<std::string>(&pubNameRight)->default_value(DEFAULT_PUBLISH_NAME_RIGHT), "right glove Name")(
			"register-as-left", po::value<std::string>(&pubNameLeft)->default_value(DEFAULT_PUBLISH_NAME_LEFT),
			"left glove Name")("debug-level,d", po::value<unsigned int>(&debugLevel)->default_value(0),
			"the more, the higher ;)");

	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 0;
	}

	ros::init(argc, argv, "gloveServer_node");

	if (!vm.count("glove-right") && !vm.count("glove-left")) {
		printf("No glove selected! Exit.\n");
		exit(0);
	}

	// start glove right
	if (vm.count("glove-right")) {
		printf("starting right glove on %s\n", ttyright.c_str());
		startGlove(rightGlove, ttyright, pubNameRight, calibfileRight);
	}
	// start glove left
	if (vm.count("glove-left")) {
		printf("starting left glove on %s\n", ttyleft.c_str());
		startGlove(leftGlove, ttyleft, pubNameLeft, calibfileLeft);
	}

	// start corba replacement loop
	do_loop = true;
	std::string calibFile;
	while (do_loop & ros::ok()) {
		sleep(1);
	}

	// delete objects
	if (vm.count("glove-right")) {
		gloveRight->stop();
		gloveRight->join();
		delete gloveRight;
	}
	if (vm.count("glove-left")) {
		gloveLeft->stop();
		gloveLeft->join();
		delete gloveLeft;
	}
	ROS_INFO("%s", "gloveServer exited normally. Gloves deleted.\n");

	return 0;
}

