/**

Copyright (c) 2016, Heller Florian, Meißner Pascal, Nguyen Trung, Yi Xie
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef GLOVESERVER_H
#define GLOVESERVER_H

/* system includes */
#include <string>
#include <vector>
#include <boost/thread.hpp>

/* ros */
#include <ros/ros.h>
#include <sstream>

#include <asr_msgs/AsrGlove.h>

/* my includes */
#include "gloveDevice.h"

#define DIEonERR(value) if (value<0) { \
 fprintf(stderr,"%i DIED in %s line %i with error %i\n",getpid(),__FILE__,__LINE__,-value);exit(1); }

#define DEFAULT_CALIBRATION_PATH "../calibrationData/"

typedef enum
{
	rightGlove = 1, leftGlove
} gloveID;
typedef enum
{
	VALUE, RADIAN
} noteType;

class gloveServer_impl
{
public:
	gloveServer_impl(gloveID id, std::string name);
	bool init(std::string serialPort);
	bool loadCalibFile(const char *srcFileName);
	~gloveServer_impl();

	void start();
	void stop();
	void join();

	//! flag switching on/off thread
	bool threadRunning;
	//! this method is called by static method s_workerthread
	void workerThread();
	//! Thread for delivering tracker data to nCenter
	boost::thread * boostWorkerThread;

private:

	gloveID myID;
	std::string myName;
	char* serialID;
	gloveDevice *gloveDev;
	unsigned int GLOVERAW_MAP2_GLOVEINVENTOR[22];

	/* Ros */
	ros::NodeHandle rosNode;
	ros::Publisher gloveDataPub;
	ros::Publisher gloveDataPub_radian;

	//! Calibration data. These 2 vector-pairs define 2 fulcrums for
	//! interpolation. [rawX] = digits, [radX] = radiant
	std::vector<double> FulcRaw1;
	std::vector<double> FulcRad1;

	std::vector<double> FulcRaw2;
	std::vector<double> FulcRad2;

	double getRadian(int index, unsigned char val);
	double *getRadianArray(unsigned char* data, unsigned int size);

	//! permute sensor values according to GLOVERAW_MAP2_GLOVEINVENTOR mapping
	bool permuteValues(unsigned char* valIn, unsigned char* valOut);

	// ----- Notification Thread ----- //
	//! Send sensor data to Notification Center
    bool sendNotificationDataToRos(double* data, unsigned int size, asr_msgs::AsrGlove gloveData, ros::Publisher publisher);

};

#endif /* GLOVESERVER_H */
