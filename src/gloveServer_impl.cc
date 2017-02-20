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
#include <sys/time.h>
#include <math.h>
#include <time.h>
#include <vector>
#include <algorithm>

/* ros */
#include <ros/ros.h>

#include <sstream>

/* my includes */
#include "gloveConstants.h"
#include "gloveServer_impl.h"

extern unsigned int debugLevel;

gloveServer_impl::gloveServer_impl(gloveID id, std::string name) {

	printf("Assuming %lu sensors.\n", sizeof(GLOVERAW_MAP2_GLOVEINVENTOR) / sizeof(unsigned int));
	for (unsigned int i = 0; i < sizeof(GLOVERAW_MAP2_GLOVEINVENTOR) / sizeof(unsigned int); i++)
		GLOVERAW_MAP2_GLOVEINVENTOR[i] = GLOVERAW_MAP2_GLOVEINVENTOR_DEFAULT[i];

	myID = id;
	myName = name;
	threadRunning = false;
}
//
gloveServer_impl::~gloveServer_impl() {
	delete gloveDev;
	threadRunning = false;
}

bool gloveServer_impl::init(std::string serialPort) {
	/* initial ros node */
	std::string pubName = (myID == rightGlove) ? "rightGloveData" : "leftGloveData";
    gloveDataPub = rosNode.advertise<asr_msgs::AsrGlove>(pubName, 1000);
    gloveDataPub_radian = rosNode.advertise<asr_msgs::AsrGlove>(pubName+"_randian", 1000);

	/* start gloveDevice */
	gloveDev = new gloveDevice;
	if (!gloveDev->init(serialPort.c_str())) {
		printf("Could not start GloveDevice... gloveDev->init(%s) failed. Bailing out\n", serialPort.c_str());
		return false;
	}
	gloveDev->showCurrentStatus();

	return true;
}

bool gloveServer_impl::loadCalibFile(const char * srcFileName) {
	printf("Loading calibration file \"%s\"\n", srcFileName);

	FILE *f;

	// open file
	if ((f = fopen(srcFileName, "r")) == NULL) {
		printf("ERROR loading calibration file - open file error!\n");

		std::string tmp(std::string(DEFAULT_CALIBRATION_PATH) + srcFileName);
		printf("Trying to load calibration file \"%s\"\n", tmp.c_str());
		if ((f = fopen(tmp.c_str(), "r")) == NULL) {
			printf("Could not open file.\n");
			return false;
		}
	}
	printf("Successfully opened file.\n");

	//! remember thread state to restart it after loading in case it was running
	bool threadState = threadRunning;

	if (threadState) {
		printf("Stopping server thread.\n");
		this->stop();
		usleep(65000);
	}

	//! alte Kalibrierung loeschen
	FulcRaw1.clear();
	FulcRad1.clear();
	FulcRaw2.clear();
	FulcRad2.clear();

	double tmpraw1, tmprad1, tmpraw2, tmprad2;
	// read calibration data from file
	for (int i = 0; i < MAX_SENSORS; i++) {
		if (!fscanf(f, "%lf:%lf %lf:%lf\n", &tmprad1, &tmpraw1, &tmprad2, &tmpraw2)) {
			printf("load calibration file - error: read from file !\n");
			fclose(f);
			return false;
		}
		//! check consistency
		if (fabs(tmpraw2 - tmpraw1) < MIN_JOINT_RESOLUTION) {

			printf("ERROR in calibration data!\nJoint %s does not achieve minimum resolution.\n", JOINT_NAME[i]);
			printf("Joint resolution: %d Minimum resolution: %d\n\n", (int) fabs(tmpraw2 - tmpraw1),
					MIN_JOINT_RESOLUTION);

			if (fabs(tmpraw2 - tmpraw1) < 2) {

				printf("Bailing out...");
				fclose(f);
				exit(-1);
			}
		}

		FulcRaw1.push_back(tmpraw1);
		FulcRad1.push_back(tmprad1);
		FulcRaw2.push_back(tmpraw2);
		FulcRad2.push_back(tmprad2);
	}
	fclose(f);

	printf("load calibration file successful !\n");

	if (threadState) {
		printf("Starting server thread again.\n");
		this->start();
	}

	return true;
}

void gloveServer_impl::start() {
		threadRunning = true;
}

void gloveServer_impl::stop() {
	//! stop thread
	threadRunning = false;
}

void gloveServer_impl::join() {
	boostWorkerThread->join();
}

#define MAX_SETS 3
#define PAUSE_SET 10000
#define PAUSE_WRITE 10000

unsigned char getMedian(unsigned char (&vAcc)[MAX_SETS][MAX_SENSORS], unsigned int index) {
	if (MAX_SETS == 1)
		return vAcc[0][index];

	std::vector<unsigned int> data;
	for (unsigned int i = 0; i < MAX_SETS; i++)
		data.push_back((unsigned int) vAcc[i][index]);
	std::sort(data.begin(), data.end());

	if (MAX_SETS % 2 == 0)
		return (unsigned char) ((data[MAX_SETS / 2 - 1] + data[MAX_SETS / 2]) / 2);
	else
		return (unsigned char) data[(MAX_SETS - 1) / 2];
}

void gloveServer_impl::workerThread() {

	unsigned char v[MAX_SENSORS], n, v_perm[MAX_SENSORS];
	unsigned char vAcc[MAX_SETS][MAX_SENSORS];
	statusQuery_t queryStatus;
	gloveStatusByte_t gloveStatus;

    asr_msgs::AsrGlove gloveData;
    asr_msgs::AsrGlove gloveData_radian;
	gloveData.id = (myID == leftGlove) ? "L" : "R";
	gloveData_radian.id = (myID == leftGlove) ? "L" : "R";

	while (threadRunning & ros::ok()) {

		//! get data from gloveDevice
		for (unsigned int i = 0; i < MAX_SETS; i++) {
	  		gloveDev->cmdGetSensorValues(vAcc[i], &n, &queryStatus, &gloveStatus);
//			gloveDev->cmdGetSensorDummyValues(vAcc[i], &n, &queryStatus, &gloveStatus);

			if (debugLevel > 2) {
				printf("vAcc: ");
				for (unsigned int j = 0; j < MAX_SENSORS; j++)
					printf("%3d ", vAcc[i][j]);
				printf("\n");
			}

			if (i + 1 < MAX_SETS)
				usleep(PAUSE_SET);
			else if (debugLevel > 2)
				printf("\n");
		}

		for (unsigned int i = 0; i < MAX_SENSORS; i++) {
			v[i] = getMedian(vAcc, i);
		}

		if (debugLevel > 2) {
			printf("v: ");
			for (unsigned int j = 0; j < MAX_SENSORS; j++)
				printf("%3d ", v[j]);

			printf("\n");
		}

		//! permutation according to inventor model
		if (!permuteValues(v, v_perm)) {

			printf("Could not permute values. Bailing out.\n");
		}

		time_t now;
		time(&now);
		if (debugLevel > 2)
			printf("%s", ctime(&now));

		if (debugLevel > 0) {
			printf("v_perm %s:", myID == leftGlove ? "L" : "R");
			for (unsigned int i = 0; i < n; i++)
				printf("%3d ", (int) v_perm[i]);
			printf("\n");
		}

		double *temp = new double[n];
		for (unsigned int i = 0 ; i < n ; i++) temp[i] = (double) v_perm[i];
		if (!sendNotificationDataToRos(temp, n, gloveData, gloveDataPub)) {
			printf("Couldn't write Notification to NotificationManager. Error!\n");
			exit(-1);
		}

                if (!sendNotificationDataToRos(getRadianArray(v_perm, n), n, gloveData_radian, gloveDataPub_radian)) {
                        printf("Couldn't write Notification to NotificationManager. Error!\n");
                        exit(-1);
                }

		usleep(PAUSE_WRITE); //500000);//30000);
	}
	threadRunning = false;
}

bool gloveServer_impl::permuteValues(unsigned char* valIn, unsigned char* valOut) {

	if (sizeof(valIn) > sizeof(valOut)) {
		printf("ERROR in permuteValues(). Array sizes do not fit for permutation!\n");
		return false;
	}

	for (unsigned int i = 0; i < MAX_SENSORS; i++) {
		//    printf( "valOut[%d] = valIn[%d] = %c\n", i, GLOVERAW_MAP2_GLOVEINVENTOR[i], valIn[GLOVERAW_MAP2_GLOVEINVENTOR[i]]);
		valOut[i] = valIn[GLOVERAW_MAP2_GLOVEINVENTOR[i]];
	}
	return true;
}

double gloveServer_impl::getRadian(int index, unsigned char val) {
	/*
	 unsigned char zero  = calibVals[0][index];
	 unsigned char min   = calibVals[1][index];
	 unsigned char max   = calibVals[2][index];
	 */
	double alfa;

	if (RADSIGN[index] == 1)
		alfa = -alfa;

	alfa =
			((FulcRad2[index] - FulcRad1[index]) * ((double) val - FulcRaw1[index])
					/ (FulcRaw2[index] - FulcRaw1[index])) + FulcRad1[index];
	//  printf( "rad2 %f rad1 %f raw2 %f raw1 %f val %f alfa %f\n", FulcRad2[index], FulcRad1[index], FulcRaw2[index], FulcRaw1[index], (double)val, alfa);
	return alfa;
}

double *gloveServer_impl::getRadianArray(unsigned char* data, unsigned int size){
        double * radianArray = new double[size];
        for ( unsigned int i = 0 ; i < size; i++){
          radianArray[i] = getRadian(i, data[i]);
        }
        return radianArray;
}

bool gloveServer_impl::sendNotificationDataToRos(double* data, unsigned int size, asr_msgs::AsrGlove gloveData, ros::Publisher publisher) {
	for (unsigned int i = 0; i < size; i++) {
		gloveData.data[i] = data[i];
	}

	std::stringstream output;
	output << "Glove Data Message published: " << myID << " ";
	for (unsigned int i = 0; i < size; i++) {
		output << gloveData.data[i] << " ";
	}
	ROS_INFO("%s", output.str().c_str());
	publisher.publish(gloveData);
	ros::spinOnce();

	return true;
}

#if gloveServer_impl_test
int main(int argc, char **argv)
{

	// Create a new instance of the implementation
	gloveServer_impl *impl = new gloveServer_impl;

	while (1);

	delete impl;

	return retval;
}
#endif /* gloveServer_impl_test */

