/**

Copyright (c) 2016, Heller Florian, Meißner Pascal, Nguyen Trung, Yi Xie
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef GLOVEDEVICE_H
#define GLOVEDEVICE_H

/* system includes */
#include <pthread.h>

/* my includes */
#include "gloveTypes.h"
#include "gloveStatus.h"
#include "serial.h"

/*!
 \class gloveDevice
 \brief


 */

#define CR 13
#define LF 10
#define MAX_SENSORS 22

class gloveDevice
{
public:
	gloveDevice();
	~gloveDevice();

	bool init(const char *serialDev);
	void stop();

	/* Display status */
	void showCurrentStatus();

	/* Sensors: init */

	//M
	bool cmdSetSwSensorMask(sensorMask_t mask);
	bool cmdGetSwSensorMask(sensorMask_t *mask);
	//K
	bool cmdGetHwSensorMask(sensorMask_t *mask);
	//N
	bool cmdSetNbSensorsEnabled(uchar_t n);
	bool cmdGetNbSensorsEnabled(uchar_t *n);
	//S
	bool cmdGetNbSensorsTotal(uchar_t *n);
	//T
	bool cmdSetSamplePeriod(samplePeriod_t period);
	bool cmdGetSamplePeriod(samplePeriod_t *period);

	/* Sensors: switches */

	//D
	bool cmdSetTimeStamp(bool enable);
	bool cmdGetTimeStamp(bool *enable);
	//U
	bool cmdSetGloveStatus(bool enable);
	bool cmdGetGloveStatus(bool *enable);

	/* Sensors: values */

	//G
	bool cmdGetSensorValues(vector_t values, uchar_t *n, statusQuery_t *query,
			gloveStatusByte_t *gloveStatusByte);
	//Y
	bool cmdGetSensorValuesSync(vector_t values, uchar_t *n, bool *errorSync,
			gloveStatusByte_t *gloveStatusByte);
	//S
	bool cmdSetStreamValues(bool enable);

	bool cmdGetSensorDummyValues(vector_t values, uchar_t *n,
			statusQuery_t *query, gloveStatusByte_t *gloveStatusByte);

	/* Sensors: calibrate  */

	//A
	bool cmdSetActuate(int n, vector_t index, vector_t values);
	bool cmdSetActuateAll(vector_t values);
	bool cmdGetActuateAll(vector_t values);
	//C
	bool cmdSetCalibrateAll(vector_t offset, vector_t gain);
	bool cmdSetCalibrateOffset(uchar_t index, uchar_t offset);
	bool cmdSetCalibrateGain(uchar_t index, uchar_t gain);
	bool cmdGetCalibrateAll(vector_t offset, vector_t gain);

	/* Glove: information */

	//B
	bool cmdSetBaudRate(uchar_t baudDivide);
	bool cmdGetBaudRate(uchar_t *baudDivide);
	//P
	bool cmdSetParameterFlags(parameterFlags_t flags);
	bool cmdGetParameterFlags(parameterFlags_t *flags);
	//G
	bool cmdGetConnectStatus(statusConnect_t *status);
	//R
	bool cmdGetHand(bool *rightHand);
	//V
	bool cmdGetGloveVersion(gloveVersion_t *version);
	//Y
	bool cmdSetBinarySync(bool enable);
	bool cmdGetBinarySync(bool *enable);

	/* Glove: switches */

	//F
	bool cmdSetDigitalFilter(bool enable);
	bool cmdGetDigitalFilter(bool *enable);
	//J
	bool cmdSetControlLight(bool enable);
	bool cmdGetControlLight(bool *enable);
	//L
	bool cmdSetWristLight(bool enable);
	bool cmdGetWristLight(bool *enable);
	//Q
	bool cmdSetQuantized(bool enable);
	bool cmdGetQuantized(bool *enable);
	//W
	bool cmdSetWristToggle(bool enable);
	bool cmdGetWristToggle(bool *enable);

	/* Glove: internal commands */

	//^I
	bool cmdReinitializeCyberGlove();
	//^R
	bool cmdRestartFirmware();

private:
	gloveStatus status;
	SerialDevice serial;int serialId;
	bool error;
	pthread_mutex_t serial_lock;

	void lock_serial();
	void unlock_serial();

	uchar_t getChar();
	void getString(vector_t s, int n);
	void sendChar(uchar_t);
	void sendString(vector_t s, int n);

	bool readResponseSet(uchar_t c);
	bool readResponseSet(uchar_t c1, uchar_t c2);
	bool readNullChar();
	bool readResponseGet(uchar_t c);

	bool cmdSetChar(uchar_t name, uchar_t val);
	bool cmdGetChar(uchar_t name, uchar_t *val);
	bool cmdGetBool(uchar_t name, bool *val);

	bool internalCmd(uchar_t cmd);

};

#endif /* GLOVEDEVICE_H */
