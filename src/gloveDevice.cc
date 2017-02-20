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
#include <pthread.h>
#include <stdio.h>

/* my includes */
#include "gloveDevice.h"
#include <string>

using std::string;

gloveDevice::gloveDevice()
{
	//create object for serial connection
}
gloveDevice::~gloveDevice()
{
	//destroy object for serial connection
}

bool gloveDevice::init(const char *serialDev)
{

	// create mutex
	pthread_mutexattr_t mutex_attr;
	pthread_mutexattr_init(&mutex_attr);
	pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_RECURSIVE);

	pthread_mutex_init(&serial_lock, &mutex_attr);
	pthread_mutexattr_destroy(&mutex_attr);

	// init serial
	if ((serialId = serial.open_serial(serialDev)) == -1)
	{
		printf("open_serial() failed.\n");
		error = false;
		return false;
	}

	// set SW mask as HW mask
	sensorMask_t mask;
	error = cmdGetHwSensorMask(&mask);
	if (!error)
	{
		printf("Error: cmdGetHwSensorMask(&mask) failed.\n");
	}
	error &= cmdSetSwSensorMask(mask);
	if (!error)
	{
		printf("Error: cmdSetSwSensorMask(mask) failed.\n");
	}

	// get sensor size
	uchar_t n;
	error &= cmdGetNbSensorsTotal(&n);
	if (!error)
	{
		printf("Error: cmdGetNbSensorsTotal(&n) failed.\n");
	}
	error &= cmdSetNbSensorsEnabled(n);
	if (!error)
	{
		printf("Error: cmdSetNbSensorsEnabled(n) failed.\n");
	}

	// get glove status switch
	bool enable;
	error &= cmdGetGloveStatus(&enable);
	if (!error)
	{
		printf("Error: cmdGetGloveStatus(&enable) failed.\n");
	}

	// get timestamp switch
	error &= cmdGetTimeStamp(&enable);
	if (!error)
	{
		printf("Error: cmdGetTimeStamp(&enable) failed.\n");
	}

	if (!error)
	{
		printf("Try to switch off (and on again!) the glove hardware.\n");
	}

	return error;
}

void gloveDevice::stop()
{
	serial.close_serial(serialId);
}

void gloveDevice::lock_serial()
{
	pthread_mutex_lock(&serial_lock);

}

void gloveDevice::unlock_serial()
{
	pthread_mutex_unlock(&serial_lock);

}

uchar_t gloveDevice::getChar()
{
	uchar_t c;
	error = serial.read_serial(serialId, &c, 1);
	return c;
}

void gloveDevice::getString(vector_t s, int n)
{
	error = serial.read_serial(serialId, s, n);
}

void gloveDevice::sendChar(uchar_t c)
{
	error = serial.write_serial(serialId, &c, 1);
}

void gloveDevice::sendString(vector_t s, int n)
{
	error = serial.write_serial(serialId, s, n);
}

bool gloveDevice::readResponseSet(uchar_t c)
{
	return (getChar() == c && getChar() == 0);
}

bool gloveDevice::readResponseSet(uchar_t c1, uchar_t c2)
{
	return (getChar() == c1 && getChar() == c2 && getChar() == 0);
}

bool gloveDevice::readNullChar()
{
	return (getChar() == 0);
}

bool gloveDevice::readResponseGet(uchar_t c)
{
	// send command get
	sendChar('?');
	sendChar(c);

	// read response
	return (getChar() == '?' && getChar() == c);
}

bool gloveDevice::cmdSetChar(uchar_t name, uchar_t val)
{
	lock_serial();

	// send cmd
	sendChar(name);

	// send value
	sendChar(val);

	// read response
	bool res = readResponseSet(name);

	unlock_serial();
	return res;
}

bool gloveDevice::cmdGetChar(uchar_t name, uchar_t *val)
{
	bool res = false;
	lock_serial();

	// read response
	if (readResponseGet(name))
	{
		*val = getChar();
		res = readNullChar();
	}

	unlock_serial();
	return res;
}

bool gloveDevice::cmdGetBool(uchar_t name, bool *val)
{
	uchar_t c;
	bool res = cmdGetChar(name, &c);
	*val = c;
	return res;
}

bool gloveDevice::cmdSetActuate(int n, vector_t index, vector_t values)
{
	lock_serial();

	// send cmd
	sendChar('A');
	sendChar(n);

	// send values
	for (int i = 0; i < n; i++)
	{
		sendChar(index[i]);
		sendChar(values[i]);
	}

	// read response
	bool res = readResponseSet('A');

	unlock_serial();
	return res;
}

bool gloveDevice::cmdSetActuateAll(vector_t values)
{
	lock_serial();

	// send cmd A
	sendChar('A');
	sendChar(255);

	// send values
	sendString(values, 6);

	// read response
	bool res = readResponseSet('A');

	unlock_serial();
	return res;
}

bool gloveDevice::cmdGetActuateAll(vector_t values)
{
	bool res = false;
	lock_serial();

	// read response
	if (readResponseGet('A'))
	{
		getString(values, 6);
		res = readNullChar();
	}

	unlock_serial();
	return res;
}

bool gloveDevice::cmdSetBaudRate(uchar_t baudDivide)
{
	return cmdSetChar('B', baudDivide);
}

bool gloveDevice::cmdGetBaudRate(uchar_t * baudDivide)
{
	return cmdGetChar('B', baudDivide);
}

bool gloveDevice::cmdSetCalibrateAll(vector_t offset, vector_t gain)
{
	lock_serial();

	// send cmd
	sendChar('C');
	sendChar('A');

	// send values
	sendString(offset, status.getSensorSize());
	sendString(gain, status.getSensorSize());

	// read response
	bool res = readResponseSet('C', 'A');

	unlock_serial();
	return res;
}

bool gloveDevice::cmdSetCalibrateOffset(uchar_t index, uchar_t offset)
{
	lock_serial();

	// send cmd
	sendChar('C');
	sendChar('O');

	// send values
	sendChar(index);
	sendChar(offset);

	// read response
	bool res = readResponseSet('C', 'O');

	unlock_serial();
	return res;
}

bool gloveDevice::cmdSetCalibrateGain(uchar_t index, uchar_t gain)
{
	lock_serial();

	// send cmd
	sendChar('C');
	sendChar('G');

	// send values
	sendChar(index);
	sendChar(gain);

	// read response
	bool res = readResponseSet('C', 'G');

	unlock_serial();
	return res;
}

bool gloveDevice::cmdGetCalibrateAll(vector_t offset, vector_t gain)
{
	bool res = false;
	lock_serial();

	// read response
	if (readResponseGet('C'))
	{
		getString(offset, status.getSensorSize());
		getString(gain, status.getSensorSize());
		res = readNullChar();
	}

	unlock_serial();
	return res;
}

bool gloveDevice::cmdSetTimeStamp(bool enable)
{
	bool res = false;
	//! Steffen
	//lock_serial();

	if (cmdSetChar('D', enable))
	{
		status.setTimeStamp(enable);
		res = true;
	}
	//unlock_serial();
	return res;
}

bool gloveDevice::cmdGetTimeStamp(bool *enable)
{
	bool res = false;
	//! Steffen
	//lock_serial();

	if (cmdGetBool('D', enable))
	{
		status.setTimeStamp(*enable);
		res = true;
	}
	//unlock_serial();
	return res;
}

bool gloveDevice::cmdSetDigitalFilter(bool enable)
{
	return cmdSetChar('F', enable);
}

bool gloveDevice::cmdGetDigitalFilter(bool *enable)
{
	return cmdGetBool('F', enable);
}

bool gloveDevice::cmdGetSensorValues(vector_t values, uchar_t *n,
		statusQuery_t *query, gloveStatusByte_t *gloveStatusByte)
{
	bool res = false;
	lock_serial();

	//send cmd
	sendChar('G');

	// read response
	if (getChar() == 'G')
	{
		*n = status.getSensorSize();
		getString(values, status.getSensorSize());
		if (status.isTimeStamp())
		{
			getString(query->timeStamp.raw, 5);
		}
		if (status.isGloveStatusByte())
			gloveStatusByte->raw = getChar();
		bool end = false;
		do
		{
			uchar_t c = getChar();
			switch (c)
			{
			case 'e':
			{
				uchar_t error = getChar();
				switch (error)
				{
				case 's':
					query->sample = true;
					break;
				case 'g':
					query->plug = true;
					break;
				default:
					end = true;
					break;
				}
			}; break;
			case 0:
				res = end = true;
				break;
			default:
				end = true;
				break;
			}
		} while (!end);
	}
	else
	{
		printf("Received wrong data from stream\n");
		fflush(NULL);
	}

	unlock_serial();
	return res;
}

bool gloveDevice::cmdSetStreamValues(bool enable)
{
	lock_serial();

	if (enable)
		sendChar('S');
	else
	{
		sendChar('^');
		sendChar('c');
	}

	unlock_serial();
	return true;
}

bool gloveDevice::cmdGetConnectStatus(statusConnect_t *status)
{
	return cmdGetChar('G', &(status->raw));
}

bool gloveDevice::cmdReinitializeCyberGlove()
{
	return internalCmd('I');
}

bool gloveDevice::cmdSetControlLight(bool enable)
{
	return cmdSetChar('J', enable);
}

bool gloveDevice::cmdGetControlLight(bool *enable)
{
	return cmdGetBool('J', enable);
}

bool gloveDevice::cmdGetHwSensorMask(sensorMask_t *mask)
{
	bool res = false;
	lock_serial();

	// read response
	if (readResponseGet('K'))
	{
		getString(mask->raw, 3);
		res = readNullChar();
	}

	unlock_serial();
	return res;
}

bool gloveDevice::cmdSetWristLight(bool enable)
{
	return cmdSetChar('L', enable);
}

bool gloveDevice::cmdGetWristLight(bool *enable)
{
	return cmdGetBool('L', enable);
}

bool gloveDevice::cmdSetSwSensorMask(sensorMask_t mask)
{
	bool res = false;
	lock_serial();

	// send cmd
	sendChar('M');

	// send values
	sendString(mask.raw, 3);

	// read response
	res = readResponseSet('M');

	unlock_serial();
	return res;
}

bool gloveDevice::cmdGetSwSensorMask(sensorMask_t *mask)
{
	bool res = false;
	lock_serial();

	// read response
	if (readResponseGet('M'))
	{
		getString(mask->raw, 3);
		res = readNullChar();
	}

	unlock_serial();
	return res;
}

bool gloveDevice::cmdSetNbSensorsEnabled(uchar_t n)
{
	if (cmdSetChar('N', n))
	{
		status.setSensorSize(n);
		return true;
	}
	return false;
}

bool gloveDevice::cmdGetNbSensorsEnabled(uchar_t * n)
{
	if (cmdGetChar('N', n))
	{
		status.setSensorSize(*n);
		return true;
	}
	return false;
}

bool gloveDevice::cmdSetParameterFlags(parameterFlags_t flags)
{
	bool res = false;
	lock_serial();

	// send cmd
	sendChar('P');

	// send values
	sendString(flags.raw, 3);

	// read response
	res = readResponseSet('P');

	unlock_serial();
	return res;
}

bool gloveDevice::cmdGetParameterFlags(parameterFlags_t *flags)
{
	bool res = false;
	lock_serial();

	// read response
	if (readResponseGet('P'))
	{
		getString(flags->raw, 3);
		res = readNullChar();
	}

	unlock_serial();
	return res;
}

bool gloveDevice::cmdSetQuantized(bool enable)
{
	return cmdSetChar('Q', enable);
}

bool gloveDevice::cmdGetQuantized(bool *enable)
{
	return cmdGetBool('Q', enable);
}

bool gloveDevice::cmdGetHand(bool *rightHand)
{
	return cmdGetBool('Q', rightHand);
}

bool gloveDevice::cmdGetSensorDummyValues(vector_t values, uchar_t* n,
		statusQuery_t* query, gloveStatusByte_t* gloveStatusByte)
{
	for (unsigned int i = 0; i < MAX_SENSORS; i++)
	{
		values[i] = i*3;
	}
	*n = (unsigned char) MAX_SENSORS;
	return true;
}

bool gloveDevice::internalCmd(uchar_t cmd)
{
	bool result = false;
	lock_serial();

	// send cmd
	sendChar('^');
	sendChar(cmd);

	// read response
	uchar_t res[5];
	getString(res, 5);

	result = (res[0] == '^' && res[1] == cmd && res[2] == CR && res[3] == LF
			&& res[4] == 0);

	unlock_serial();
	return result;
}

bool gloveDevice::cmdRestartFirmware()
{
	return internalCmd('R');
}

bool gloveDevice::cmdGetNbSensorsTotal(uchar_t * n)
{
	return cmdGetChar('S', n);
}

bool gloveDevice::cmdSetSamplePeriod(samplePeriod_t period)
{
	bool res = false;
	lock_serial();

	// send cmd
	sendChar('T');

	// send values
	sendString(period.raw, 4);

	// read response
	res = readResponseSet('T');

	unlock_serial();
	return res;
}

bool gloveDevice::cmdGetSamplePeriod(samplePeriod_t *period)
{
	bool res = false;
	lock_serial();

	// read response
	if (readResponseGet('T'))
	{
		getString(period->raw, 4);
		res = readNullChar();
	}

	unlock_serial();
	return res;
}

bool gloveDevice::cmdSetGloveStatus(bool enable)
{
	if (cmdSetChar('U', enable))
	{
		status.setGloveStatusByte(enable);
		return true;
	}
	return false;
}

bool gloveDevice::cmdGetGloveStatus(bool *enable)
{
	if (cmdGetBool('U', enable))
	{
		status.setGloveStatusByte(*enable);
		return true;
	}
	return false;
}

bool gloveDevice::cmdGetGloveVersion(gloveVersion_t *version)
{
	bool res = false;
	lock_serial();

	// read response
	if (readResponseGet('V'))
	{
		version->raw[0] = getChar();
		version->raw[1] = getChar();
		version->raw[2] = getChar();
		if (version->raw[2])
		{
			version->raw[3] = getChar();
			if (version->raw[3])
			{
				res = readNullChar();
			}
			else
				res = true;
		}
		else
			res = true;
	}

	unlock_serial();
	return res;
}

bool gloveDevice::cmdSetWristToggle(bool enable)
{
	return cmdSetChar('W', enable);
}

bool gloveDevice::cmdGetWristToggle(bool *enable)
{
	return cmdGetBool('W', enable);
}

bool gloveDevice::cmdSetBinarySync(bool enable)
{
	return cmdSetChar('W', enable);
}

bool gloveDevice::cmdGetBinarySync(bool *enable)
{
	return cmdGetBool('W', enable);
}

bool gloveDevice::cmdGetSensorValuesSync(vector_t values, uchar_t *n,
		bool *errorSync, gloveStatusByte_t *gloveStatusByte)
{
	bool res = false;
	lock_serial();

	// read response
	if (getChar() == 'Y')
	{
		*n = status.getSensorSize();
		getString(values, status.getSensorSize());
		if (status.isGloveStatusByte())
			gloveStatusByte->raw = getChar();

		uchar_t c = getChar();
		if (c)
		{
			*errorSync = c;
			res = readNullChar();
		}
		else
			res = true;
	}

	unlock_serial();
	return res;
}

void gloveDevice::showCurrentStatus()
{
	printf("\n");
	printf("--- Glove Current Status ---\n");
	printf("Nb of sensors sampled: %d\n", status.getSensorSize());
	printf("Glove Status Byte: %s\n",
			status.isGloveStatusByte() ? "on" : "off");
	printf("Time-Stamp: %s\n", status.isTimeStamp() ? "on" : "off");
	printf("Errors detected: %s\n", error ? "no" : "yes");
	printf("\n");
}

#if gloveDevice_test
#include <stdio.h>
int main(int argc, char **argv)
{
	// This is a module-test block. You can put code here that tests
	// just the contents of this C file, and build it by saying
	//             make gloveDevice_test
	// Then, run the resulting executable (gloveDevice_test).
	// If it works as expected, the module is probably correct. ;-)

	fprintf(stderr, "Testing gloveDevice\n");

	printf("testing gloveDevice\n");

	return 0;
}
#endif /* gloveDevice_test */
