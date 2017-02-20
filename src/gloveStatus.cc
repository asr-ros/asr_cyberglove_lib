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
/* (none) */

/* my includes */
#include "gloveStatus.h"


gloveStatus::gloveStatus()
{
}

gloveStatus::~gloveStatus()
{ 
}

uchar_t gloveStatus::getSensorSize()
{
  return sensorSize;
}

void gloveStatus::setSensorSize(uchar_t n)
{
  sensorSize = n;
}

void gloveStatus::setGloveStatusByte(bool activate)
{
  flags.X.is_glove_byte = activate;
}

bool gloveStatus::isGloveStatusByte()
{
  return flags.X.is_glove_byte;
}

void gloveStatus::setTimeStamp(bool activate)
{
  flags.X.is_time_stamp = activate;
}

bool gloveStatus::isTimeStamp()
{
  return flags.X.is_time_stamp;  
}

void gloveStatus:: setParameterFlags(parameterFlags_t flg)
{
  flags = flg;
}



#if gloveStatus_test
#include <stdio.h>
int main(int argc, char **argv)
{
  // This is a module-test block. You can put code here that tests
  // just the contents of this C file, and build it by saying
  //             make gloveStatus_test
  // Then, run the resulting executable (gloveStatus_test).
  // If it works as expected, the module is probably correct. ;-)

  fprintf(stderr, "Testing gloveStatus\n");

  return 0;
}
#endif /* gloveStatus_test */
