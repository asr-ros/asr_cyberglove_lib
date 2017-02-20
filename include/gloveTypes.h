/**

Copyright (c) 2016, Heller Florian, Meißner Pascal, Nguyen Trung, Yi Xie
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef GLOVETYPES_H
#define GLOVETYPES_H

/* system includes */

/* my includes */
/* (none) */


typedef unsigned char * vector_t;
typedef unsigned char uchar_t;

union timeStamp_t
{
  uchar_t raw[5];
  struct{
    uchar_t is_b0_null: 1;
    uchar_t is_b1_null: 1;
    uchar_t is_b2_null: 1;
    uchar_t is_b3_null: 1;
    uchar_t _rv0:       4;
    
    uchar_t b0;
    uchar_t b1;
    uchar_t b2;
    uchar_t b3;
  }X;
};

union gloveStatusByte_t
{
  uchar_t raw;
  struct{
    uchar_t glove_plugged: 1;
    uchar_t switch_wrist:  1;
    uchar_t light_wrist:   1;
    uchar_t _rv0:          5;
  }X;
};

struct sensorMask_t
{
  uchar_t raw[3];
};

union gloveVersion_t
{
  uchar_t raw[4];
  struct{
    uchar_t msb1;
    uchar_t lsb1;
    uchar_t msb2;
    uchar_t lsb2;
  }X;
};


union statusQuery_t
{
  timeStamp_t timeStamp;
  bool sample;
  bool plug;
};


union statusConnect_t
{
  uchar_t raw;
  struct{
    uchar_t glove_init:    1;
    uchar_t glove_plugged: 1;
    uchar_t _rv0:          6;
  }X;
};

union samplePeriod_t
{
  uchar_t raw[4];
  struct{
    unsigned short w1;
    unsigned short w2;
  }X;
};

union parameterFlags_t
{
  uchar_t raw[3];
  struct
  {
    uchar_t glove_inout:    1;
    uchar_t switch_status:  1;
    uchar_t light_status:   1;
    uchar_t _rv0:           5;

    uchar_t binary_sync:    1;
    uchar_t ascii_sync:     1;
    uchar_t is_glove_byte:  1;
    uchar_t control_light:  1;
    uchar_t digital_filter: 1;
    uchar_t is_time_stamp:  1;
    uchar_t glove_hand:     1;
    uchar_t glove_valid:    1;

    uchar_t send_quant:     1;
    uchar_t cyber_option:   1;
    uchar_t _rv1:           6;
  }X;
}; 
    
  

#endif /* GLOVETYPES_H */
