#pragma once

#include "common/streambuf.h"
#include "msp/msp.h"
#include <stdbool.h>

// 5 Inputs states, angular velocities around each axis (ROLL,PITCH,YAW), thrust
// command and a channel for arming
#define PACKET_SIZE_BYTES 14

typedef int mspDescriptor_t;

typedef struct
{   
    float pGains[3];
    float dGains[3];
} askariGains_t;

mspResult_e mspProcessAskariCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src,sbuf_t *dst);


void updateAskariQuaternions(void);
float pidLevelAskari(int axis);
