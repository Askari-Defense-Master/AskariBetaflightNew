#pragma once

#include "common/streambuf.h"
#include "msp/msp.h"
#include <stdbool.h>


#define ASKARI_MODE

#ifdef ASKARI_MODE
// 5 Inputs states, angular velocities around each axis (ROLL,PITCH,YAW), thrust
// command and a channel for arming
#define SUPPORTED_STATE_CHANNEL_COUNT 6

typedef int mspDescriptor_t;

mspResult_e mspProcessAskariCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src,sbuf_t *dst);

float getAskariSetpointRates(int axis);
bool useAskari(void);






#endif