#pragma once

#include "common/streambuf.h"
#include "msp/msp.h"
#include <stdbool.h>

// 5 Inputs states, angular velocities around each axis (ROLL,PITCH,YAW), thrust
// command and a channel for arming
#define SUPPORTED_STATE_CHANNEL_COUNT 6
extern int16_t askariSetpoints[2];
extern bool useAskari;

typedef int mspDescriptor_t;

mspResult_e mspProcessAskariCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src,sbuf_t *dst);

float getAskariSetpointRates(int axis);

