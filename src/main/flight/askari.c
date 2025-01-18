#include "askari.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common/maths.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "msp/msp_protocol.h"

#include "flight/imu.h"

#include "sensors/acceleration.h"
#include "sensors/compass.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"

#include "rx/rx.h"
#include "rx/msp.h"


enum AXIS { // roll, pitch, throttle, yaw, aux1, aux2
  ROLL = 0,
  PITCH,
  THROTTLE,
  YAW,
  AUX1,
  AUX2
};

int16_t askariSetpoints[3] = {0,0,0}; //This holds roll [Decidegrees],pitch [Decidegrees], and maybe yaw [Degrees/s] commands
bool useAskari = false;

// static void askariMspFrameReceive(const uint16_t *frame, int channelCount)
// {
//   uint16_t rxFrame[channelCount];
//   for (int i = 0; i<channelCount;i++)
//   {
//     if (i == ROLL || i == PITCH)
//     {
//       rxFrame[i] = 1500; //To ensure that the system does to RX failsage
//       askariSetpoints[i]  = (int16_t)frame[i]; // Reinterpret as int16_t
//     }else 
//     {
//       rxFrame[i] = frame[i];
//     }
//   }
//   rxMspFrameReceive(rxFrame, channelCount); //to set aux1,aux2,throttle and yaw
// }

mspResult_e mspProcessAskariCommand(mspDescriptor_t srcDesc, int16_t cmdMSP,
                                    sbuf_t *src, sbuf_t *dst) {
  UNUSED(srcDesc);

  const unsigned int dataSize = sbufBytesRemaining(src);
  //---
  switch (cmdMSP) {
  case MSP_ASKARI: {
    //Handle the RX receive
    uint8_t channelCount = dataSize / sizeof(uint16_t);
    if (channelCount > SUPPORTED_STATE_CHANNEL_COUNT) {
      return MSP_RESULT_ERROR;
    } else {
      uint16_t frame[SUPPORTED_STATE_CHANNEL_COUNT];
      for (int i = 0; i < channelCount; i++) {
        frame[i] = sbufReadU16(src);
      }
      // rxMspFrameReceive(frame, channelCount);
      // askariMspFrameReceive(frame, channelCount);

      rxMspFrameReceive(frame, channelCount); //to set aux1,aux2,throttle and yaw

    }

    // SENDING BACK ATTITUDE DATA
    sbufWriteU16(dst, attitude.values.roll);
    sbufWriteU16(dst, attitude.values.pitch);
    sbufWriteU16(dst, attitude.values.yaw);

    // SENDING BACK IMU DATA
    for (int i = 0; i < 3; i++) {
#if defined(USE_ACC)
      sbufWriteU16(dst, lrintf(acc.accADC.v[i]));
#else
      sbufWriteU16(dst, 0);
#endif
    }
    for (int i = 0; i < 3; i++) {
      sbufWriteU16(dst, gyroRateDps(i));
    }
    for (int i = 0; i < 3; i++) {
#if defined(USE_MAG)
      sbufWriteU16(dst, lrintf(mag.magADC.v[i]));
#else
      sbufWriteU16(dst, 0);
#endif
    }

    // SENDING BACK MOTOR DATA
    /*TODO:*/
    break;
  }
  default:
    return MSP_RESULT_CMD_UNKNOWN;
  }
  return MSP_RESULT_ACK;
}