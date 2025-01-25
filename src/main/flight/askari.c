#include "askari.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common/maths.h"
#include "common/streambuf.h"
#include "common/utils.h"
#include "fc/runtime_config.h"

#include "msp/msp_protocol.h"

#include "flight/imu.h"
#include "flight/position.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
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

//Quaternion variables used in the controller
quaternion_t currentQuaternion = QUATERNION_INITIALIZE;
quaternion_t desiredQuaternion = QUATERNION_INITIALIZE;
quaternion_t errorQuaternion = QUATERNION_INITIALIZE;
quaternion_t errorTiltQuaternion = QUATERNION_INITIALIZE;
quaternion_t errorYawQuaternion = QUATERNION_INITIALIZE;
quaternionProducts errorQuaternionP = QUATERNION_PRODUCTS_INITIALIZE;


//NOTE: Integral will probably not be used
askariGains_t pidAskari = {
  .pGains = {12.0,12.0,5.0},
  .dGains = {0.3,0.3,0.3} 
};

int16_t askariSetpoints[3] = {0,0,0}; //This holds roll [Decidegrees],pitch [Decidegrees], and maybe yaw [Degrees/s] commands
bool useAskari = false;

static void askariMspFrameReceive(const uint16_t *frame, int channelCount)
{
  uint16_t rxFrame[channelCount];
  for (int i = 0; i<channelCount;i++)
  {
    if (i == ROLL || i == PITCH)
    {
      rxFrame[i] = 1500; //To ensure that the system does to RX failsage
      askariSetpoints[i]  = (int16_t)frame[i]; // Reinterpret as int16_t
    }else 
    {
      rxFrame[i] = frame[i];
    }
  }
  rxMspFrameReceive(rxFrame, channelCount); //to set aux1,aux2,throttle and yaw
}

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
      if (FLIGHT_MODE(ASKARI_MODE))
      {
        askariMspFrameReceive(frame, channelCount);
      } 
      else
      {
        rxMspFrameReceive(frame, channelCount); //to set aux1,aux2,throttle and yaw
      }
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

    // SENDING BACK MAGNETOMETER DATA
    for (int i = 0; i < 3; i++) {
#if defined(USE_MAG)
      sbufWriteU16(dst, lrintf(mag.magADC.v[i]));
#else
      sbufWriteU16(dst, 0);
#endif
    }

    // SENDING BACK ALTITUDE DATA
    sbufWriteU32(dst, getEstimatedAltitudeCm());
#ifdef USE_VARIO
        sbufWriteU16(dst, getEstimatedVario());
#else
        sbufWriteU16(dst, 0);
#endif

    // SENDING BACK MOTOR DATA
    /*TODO:*/
    break;
  }
  default:
    return MSP_RESULT_CMD_UNKNOWN;
  }

  pidLevelAskari(ROLL);
  return MSP_RESULT_ACK;
}


///THE MAGIC HAPPENS HERE
//ASKARI ATTITUDE CONTROL

//Here are some papers that helped me out
/*
Tilt-Prioritized Quadrocopter Attitude Control
Dario Brescianini, Student Member, IEEE
Raffaello D'Andrea, Fellow, IEEE

Rotational Error Metrics for Quadrotor Control
Alexander Spitzer, Robotics Institute, Carnegie Mellon University, Pittsburgh, PA, USA
spitzer@cmu.edu

Nathan Michael, Robotics Institute, Carnegie Mellon University, Pittsburgh, PA, USA
nmichael@cmu.edu

*/

static void imuQuaternionComputeProducts(quaternion_t *quat, quaternionProducts *quatProd)
{
    quatProd->ww = quat->w * quat->w;
    quatProd->wx = quat->w * quat->x;
    quatProd->wy = quat->w * quat->y;
    quatProd->wz = quat->w * quat->z;
    quatProd->xx = quat->x * quat->x;
    quatProd->xy = quat->x * quat->y;
    quatProd->xz = quat->x * quat->z;
    quatProd->yy = quat->y * quat->y;
    quatProd->yz = quat->y * quat->z;
    quatProd->zz = quat->z * quat->z;
}

static void quaternionConjugate(quaternion_t *q) {
    q->x = -q->x;
    q->y = -q->y;
    q->z = -q->z;
}

FAST_CODE_NOINLINE void updateAskariQuaternions(void)
{
  getQuaternion(&currentQuaternion);
  quaternionConjugate(&currentQuaternion);
  imuQuaternionMultiplication(&currentQuaternion, &desiredQuaternion, &errorQuaternion);
  imuQuaternionComputeProducts(&errorQuaternion, &errorQuaternionP); // I only need 6 of the 9 products might be able to save some cycles if I just use it myself

  float den = sqrtf(errorQuaternionP.ww + errorQuaternionP.zz);
  float inv_den = 1.0f / den;

  errorTiltQuaternion.w = (errorQuaternionP.ww + errorQuaternionP.zz)*inv_den;
  errorTiltQuaternion.x = (errorQuaternionP.wx - errorQuaternionP.yz)*inv_den;
  errorTiltQuaternion.y = (errorQuaternionP.wy + errorQuaternionP.xz)*inv_den;
  errorTiltQuaternion.z = 0;

  errorYawQuaternion.w = errorQuaternion.w*inv_den;
  errorYawQuaternion.x = 0.0;
  errorYawQuaternion.y = 0.0;
  errorYawQuaternion.z = errorQuaternion.z*inv_den;
}


// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
FAST_CODE_NOINLINE float pidLevelAskari(int axis)
{
    //Ensure that quaternions are updated --> called from main loop

    ///PD controller
    const float gyroRate = gyro.gyroADCf[axis];

    UNUSED(gyroRate);
    
    //pidRuntime.angleGain
    float angleRate = pidAskari.pGains[axis] * errorTiltQuaternion.v[axis+1] + pidAskari.pGains[axis] * errorYawQuaternion.v[axis+1] - pidAskari.dGains[axis] * gyroRate;
    
    return angleRate;
}
