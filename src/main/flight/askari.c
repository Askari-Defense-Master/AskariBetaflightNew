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


enum RX_CHANNELS { // roll, pitch, throttle, yaw, aux1, aux2
  RX_ROLL = 0,
  RX_PITCH,
  RX_THROTTLE,
  RX_YAW,
  RX_AUX1,
  RX_AUX2,
  RX_CHANNEL_COUNT // Total number of RX channels
};

enum ASKARI_PACKET_INDEX { // roll, pitch, throttle, yaw, aux1, aux2
  QUATERNION_W = 0,
  QUATERNION_X,
  QUATERNION_Y,
  QUATERNION_Z,
  THROTTLE,
  AUX1,
  AUX2,
  PACKET_SIZE
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
  .pGains = {550.0,550.0,200.0},
  .dGains = {0.3,0.3,0.3} 
};

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

//Askari packet: {quaternion.w,quaternion.x,quaternion.y,quaternion.z,throttle,aux1,aux2}
static void askariMspFrameReceive(const uint16_t *frame)
{
  //Frame to be sent back to the RX side
  uint16_t rxFrame[RX_CHANNEL_COUNT] = {1500, 1500, 1500, 1500, 1500, 1500}; 

  // Convert quaternion elements
  for (int i = QUATERNION_W; i <= QUATERNION_Z; i++) {
      desiredQuaternion.v[i] = (float)((int16_t)frame[i]);
  }

  //normalize the quaternion
  float recipNorm = invSqrt(sq(desiredQuaternion.w) + sq(desiredQuaternion.x) + sq(desiredQuaternion.y) + sq(desiredQuaternion.z));
  desiredQuaternion.w *= recipNorm;
  desiredQuaternion.x *= recipNorm;
  desiredQuaternion.y *= recipNorm;
  desiredQuaternion.z *= recipNorm;

  // Map relevant channels from frame to rxFrame
  rxFrame[RX_THROTTLE] = frame[THROTTLE];
  rxFrame[RX_AUX1] = frame[AUX1];
  rxFrame[RX_AUX2] = frame[AUX2];

  // Send the mapped RX frame
  rxMspFrameReceive(rxFrame, RX_CHANNEL_COUNT);
}

mspResult_e mspProcessAskariCommand(mspDescriptor_t srcDesc, int16_t cmdMSP,
                                    sbuf_t *src, sbuf_t *dst) {
  UNUSED(srcDesc);

  const unsigned int dataSize = sbufBytesRemaining(src);
  //---
  switch (cmdMSP) {
  case MSP_ASKARI: {
    //Handle the RX receive
    if (dataSize != PACKET_SIZE_BYTES) {
      return MSP_RESULT_ERROR;
    } 

    uint8_t channelCount = dataSize / sizeof(uint16_t);
    uint16_t frame[channelCount];
    for (int i = 0; i < channelCount; i++) {
      frame[i] = sbufReadU16(src);
    }
    askariMspFrameReceive(frame);

    // SENDING BACK ATTITUDE DATA in quaternions
    sbufWriteU16(dst, lrintf(imuAttitudeQuaternion.w*1000));
    sbufWriteU16(dst, lrintf(imuAttitudeQuaternion.x*1000));
    sbufWriteU16(dst, lrintf(imuAttitudeQuaternion.y*1000));
    sbufWriteU16(dst, lrintf(imuAttitudeQuaternion.z*1000));


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

  return MSP_RESULT_ACK;
}


///THE MAGIC HAPPENS HERE
//NOTE: ASKARI ATTITUDE CONTROL

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
    quatProd->ww = quat->w * quat->w;//need
    quatProd->wx = quat->w * quat->x;//need
    quatProd->wy = quat->w * quat->y;//need
    // quatProd->wz = quat->w * quat->z;
    // quatProd->xx = quat->x * quat->x;
    // quatProd->xy = quat->x * quat->y;
    quatProd->xz = quat->x * quat->z;//need
    // quatProd->yy = quat->y * quat->y;
    quatProd->yz = quat->y * quat->z;//need
    quatProd->zz = quat->z * quat->z;//need
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
  imuQuaternionComputeProducts(&errorQuaternion, &errorQuaternionP); // I only need 6 of the 9 products might be able to save some cycles if I only compute the ones I need

  float inv_den = invSqrt(errorQuaternionP.ww + errorQuaternionP.zz);

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
    //NB: Ensure that quaternions are updated --> called from main loop

    ///PD controller
    const float gyroRate = gyro.gyroADCf[axis];

    UNUSED(gyroRate);
    
    //pidRuntime.angleGain
    float angleRate = (pidAskari.pGains[axis] * errorTiltQuaternion.v[axis+1]) + (pidAskari.pGains[axis] * SIGN(errorYawQuaternion.w) *errorYawQuaternion.v[axis+1]) - pidAskari.dGains[axis] * gyroRate;
    
    return angleRate;
}
