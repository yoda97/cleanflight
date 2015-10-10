/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

// Inertial Measurement Unit (IMU)

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "common/maths.h"

#include "platform.h"
#include "debug.h"

#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "config/runtime_config.h"

// Accelerations in NEU coordinates
t_fp_vector imuAverageAcceleration;
int16_t accSmooth[XYZ_AXIS_COUNT];

float smallAngleCosZ = 0;
float magneticDeclination = 0.0f;       // calculated at startup from config

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    // quaternion of sensor frame relative to earth frame
static float q0q0, q1q1, q2q2, q3q3, q0q1, q0q2, q0q3, q1q2, q1q3, q2q3;

attitudeEulerAngles_t attitude = { { 0, 0, 0 } };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

static imuRuntimeConfig_t *imuRuntimeConfig;
static pidProfile_t *pidProfile;

static float cosTiltAngleZ = 1.0f;
static float gyroScale;

static void imuComputeCommonQuaternionParameters(void)
{
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;
    
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q3 = q2 * q3;
}

void imuConfigure(imuRuntimeConfig_t *initialImuRuntimeConfig, pidProfile_t *initialPidProfile)
{
    imuRuntimeConfig = initialImuRuntimeConfig;
    pidProfile = initialPidProfile;
}

void imuInit()
{
    int axis;

    smallAngleCosZ = cos_approx(degreesToRadians(imuRuntimeConfig->small_angle));
    gyroScale = gyro.scale * (M_PIf / 180.0f);  // gyro output scaled to rad per second

    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        imuAverageAcceleration.A[axis] = 0;
    }

    imuComputeCommonQuaternionParameters();
}

float calculateThrottleAngleScale(uint16_t throttle_correction_angle)
{
    return (1800.0f / M_PIf) * (900.0f / throttle_correction_angle);
}

static void imuTransformVectorBodyToEarth(t_fp_vector * v)
{
    float x,y,z;

    /* From body frame to earth frame */
    x = (1.0f - 2.0f * q2q2 - 2.0f * q3q3) * v->V.X + 2.0f * (q1q2 + -q0q3) * v->V.Y + 2.0f * (q1q3 - -q0q2) * v->V.Z;
    y = 2.0f * (q1q2 - -q0q3) * v->V.X + (1.0f - 2.0f * q1q1 - 2.0f * q3q3) * v->V.Y + 2.0f * (q2q3 + -q0q1) * v->V.Z;
    z = 2.0f * (q1q3 + -q0q2) * v->V.X + 2.0f * (q2q3 - -q0q1) * v->V.Y + (1.0f - 2.0f * q1q1 - 2.0f * q2q2) * v->V.Z;

    v->V.X = x;
    v->V.Y = y;
    v->V.Z = z;
}

#define ACCELERATION_LPF_HZ             10

// rotate acc into Earth frame and calculate acceleration in it
static void imuCalculateAccelerationNorthEastUp(uint32_t deltaT)
{
    static int32_t accZoffset = 0;
    float accDt = deltaT * 1e-6;
    t_fp_vector accel_ned;

    // the accel values have to be rotated into the earth frame
    accel_ned.V.X = accADC[0];
    accel_ned.V.Y = accADC[1];
    accel_ned.V.Z = accADC[2];
    imuTransformVectorBodyToEarth(&accel_ned);

    if (imuRuntimeConfig->acc_unarmedcal == 1) {
        if (!ARMING_FLAG(ARMED)) {
            accZoffset -= accZoffset / 64;
            accZoffset += accel_ned.V.Z;
        }
        accel_ned.V.Z -= accZoffset / 64;  // compensate for gravitation on z-axis
    } else
        accel_ned.V.Z -= acc_1G;

    // FIXME: accel_ned is actually not NED, but NWU (rotated 180deg around X axis). We need NEU coordinates, so we simply reverse Y axis
    accel_ned.V.Y = -accel_ned.V.Y;

    // Calculate acceleration in earth frame
    int axis;
    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        // Convert to cm/s^2
        float accValueCMSS = accel_ned.A[axis] * (100.0f * 9.80665f / acc_1G);

        // Apply LPF to acceleration: y[i] = y[i-1] + alpha * (x[i] - y[i-1])
        // TODO: Verify if LPF is needed here
        imuAverageAcceleration.A[axis] += (accDt / ((0.5f / (M_PIf * ACCELERATION_LPF_HZ)) + accDt)) * (accValueCMSS - imuAverageAcceleration.A[axis]);
    }
}

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static void imuMahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt, bool useAcc, bool useMag)
{
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki
    float recipNorm;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex = 0, halfey = 0, halfez = 0;
    float qa, qb, qc;

    if (useMag) {
        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of magnetic field
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        halfex += (my * halfwz - mz * halfwy);
        halfey += (mz * halfwx - mx * halfwz);
        halfez += (mx * halfwy - my * halfwx);
    }

    if (useAcc) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex += (ay * halfvz - az * halfvy);
        halfey += (az * halfvx - ax * halfvz);
        halfez += (ax * halfvy - ay * halfvx);
    }

    // Compute and apply integral feedback if enabled
    if(imuRuntimeConfig->dcm_ki > 0.0f) {
        integralFBx += 2.0f * imuRuntimeConfig->dcm_ki * halfex * dt;    // integral error scaled by Ki
        integralFBy += 2.0f * imuRuntimeConfig->dcm_ki * halfey * dt;
        integralFBz += 2.0f * imuRuntimeConfig->dcm_ki * halfez * dt;
    }
    else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional and integral feedback
    gx += 2.0f * imuRuntimeConfig->dcm_kp * halfex + integralFBx;
    gy += 2.0f * imuRuntimeConfig->dcm_kp * halfey + integralFBy;
    gz += 2.0f * imuRuntimeConfig->dcm_kp * halfez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);        // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Pre-compute frequently used parameters
    imuComputeCommonQuaternionParameters();
}

static void imuUpdateEulerAngles(void)
{
    /* Compute pitch/roll angles */
    attitude.values.roll = lrintf(atan2_approx(2.0f * (q2q3 + q0q1), 1.0f - 2.0f * (q1q1 + q2q2)) * (1800.0f / M_PIf));
    attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(2.0f * (q0q2 - q1q3))) * (1800.0f / M_PIf));
    attitude.values.yaw = lrintf((-atan2_approx(2.0f * (q1q2 + q0q3), 1.0f - 2.0f * (q2q2 + q3q3)) * (1800.0f / M_PIf) + magneticDeclination));

    if (attitude.values.yaw < 0)
        attitude.values.yaw += 3600;

    debug[0] = attitude.values.roll;
    debug[1] = attitude.values.pitch;
    debug[2] = attitude.values.yaw;

    /* tilt angle cosine is directly computable from orientation quaternion */
    cosTiltAngleZ = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;

    /* Update small angle state */
    if (cosTiltAngleZ > smallAngleCosZ) {
        ENABLE_STATE(SMALL_ANGLE);
    } else {
        DISABLE_STATE(SMALL_ANGLE);
    }
}

static void imuCalculateEstimatedAttitude(void)
{
    static float accLPF[3];
    static uint32_t previousIMUUpdateTime;
    int32_t axis;

    uint32_t currentTime = micros();
    uint32_t deltaT = currentTime - previousIMUUpdateTime;
    previousIMUUpdateTime = currentTime;

    float gx = gyroADC[X] * gyroScale;
    float gy = gyroADC[Y] * gyroScale;
    float gz = gyroADC[Z] * gyroScale;

    int32_t accMagnitude = 0;

    // Initialization
    for (axis = 0; axis < 3; axis++) {
        if (imuRuntimeConfig->acc_lpf_factor > 0) {
            accLPF[axis] = accLPF[axis] * (1.0f - (1.0f / imuRuntimeConfig->acc_lpf_factor)) + accADC[axis] * (1.0f / imuRuntimeConfig->acc_lpf_factor);
            accSmooth[axis] = accLPF[axis];
        } else {
            accSmooth[axis] = accADC[axis];
        }
        accMagnitude += (int32_t)accSmooth[axis] * accSmooth[axis];
    }
    accMagnitude = accMagnitude * 100 / ((int32_t)acc_1G * acc_1G);

    // Calculate accelerometer magnitude and ignore it if accelerating/decelerating too much
    bool useAcc = (72 < (uint16_t)accMagnitude) && ((uint16_t)accMagnitude < 133);
    bool useMag = sensors(SENSOR_MAG) && (magADC[X] != 0) && (magADC[Y] != 0) && (magADC[Z] != 0);

    imuMahonyAHRSupdate(gx, gy, gz,
                        accSmooth[X], accSmooth[Y], accSmooth[Z],
                        magADC[X], magADC[Y], magADC[Z],
                        deltaT * 1e-6f,
                        useAcc,
                        useMag);

    imuUpdateEulerAngles();

    imuCalculateAccelerationNorthEastUp(deltaT); // rotate acc vector into earth frame and store it for future usage
}

void imuUpdate(rollAndPitchTrims_t *accelerometerTrims)
{
    gyroUpdate();

    if (sensors(SENSOR_ACC)) {
        updateAccelerationReadings(accelerometerTrims);
        imuCalculateEstimatedAttitude();
    } else {
        accADC[X] = 0;
        accADC[Y] = 0;
        accADC[Z] = 0;
    }
}

float calculateCosTiltAngle(void)
{
    return cosTiltAngleZ;
}

int16_t calculateTiltAngle(void)
{
    return lrintf(fabsf(acos_approx(cosTiltAngleZ) * (1800.0f / M_PIf)));
}

int16_t calculateThrottleAngleCorrection(uint8_t throttle_correction_value, int16_t throttle_correction_angle)
{
    if (cosTiltAngleZ <= 0.015f)
        return 0;

    int angle = lrintf(acos_approx(cosTiltAngleZ) * calculateThrottleAngleScale(throttle_correction_angle));
    if (angle > 900)
        angle = 900;
    return lrintf(throttle_correction_value * sin_approx((angle / 900.0f) * (M_PIf / 2.0f)));
}
