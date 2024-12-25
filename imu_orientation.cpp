/***************************************************************************
 *                                                                         *
 * Sensor Fusion code for estimating orientation of Arduino-based IMU      *
 * 2022 Joseph Malloch, Brady Boettcher                                    *
 * Graphics and Experiential Media (GEM) Laboratory                        *
 * Input Devices and Music Interaction Laboratory (IDMIL)                  *
 *                                                                         *
 ***************************************************************************
 *                                                                         *
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU License V2 for more details.                                        *
 *                                                                         *
 ***************************************************************************/

#include "imu_orientation.h"
#include <cmath>

#define DEGREE_TO_RAD (M_PIF / 180.f)
#define GYRO_BIAS_COEFF 0.99999f

void IMU_Orientation::setAccelerometerValues(float x, float y, float z)
{
    accel.x = x;
    accel.y = y;
    accel.z = z;
}

void IMU_Orientation::setGyroscopeRadianValues(float x, float y, float z, float period_sec)
{
    // convert to delta radians before storing
    // x and y axes need to be inverted
    gyro.x = x * period_sec;
    gyro.y = y * -period_sec;
    gyro.z = z * -period_sec;
}

void IMU_Orientation::setGyroscopeDegreeValues(float x, float y, float z, float period_sec)
{
    // convert to delta radians before storing
    // x and y axes need to be inverted
    gyro.x = x * DEGREE_TO_RAD * period_sec;
    gyro.y = y * -DEGREE_TO_RAD * period_sec;
    gyro.z = z * -DEGREE_TO_RAD * period_sec;
}

void IMU_Orientation::setMagnetometerValues(float x, float y, float z)
{
    mag.x = x;
    mag.y = y;
    mag.z = z;
}

void IMU_Orientation::update(float weight)
{
    Quaternion q_temp;
    // Euler etemp;
    Cartesian c_temp;

    /**********************/
    /* ACCELEROMETER DATA */
    /**********************/

    // build quaternions from polar representation of accelerometer data
    accel.magnitude = sqrtf(powf(accel.z, 2) + powf(accel.y, 2));

    float half_roll = atan2f(accel.z, accel.y) * 0.5f;
    Quaternion q_accel_roll(cosf(half_roll), 0, sinf(half_roll), 0);
    
    float half_tilt = atan2f(accel.x, accel.magnitude) * 0.5f;
    Quaternion q_accel_tilt(cosf(half_tilt), sinf(half_tilt), 0, 0);

    accel.magnitude = sqrtf(powf(accel.x, 2) + powf(accel.magnitude, 2));

    Quaternion q_accel = q_accel_tilt * q_accel_roll;

    /**********************/
    /*   GYROSCOPE DATA   */
    /**********************/

    // continuously adjust for changing gyro bias
    gyro_bias.x = gyro_bias.x * GYRO_BIAS_COEFF + gyro.x * (1.0f - GYRO_BIAS_COEFF);
    gyro_bias.y = gyro_bias.y * GYRO_BIAS_COEFF + gyro.x * (1.0f - GYRO_BIAS_COEFF);
    gyro_bias.z = gyro_bias.z * GYRO_BIAS_COEFF + gyro.x * (1.0f - GYRO_BIAS_COEFF);

    gyro.x -= gyro_bias.x;
    gyro.y -= gyro_bias.y;
    gyro.z -= gyro_bias.z;

    // construct quaternion from gyroscope axes
    Quaternion q_gyro(cosf((gyro.x + gyro.y + gyro.z) * 0.5f), sinf(gyro.z * 0.5f),
                      sinf(gyro.x * 0.5f), sinf(gyro.y * 0.5f));

    // integrate latest gyro quaternion with previous orientation
    quaternion = quaternion * q_gyro;

    // calculate inverse quaternion for tilt estimate
    half_tilt = euler.tilt * 0.5f;
    // TODO: optimize by building inverse quaternion directly
    Quaternion q_tilt_inv(cosf(half_tilt), sinf(half_tilt), 0, 0);
    q_tilt_inv = q_tilt_inv.inverse();

    // calculate inverse quaternion for roll estimate
    half_roll = euler.roll * 0.5f;
    // TODO: optimize by building inverse quaternion directly
    Quaternion q_roll_inv(cosf(half_roll), 0, sinf(half_roll), 0);
    q_roll_inv = q_roll_inv.inverse();

    // Compute euler angles
    /* extract tilt and azimuth by rotating vector [0,1,0]: Q * [0,0,1,0] * Q^-1
     * before optimization:
        Quaternion right(0,0,1,0);
        Quaternion conj = quaternion.conjugate();
        Quaternion temp = right * conj;
        temp = quaternion * temp;
        euler.azimuth = atan2(temp.x, temp.y);
        euler.tilt = temp.z * M_PI_2;
     */
    c_temp.x = 2 * (quaternion.x * quaternion.y - quaternion.w * quaternion.z);
    c_temp.y = quaternion.w * quaternion.w - quaternion.x * quaternion.x + quaternion.y * quaternion.y - quaternion.z * quaternion.z;
    c_temp.z = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
    euler.azimuth = atan2f(c_temp.x, c_temp.y);
    euler.tilt = c_temp.z * M_PI_2F;

    /* extract roll by rotating vector [0,0,1]: Q * [0,0,0,1] * Q^-1
     * before optimization:
        Quaternion up(0,0,0,1);
        temp = up * conj;
        temp = quaternion * temp;
        euler.roll = atan2(cos(euler.azimuth) * temp.x + sin(euler.azimuth) * -temp.y, temp.z);
     */
    c_temp.x = 2 * (quaternion.x * quaternion.z + quaternion.w * quaternion.y);
    c_temp.y = 2 * (quaternion.y * quaternion.z - quaternion.w * quaternion.x);
    c_temp.z = quaternion.w * quaternion.w - quaternion.x * quaternion.x - quaternion.y * quaternion.y + quaternion.z * quaternion.z;
    euler.roll = atan2f(cosf(euler.azimuth) * c_temp.x + sinf(euler.azimuth) * -c_temp.y, c_temp.z);

    /**********************/
    /* MAGNETOMETER DATA  */
    /**********************/

    // assuming magnetometer data has already been calibrated

    // build quaternions from polar representation of magnetometer data
    half_roll = atan2f(mag.z, mag.y) * 0.5f;
    Quaternion q_mag_roll(cosf(half_roll), 0, sinf(half_roll), 0);

    mag.magnitude = sqrtf(powf(mag.z, 2) + powf(mag.y, 2));
    half_tilt = atan2f(mag.x, mag.magnitude) * 0.5f;
    
    Quaternion q_mag_tilt(cosf(half_tilt), sinf(half_tilt), 0, 0);
    Quaternion q_mag = q_mag_tilt * q_mag_roll;

    // rotate the magnetometer quaternion
    q_mag = q_mag * q_roll_inv;
    q_mag = q_mag * q_tilt_inv;

    /* extract azimuth by rotating vector [0,0,1]: Q * [0,0,0,1] * Q^-1
     * before optimization:
        conj = q_mag.conjugate();
        temp = up * conj;
        temp = q_mag * temp;
        float half_azimuth = atan2(temp.x, temp.y) * 0.5;
     */
    c_temp.x = 2 * (q_mag.x * q_mag.z + q_mag.w * q_mag.y);
    c_temp.y = 2 * (q_mag.y * q_mag.z - q_mag.w * q_mag.x);
    float half_azimuth = atan2f(c_temp.x, c_temp.y) * 0.5f;

    // replace q_mag with just azimuth
    q_mag.w = cosf(half_azimuth);
    q_mag.x = q_mag.y = 0;
    q_mag.z = sinf(half_azimuth);

    // construct quaternion from combined accelerometer and magnetometer azimuth data
    Quaternion q_accel_mag = q_mag * q_accel;

    // SLERP between stored orientation and new accel + mag estimate
    Quaternion delayed = quaternion;
    quaternion = quaternion.slerp(q_accel_mag, weight);

    // use the shortest distance from previous orientation
    quaternion.minimizeDistance(delayed);
}

IMU_Orientation::Quaternion IMU_Orientation::Quaternion::operator*(Quaternion& rhs)
{
    Quaternion o;
    o.w = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
    o.x = x * rhs.w + w * rhs.x + y * rhs.z - z * rhs.y;
    o.y = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
    o.z = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;
    return o;
}

IMU_Orientation::Quaternion IMU_Orientation::Quaternion::inverse()
{
    Quaternion o;
    float norm_squared = w * w + x * x + y * y + z * z;
    if (norm_squared == 0) norm_squared = 0.0000001f;
    o.w = w / norm_squared;
    o.x = x * -1 / norm_squared;
    o.y = y * -1 / norm_squared;
    o.z = z * -1 / norm_squared;
    return o;
}

IMU_Orientation::Quaternion IMU_Orientation::Quaternion::conjugate()
{
    return Quaternion(w, -x, -y, -z);
}

IMU_Orientation::Quaternion IMU_Orientation::Quaternion::slerp(Quaternion &q, float weight)
{
    Quaternion o;
    float dot = dotProduct(q);
    if (dot > 0.9995f) {
        o.w = w + (q.w - w) * weight;
        o.x = x + (q.x - x) * weight;
        o.y = y + (q.y - y) * weight;
        o.z = z + (q.z - z) * weight;
        o.normalize();
        return o;
    }

    if (dot > 1)
        dot = 1;
    else if (dot < -1)
        dot = -1;

    float theta_0 = acosf(dot);
    float theta = (0.f < theta_0 && theta_0 < M_PI_2F) ? theta_0 * weight : (theta_0 - M_PIF) * weight;

    o.w = q.w - w * dot;
    o.x = q.x - x * dot;
    o.y = q.y - y * dot;
    o.z = q.z - z * dot;
    o.normalize();

    o.w = w * cosf(theta) + o.w * sinf(theta);
    o.x = x * cosf(theta) + o.x * sinf(theta);
    o.y = y * cosf(theta) + o.y * sinf(theta);
    o.z = z * cosf(theta) + o.z * sinf(theta);
    o.normalize();
    return o;
}

float IMU_Orientation::Quaternion::dotProduct(Quaternion &q)
{
    return w * q.w + x * q.x + y * q.y + z * q.z;
}

void IMU_Orientation::Quaternion::normalize()
{
    float norm = sqrtf(w * w + x * x + y * y + z * z);
    if (norm == 0)
      norm = 0.0000001f;
    float inv_norm = 1.0f / norm;
    w *= inv_norm;
    x *= inv_norm;
    y *= inv_norm;
    z *= inv_norm;
}

void IMU_Orientation::Quaternion::minimizeDistance(Quaternion &old)
{
    // use the shortest distance
    if (dotProduct(old) < 0) {
        w *= -1.0f;
        x *= -1.0f;
        y *= -1.0f;
        z *= -1.0f;
    }
}
