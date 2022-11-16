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

#define DEGREE_TO_RAD (M_PI / 180.)
#define GYRO_BIAS_COEFF 0.99999

void IMU_Orientation::setAccelerometerValues(double x, double y, double z)
{
    accel.x = x;
    accel.y = y;
    accel.z = z;
}

void IMU_Orientation::setGyroscopeRadianValues(double x, double y, double z, double period_sec)
{
    // convert to delta radians before storing
    // x and y axes need to be inverted
    gyro.x = x * -period_sec;
    gyro.y = y * -period_sec;
    gyro.z = z * period_sec;
}

void IMU_Orientation::setGyroscopeDegreeValues(double x, double y, double z, double period_sec)
{
    // convert to delta radians before storing
    // x and y axes need to be inverted
    gyro.x = x * -DEGREE_TO_RAD * period_sec;
    gyro.y = y * -DEGREE_TO_RAD * period_sec;
    gyro.z = z * DEGREE_TO_RAD * period_sec;
}

void IMU_Orientation::setMagnetometerValues(double x, double y, double z)
{
    mag.x = x;
    mag.y = y;
    mag.z = z;
}

void IMU_Orientation::update(double weight)
{
    Quaternion q_temp;
    // Euler etemp;
    Cartesian c_temp;

    /**********************/
    /* ACCELEROMETER DATA */
    /**********************/

    // build quaternions from polar representation of accelerometer data
    accel.magnitude = sqrt(pow(accel.z, 2) + pow(accel.y, 2));

    double half_roll = atan2(accel.z, accel.y) * 0.5;
    Quaternion q_accel_roll(cos(half_roll), 0, sin(half_roll), 0);
    
    double half_tilt = atan2(accel.x, accel.magnitude) * 0.5;
    Quaternion q_accel_tilt(cos(half_tilt), sin(half_tilt), 0, 0);

    accel.magnitude = sqrt(pow(accel.x, 2) + pow(accel.magnitude, 2));

    Quaternion q_accel = q_accel_tilt * q_accel_roll;

    /**********************/
    /*   GYROSCOPE DATA   */
    /**********************/

    // continuously adjust for changing gyro bias
    gyro_bias.x = gyro_bias.x * GYRO_BIAS_COEFF + gyro.x * (1.0 - GYRO_BIAS_COEFF);
    gyro_bias.y = gyro_bias.y * GYRO_BIAS_COEFF + gyro.x * (1.0 - GYRO_BIAS_COEFF);
    gyro_bias.z = gyro_bias.z * GYRO_BIAS_COEFF + gyro.x * (1.0 - GYRO_BIAS_COEFF);

    gyro.x -= gyro_bias.x;
    gyro.y -= gyro_bias.y;
    gyro.z -= gyro_bias.z;

    // construct quaternion from gyroscope axes
    Quaternion q_gyro(cos((gyro.x + gyro.y + gyro.z) * 0.5), sin(gyro.z * 0.5),
                      sin(gyro.x * 0.5), sin(gyro.y * 0.5));

    // integrate latest gyro quaternion with previous orientation
    quaternion = quaternion * q_gyro;

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
    euler.azimuth = atan2(c_temp.x, c_temp.y);
    euler.tilt = c_temp.z * M_PI_2;

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
    euler.roll = atan2(cos(euler.azimuth) * c_temp.x + sin(euler.azimuth) * -c_temp.y, c_temp.z);

    // calculate inverse quaternion for tilt estimate
    half_tilt = euler.tilt * 0.5;
    // TODO: optimize by building inverse quaternion directly
    Quaternion q_tilt_inv(cos(half_tilt), sin(half_tilt), 0, 0);
    q_tilt_inv = q_tilt_inv.inverse();

    // calculate inverse quaternion for roll estimate
    half_roll = euler.roll * 0.5;
    // TODO: optimize by building inverse quaternion directly
    Quaternion q_roll_inv(cos(half_roll), 0, sin(half_roll), 0);
    q_roll_inv = q_roll_inv.inverse();

    /**********************/
    /* MAGNETOMETER DATA  */
    /**********************/

    // assuming magnetometer data has already been calibrated

    // build quaternions from polar representation of magnetometer data
    half_roll = atan2(mag.z, mag.y) * -0.5;
    Quaternion q_mag_roll(cos(half_roll), 0, sin(half_roll), 0);

    mag.magnitude = sqrt(pow(mag.z, 2) + pow(mag.y, 2));
    half_tilt = atan2(mag.x, mag.magnitude) * 0.5;
    
    Quaternion q_mag_tilt(cos(half_tilt), sin(half_tilt), 0, 0);
    Quaternion q_mag = q_mag_tilt * q_mag_roll;

    // rotate the magnetometer quaternion
    q_mag = q_mag * q_roll_inv;
    q_mag = q_mag * q_tilt_inv;

    /* extract azimuth by rotating vector [0,0,1]: Q * [0,0,0,1] * Q^-1
     * before optimization:
        conj = q_mag.conjugate();
        temp = up * conj;
        temp = q_mag * temp;
        double half_azimuth = atan2(temp.x, temp.y) * 0.5;
     */
    c_temp.x = 2 * (q_mag.x * q_mag.z + q_mag.w * q_mag.y);
    c_temp.y = 2 * (q_mag.y * q_mag.z - q_mag.w * q_mag.x);
    double half_azimuth = atan2(c_temp.x, c_temp.y) * 0.5;

    // replace q_mag with just azimuth
    q_mag.w = cos(half_azimuth);
    q_mag.x = q_mag.y = 0;
    q_mag.z = sin(half_azimuth);

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
    if (norm_squared == 0) norm_squared = 0.0000001;
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

IMU_Orientation::Quaternion IMU_Orientation::Quaternion::slerp(Quaternion &q, double weight)
{
    Quaternion o;
    double dot = dotProduct(q);
    if (dot > 0.9995) {
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

    double theta_0 = acos(dot);
    double theta = (0. < theta_0 && theta_0 < M_PI_2) ? theta_0 * weight : (theta_0 - M_PI) * weight;

    o.w = q.w - w * dot;
    o.x = q.x - x * dot;
    o.y = q.y - y * dot;
    o.z = q.z - z * dot;
    o.normalize();

    o.w = w * cos(theta) + o.w * sin(theta);
    o.x = x * cos(theta) + o.x * sin(theta);
    o.y = y * cos(theta) + o.y * sin(theta);
    o.z = z * cos(theta) + o.z * sin(theta);
    o.normalize();
    return o;
}

double IMU_Orientation::Quaternion::dotProduct(Quaternion &q)
{
    return w * q.w + x * q.x + y * q.y + z * q.z;
}

void IMU_Orientation::Quaternion::normalize()
{
    double norm = sqrt(w * w + x * x + y * y + z * z);
    if (norm == 0)
      norm = 0.0000001;
    double inv_norm = 1.0 / norm;
    w *= inv_norm;
    x *= inv_norm;
    y *= inv_norm;
    z *= inv_norm;
}

void IMU_Orientation::Quaternion::minimizeDistance(Quaternion &old)
{
    // use the shortest distance
    if (dotProduct(old) < 0) {
        w *= -1.0;
        x *= -1.0;
        y *= -1.0;
        z *= -1.0;
    }
}
