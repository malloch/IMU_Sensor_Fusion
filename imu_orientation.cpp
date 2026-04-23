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
#include <cstdio>

#define DEGREE_TO_RAD sfFloat(M_PIF / sfFloat(180.))
#define GYROMETER_BIAS_COEFF sfFloat(0.999)

#ifdef SENSOR_FUSION_FLOATS
using namespace std;
#endif

static sfFloat minimum(sfFloat a, sfFloat b) {
  return a < b ? a : b;
}

void IMU_Orientation::setAccelerometerValues(sfFloat x, sfFloat y, sfFloat z) {
  sensors.accelerometer.set(x, y, z);
}

void IMU_Orientation::setGyrometerRadianValues(sfFloat x, sfFloat y, sfFloat z) {
  // depending on IMU model, axes need to be swapped and inverted
  Cartesian new_val(-y, x, z);

  // remove current bias estimate and store the value
  sensors.gyrometer = new_val - gyrometer_bias;

  // use a fast-adapting emd to estimate whether gyro is at rest
  Cartesian diff(new_val - gyrometer_ema);
  gyrometer_ema += diff * 0.1;
  gyrometer_emd += (abs(diff) - gyrometer_emd) * 0.1;

  // update bias estimate with a dynamic adaptation rate
  sfFloat bias_coef = (gyrometer_emd.x + gyrometer_emd.y + gyrometer_emd.z) < 0.1 ? 0.001 : 0.00001;
  gyrometer_bias += (new_val - gyrometer_bias) * bias_coef;

  // updated flag should have been set during assignment above
  sensors.gyrometer.updated = true;
}

void IMU_Orientation::setGyrometerDegreeValues(sfFloat x, sfFloat y, sfFloat z) {
  // convert to radians
  x *= DEGREE_TO_RAD;
  y *= DEGREE_TO_RAD;
  z *= DEGREE_TO_RAD;

  setGyrometerRadianValues(x, y, z);
}

void IMU_Orientation::setMagnetometerValues(sfFloat x, sfFloat y, sfFloat z) {
  sensors.magnetometer.set(x, y, z);
}

void IMU_Orientation::update(sfFloat period, sfFloat weight) {
  Quaternion Q = orientation.quaternion;
  Quaternion q_az, q_tilt_inv, q_roll_inv;
  Cartesian c_temp;

  sfFloat half_roll, half_tilt, half_azimuth;

  /**********************/
  /*   GYROMETER DATA   */
  /**********************/

  if (sensors.gyrometer.updated) {
    /* Use a 2-sample average for integration (assumes linear ramp over <period> from last sample)
     * The last sample of gyro data is stored in local_frame.angular_velocity */
    Cartesian rotation = (sensors.gyrometer + local_frame.angular_velocity) * 0.5 * period;

    local_frame.angular_velocity = sensors.gyrometer;

    // construct quaternion from rotation axes
    Quaternion q_rotation(cos((rotation.x + rotation.y + rotation.z) * sfFloat(0.5)),
                          sin(rotation.x * sfFloat(0.5)),
                          sin(rotation.y * sfFloat(0.5)),
                          sin(rotation.z * sfFloat(0.5)));

    // integrate latest rotation quaternion with previous orientation
    Q = Q * q_rotation;

    // test: remove azimuth
    // Quaternion tmp(0, 1, 0, 0);
    // Quaternion conj = quaternion.conjugate();
    // tmp = tmp * conj;
    // tmp = quaternion * tmp;
    // quaternion = tmp;

    // calculate inverse quaternion for tilt estimate
    // half_tilt = euler.tilt * sfFloat(0.5);
    // TODO: optimize by building inverse quaternion directly
    // Quaternion q_tilt(cos(half_tilt), sin(half_tilt), 0, 0);
    // q_tilt_inv = q_tilt.inverse();

    // calculate inverse quaternion for roll estimate
    // half_roll = euler.roll * sfFloat(0.5);
    // TODO: optimize by building inverse quaternion directly
    // Quaternion q_roll(cos(half_roll), 0, sin(half_roll), 0);
    // q_roll_inv = q_roll.inverse();

    // Compute euler angles
    /* extract tilt (and azimuth?) by rotating vector [1,0,0]: Q * [0,1,0,0] * Q^-1
         * before optimization:
            Quaternion right(0,1,0,0);
            Quaternion conj = quaternion.conjugate();
            right = right * conj;
            right = quaternion * right;
            orientation.euler.tilt = atan2(right.x, right.y);
            orientation.euler.azimuth = temp.z * M_PI_2;
         */
    // c_temp.x = Q.w * Q.w + Q.x * Q.x - Q.y * Q.y - Q.z * Q.z;
    // c_temp.y = 2 * (Q.w * Q.z + Q.x * Q.y);
    // c_temp.z = 2 * (Q.x * Q.z - Q.w * Q.y);
    // orientation.euler.tilt = atan2(c_temp.x, c_temp.y);
    // orientation.euler.azimuth = c_temp.z * M_PI_2;

    // extract roll by rotating vector [?, ?, ?]...
    // TBD:

    // TODO:
    // orientation.euler.roll =
    // TODO:
    // orientation.euler.azimuth =

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
    c_temp.x = 2 * (Q.x * Q.y - Q.w * Q.z);
    c_temp.y = Q.w * Q.w - Q.x * Q.x + Q.y * Q.y - Q.z * Q.z;
    c_temp.z = 2 * (Q.w * Q.x + Q.y * Q.z);
    // orientation.euler.tilt = atan2(c_temp.y, -c_temp.x);
    // orientation.euler.azimuth = c_temp.z * M_PI_2F;
    orientation.euler.tilt = atan2(c_temp.x, c_temp.y);
    // orientation.euler.roll = atan2(c_temp.y, c_temp.z) - M_PI_2F;
    // if (orientation.euler.roll < -M_PIF)
    //     orientation.euler.roll += M_PIF * 2;

    /* extract roll by rotating vector [0,0,1]: Q * [0,0,0,1] * Q^-1
        * before optimization:
            Quaternion up(0,0,0,1);
            temp = up * conj;
            temp = quaternion * temp;
            euler.roll = atan2(cos(euler.azimuth) * temp.x + sin(euler.azimuth) * -temp.y, temp.z);
        */
    // c_temp.x = 2 * (Q.x * Q.z + Q.w * Q.y);
    // c_temp.y = 2 * (Q.y * Q.z - Q.w * Q.x);
    // c_temp.z = Q.w * Q.w - Q.x * Q.x - Q.y * Q.y + Q.z * Q.z;
    // euler.roll = atan2(cos(euler.azimuth) * c_temp.x + sin(euler.azimuth) * -c_temp.y, c_temp.z);
    // orientation.euler.roll = atan2(c_temp.y, c_temp.z);
    float temp = 2. * (Q.w * Q.y - Q.x * Q.z);
    orientation.euler.roll = -M_PI_2F + 2. * atan2(sqrt(1 + temp), 1. - 2. * sqrt(1. - temp));

    // orientation.euler.azimuth = -atan2(c_temp.z, c_temp.x) + M_PI_2F;
    // if (orientation.euler.azimuth > M_PIF)
    //     orientation.euler.azimuth -= M_PIF * 2;

    orientation.euler.azimuth = atan2(2. * (Q.w * Q.z + Q.x * Q.y), 1. - 2. * (Q.y * Q.y + Q.z * Q.z));
  } else {
    goto done;
  }

  /**********************/
  /* MAGNETOMETER DATA  */
  /**********************/
  if (sensors.magnetometer.updated) {
    // assuming magnetometer data has already been calibrated

    // build quaternions from polar representation of magnetometer data
    // half_roll = atan2(magnetometer.z, magnetometer.y) * sfFloat(0.5);
    // Quaternion q_mag_roll(cos(half_roll), 0, sin(half_roll), 0);

    // magnetometer.magnitude = sqrt(pow(magnetometer.z, sfFloat(2)) + pow(magnetometer.y, sfFloat(2)));
    // half_tilt = atan2(magnetometer.x, magnetometer.magnitude) * sfFloat(0.5);

    // Quaternion q_mag_tilt(cos(half_tilt), sin(half_tilt), 0, 0);
    // Quaternion q_mag = q_mag_tilt * q_mag_roll;

    // // rotate the magnetometer quaternion
    // q_mag = q_mag * q_roll_inv;
    // q_mag = q_mag * q_tilt_inv;

    // /* extract azimuth by rotating vector [0,0,1]: Q * [0,0,0,1] * Q^-1
    // * before optimization:
    //     conj = q_magnetometer.conjugate();
    //     temp = up * conj;
    //     temp = q_mag * temp;
    //     sfFloat half_azimuth = atan2(temp.x, temp.y) * 0.5;
    // */
    // c_temp.x = 2 * (q_magnetometer.x * q_magnetometer.z + q_magnetometer.w * q_magnetometer.y);
    // c_temp.y = 2 * (q_magnetometer.y * q_magnetometer.z - q_magnetometer.w * q_magnetometer.x);
    // half_azimuth = atan2(c_temp.x, c_temp.y) * sfFloat(0.5);
  } else {
    // comment out this block if magnetometer is present
    // strategy for implementation with no magnetometer
    // 1) extract azimuth from fused orientation
    // 2) slowly steer axzimuth toward 0 or PI depending on whether sensor is upside down
    // 3) in ambiguous orientations we could just avoid nudge
    // 4) TODO: add startup sequence with faster adaptation

    float diff = (orientation.euler.azimuth > 0.) == (abs(orientation.euler.azimuth) < M_PIF) ? -0.0174 : 0.0174;
    orientation.euler.azimuth += diff;

    // use azimuth from gyro integration for sensor fusion below
    half_azimuth = orientation.euler.azimuth * sfFloat(0.5);
  }

  /**********************/
  /* ACCELEROMETER DATA */
  /**********************/

  if (sensors.accelerometer.updated) {
    // TODO: check for free-fall condition
    // TODO: consider only updating orientation if acceleration magnitude is ~9.8m/s

    // build quaternions from polar representation of accelerometer data
    {
      float xx = sensors.accelerometer.x * sensors.accelerometer.x;
      float yy = sensors.accelerometer.y * sensors.accelerometer.y;
      float zz = sensors.accelerometer.z * sensors.accelerometer.z;

      float magnitude_xz = sqrt(xx + zz);
      half_roll = atan2(sensors.accelerometer.y, magnitude_xz);

      float magnitude_yz = sqrt(yy + zz);
      float magnitude_xy = sqrt(xx + yy);
      half_tilt = atan2(atan2(sensors.accelerometer.x, magnitude_yz),
                        atan2(sensors.accelerometer.z, magnitude_xy));
      half_tilt *= minimum(M_PI_2F - abs(half_roll), 1.0) * sfFloat(0.5);
      half_roll *= sfFloat(0.5);
    }

    /* before optimization: */
    // Quaternion q_accel_roll(cos(half_roll), 0, sin(half_roll), 0);
    // Quaternion q_accel_tilt(cos(half_tilt), sin(half_tilt), 0, 0);
    // Quaternion q_accel = q_accel_roll * q_accel_tilt;
    /* optimized version (4 multiplications instead of 16 multiplication and 12 additions) */
    float chr = cos(half_roll);
    float shr = sin(half_roll);
    float cht = cos(half_tilt);
    float sht = sin(half_tilt);
    // Quaternion q_accel(chr * cht, shr * cht, -shr * sht, chr * sht);
    Quaternion q_accel(chr * cht, chr * sht, shr * cht, -shr * sht);

    q_az.w = cos(half_azimuth);
    q_az.x = q_az.y = 0;
    q_az.z = sin(half_azimuth);

    // construct quaternion from combined accelerometer and azimuth data
    Quaternion q_accel_mag = q_az * q_accel;

    // SLERP between stored orientation and new accel + mag estimate
    Quaternion delayed = Q;
    // quaternion = quaternion.slerp(q_accel_mag, sfFloat(weight));
    // quaternion = quaternion.slerp(q_accel, sfFloat(weight));
    Q = Q.slerp(q_accel_mag, 0.001);

    // use the shortest distance from previous orientation
    Q.minimizeDistance(delayed);

    /* Calculate world-frame acceleration */
    Cartesian accel_wf(-sensors.accelerometer.y, sensors.accelerometer.x, sensors.accelerometer.z);

    accel_wf.rotate(Q);

    // rearrange elements
    float temp = accel_wf.x;
    accel_wf.x = accel_wf.y;
    accel_wf.y = temp;

    // subtract gravity acceleration
    accel_wf.z -= 9.8;

    for (int i = 0; i < 3; i++) {
      // TODO: smooth the world-frame acceleration used for serial/libmapper output
      // world_frame.acceleration.elements[i] *= 0.9;
      // world_frame.acceleration.elements[i] += q_accel_wf.elements[i+1] * 0.1;

      world_frame.acceleration.elements[i] = accel_wf.elements[i];
    }

    // integrate world-frame acceleration to estimate velocity
    // TODO: integration should use a 2-sample moving average
    world_frame.linear_velocity += world_frame.acceleration * period;

    // integrate world-frame velocity to estimate displacement
    // TODO: integration should use a 2-sample moving average
    world_frame.displacement += world_frame.linear_velocity * period;
  }

  orientation.quaternion = Q;

done:
  sensors.accelerometer.updated = sensors.gyrometer.updated = sensors.magnetometer.updated = 0;
}

void IMU_Orientation::reset() {
  // reset orienation estimate
  orientation.quaternion.set(1, 0, 0, 0);

  // reset accumulators
  world_frame.linear_velocity.reset();
  world_frame.displacement.reset();

  // reset bias estimates
  gyrometer_bias.set(0, 0, 0);
  gyrometer_ema.set(0, 0, 0);
  gyrometer_emd.set(0, 0, 0);
}

Quaternion Quaternion::operator*(Quaternion& rhs) {
  Quaternion o;
  o.w = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
  o.x = x * rhs.w + w * rhs.x + y * rhs.z - z * rhs.y;
  o.y = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
  o.z = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;
  return o;
}

Quaternion Quaternion::inverse() {
  Quaternion o;
  sfFloat norm_squared = w * w + x * x + y * y + z * z;
  if (norm_squared == 0) norm_squared = sfFloat(0.0000001);
  o.w = w / norm_squared;
  o.x = x * -1 / norm_squared;
  o.y = y * -1 / norm_squared;
  o.z = z * -1 / norm_squared;
  return o;
}

Quaternion Quaternion::conjugate() {
  return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::slerp(Quaternion& q, sfFloat weight) {
  Quaternion o;
  sfFloat dot = dotProduct(q);
  if (dot > sfFloat(0.9995f)) {
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

  sfFloat theta_0 = acos(dot);
  sfFloat theta = (sfFloat(0.) < theta_0 && theta_0 < M_PI_2F) ? theta_0 * weight : (theta_0 - M_PIF) * weight;

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

sfFloat Quaternion::dotProduct(Quaternion& q) {
  return w * q.w + x * q.x + y * q.y + z * q.z;
}

void Quaternion::normalize() {
  sfFloat norm = sqrt(w * w + x * x + y * y + z * z);
  if (norm == 0)
    norm = sfFloat(0.0000001);
  sfFloat inv_norm = sfFloat(1.0) / norm;
  w *= inv_norm;
  x *= inv_norm;
  y *= inv_norm;
  z *= inv_norm;
}

void Quaternion::minimizeDistance(Quaternion& rhs) {
  // use the shortest distance
  if (dotProduct(rhs) < 0) {
    w *= sfFloat(-1.0);
    x *= sfFloat(-1.0);
    y *= sfFloat(-1.0);
    z *= sfFloat(-1.0);
  }
}

Cartesian& Cartesian::rotate(Quaternion& q) {
  // before optimization:
  // Quaternion q_vector(0, x, y, z);
  // Quaternion q_conj = q.conjugate();
  // q_vector = q_vector * conj;
  // q_vector = q * q_vector;

  // optimized:
  Quaternion o;

  // multiply vector quaternion by conjugate of rotation quaternion
  o.w = x * q.x + y * q.y + z * q.z;
  o.x = x * q.w - y * q.z + z * q.y;
  o.y = x * q.z + y * q.w - z * q.x;
  o.z = y * q.x + z * q.w - x * q.y;

  // multiply rotation quaternion by result, ignoring w element
  x = q.x * o.w + q.w * o.x + q.y * o.z - q.z * o.y;
  y = q.w * o.y - q.x * o.z + q.y * o.w + q.z * o.x;
  z = q.w * o.z + q.x * o.y - q.y * o.x + q.z * o.w;

  return *this;
}
