# IMU Sensor Fusion

Complementary-filter sensor fusion code for combining accelerometer, magnetometer, and rate-gyroscope data into a single stable estimate of orientation.

This code is currently adapted for the LSM9DS1 IMU but should be trivial to adapt to other sensors.

## Usage example

~~~ C++
#include "imu_orientation.h"

IMU_Orientation orientation;

// set raw data values read from IMU sensors
orientation.setAccelerometerValues(accel_x, accel_y, accel_z);
orientation.setMagnetometerValues(mag_x, mag_y, mag_z);
orientation.setGyroscopeValues(gyro_x, gyro_y, gyro_z);

// update sensor fusion
orientation.update();

// print the estimated quaternion
std::cout << "Quaternion: "
          << orientation.quaternion.w << ", "
          << orientation.quaternion.x << ", "
          << orientation.quaternion.y << ", "
          << orientation.quaternion.z << std::endl;

// print the estimated Euler angles
std::cout << "Euler angles: "
          << orientation.euler.tilt << ", "
          << orientation.euler.roll << ", "
          << orientation.euler.azimuth << ", "
          << std::endl;

~~~
