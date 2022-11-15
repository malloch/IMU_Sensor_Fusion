/***************************************************************************
 *                                                                         *
 * Sensor Fusion code for estimating orientation of an IMU                 *
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

#ifndef IMU_ORIENTATION_H
#define IMU_ORIENTATION_H

class IMU_Orientation {
  public:
    IMU_Orientation() {}
    ~IMU_Orientation() {}
  
    // Data structures
    class Quaternion
    {
    public:
        Quaternion(): w(1), x(0), y(0), z(0) {}
        Quaternion(double w, double x, double y, double z): w(w), x(x), y(y), z(z) {}
        Quaternion(const Quaternion&) = default;
        Quaternion& operator= (const Quaternion&) = default;

        ~Quaternion() {}

        Quaternion operator*(Quaternion& rhs);

        Quaternion inverse();
        Quaternion conjugate();
        double dotProduct(Quaternion &q);
        Quaternion slerp(Quaternion &q, double weight);
        void normalize();
        void minimizeDistance(Quaternion &q);

        // Quaternion members
        double  w;
        double  x;
        double  y;
        double  z;
    private:
    };

    typedef struct Euler
    {
        union {
            double tilt;
            double pitch;
        };
        double roll;
        union {
            double azimuth;
            double heading;
        };
    } Euler;

    void setAccelerometerValues(double x, double y, double z);
    void setMagnetometerValues(double x, double y, double z);
    void setGyroscopeDegreeValues(double x, double y, double z, double period);
    void setGyroscopeRadianValues(double x, double y, double z, double period);

    void update(double weight = 0.99);

    Quaternion quaternion;
    Euler euler;

  private:
    typedef struct Cartesian
    {
      double x;
      double y;
      double z;
      double magnitude;
    } Cartesian;

    Cartesian accel;
    Cartesian gyro;
    Cartesian gyro_bias;
    Cartesian mag;
};

#endif
