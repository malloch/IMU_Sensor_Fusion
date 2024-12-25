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

// Add definitions of M_PI just in case
#ifndef M_PI
    #define M_PI        3.14159265358979323846
#endif
#ifndef M_PI_2
    #define M_PI_2      1.57079632679489661923
#endif

class IMU_Orientation {
  public:
    IMU_Orientation(): quaternion(1,0,0,0) {}
    ~IMU_Orientation() {}
  
    // Data structures
    class Quaternion
    {
    public:
        Quaternion(): w(1), x(0), y(0), z(0) {}
        Quaternion(float w, float x, float y, float z): w(w), x(x), y(y), z(z) {}
        Quaternion(const Quaternion&) = default;
        Quaternion& operator= (const Quaternion&) = default;

        ~Quaternion() {}

        Quaternion operator*(Quaternion& rhs);

        Quaternion inverse();
        Quaternion conjugate();
        float dotProduct(Quaternion &q);
        Quaternion slerp(Quaternion &q, float weight);
        void normalize();
        void minimizeDistance(Quaternion &q);

        // Quaternion members
        float  w;
        float  x;
        float  y;
        float  z;
    private:
    };

    typedef struct Euler
    {
        union {
            float tilt;
            float pitch;
        };
        float roll;
        union {
            float azimuth;
            float heading;
        };
    } Euler;

    void setAccelerometerValues(float x, float y, float z);
    void setMagnetometerValues(float x, float y, float z);
    void setGyroscopeDegreeValues(float x, float y, float z, float period);
    void setGyroscopeRadianValues(float x, float y, float z, float period);

    void update(float weight = 0.01);

    Quaternion quaternion;
    Euler euler;

  private:
    typedef struct Cartesian
    {
      float x;
      float y;
      float z;
      float magnitude;
    } Cartesian;

    Cartesian accel;
    Cartesian gyro;
    Cartesian gyro_bias;
    Cartesian mag;
};

#endif
