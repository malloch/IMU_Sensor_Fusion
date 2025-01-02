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

// Add our own type
#ifdef SENSOR_FUSION_FLOATS
    typedef float sfFloat;
#else
    typedef double sfFloat;
#endif

// Add definitions of M_PI just in case
// Define sfFloat versions of M_PI and M_PI_2

#ifdef M_PI
    #define M_PIF        sfFloat(M_PI)
#else
    #define M_PIF        sfFloat(3.14159265358979323846)
#endif
#ifdef M_PI_2
    #define M_PI_2F      sfFloat(M_PI_2)
#else
    #define M_PI_2F      sfFloat(1.57079632679489661923f)
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
        Quaternion(sfFloat w, sfFloat x, sfFloat y, sfFloat z): w(w), x(x), y(y), z(z) {}
        Quaternion(const Quaternion&) = default;
        Quaternion& operator= (const Quaternion&) = default;

        ~Quaternion() {}

        Quaternion operator*(Quaternion& rhs);

        Quaternion inverse();
        Quaternion conjugate();
        sfFloat dotProduct(Quaternion &q);
        Quaternion slerp(Quaternion &q, sfFloat weight);
        void normalize();
        void minimizeDistance(Quaternion &q);

        // Quaternion members
        sfFloat  w;
        sfFloat  x;
        sfFloat  y;
        sfFloat  z;
    private:
    };

    typedef struct Euler
    {
        union {
            sfFloat tilt;
            sfFloat pitch;
        };
        sfFloat roll;
        union {
            sfFloat azimuth;
            sfFloat heading;
        };
    } Euler;

    void setAccelerometerValues(sfFloat x, sfFloat y, sfFloat z);
    void setMagnetometerValues(sfFloat x, sfFloat y, sfFloat z);
    void setGyroscopeDegreeValues(sfFloat x, sfFloat y, sfFloat z, sfFloat period);
    void setGyroscopeRadianValues(sfFloat x, sfFloat y, sfFloat z, sfFloat period);

    void update(sfFloat weight = 0.01);

    Quaternion quaternion;
    Euler euler;

  private:
    typedef struct Cartesian
    {
      sfFloat x;
      sfFloat y;
      sfFloat z;
      sfFloat magnitude;
    } Cartesian;

    Cartesian accel;
    Cartesian gyro;
    Cartesian gyro_bias;
    Cartesian mag;
};

#endif
