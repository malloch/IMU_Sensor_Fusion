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

#include <cmath>

#define SENSOR_FUSION_FLOATS

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

class Cartesian;

class Quaternion {
public:
    Quaternion(): w(1), x(0), y(0), z(0) {}
    Quaternion(sfFloat w, sfFloat x, sfFloat y, sfFloat z): w(w), x(x), y(y), z(z) {}
    Quaternion(const Quaternion&) = default;
    Quaternion(const Cartesian&);
    Quaternion& operator= (const Quaternion&) = default;

    ~Quaternion() {}

    Quaternion& set(sfFloat _w, sfFloat _x, sfFloat _y, sfFloat _z)
    {
        w = _w;
        x = _x;
        y = _y;
        z = _z;
        return *this;
    }

    Quaternion operator*(Quaternion& rhs);

    Quaternion inverse();
    Quaternion conjugate();
    sfFloat dotProduct(Quaternion &q);
    Quaternion slerp(Quaternion &q, sfFloat weight);
    void normalize();
    void minimizeDistance(Quaternion &q);

    // Quaternion members
    union {
        sfFloat elements[4];
        struct {
            sfFloat  w;
            sfFloat  x;
            sfFloat  y;
            sfFloat  z;
        };
    };
private:
};

class Cartesian {
  public:
    Cartesian(): x(0), y(0), z(0), updated(false), num_samples(0) {}
    Cartesian(sfFloat x, sfFloat y, sfFloat z): x(x), y(y), z(z), updated(true), num_samples(0) {}
    Cartesian& operator= (const Cartesian& rhs) {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        updated = true;
        return *this;
    }

    ~Cartesian() {}
private:
    inline void update_means()
    {
        int i;
        ++num_samples;
        float multiplier = 1.0 / num_samples;
        // float complement = 1.0 - multiplier;
        for (i = 0; i < 3; i++) {
            // means[i] = elements[i] * multiplier + means[i] * (1.0 - complement);
            means[i] += (elements[i] - means[i]) * multiplier;
        }
    }

    // also need func for resetting mean

public:
    Cartesian& set(sfFloat _x, sfFloat _y, sfFloat _z)
    {
        // each time we call set() we will also update the running mean
        x = _x;
        y = _y;
        z = _z;
        updated = true;
        update_means();
        return *this;
    }

    inline Cartesian& set(sfFloat *values)
    {
        return set(values[0], values[1], values[2]);
    }

    virtual Cartesian& operator+=(const Cartesian& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        updated = true;
        return *this;
    };

    friend Cartesian operator+(Cartesian lhs, const Cartesian& rhs)
    {
        lhs += rhs;
        return lhs;
    };

    Cartesian& operator-=(Cartesian rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        updated = true;
        return *this;
    };

    friend Cartesian operator-(Cartesian lhs, const Cartesian& rhs)
    {
        lhs -= rhs;
        return lhs;
    };

    // Cartesian operator*(Cartesian& rhs)
    // {
    //     return Cartesian(x * rhs.x, y * rhs.y, z * rhs.z);
    // };

    // Cartesian operator*=(Cartesian& rhs)
    // {
    //     x *= rhs.x;
    //     y *= rhs.y;
    //     z *= rhs.z;
    // };

    // Cartesian operator*(float coef)
    // {
    //     return Cartesian(x * coef, y * coef, z * coef);
    // };

    Cartesian& operator*=(float coef)
    {
        x *= coef;
        y *= coef;
        z *= coef;
        updated = true;
        return *this;
    };

    friend Cartesian operator*(Cartesian lhs, float coef)
    {
        lhs.x *= coef;
        lhs.y *= coef;
        lhs.z *= coef;
        lhs.updated = true;
        return lhs;
    };

    friend Cartesian abs(Cartesian c)
    {
        return Cartesian(fabsf(c.x), fabsf(c.y), fabsf(c.z));
    }

    Cartesian& abs()
    {
        x = fabsf(x);
        y = fabsf(y);
        z = fabsf(z);
        updated = true;
        return *this;
    }

    virtual Cartesian& reset()
    {
        x = y = z = 0;
        updated = false;
        return *this;
    }

    Cartesian& rotate(Quaternion& q);

    Cartesian& reset_means()
    {
        num_samples = 0;
        return *this;
    }

    union {
        sfFloat elements[3];
        struct {
            sfFloat x;
            sfFloat y;
            sfFloat z;
        };
    };
    sfFloat means[3];

    // sfFloat magnitude;
    bool updated;
private:
    int num_samples;
};

class Euler {
  public:
    Euler(): tilt(0), roll(0), azimuth(0) {}
    Euler(sfFloat tilt, sfFloat roll, sfFloat azimuth): tilt(tilt), roll(roll), azimuth(azimuth) {}
    Euler& operator= (const Euler&) = default;

    ~Euler() {}

    Euler operator+(Euler rhs)
    {
        return Euler(tilt + rhs.tilt, roll + rhs.roll, azimuth + rhs.azimuth);
    };

    Euler operator*(Euler rhs)
    {
        return Euler(tilt * rhs.tilt, roll * rhs.roll, azimuth * rhs.azimuth);
    };

    Euler operator*(float coef)
    {
        return Euler(tilt * coef, roll * coef, azimuth * coef);
    };

    union {
        sfFloat elements[3];
        struct {
            union {
                sfFloat tilt;
                sfFloat pitch;
            };
            sfFloat roll;
            union {
                sfFloat azimuth;
                sfFloat heading;
            };
        };
    };
};

class Leaky_Cartesian : public Cartesian {
  public:
    Leaky_Cartesian(): gain(1), ff1(0), ff2(0), fb1(0), fb2(0) {}
    Leaky_Cartesian(sfFloat gain, sfFloat ff1, sfFloat ff2, sfFloat fb1, sfFloat fb2): gain(gain), ff1(ff1), ff2(ff2), fb1(fb1), fb2(fb2) {}
    ~Leaky_Cartesian() {}

    Leaky_Cartesian& set_coefficients(sfFloat _gain, sfFloat _ff1, sfFloat _ff2, sfFloat _fb1, sfFloat _fb2)
    {
        gain = _gain;
        ff1  = _ff1;
        ff2  = _ff2;
        fb1  = _fb1;
        fb2  = _fb2;
        return *this;
    }

    Leaky_Cartesian& set_coefficients(float* coefficients)
    {
        gain = coefficients[0];
        ff1  = coefficients[1];
        ff2  = coefficients[2];
        fb1  = coefficients[3];
        fb2  = coefficients[4];
        return *this;
    }

    Leaky_Cartesian& set_filter(float SR, float cutoff, float Q)
    {
        if ((SR > 0) && (cutoff > 0) && (Q > 0)) {
            float w0 = cutoff / SR * M_PI * 2.0;
            float a = sin(w0) / (Q * 2.0);
            float norm = 1.0 / (1 + a);

            gain = ff2 = (1.0 + cos(w0)) / 2.0 * norm;
            ff1 = -(1.0 + cos(w0)) * norm;
            fb1 = -2.0 * cos(w0) * norm;
            fb2 = (1.0 - a) * norm;
        }
        return *this;
    }

    Leaky_Cartesian& operator+=(const Cartesian& rhs) override
    {
        // add value to internal store
        input += rhs;
        // we don't want raw integrated value to get too large and lose FP precision
        for (int i = 0; i < 3; i++) {
            if (elements[0] > 1000) {
                elements[0] -= 1000;
                ff_samps[0].elements[0] -= 1000;
                ff_samps[1].elements[0] -= 1000;
            }
            else if (elements[0] < -1000) {
                elements[0] += 1000;
                ff_samps[0].elements[0] += 1000;
                ff_samps[1].elements[0] += 1000;
            }
        }
        Cartesian::operator=(  input       * gain
                             + ff_samps[0] * ff1
                             + ff_samps[1] * ff2
                             - fb_samps[0] * fb1
                             - fb_samps[1] * fb2);
        ff_samps[1] = ff_samps[0];
        ff_samps[0] = input;
        fb_samps[1] = fb_samps[0];
        // fb_samps[0] = (*this);
        fb_samps[0].set(elements);
        return *this;
    }

    Leaky_Cartesian& reset() override
    {
        set(0, 0, 0);
        updated = false;
        input.set(0, 0, 0);
        ff_samps[0] = ff_samps[1] = fb_samps[0] = fb_samps[1] = input;
        return *this;
    }

  private:
    // filter coefficients
    sfFloat gain;
    sfFloat ff1;
    sfFloat ff2;
    sfFloat fb1;
    sfFloat fb2;

    // circular buffers for delayed samples
    Cartesian input;
    Cartesian ff_samps[2];
    Cartesian fb_samps[2];
};

class IMU_Orientation {
  public:
    IMU_Orientation() {}
    ~IMU_Orientation() {}

    void setAccelerometerValues(sfFloat x, sfFloat y, sfFloat z);
    inline void setAccelerometerValues(sfFloat *values)
        { setAccelerometerValues(values[0], values[1], values[2]); }

    void setMagnetometerValues(sfFloat x, sfFloat y, sfFloat z);
    inline void setMagnetometerValues(sfFloat *values)
        { setMagnetometerValues(values[0], values[1], values[2]); }

    void setGyrometerDegreeValues(sfFloat x, sfFloat y, sfFloat z);
    inline void setGyrometerDegreeValues(sfFloat *values)
        { setGyrometerDegreeValues(values[0], values[1], values[2]); }

    void setGyrometerRadianValues(sfFloat x, sfFloat y, sfFloat z);
    inline void setGyrometerRadianValues(sfFloat *values)
        { setGyrometerRadianValues(values[0], values[1], values[2]); }

    void update(sfFloat period, sfFloat weight = 0.01);

    void reset();

    struct {
        Quaternion quaternion;
        Euler euler;
    } orientation;

    struct {
        Cartesian accelerometer;
        Cartesian gyrometer;
        Cartesian magnetometer;

        struct {
            Cartesian accelerometer;
            Cartesian gyrometer;
            Cartesian magnetometer;
        } smoothed;
    } sensors;

    struct {
        Cartesian acceleration;
        Cartesian angular_velocity;
        Cartesian magnetic_field;
    } local_frame;

    struct {
        Cartesian acceleration;
        Leaky_Cartesian linear_velocity;
        Cartesian angular_velocity;
        Leaky_Cartesian displacement;
    } world_frame;

    Quaternion debug;

private:
    Cartesian gyrometer_bias;
    Cartesian gyrometer_ema;
    Cartesian gyrometer_emd;
    Cartesian accel_wf_bias;
};

#endif
