// AHRS: Attitude and Reference Heading System

// ahrs.h
#ifndef AHRS_H
#define AHRS_H

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} Quaternion;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} EulerAngles;

typedef struct
{
    Quaternion q;
    float alpha;
} AHRS;

void ahrs_init(AHRS *ahrs, float alpha);

void ahrs_update(
    AHRS *ahrs,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float mx, float my, float mz,
    float dt);

void ahrs_get_euler(
    AHRS *ahrs,
    EulerAngles *euler);

Quaternion ahrs_get_quaternion(
    AHRS *ahrs);

#endif

------------------------------------------------------------------------------------------------------------------------------------------------------------------

// ahrs.c
#include "ahrs.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static void normalize_quaternion(Quaternion *q)
{
    float norm = sqrtf(q->w * q->w +q->x * q->x + q->y * q->y + q->z * q->z);

    if(norm > 0.0f)
    {
        q->w /= norm;
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
    }
}

static void get_accelerometer_angles(float ax, float ay, float az, float *roll, float *pitch)
{
    *roll = atan2f(ay, az);
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
}

static float get_magnetometer_yaw(float mx, float my, float mz, float roll, float pitch)
{
    float mx_comp = mx * cosf(pitch) + mz * sinf(pitch);
    float my_comp = mx * sinf(roll) * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);
    return atan2f(-my_comp, mx_comp);
}

static Quaternion euler_to_quaternion(float roll, float pitch, float yaw)
{
    Quaternion q;

    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

static void quaternion_to_euler(const Quaternion *q, EulerAngles *euler)
{
    euler->roll = atan2f(2.0f * (q->w * q->x + q->y * q->z), 1.0f - 2.0f * (q->x * q->x + q->y * q->y)) * 180.0f / M_PI;
    euler->pitch = asinf(2.0f * (q->w * q->y - q->z * q->x)) * 180.0f / M_PI;
    euler->yaw = atan2f(2.0f * (q->w * q->z + q->x * q->y), 1.0f - 2.0f * (q->y * q->y + q->z * q->z)) * 180.0f / M_PI;
}

void ahrs_init(AHRS *ahrs, float alpha)
{
    ahrs->q.w = 1.0f;
    ahrs->q.x = 0.0f;
    ahrs->q.y = 0.0f;
    ahrs->q.z = 0.0f;

    ahrs->alpha = alpha;
}

void ahrs_update(AHRS *ahrs, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
    float roll;
    float pitch;
    float yaw;

    get_accelerometer_angles(ax, ay, az, &roll, &pitch);

    yaw = get_magnetometer_yaw(mx, my, mz, roll, pitch);

    Quaternion qa = euler_to_quaternion(roll, pitch, yaw);

    gx *= M_PI / 180.0f;
    gy *= M_PI / 180.0f;
    gz *= M_PI / 180.0f;

    Quaternion qg;

    float q_dot_w = 0.5f * (-ahrs->q.x * gx -ahrs->q.y * gy -ahrs->q.z * gz);
    float q_dot_x = 0.5f * (ahrs->q.w * gx + ahrs->q.y * gz - ahrs->q.z * gy);
    float q_dot_y = 0.5f * (ahrs->q.w * gy - ahrs->q.x * gz + ahrs->q.z * gx);
    float q_dot_z = 0.5f * (ahrs->q.w * gz + ahrs->q.x * gy - ahrs->q.y * gx);

    qg.w = ahrs->q.w + q_dot_w * dt;
    qg.x = ahrs->q.x + q_dot_x * dt;
    qg.y = ahrs->q.y + q_dot_y * dt;
    qg.z = ahrs->q.z + q_dot_z * dt;

    ahrs->q.w = ahrs->alpha * qg.w + (1.0f - ahrs->alpha) * qa.w;
    ahrs->q.x = ahrs->alpha * qg.x + (1.0f - ahrs->alpha) * qa.x;
    ahrs->q.y = ahrs->alpha * qg.y + (1.0f - ahrs->alpha) * qa.y;
    ahrs->q.z = ahrs->alpha * qg.z + (1.0f - ahrs->alpha) * qa.z;

    normalize_quaternion(&ahrs->q);
}

void ahrs_get_euler(AHRS *ahrs, EulerAngles *euler)
{
    quaternion_to_euler(&ahrs->q, euler);
}

Quaternion ahrs_get_quaternion(AHRS *ahrs)
{
    return ahrs->q;
}
