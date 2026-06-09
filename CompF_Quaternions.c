#include <stdlib.h>
# include <math.h>

#ifndef M_PI // checks for if M_PI is already defined, if not defines it
#define M_PI 3.14159265358979323846f
#endif  // marks the end of the M_PI definition block

// sleep = 0.01s means 100Hz update rate
int iterations = 1000;
float sleep = 0.01;
float alpha = 0.98;
// 98% reliance on gyroscope data and 2% reliance on accelerometer/magnetometer data
float dt = 0.0;

float phi_angle, theta_angle, psi_angle;

// a quaternion is a four-dimensional number used to represent 3D orientations and rotations.
// it consists of a real scalar part 'w' representing the magnitude of rotation, related to half cosine of the angle
// it consists of three imaginary vector components: 'x','y','z' representing the axes of rotation, related to half sine of the rotation angle multiplied by the unit vector along the axis of rotation
//for a unit quaternion, √(q0^2 + q1^2 + q2^2 + q3^2) = 1, for a valid 3D rotation representation, the quaternion must be normalized to ensure it represents a valid rotation without scaling effects.
// also, i^2 = j^2 = k^2 = ijk = -1, where i, j, k are the imaginary units of the quaternion, and they follow specific multiplication rules that define how quaternions combine and rotate in 3D space.

// q0 = cos(theta/2) 
// q1 = X * sin(theta/2)
// q2 = Y * sin(theta/2)
// q3 = Z * sin(theta/2)

float q0 = 1.0f; //scalar part of quaternion 'w'
float q1 = 0.0f; // x component of quaternion
float q2 = 0.0f; // y component of quaternion
float q3 = 0.0f; // z component of quaternion

void get_gyro_bias(float *gyro_bias_x, float *gyro_bias_y, float *gyro_bias_z) 
{
    float gyro_x, gyro_y, gyro_z;
    float sum_gyro_x = 0.0;
    float sum_gyro_y = 0.0;
    float sum_gyro_z = 0.0;
    
    for (int i = 0; i < 200; i++) 
    {
        float gyro_x, gyro_y, gyro_z;
        sum_gyro_x += gyro_x;
        sum_gyro_y += gyro_y;
        sum_gyro_z += gyro_z;

    }

    *gyro_bias_x = sum_gyro_x / 200;
    *gyro_bias_y = sum_gyro_y / 200;
    *gyro_bias_z = sum_gyro_z / 200;
}

// to calculate gyrometer angles with bias correction
void get_raw_gyro_angles(float *gx, float *gy, float *gz, float gyro_bias_x, float gyro_bias_y, float gyro_bias_z)
{
    *gx -= gyro_bias_x;
    *gy -= gyro_bias_y;
    *gz -= gyro_bias_z;
}

void measure_dt(int iterations)
{
    float dt = 0.0f;
    float start_time = time();

    for (int i = 0; i < iterations; i++)
    {
        dt = time() - start_time;
        start_time = time();
    }
}

void get_accelerometer_angles(float *acc_x, float *acc_y, float *acc_z)
{
    phi_angle = atan2f(*acc_y, *acc_z);
    theta_angle = atan2f(-*acc_x, sqrtf(*acc_y * *acc_y + *acc_z * *acc_z));
}

void get_magnetometer_angles(float *mag_x, float *mag_y, float *mag_z)
{
    float mx_comp = *mag_x * cosf(theta_angle) + *mag_z * sinf(theta_angle);
    float my_comp = *mag_x * sinf(phi_angle) * sinf(theta_angle) + *mag_y * cosf(phi_angle) - *mag_z * sinf(phi_angle) * cosf(theta_angle);
    psi_angle = atan2f(-my_comp, mx_comp);
}

void complementary_filter(float *qg0, float *qg1, float *qg2, float *qg3, float *qa0, float *qa1, float *qa2, float *qa3, float gx, float gy, float gz, float q_dot0, float q_dot1, float q_dot2, float q_dot3, float dt)
{
    // Convert Euler angles to quaternion
    float cr = cosf(phi_angle * 0.5f);
    float sr = sinf(phi_angle * 0.5f);

    float cp = cosf(theta_angle * 0.5f);
    float sp = sinf(theta_angle * 0.5f);

    float cy = cosf(psi_angle * 0.5f);
    float sy = sinf(psi_angle * 0.5f);

    // Calculate quaternion from accelerometer and magnetometer angles
    // \(w = \cos\left(\frac{\phi}{2}\right) \cos\left(\frac{\theta}{2}\right) \cos\left(\frac{\psi}{2}\right) + \sin\left(\frac{\phi}{2}\right) \sin\left(\frac{\theta}{2}\right) \sin\left(\frac{\psi}{2}\right)\)\(x = \sin\left(\frac{\phi}{2}\right) \cos\left(\frac{\theta}{2}\right) \cos\left(\frac{\psi}{2}\right) - \cos\left(\frac{\phi}{2}\right) \sin\left(\frac{\theta}{2}\right) \sin\left(\frac{\psi}{2}\right)\)\(y = \cos\left(\frac{\phi}{2}\right) \sin\left(\frac{\theta}{2}\right) \cos\left(\frac{\psi}{2}\right) + \sin\left(\frac{\phi}{2}\right) \cos\left(\frac{\theta}{2}\right) \sin\left(\frac{\psi}{2}\right)\)\(z = \cos\left(\frac{\phi}{2}\right) \cos\left(\frac{\theta}{2}\right) \sin\left(\frac{\psi}{2}\right) - \sin\left(\frac{\phi}{2}\right) \sin\left(\frac{\theta}{2}\right) \cos\left(\frac{\psi}{2}\right)\)
    *qa0 = cr * cp * cy + sr * sp * sy;
    *qa1 = sr * cp * cy - cr * sp * sy;
    *qa2 = cr * sp * cy + sr * cp * sy;  
    *qa3 = cr * cp * sy - sr * sp * cy;

    // Convert gyroscope readings from degrees/s to radians/s
    gx *= M_PI / 180.0f; 
    gy *= M_PI / 180.0f;
    gz *= M_PI / 180.0f;

    // Calculate quaternion derivative from gyroscope data
    // The quaternion derivative describes the rate of change of the quaternion based on the object's angular velocity
    q_dot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    q_dot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    q_dot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    q_dot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    *qg0 = q0 + q_dot0 * dt;
    *qg1 = q1 + q_dot1 * dt; 
    *qg2 = q2 + q_dot2 * dt;
    *qg3 = q3 + q_dot3 * dt;

    //Apply complementary filter to combine gyroscope and accelerometer/magnetometer quaternions
    q0 = alpha * *qg0 + (1.0f - alpha) * *qa0;
    q1 = alpha * *qg1 + (1.0f - alpha) * *qa1;
    q2 = alpha * *qg2 + (1.0f - alpha) * *qa2;
    q3 = alpha * *qg3 + (1.0f - alpha) * *qa3;
}

void normalize_quaternion() 
{
    // Normalize the quaternion to maintain unit length
    // a normalizes quaternion is one in which all its individual parts are squared and then summed and rooted to equal 1, ensuring it represents a valid rotation without scaling effects.
    float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm > 0.0f) 
    {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
}

void quaterion_to_euler(float *phi_angle, float *theta_angle, float *psi_angle)
{
    // Convert quaternion to Euler angles
    *phi_angle = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    *theta_angle = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
    *psi_angle = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
}
