#include <stdlib.h>
# include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

int iterations = 1000;
float sleep = 0.01;
float alpha = 0.98;
float dt = 0.0;
float phi_angle = 0.0;
float theta_angle = 0.0;

void get_gyro_bias(float *gyro_bias_x, float *gyro_bias_y, float *gyro_bias_z) 
{
    float gyro_x, gyro_y, gyro_z;
    float sum_gyro_x = 0.0;
    float sum_gyro_y = 0.0;
    float sum_gyro_z = 0.0;
    
    for (int i = 0; i < 200; i++) 
    {
        get_gyro(&gyro_x, &gyro_y, &gyro_z); 
        sum_gyro_x += gyro_x;
        sum_gyro_y += gyro_y;
        sum_gyro_z += gyro_z;

    }

    *gyro_bias_x = sum_gyro_x / 200;
    *gyro_bias_y = sum_gyro_y / 200;
    *gyro_bias_z = sum_gyro_z / 200;
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
void get_accelerometer_angles(float *phi_angle, float *theta_angle)
{
    float acc_x, acc_y, acc_z;
    get_accelerometer(&acc_x, &acc_y, &acc_z); 

    *phi_angle = atan2f(acc_y, acc_z) * 180.0 / M_PI; 
    *theta_angle = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 180.0 / M_PI;
}

void get_raw_gyro_angles(float *gyro_bias_x, float *gyro_bias_y, float *gyro_bias_z)
{
    float p,q,r;
    p -= *gyro_bias_x;
    q -= *gyro_bias_y;
    r -= *gyro_bias_z;
}

void euler_angles(float p, float q, float r, float phi_angle, float theta_angle)
{
    float phi_dot = p + q * sinf(phi_angle * M_PI / 180.0) * tanf(theta_angle * M_PI / 180.0) + r * cosf(phi_angle * M_PI / 180.0) * tanf(theta_angle * M_PI / 180.0);
    float theta_dot = q * cosf(phi_angle * M_PI / 180.0) - r * sinf(phi_angle * M_PI / 180.0);
}
void complementary_filter(float phi_angle, float theta_angle, float phi_dot, float theta_dot, float dt)
{
    phi_angle = alpha * (phi_angle + phi_dot * dt) + (1 - alpha) * phi_angle;
    theta_angle = alpha * (theta_angle + theta_dot * dt) + (1 - alpha) * theta_angle;
}

