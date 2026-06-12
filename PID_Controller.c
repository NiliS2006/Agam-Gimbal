PID controllers for a drone system with a gimbal system, with the use of quaternions.

// this is the structure for Quaternions that are going to be used in the drone.
// w is the scalar part of the representation
// x,y,z are the vector parts of the representation
typedef struct
{
    float w;
    float x;
    float y;
    float z;
} Quaternion;

// Structure for the PID controller that is going to be used in the drone.
typedef struct
{
    float kp; // proportional gain
    float ki; // integral gain
    float kd; // derivative gain

    float integral; // integral term
    float previous_error; // previous error for derivative term
} PIDController;

// Structure for drone's controller that contains the PID controllers for roll, pitch, and yaw.
typedef struct
{
    PIDController roll; // PID controller for roll
    PIDController pitch; // PID controller for pitch
    PIDController yaw; // PID controller for yaw
} QuaternionController;

// Structure for the drone's output that contains the desired roll, pitch, and yaw angles.
typedef struct
{
    float roll; // desired roll angle
    float pitch; // desired pitch angle
    float yaw; // desired yaw angle
} DroneOutput;

// Function to find Quaternion Conjugate
// if the quaternion is represented as q = w + xi + yj + zk, then the conjugate is given by q* = w - xi - yj - zk
Quaternion quaternion_conjugate(Quaternion q)
{
    Quaternion conjugate;
    conjugate.w = q.w;
    conjugate.x = -q.x;
    conjugate.y = -q.y;
    conjugate.z = -q.z;
    return conjugate;
}

// Function to find Quaternion Multiplication
// this is very essential in finding the error between the desired and current orientation of the drone.
// if we have two quaternions q1 = w1 + x1i + y1j + z1k and q2 = w2 + x2i + y2j + z2k, then the multiplication is given by:
// q1 * q2 = (w1w2 - x1x2 - y1y2 - z1z2) + (w1x2 + x1w2 + y1z2 - z1y2)i + (w1y2 - x1z2 + y1w2 + z1x2)j + (w1z2 + x1y2 - y1x2 + z1w2)k
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2)
{
    Quaternion result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    // remember the multiplication rules of quaternions
    // i^2 = j^2 = k^2 = ijk = -1
    // i * j = k, j * k = i, k * i = j
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

// Function to find the error between the desired and current orientation of the drone.
// the error is calculated by multiplying the desired quaternion with the conjugate of the current quaternion.
Quaternion quaternion_error(Quaternion desired, Quaternion current)
{
    Quaternion current_conjugate = quaternion_conjugate(current);
    return quaternion_multiply(desired, current_conjugate);
}

// Function to update the PID controller for a given error and time step.
float update_pid(PIDController *pid, float error, float dt)
{
    // Update integral term
    // adds the error upto the time step to the integral term. This is used to eliminate the steady-state error in the system.
    pid->integral += error * dt;

    // Calculate derivative term
    // finds the rate of change of error with respect to time. This is used to predict the future behavior of the system and to provide a damping effect to the system.
    float derivative = (error - pid->previous_error) / dt;

    // Update previous error
    // stores the current error as the previous error for the next iteration. This is used to calculate the derivative term in the next iteration.
    pid->previous_error = error;

    // Calculate PID output
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

// Main Quaternion Controller Function
// this function takes the desired and current quaternions, the time step, and the controller as input and returns the desired roll, pitch, and yaw angles for the drone.
DroneOutput quaternion_controller(Quaternion desired, Quaternion current, float dt, QuaternionController *controller)
{
    // Calculate the error quaternion
    Quaternion error = quaternion_error(desired, current);

    // Convert the error quaternion to roll, pitch, and yaw angles
    // this is done by using the following formulas:
    // roll = atan2(2(q.w * q.x + q.y * q.z), 1 - 2(q.x^2 + q.y^2))
    // pitch = asin(2(q.w * q.y - q.z * q.x))
    // yaw = atan2(2(q.w * q.z + q.x * q.y), 1 - 2(q.y^2 + q.z^2))
    float roll_error = atan2f(2.0f * (error.w * error.x + error.y * error.z), 1.0f - 2.0f * (error.x * error.x + error.y * error.y));
    float pitch_error = asinf(2.0f * (error.w * error.y - error.z * error.x));
    float yaw_error = atan2f(2.0f * (error.w * error.z + error.x * error.y), 1.0f - 2.0f * (error.y * error.y + error.z * error.z));

    // Update PID controllers for roll, pitch, and yaw
    float roll_output = update_pid(&controller->roll, roll_error, dt);
    float pitch_output = update_pid(&controller->pitch, pitch_error, dt);
    float yaw_output = update_pid(&controller->yaw, yaw_error, dt);

    // Return the desired roll, pitch, and yaw angles for the drone
    DroneOutput output;
    output.roll = roll_output;
    output.pitch = pitch_output;
    output.yaw = yaw_output;
    return output;
}
