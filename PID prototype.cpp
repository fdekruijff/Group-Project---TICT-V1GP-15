#include <math.h>

void steer_left(float amount);
void steer_right(float amount);


int main() {
    float sensor_value = 0.0;                   // Should be a function return.
    float offset = 0.0;                         // Should be defined after calibration.

    float error = sensor_value - offset;
    float last_error = 0.0;
    float constant = 1.0;
    float reset = 0.0;
    float motor = 0.0;                         // Value for steering straight ahead.
    float output = 0.0;

    float P_pid = constant * error;
    float I_pid = reset + (constant / 2 * float(M_PI)) * error;
    float D_pid = (constant / 2 * float(M_PI)) * (error - last_error);
    last_error = error;

    output = P_pid + I_pid + D_pid;
    steer_left(motor - output);
    steer_right(motor + output);
}