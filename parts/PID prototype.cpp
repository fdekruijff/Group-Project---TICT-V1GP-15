#include <cmath>
#include <iostream>
#include <unistd.h>
#include "../piprograms/BrickPi3.cpp"

BrickPi3 BP;
uint8_t s_contrast = PORT_2;            // Light sensor
uint8_t m_left = PORT_B;                // Left motor
uint8_t m_right = PORT_C;               // Right motor

sensor_light_t contrast_struct;

using namespace std;

void exit_signal_handler(int signo);

void brick_py_setup() {
    signal(SIGINT, exit_signal_handler);
    BP.detect();
    BP.set_sensor_type(s_contrast, SENSOR_TYPE_NXT_LIGHT_ON);
}

int16_t get_contrast() {
    BP.get_sensor(s_contrast, contrast_struct);
    return contrast_struct.reflected;
}

void steer_left(float amount);
void steer_right(float amount);


int main() {
    brick_py_setup();

    while (true) {
        cout << "reflected: " << get_contrast() << endl;
        sleep(1);
    }

//    float sensor_value = get_contrast();                   // Should be a function return.
//    float offset = 0.0;                         // Should be defined after calibration.
//
//    float error = sensor_value - offset;
//    float last_error = 0.0;
//    float constant = 1.0;
//    float reset = 0.0;
//    float motor = 0.0;                         // Value for steering straight ahead.
//    float output = 0.0;
//
//    float P_pid = constant * error;
//    float I_pid = reset + (constant / 2 * float(M_PI)) * error;
//    float D_pid = (constant / 2 * float(M_PI)) * (error - last_error);
//    last_error = error;
//
//    output = P_pid + I_pid + D_pid;
//    steer_left(motor - output);
//    steer_right(motor + output);
}