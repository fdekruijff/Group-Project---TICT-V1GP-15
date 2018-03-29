#include <cmath>
#include <iostream>
#include <unistd.h>
#include "./piprograms/BrickPi3.cpp"
#include <thread>
#include <vector>

using namespace std;


BrickPi3 BP;
uint8_t s_contrast = PORT_2;            // Light sensor
uint8_t m_head = PORT_A;                // Head motor
uint8_t m_left = PORT_B;                // Left motor
uint8_t m_right = PORT_C;               // Right motor

int16_t high_reflection = 0;
int16_t low_reflection = 0;

bool calibrating = false;

sensor_light_t contrast_struct;

void brick_py_setup() {
    BP.detect();
    BP.set_sensor_type(s_contrast, SENSOR_TYPE_NXT_LIGHT_ON);
}

void turn_off_motors() {
    BP.set_motor_power(m_right, 0);
    BP.set_motor_power(m_left, 0);
}

void motor_power(int power) {
    BP.set_motor_limits(m_left, uint8_t(power), 0);
    BP.set_motor_limits(m_right, uint8_t(power), 0);
}

int16_t get_contrast() {
    BP.get_sensor(s_contrast, contrast_struct);
    return contrast_struct.reflected;
}

int16_t max_vector(vector<int16_t> const vec) {
    int16_t tmp = vec[0];
    for (unsigned int i = 0; i < vec.size(); i++) {
        if (vec[i] > tmp) {
            tmp = vec[i];
        }
    }
    return tmp;
}

int16_t min_vector(vector<int16_t> const vec) {
    int16_t tmp = 4000;
    for (unsigned int i = 0; i < vec.size(); i++) {
        if (vec[i] < tmp) {
            tmp = vec[i];
        }
    }
    return tmp;
}

void measure_contrast() {
    vector<int16_t> tmp;
    while (calibrating) {
        tmp.push_back(get_contrast());
        usleep(25000);
    }
    high_reflection = max_vector(tmp);
    low_reflection = min_vector(tmp);
}

void calibrate() {
    calibrating = true;
    thread measure(measure_contrast);
    int turn = 180;
    vector<vector<int>> power_profile = {{turn,  -turn},
                                         {-turn, turn},
                                         {-turn, turn},
                                         {turn,  -turn},
                                         {turn,  -turn},
                                         {-turn, turn},
                                         {-turn, turn},
                                         {turn,  -turn}};

    motor_power(40);
    for (int i = 0; i < power_profile.size(); i++) {
        BP.set_motor_position_relative(m_right, power_profile[i][0]);
        BP.set_motor_position_relative(m_left, power_profile[i][1]);
        usleep(500000);
    }
    turn_off_motors();
    motor_power(100);
    calibrating = false;
    measure.join();
}

int main() {
    brick_py_setup();
    sleep(1);
    calibrate();

    float sensor_value = get_contrast();        // Should be a function return.
    float offset = ;                         // Should be defined after calibration.

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
//    steer_left(motor - output);
//    steer_right(motor + output);
}