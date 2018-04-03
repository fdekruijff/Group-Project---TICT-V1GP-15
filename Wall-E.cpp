#include <cmath>
#include <iostream>
#include <unistd.h>
#include "./piprograms/BrickPi3.cpp"
#include <thread>
#include <vector>

using namespace std;


BrickPi3 BP;

// Motor / sensor variables
uint8_t s_contrast = PORT_2;            // Light sensor
uint8_t m_head = PORT_A;                // Head motor
uint8_t m_left = PORT_B;                // Left motor
uint8_t m_right = PORT_C;               // Right motor
sensor_light_t contrast_struct;

// Calibration variables
bool calibrating = false;
float set_point = 0.0;
int16_t high_reflection = 0;
int16_t low_reflection = 0;

// PID variables
struct wall_e_settings {
    float last_error;
    float last_time;
    float p_gain;
    float i_gain;
    float d_gain;
    float i_error;
};

wall_e_settings brain;

// Driving modes
bool driving_mode_line = false;
bool grid = false;

void exit_signal_handler(int signo);

void exit_signal_handler(int signo){
    if(signo == SIGINT){
        BP.reset_all();    // Reset everything so there are no run-away motors
        exit(-2);
    }
}

void setup() {
    signal(SIGINT, exit_signal_handler);
    BP.detect();
    BP.set_sensor_type(s_contrast, SENSOR_TYPE_NXT_LIGHT_ON);

    brain.last_error = 0.0;
    brain.last_time = 1.0;
    brain.p_gain = 0.275;
    brain.i_gain = 0.01;
    brain.d_gain = 2.8;
    brain.i_error = 0.0;
    brain.set_point = 0.0;
}

void stop() {
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
        if (vec[i] < tmp && vec[i] != 0) {
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
    vector<vector<int>> power_profile = {
            {turn,  -turn},
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
    calibrating = false;
    stop();
    motor_power(100);
    measure.join();
    set_point = (high_reflection + low_reflection) / 2;
    cout << "Calibration finished. high:" << int(high_reflection) << " low:" << low_reflection << " set:" << set_point << endl;
}

void steer_left(uint8_t amount) {
    BP.set_motor_power(m_left, amount);
}

void steer_right(uint8_t amount) {
    BP.set_motor_power(m_right, amount);
}

float calculate_correction() {
    float sensor = get_contrast();
    float error = set_point - sensor;

    float p_error = error;
    brain.i_error += (error + brain.last_error) * brain.last_time;
    float d_error = (error - brain.last_error) / brain.last_time;

    float p_output = brain.p_gain * p_error;
    float i_output = brain.i_gain * brain.i_error;
    float d_output = brain.d_gain * d_error;
    brain.last_error = error;

    // TODO: map output to power profile
    return p_output;
}

void drive_line() {
    while (driving_mode_line) {
        float output = calculate_correction();
        steer_left(65 - output);
        steer_right(65 + output);
        usleep(15000);
    }

}

int main() {
    setup();
    calibrate();
    drive_line();
}