#include <cmath>
#include <iostream>
#include <unistd.h>
#include "./piprograms/BrickPi3.cpp"
#include <thread>
#include <signal.h>
#include <vector>

using namespace std;


BrickPi3 BP;
uint8_t s_contrast = PORT_2;            // Light sensor
uint8_t m_head = PORT_A;                // Head motor
uint8_t m_left = PORT_B;                // Left motor
uint8_t m_right = PORT_C;               // Right motor

int16_t high_reflection = 0;
int16_t low_reflection = 0;

int power = 30;
float K = 2.0;
bool calibrating = false;

sensor_light_t contrast_struct;

void exit_signal_handler(int signo);

void exit_signal_handler(int signo){
    if(signo == SIGINT){
        BP.reset_all();    // Reset everything so there are no run-away motors
        exit(-2);
    }
}

void brick_py_setup() {
    signal(SIGINT, exit_signal_handler);
    BP.detect();
    BP.set_sensor_type(s_contrast, SENSOR_TYPE_NXT_LIGHT_ON);
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
}

void steer_left(float amount) {
    BP.set_motor_power(m_left, amount);
}

void steer_right(float amount) {
    BP.set_motor_power(m_right, amount);
}

void drive_line() {
    float reset = 0.0;
    float constant = K;
    float last_error = 0.0;
    float output = 0.0;
    float P_pid = 0.0;
    float I_pid = 0.0;
    float D_pid = 0.0;

    while (true) {
        float sensor_value = get_contrast();
        float offset = (high_reflection + low_reflection) / 2;
        float error = sensor_value - offset;

//        float P_pid = constant * error;
//        float I_pid = reset + (constant / 2 * float(M_PI)) * error;
//        float D_pid = (constant / 2 * float(M_PI)) * (error - last_error);//

        P_pid = constant * error;
        I_pid += error;
        D_pid = error - last_error;
        last_error = error;

        output = P_pid + I_pid + D_pid;
        steer_left(power - output);
        steer_right(power + output);
        cout << "error: " << error << endl;
        cout << "l pwr: " << power-output;
        cout << "r pwr: " << power+output;
        usleep(75000);
    }
}

int main() {
    brick_py_setup();
    sleep(1);
    calibrate();

    drive_line();
}