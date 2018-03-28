#include <cmath>
#include <iostream>
#include <thread>
#include "../piprograms/BrickPi3.cpp"
#include <vector>
#include <unistd.h>
#include <algorithm>

BrickPi3 BP;
uint8_t s_contrast = PORT_2;            // Light sensor
uint8_t m_left = PORT_B;                // Left motor
uint8_t m_right = PORT_C;               // Right motor

bool calibrating = false;
int16_t high_reflection = 0;
int16_t low_reflection = 0;

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

void measure_contrast() {
    vector<int16_t> tmp;
    while (calibrating) {
        tmp.push_back(get_contrast());
        usleep(250000);
    }
    low_reflection = uint16_t(minmax_element(tmp.begin(), tmp.end()).first - tmp.begin());
    high_reflection = uint16_t(minmax_element(tmp.begin(), tmp.end()).second - tmp.begin());
}

vector<int16_t> calibrate() {
    calibrating = true;
    thread measure (measure_contrast);
    // turn 360 degrees and read values


}

int main() {

}