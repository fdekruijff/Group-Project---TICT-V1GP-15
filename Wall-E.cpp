#include <cmath>
#include <iostream>
#include <unistd.h>
#include "./piprograms/BrickPi3.cpp"
#include <thread>
#include <signal.h>
#include <vector>

using namespace std;

/// BRICK PI 3 variable declaration
BrickPi3 BP;

/// Motor / Sensor variable declaration
uint8_t s_ultrasonic = PORT_3;                       // Ultrasonic sensor
uint8_t s_color = PORT_1;                       // Color sensor
uint8_t s_contrast = PORT_2;                    // Light sensor
uint8_t m_head = PORT_A;                        // Head motor
uint8_t m_left = PORT_B;                        // Left motor
uint8_t m_right = PORT_C;                       // Right motor

sensor_light_t      contrast_struct;
sensor_ultrasonic_t sonic_struct;

/// Ultrasonic sensor variable declaration
int distance_to_object = 0; 

/// limited distance stops PID 
int limited_distance = 10;

/// Calibration variable declaration
bool calibrating = false;
int16_t high_reflection = 0;                    // Black
int16_t low_reflection = 0;                     // White

/// Driving modes variable declaration
const string LINE = "LINE";
const string STOP = "STOP";
const string FREE = "FREE";

/// Wall-E brain settings struct declaration
struct wall_e_settings {
    float last_error = 0.0;                     // Value set by PID
    float i_error = 0.0;                        // Value set by PID
    float set_point = 0.0;                      // Value set by sensor
    float last_time = 1.0;                      // Domain unknown
    float i_gain = 0.01;                        // Domain: unknown
    float d_gain = 2.8;                         // Domain: unknown
    float p_gain = 0.295;                       // Domain: [0.275, 0.325]
    float compensation_multiplier = 950.0;      // Domain: [750, 1200]
    int motor_power = 80;                       // Domain: [10, 80]
    int pid_update_frequency_ms = 11000;        // Domain: [10000, 175000]
    string driving_mode = STOP;                 // Default driving mode
};

wall_e_settings brain;

void exit_signal_handler(int sig);

void exit_signal_handler(int sig) {
    /// Control-C handler that resets Brick Pi and exits application.
    if (sig == SIGINT) {
        BP.reset_all();                         // Reset everything so there are no run-away motors
        exit(-2);
    }
}

void setup() {
    /// Brick PI and exit handler are initialised here.
    signal(SIGINT, exit_signal_handler);
    BP.detect();
    BP.set_sensor_type(s_contrast, SENSOR_TYPE_NXT_LIGHT_ON);
    BP.set_sensor_type(s_color, SENSOR_TYPE_NXT_COLOR_FULL);
    BP.set_sensor_type(s_ultrasonic, SENSOR_TYPE_NXT_ULTRASONIC);
}

void stop() {
    /// Stops driving Wall-E.
    BP.set_motor_power(m_right, 0);
    BP.set_motor_power(m_left, 0);
}

void motor_power(int power) {
    /// Set motor power to specific power level simultaneously.
    BP.set_motor_power(m_left, uint8_t(power));
    BP.set_motor_power(m_right, uint8_t(power));
}

void motor_power_limit(int power) {
    /// Set motor power to specific power limit simultaneously.
    BP.set_motor_limits(m_left, uint8_t(power), 0);
    BP.set_motor_limits(m_right, uint8_t(power), 0);
}

void scan_ultrasonic(){
    //updates the distance_to_object, 0 / 25. 0 is error code.
	while (true){
		BP.get_sensor(s_ultrasonic, sonic_struct);
        if (sonic_struct.cm > 0 and sonic_struct.cm <25){
               distance_to_object = sonic_struct.cm;
        }else{ 
            distance_to_object = 0;
        }
        cout << distance_to_object << endl;
    usleep(200000);
    }
} 

int16_t get_contrast() {
    /// Returns the reflected black / white contrast.
    BP.get_sensor(s_contrast, contrast_struct);
    return contrast_struct.reflected;
}

int16_t max_vector(vector<int16_t> const vec) {
    /// Returns the highest integer value of a vector.
    // TODO: fix  duplicate code here
    int16_t tmp = vec[0];
    for (unsigned int i = 0; i < vec.size(); i++) {
        if (vec[i] > tmp) {
            tmp = vec[i];
        }
    }
    return tmp;
}

int16_t min_vector(vector<int16_t> const vec) {
    /// Returns the lowest integer value of a vector.
    // TODO: fix  duplicate code here
    int16_t tmp = 4000;
    for (unsigned int i = 0; i < vec.size(); i++) {
        if (vec[i] < tmp && vec[i] != 0) {
            tmp = vec[i];
        }
    }
    return tmp;
}

void measure_contrast() {
    /// Thread function that reads sensor values and calculates maximum and minimum from local vector.
    vector<int16_t> tmp;
    while (calibrating) {
        tmp.push_back(get_contrast());
        usleep(25000);
    }
    high_reflection = max_vector(tmp);
    low_reflection = min_vector(tmp);
}

//if a object is in the way of the PID it stops the PID.
void object_in_the_way(){
	while (true){
		if (distance_to_object < limited_distance and distance_to_object != 0){
			brain.driving_mode = STOP;
            stop();
		}
	}
}

void calibrate() {
    /// Function reads sensor values while driving over the tape. Sets maximum, minimum and set point for PID.
    calibrating = true;
    thread measure(measure_contrast);
    int turn = 180;
    vector<vector<int>> power_profile = {
            {turn,  -turn},
            {-turn, turn},
            {-turn, turn},
            {turn,  -turn}};

    motor_power_limit(40);
    for (int i = 0; i < power_profile.size(); i++) {
        BP.set_motor_position_relative(m_right, power_profile[i][0]);
        BP.set_motor_position_relative(m_left, power_profile[i][1]);
        usleep(650000);
    }
    calibrating = false;
    stop();
    motor_power_limit(100);
    measure.join();
    brain.set_point = (high_reflection + low_reflection) / 2;
    cout << "Calibration finished." << endl <<
         " high:" << int(high_reflection) << " low:" << int(low_reflection) << " set:" << brain.set_point << endl;
}

void steer_left(uint8_t amount) {
    /// Steer left motor.
    BP.set_motor_power(m_left, amount);
}

void steer_right(uint8_t amount) {
    /// Steer right motor.
    BP.set_motor_power(m_right, amount);
}

float bound(float value, float begin, float end) {
    /// Cap value between begin and end range. Used to keep PID motor values in boundaries.
    if (value < begin) return begin;
    if (value > end) return end;
    return value;
}

float calc_compensation(float x) {
    return float((1.0 / brain.compensation_multiplier) * (x * x));
}

float calculate_correction() {
    /// PID calculations are performed here. Magic for the most part.
    float sensor = get_contrast();
    float error = brain.set_point - sensor;

    float p_error = error;
    brain.i_error += (error + brain.last_error) * brain.last_time;
    float d_error = (error - brain.last_error) / brain.last_time;

    float p_output = brain.p_gain * p_error;
    float i_output = brain.i_gain * brain.i_error;
    float d_output = brain.d_gain * d_error;
    brain.last_error = error;

    // TODO: FIX Integral and Derivative of PID.
    return p_output;
}


bool is_black() {
    /// Is sensor value in the black domain?
    float sensor = get_contrast();
    return sensor > brain.set_point && sensor < high_reflection;
}

bool is_white() {
    /// Is sensor value in the white domain?
    float sensor = get_contrast();
    return sensor < high_reflection && sensor > brain.set_point;
}

void drive_line() {
    /// Threaded function that follows measured contrast based on sensor reading and set point.
    brain.driving_mode = LINE;
    while (brain.driving_mode == LINE) {
        float output = calculate_correction();
        float comp = calc_compensation(brain.last_error);
        steer_left(uint8_t( bound(brain.motor_power - comp - output, 5, 100)));
        steer_right(uint8_t(bound(brain.motor_power - comp + output, 5, 100)));
        usleep(brain.pid_update_frequency_ms);
    }
}

void find_line() {
    brain.driving_mode = FREE;
    motor_power(20);
    while (brain.driving_mode == FREE && !is_black()) {
        usleep(500000);
    }
    stop();
    drive_line();
}

int main() {
    setup();
    calibrate();

    // Start sensor threads
    thread scan_distance (scan_ultrasonic);
    // TODO: right here
    thread stop_opbeject (object_in_the_way);
    
    drive_line();
}
