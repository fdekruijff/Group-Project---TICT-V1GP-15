#include <cmath>
#include <iostream>
#include <unistd.h>
#include "./piprograms/BrickPi3.cpp"
#include <signal.h>
#include <thread>
#include <vector>
#include <map>

using namespace std;


BrickPi3 BP;

// Motor / sensor variables
uint8_t s_colour = PORT_1;				// Colour sensor
uint8_t s_contrast = PORT_2;            // Light sensor
uint8_t m_head = PORT_A;                // Head motor
uint8_t m_left = PORT_B;                // Left motor
uint8_t m_right = PORT_C;               // Right motor
sensor_light_t contrast_struct;

// Calibration variables
bool calibrating = false;
bool search_colour = true;
bool intersection = true;
bool c_control =false;
int16_t high_reflection = 0;
int16_t low_reflection = 0;
sensor_color_t Colour1;

// Driving modes
const string LINE = "LINE";
const string STOP = "STOP";

// PID variables
struct wall_e_settings {
    float last_error = 0.0;
    float last_time = 1.0;
    float p_gain = 0.275;
    float i_gain = 0.01;
    float d_gain = 2.8;
    float i_error = 0.0;
    float set_point = 0.0;
    int motor_power = 65;
    int pid_update_frequency_ms = 15000;
    string driving_mode = STOP;
};

wall_e_settings brain;

void exit_signal_handler(int signo);

void exit_signal_handler(int signo) {
    if (signo == SIGINT) {
        BP.reset_all();    // Reset everything so there are no run-away motors
        exit(-2);
    }
}

void setup() {
    signal(SIGINT, exit_signal_handler);
    BP.detect();
    BP.set_sensor_type(s_contrast, SENSOR_TYPE_NXT_LIGHT_ON);
    BP.set_sensor_type(s_colour, SENSOR_TYPE_NXT_COLOR_FULL);
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
    brain.set_point = (high_reflection + low_reflection) / 2;
    cout << "Calibration finished. high:" << int(high_reflection) << " low:" << low_reflection << " set:" << brain.set_point
         << endl;
}

void steer_left(uint8_t amount) {
    BP.set_motor_power(m_left, amount);
}

void steer_right(uint8_t amount) {
    BP.set_motor_power(m_right, amount);
}

float calculate_correction() {
    float sensor = get_contrast();
    float error = brain.set_point - sensor;

    float p_error = error;
    brain.i_error += (error + brain.last_error) * brain.last_time;
    float d_error = (error - brain.last_error) / brain.last_time;

    float p_output = brain.p_gain * p_error;
    float i_output = brain.i_gain * brain.i_error;
    float d_output = brain.d_gain * d_error;
    brain.last_error = error;

    // TODO: FIX pid
    return p_output;
}

void drive_line() {
    while (brain.driving_mode == LINE) {
        float output = calculate_correction();
        steer_left(uint8_t(brain.motor_power - output));
        steer_right(uint8_t(brain.motor_power + output));
        usleep(brain.pid_update_frequency_ms);
    }
}

int find_colour()
{
	while(search_colour == true)
	{
		BP.get_sensor(s_colour, Colour1);
		if(Colour1.reflected_red >= 350 && Colour1.reflected_green < 350 && Colour1.reflected_blue < 350)
		{
			return 1;
		}
		if(Colour1.reflected_red < 350 && Colour1.reflected_green >= 350 && Colour1.reflected_blue < 350)
		{
			return 2;
		}
		if(Colour1.reflected_red < 350 && Colour1.reflected_green < 350 && Colour1.reflected_blue >= 350)
		{
			return 3;
		}
		sleep(0.1);
	}
}

void folour_control()
{
	while(c_control == true)
	{
		if(find_colour() == 1)
		{
			cout << "Found red";
//			stop();
//			break;
		}
		if(find_colour() == 2)
		{
			cout << "Found green";
		}
		if(find_colour() == 3)
		{
			cout << "Found blue";
		}
	}
}

void find_colour_values()
{
	while(intersection == true)
	{
		BP.get_sensor(s_colour, Colour1);
		cout << "high ref: " << int(high_reflection) << "  red: " << Colour1.reflected_red << "  green: " << Colour1.reflected_green << "  blue: " << Colour1.reflected_blue << endl;
		sleep(1);
	}
}

void find_intersection()
{
	while(intersection == true)
	{
		BP.get_sensor(s_colour, Colour1);
		if(Colour1.reflected_red <= 350 && Colour1.reflected_green <= 350 && Colour1.reflected_blue <= 350)
		{
			cout << "found intersection" << endl;
		}
		if(Colour1.reflected_red >= 350 && Colour1.reflected_green <= 350 && Colour1.reflected_blue <= 350)
		{
			cout << "found closed intersection" << endl;
		}
		sleep(1);
	}
}

bool find_intersection()
{
	while(intersection == true)
	{
		BP.get_sensor(s_colour, Colour1);
		if(Colour1.reflected_red <= 350 && Colour1.reflected_green <= 350 && Colour1.reflected_blue <= 350)
		{
			cout << "found intersection" << endl;
			return true;
		}
	}
}

int main() {
    setup();
    calibrate();
//    brain.driving_mode = LINE;
//    drive_line();
//	find_colour_values();
	find_intersection();
}