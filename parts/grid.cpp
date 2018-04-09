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
uint8_t s_ultrasonic = PORT_3;                  // Ultrasonic sensor
uint8_t s_colour = PORT_1;                       // Color sensor
uint8_t s_contrast = PORT_2;                    // Light sensor
uint8_t m_head = PORT_A;                        // Head motor
uint8_t m_left = PORT_B;                        // Left motor
uint8_t m_right = PORT_C;                       // Right motor

sensor_light_t      contrast_struct;
sensor_ultrasonic_t sonic_struct;
sensor_color_t Colour_struct;

/// Ultrasonic sensor variable declaration
int distance_to_object = 0;

/// limited distance stops PID
int limited_distance = 10;

/// Calibration variable declaration
bool calibrating = false;

int16_t high_reflection = 0;                    // Black
int16_t low_reflection = 0;                     // White

int16_t red_high_reflection = 0;				// much Red
int16_t red_low_reflection = 0;					// little Red
int16_t blue_high_reflection = 0;				// much blue
int16_t blue_low_reflection = 0;				// little blue
int16_t green_high_reflection = 0;				// much green
int16_t green_low_reflection = 0;				// little green

vector<int> colour_set_point = {0,0,0}; 

/// Driving modes variable declaration
const string LINE = "LINE";
const string STOP = "STOP";
const string GRID = "GRID";
const string FREE = "FREE";

/// Wall-E brain settings struct declaration
struct wall_e_settings {
    float last_error = 0.0;                     // Value set by PID
    float i_error = 0.0;                        // Value set by PID
    float set_point = 0.0;                      // Value set by sensor
    float last_time = 1.0;                      // Domain: unknown
    float i_gain = 0.01;                        // Domain: unknown
    float d_gain = 2.8;                         // Domain: unknown
    float p_gain = 0.295;                       // Domain: [0.275, 0.325]
    float compensation_multiplier = 950.0;      // Domain: [750, 1200]
    int motor_power = 30;                       // Domain: [10, 80]
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
    BP.set_sensor_type(s_colour, SENSOR_TYPE_NXT_COLOR_FULL);
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
    while (true){
        BP.get_sensor(s_ultrasonic, sonic_struct);
        cout << sonic_struct.cm << endl;
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

void measure_colour_contrast()
{
    /// Thread function that reads colour sensor values and calculates maximum and minimum from local vector.
    vector<int16_t> red_tmp;
	vector<int16_t> blue_tmp;
	vector<int16_t> green_tmp;
	
    while (calibrating)
	{
		BP.get_sensor(s_colour, Colour_struct);
		red_tmp.push_back(Colour_struct.reflected_red);
        blue_tmp.push_back(Colour_struct.reflected_blue);
		green_tmp.push_back(Colour_struct.reflected_green);
        usleep(25000);
    }
    red_high_reflection = max_vector(red_tmp);
    red_low_reflection = min_vector(red_tmp);
	blue_high_reflection = max_vector(blue_tmp);
	blue_low_reflection = min_vector(blue_tmp);
	green_high_reflection = max_vector(green_tmp);
	green_low_reflection = min_vector(green_tmp);
}

//if a object is in the way of the PID it stops the PID.
void object_in_the_way(){
    sleep(3);
    while (true){
        if (sonic_struct.cm  < limited_distance){
            brain.driving_mode = STOP;
            stop();
        }
    }
}

void calibrate() {
    /// Function reads sensor values while driving over the tape. Sets maximum, minimum and set point for PID.
    calibrating = true;
    thread measure(measure_contrast);
	thread colour_measure(measure_colour_contrast);
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
	colour_measure.join();
    brain.set_point = (high_reflection + low_reflection) / 2;
	colour_set_point[0] = (red_high_reflection + red_low_reflection) / 2;
	colour_set_point[1] = (blue_high_reflection + blue_low_reflection) / 2;
	colour_set_point[2] = (green_high_reflection + green_low_reflection) / 2;
    cout << "Calibration finished." << endl <<
         " high:" << int(high_reflection) << " low:" << int(low_reflection) << " set:" << brain.set_point << endl;
	cout << "Red_high:   " << red_high_reflection << " Red_low:   " << red_low_reflection << endl << 
	"Blue_high:  " << blue_high_reflection << " Blue_low:  " << blue_low_reflection << endl << 
	"Green_high: " << green_high_reflection << " Green_low: " << green_low_reflection << endl;
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

bool colour_is_black()
	/// Is colour sensor value in the black domain?
{
	float red_sensor = Colour_struct.reflected_red;
	float blue_sensor = Colour_struct.reflected_blue;
	float green_sensor = Colour_struct.reflected_green;
	return (red_sensor > colour_set_point[0] && red_sensor < red_high_reflection) &&
			(blue_sensor > colour_set_point[1] && blue_sensor < blue_high_reflection) &&
			(green_sensor > colour_set_point[2] && green_sensor < green_high_reflection);
}

void drive() {
    /// Threaded function that applies certain drive mode.
    while (true) {
        if (brain.driving_mode == STOP || brain.driving_mode == FREE)  {
            usleep(250000);
            continue;
        }
        if (brain.driving_mode == LINE) {
            float output = calculate_correction();
            float comp = calc_compensation(brain.last_error);
            steer_left(uint8_t(int(bound(brain.motor_power - (comp + 5) - output, -90, 90))));
            steer_right(uint8_t(int(bound(brain.motor_power - (comp + 5) + output, -90, 90))));
            usleep(brain.pid_update_frequency_ms);
        }
        if (brain.driving_mode == GRID) {
            // TODO: implement GRID driving code here.
        }
    }
}

void find_line() {
    brain.driving_mode = FREE;
    motor_power(20);
    while (brain.driving_mode == FREE && !is_black()) {
        usleep(500000);
    }
    stop();
    brain.driving_mode == LINE;
}

void find_colour_values()
// uses the colour sensor to return the colour values to use them for calibation.
{
	while(true)
	{
		BP.get_sensor(s_contrast, contrast_struct);
		BP.get_sensor(s_colour, Colour_struct);
		cout << "high ref: " << contrast_struct.reflected << "  red: " << Colour_struct.reflected_red << 
			"  green: " << Colour_struct.reflected_green << "  blue: " << Colour_struct.reflected_blue << endl;
		usleep(100000);
	}
}

void find_intersection()
// finds intersections in the back line and finds intersections that are closed.
{
	sleep(3);
	while(true)
	{
		BP.get_sensor(s_colour, Colour_struct);
		if(Colour_struct.reflected_red <= 350 && Colour_struct.reflected_green <= 350 && Colour_struct.reflected_blue <= 350)
		{
			cout << "found intersection" << endl;
		}
		if(Colour_struct.reflected_red >= 400 && Colour_struct.reflected_green <= 350 && Colour_struct.reflected_blue <= 350)
		{
			cout << "found closed intersection" << endl;
			stop();
			break;
		}
		usleep(100000);//Sleep so it doesn't spam the console.
	}
}

bool intersection()
{

	return is_black() && colour_is_black();
}

void print()
{
	while(true)
	{
		cout << "****intersection:" << intersection() << "****" << endl;
		usleep(100000);
	}
}

//bool closed_intersection()

int main() {
    setup();
    calibrate();

    brain.driving_mode = LINE;

    // Start sensor threads
   // thread scan_distance (scan_ultrasonic);
    //thread stop_opbeject (object_in_the_way);
	

    // Start driving thread
    thread init_drive (drive);
	
	// Start intersection thread
//	thread scan_intersection(find_intersection);
	thread scan_colour_values(find_colour_values);
//	thread scanfor_intersection(intersection);
	thread print_intersection(print);
	while(true)
	{
		if(intersection() == true)
		{
			cout << "found intersection" << endl;
//			brain.driving_mode = STOP;
//			stop();
		}
		usleep(100000);
	}

    while (true) {
        sleep(5);
    }

}