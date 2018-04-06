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
uint8_t s_color = PORT_1;                       // Color sensor
uint8_t s_contrast = PORT_2;                    // Light sensor
uint8_t s_ultrasonic = PORT_3;                  // Ultrasonic sensor
uint8_t m_head = PORT_A;                        // Head motor
uint8_t m_left = PORT_B;                        // Left motor
uint8_t m_right = PORT_C;                       // Right motor

/// Sensor data structures
sensor_light_t contrast_struct;
sensor_ultrasonic_t sonic_struct;

/// limited distance stops PID
int limited_distance = 10;

/// Calibration variable declaration
bool calibrating = false;
int16_t high_reflection = 0;                    // Black
int16_t low_reflection = 0;                     // White

/// Driving modes variable declaration
const string LINE = "LINE";
const string STOP = "STOP";
const string GRID = "GRID";
const string FREE = "FREE";

/// Thread declaration
thread scan_distance;
thread stop_object;
thread init_drive;

/// Wall-E brain settings data structure declaration
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
    bool exit = false;                          // Exit boolean to stop Wall-E
    string driving_mode = STOP;                 // Default driving mode
};

///  Declare the brain!
wall_e_settings brain;

void exit_signal_handler(int sig);

void stop() {
    /// Stops driving Wall-E and exit the threads.
    BP.set_motor_power(m_right, 0);
    BP.set_motor_power(m_left, 0);

    scan_distance.join();
    stop_object.join();
    init_drive.join();
}

void exit_signal_handler(int sig) {
    /// Control-C handler that resets Brick Pi and exits application.
    if (sig == SIGINT) {
        stop();
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

void driving(int turn_drive, int degrees, int distance){
	//Makes Wall-E turn and drive straight
	int power =40;
	float degrees /= 5.625; //360 conversion to 64
	
	//turn
	if(turn_drive== 0){
		if(degrees < 0){
			degrees *= -1;
			power *= -1;
		}
		
		BP.set_motor_power(m_left, power);
		BP.set_motor_power(m_right, power*-1);
		usleep(100000*degrees);
		stop();
		
	//drive
	}else if(turn_drive==1){
		if(distance < 0){
			distance *= -1;
			power *= -1;
		}
		BP.set_motor_power(m_left, power);
		BP.set_motor_power(m_right, power);
		usleep(76927*distance);
		stop();
	}
	
}

void scan_ultrasonic() {
    /// Returns ultrasonic value
    // TODO: maybe refactor this code.
    while (true) {
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


void object_in_the_way() {
    /// If a object is in the way of the PID it stops the PID.
    sleep(1); //TODO: this is bad practice
    while (true) {
        if (sonic_struct.cm < limited_distance) {
            brain.driving_mode = STOP;
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

int turn_head(int degree) {
    BP.set_motor_position(m_head, degree);
}

int no_object() {
    ///keeps on driving till there is no object.
    while (sonic_struct.cm < 20) {
        driving(1, 0, 1)  
    }
}

void steer_left(uint8_t amount) {
    /// Steer left motor.
    BP.set_motor_power(m_left, amount);
}

void steer_right(uint8_t amount) {
    /// Steer right motor.
    BP.set_motor_power(m_right, amount);
}

void turn_head_body(int degrees) {
    ///turns head and body at the same time in threads.
    thread turn (driving(0, degrees, 0));
    thread head_turn (turn_head(degrees));
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

void drive() {
    /// Threaded function that applies certain drive mode.
    while (true) {
        if (brain.driving_mode == STOP || brain.driving_mode == FREE) {
            usleep(250000);
            continue;
        }
        if (brain.driving_mode == LINE) {
            float output = calculate_correction();
            float comp = calc_compensation(brain.last_error);

            float left = int(bound(brain.motor_power - (comp + 5) - output, -90, 90));
            float right = int(bound(brain.motor_power - (comp + 5) + output, -90, 90));

            cout << brain.last_error << endl;
            steer_left(uint8_t(left));
            steer_right(uint8_t(right));
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


int around_object() {
    /// main function to drive around the obstacle. it calls all the functions in the right order
    turn_head_body(90);
    no_object();
    driving(1, 0, 10) 
    turn_head_body(90 * -1);
    driving(1, 0, 10)
    turn_head(90);
    no_object();
    turn_head_body(90*-1);
    find_line();
}


int main() {
    setup();
    calibrate();

    // Default driving mode after starting thread.
    brain.driving_mode = LINE;

    // Start sensor threads
    thread scan_distance(scan_ultrasonic);
    thread stop_object(object_in_the_way);

    // Start driving thread
    thread init_drive(drive);

    while (brain.exit) {
        // Just a infinite loop to keep the threads
        usleep(500000);
    }
    stop();
}
