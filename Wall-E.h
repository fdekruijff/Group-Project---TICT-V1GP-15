#ifndef CLION_WALL_E_H
#define CLION_WALL_E_H

#include <cmath>
#include <iostream>
#include <unistd.h>
#include "./piprograms/BrickPi3.cpp"
#include <thread>
#include <signal.h>
#include <iomanip>
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
sensor_color_t color_struct;

/// Calibration variable declaration
bool calibrating = false;
int16_t high_reflection = 0;                    // Black
int16_t low_reflection = 0;                     // White

int16_t red_high_reflection = 0;                // + red
int16_t red_low_reflection = 0;                 // - red

vector<int> color_set_point = {0, 0, 0};

/// Driving modes variable declaration
const string LINE = "DRIVE_MODE_LINE";
const string STOP = "DRIVE_MODE_STOP";
const string GRID = "DRIVE_MODE_GRID";
const string FREE = "DRIVE_MODE_FREE";
const string OBJECT = "DRIVE_MODE_OBJECT";
const int RIGHT = 1;
const int UP = 2;
const int LEFT = 3;
const int DOWN = 4;

/// Thread declaration
thread scan_distance;
thread stop_object;
thread init_drive;
thread x;

struct direction {
    int dir;
    int code;
};

/// Wall-E brain settings data structure declaration
struct wall_e_settings {
    float last_error = 0.0;                     // Value set by PID
    float i_error = 0.0;                        // Value set by PID
    float set_point = 0.0;                      // Value set by sensor
    float color_set_point = 0.0;                // Value set by sensor
    float last_time = 1.0;                      // Domain: unknown
    float i_gain = 0.01;                        // Domain: unknown
    float d_gain = 2.8;                         // Domain: unknown
    float p_gain = 0.225;                       // Domain: [0.275, 0.325]
    float compensation_multiplier = 1000.0;      // Domain: [750, 1200]
    int motor_power = 50;                       // Domain: [10, 80]
    int pid_update_frequency_ms = 13750;        // Domain: [10000, 175000]
    int driving_direction = RIGHT;              // Driving direction on GRID as seen from below the coordinate system
    int limited_distance = 2;                  // Sensor distance to stop PID
    bool exit = false;                          // Exit boolean to stop Wall-E
    bool found_eve = false;
    string driving_mode = STOP;                 // Default driving mode
    vector<int> current_coordinates = {0, 0};   // Current position of Wall-E on the GRID.
    vector<int> destination_coordinates = {0, 0};   // Current position of Wall-E on the GRID.
    vector<int> last_coordinates = {0, 0};      // Previous position of Wall-E on the GRID.
    vector<vector<int>> grid;                   // -1 = out of bounds 0 = obstruction, 1 = explored, 2 = unexplored, 3 = destination, 4 = Wall-E
};

///  Declare the brain!
wall_e_settings brain;

///Wall-E
void exit_signal_handler(int sig);              // Control-C handler that resets Brick Pi and exits application.
void stop();                                    // Stops driving Wall-E and exit the threads.
void setup();									// Brick PI and exit handler are initialised here.
void drive();									// Threaded function that applies certain drive mode.
void set_drive_mode();							// Sets driving mode of Wall-E

///motor_func
void stop_driving();							// Stops driving Wall-E.
void motor_power(int power);					// Set motor power to specific power level simultaneously.
void motor_power_limit(int power, int dps);  	// Set motor power to specific power limit simultaneously.
void dodge(bool turn_drive, int degrees, int distance);	//Makes Wall-E turn and drive straight.
void steer_left(int amount);					// Steer left motor.
void steer_right(int amount);					// Steer right motor.
void around_object();							// Main function to drive around the obstacle. it calls all the functions in the right order.
void object_in_the_way();						// If a object is in the way of the PID it stops the PID.
void set_drive_mode();							// Sets driving mode of Wall-E
void set_grid_parameters();						// Sets parameters for grid.
void measure_contrast();						// Thread function that reads sensor values and calculates maximum and minimum from local vector.
void calibrate();								// Function reads sensor values while driving over the tape. Sets maximum, minimum and set point for PID.
void print_grid();								// Visualise the virtual grid
float calc_compensation(float x);				// Calculates compensation for motor correction.
float calculate_correction();					// PID calculations are performed here. Magic for the most part.
vector<int> translate_xy_to_vector(int x,int y);// Translates coordinate system coordinates to nested vector coordinates.
int scan_surroundings();						// Returns the information in the surrounding tiles.
void scan_ultrasonic();							// Returns ultrasonic value.
int bound(float value, int begin, int end);		// Cap value between begin and end range. Used to keep PID motor values in boundaries.
int turn_head(int degree);						// Turns head to disered position.

///sensor_func
int no_object();								// Keeps on driving till there is no object.
bool is_black();								// Is sensor value in the black domain?
bool color_is_black();							// Is color sensor value in the black domain?
bool intersection();							// Detects intersections.
void object_in_the_way();						// If a object is in the way of the PID it stops the PID.
void measure_contrast();						// Thread function that reads sensor values and calculates maximum and minimum from local vector.
void calibrate();								// Function reads sensor values while driving over the tape. Sets maximum, minimum and set point for PID.
int scan_surroundings();						// Returns the information in the surrounding tiles.
void scan_ultrasonic();							// Returns ultrasonic value.
vector<int16_t> get_contrast();					// Returns the reflected black / white contrast.
int16_t max_vector(vector<int16_t> vec);        // Returns the highest integer value of a vector.
int16_t min_vector(vector<int16_t> vec);        // Returns the lowest integer value of a vector.
int16_t line_val();								// Returns the reflected black / white contrast.

///pid_func
void find_line();								// Finds line when driving straight.
float calc_compensation(float x);				// Calculates compensation for motor correction.
float calculate_correction();					// PID calculations are performed here. Magic for the most part.
vector<int> motor_correction();					// Returns PID calculated motor correction for left and right motor.

///grid_func
void turn_to_destination(int direction);		// Turns Wall-E on grid to next intersection.
void update_virtual_grid();						// Update virtual grid based on position and driving direction.
void set_grid_parameters();						// Sets parameters for grid.
void print_grid();								// Visualise the virtual grid.
vector<int> translate_xy_to_vector(int x,int y);// Translates coordinate system coordinates to nested vector coordinates.
vector<int> get_new_coordinates(int direction, vector<int> current_position);   // Gets new coordinates of Wall-E in grid.














#endif //CLION_WALL_E_H