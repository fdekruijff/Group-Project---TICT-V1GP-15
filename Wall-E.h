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
thread findLine;

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
    float compensation_multiplier = 950.0;      // Domain: [750, 1200]
    int motor_power = 40;                       // Domain: [10, 80]
    int pid_update_frequency_ms = 13500;        // Domain: [10000, 175000]
    int driving_direction = RIGHT;              // Driving direction on GRID as seen from below the coordinate system
    int limited_distance = 10;                  // Sensor distance to stop PID
    bool exit = false;                          // Exit boolean to stop Wall-E
    string driving_mode = STOP;                 // Default driving mode
    vector<int> current_coordinates = {1, 1};   // Current position of Wall-E on the GRID.
    vector<int> last_coordinates = {0, 0};      // Previous position of Wall-E on the GRID.
    vector<vector<int>> grid;                   // -1 = out of bounds 0 = obstruction, 1 = explored, 2 = unexplored, 3 = destination, 4 = Wall-E
};

///  Declare the brain!
wall_e_settings brain;

void exit_signal_handler(int sig);              // Control-C handler that resets Brick Pi and exits application.
void stop();                                    // Stops driving Wall-E and exit the threads.
void stop_driving();
void setup();
void motor_power(int power);
void motor_power_limit(int power);
void dodge(int turn_drive, int degrees, int distance);
void scan_ultrasonic();
void steer_left(int amount);
void steer_right(int amount);
void turn_to_destination(int direction);
void update_virtual_grid();
void drive();
void find_line();
void around_object();
void object_in_the_way();
void set_drive_mode();
void set_grid_parameters();
void measure_contrast();
void calibrate();
void print_grid();
float calc_compensation(float x);
float calculate_correction();
int translate_y(int y);
int scan_surroundings();
int bound(float value, int begin, int end);
int turn_head(int degree);
int no_object();
bool is_black();
bool color_is_black();
bool intersection();
vector<int16_t> get_contrast();
vector<int> motor_correction();
vector<int> get_new_coordinates(int direction, vector<int> current_position);
int16_t max_vector(vector<int16_t> const vec);
int16_t min_vector(vector<int16_t> const vec);
int16_t line_val();

#endif //CLION_WALL_E_H