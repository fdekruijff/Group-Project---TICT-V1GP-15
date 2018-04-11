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

/// limited distance stops PID
int limited_distance = 10;

/// Calibration variable declaration
bool calibrating = false;
int16_t high_reflection = 0;                    // Black
int16_t low_reflection = 0;                     // White

int16_t red_high_reflection = 0;                // + red
int16_t red_low_reflection = 0;                 // - red

int color_set_point = 0;

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
    float last_time = 1.0;                      // Domain: unknown
    float i_gain = 0.01;                        // Domain: unknown
    float d_gain = 2.8;                         // Domain: unknown
    float p_gain = 0.225;                       // Domain: [0.275, 0.325]
    float compensation_multiplier = 950.0;      // Domain: [750, 1200]
    int motor_power = 40;                       // Domain: [10, 80]
    int pid_update_frequency_ms = 13750;        // Domain: [10000, 175000]
    int max_x = 5;                              // GRID, max x value
    int max_y = 3;                              // GRID, max y value
    int driving_direction = RIGHT;              // Driving direction on GRID as seen from below the coordinate system
    bool exit = false;                          // Exit boolean to stop Wall-E
    string driving_mode = STOP;                 // Default driving mode
    vector<int> current_coordinates = {0, 0};   // Current position of Wall-E on the GRID.
    vector<int> last_coordinates = {0, 0};      // Previous position of Wall-E on the GRID.
    vector<vector<int>> grid;                   // -1 = out of bounds 0 = obstruction, 1 = explored, 2 = unexplored, 3 = destination, 4 = Wall-E
};

///  Declare the brain!
wall_e_settings brain;


void exit_signal_handler(int sig);

void stop() {
    /// Stops driving Wall-E and exit the threads.
    BP.set_motor_power(m_right, 0);
    BP.set_motor_power(m_left, 0);

    if (scan_distance.joinable()) {
        scan_distance.join();
    }
    if (stop_object.joinable()) {
        stop_object.join();
    }
    if (init_drive.joinable()) {
        init_drive.join();
    }
}

void stop_driving() {
    /// Stops driving Wall-E and exit the threads.
    BP.set_motor_power(m_right, 0);
    BP.set_motor_power(m_left, 0);
}

void exit_signal_handler(int sig) {
    /// Control-C handler that resets Brick Pi and exits application.
    if (sig == SIGINT) {
        stop();
        BP.reset_all();
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
    // TODO: calibrate head position
}

void motor_power(int power) {
    /// Set motor power to specific power level simultaneously.
    BP.set_motor_power(m_left, uint8_t(power));
    BP.set_motor_power(m_right, uint8_t(power));
}

void motor_power_limit(int power, int limit) {
    /// Set motor power to specific power limit simultaneously.
    BP.set_motor_limits(m_left, uint8_t(power), uint8_t(limit));
    BP.set_motor_limits(m_right, uint8_t(power), uint8_t(limit));
}

void dodge(int turn_drive, int degrees, int distance) {
    // TODO: fix static motor calls and static values
    // TODO: fix bool mess
    //Makes Wall-E turn and drive straight
    int power = 40;

    //turn
    if (!bool(turn_drive)) {
        motor_power_limit(35, 1200);
        BP.set_motor_position_relative(m_left, int32_t(degrees * 5.95));
        BP.set_motor_position_relative(m_right, int32_t(degrees * 5.85 * -1));
        usleep(degrees * 40000);

        //drive
    } else if (bool(turn_drive)) {
        if (distance < 0) {
            distance *= -1;
            power *= -1;
        }
        motor_power(int8_t(power));
        usleep(76927 * distance);
        stop_driving();
    }
}

void scan_ultrasonic() {
    /// Returns ultrasonic value
    // TODO: maybe refactor this code.
    while (!brain.exit) {
        BP.get_sensor(s_ultrasonic, sonic_struct);
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
}

int16_t get_red_contrast() {
    /// Returns the reflected red value.
    BP.get_sensor(s_color, color_struct);
    return color_struct.reflected_red;
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
        usleep(12500);
    }
    high_reflection = max_vector(tmp);
    low_reflection = min_vector(tmp);
}

void measure_color_contrast() {
    /// Thread function that reads color sensor values and calculates maximum and minimum from local vector.
    // TODO: remove redundant variable declaration and redundant assinging of values
    vector<int16_t> red_tmp;

    while (calibrating) {
        red_tmp.push_back(get_red_contrast());
        usleep(12500);
    }
    red_high_reflection = max_vector(red_tmp);
    red_low_reflection = min_vector(red_tmp);
}

void calibrate() {
    /// Function reads sensor values while driving over the tape. Sets maximum, minimum and set point for PID.
    calibrating = true;
    thread measure(measure_contrast);
    thread color_measure(measure_color_contrast);
    int turn = 180;
    vector<vector<int>> power_profile = {
            {turn,  -turn},
            {-turn, turn},
            {-turn, turn},
            {turn,  -turn}};

    motor_power_limit(40, 0);
    for (int i = 0; i < power_profile.size(); i++) {
        BP.set_motor_position_relative(m_right, power_profile[i][0]);
        BP.set_motor_position_relative(m_left, power_profile[i][1]);
        usleep(650000);
    }
    calibrating = false;
    stop_driving();
    motor_power_limit(100, 0);
    measure.join();
    color_measure.join();
    brain.set_point = (high_reflection + low_reflection) / 2;
    color_set_point = (red_high_reflection + red_low_reflection) / 2;
}

int turn_head(int degree) {
    // TODO: this doesn't work
    BP.set_motor_position(m_head, degree);
}

int no_object() {
    ///keeps on driving till there is no object.
    // TODO: make sure the boolean logic works here
    bool to_object = false;
    bool object = false;
    bool end_of_object = false;
    while (!to_object) {
        if (!object and !end_of_object) {
            dodge(1, 0, 1);
            if (sonic_struct.cm < 20) {
                object = true;
            }
        }
        if (object and !end_of_object) {
            dodge(1, 0, 1);
            if (sonic_struct.cm > 30) {
                end_of_object = true;
            }
        }
        if (object and end_of_object) {
            dodge(1, 0, 20);
            to_object = true;
        }
    }
}


void steer_left(int amount) {
    /// Steer left motor.
    BP.set_motor_power(m_left, uint8_t(amount));
}

void steer_right(int amount) {
    /// Steer right motor.
    BP.set_motor_power(m_right, uint8_t(amount));
}

int bound(float value, int begin, int end) {
    /// Cap value between begin and end range. Used to keep PID motor values in boundaries.
    if (value < begin) return begin;
    if (value > end) return end;
    return int(value);
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

void find_color_values() {
    /// Uses color sensor to update color data structures.
    while (!brain.exit) {
        BP.get_sensor(s_contrast, contrast_struct);
        BP.get_sensor(s_color, color_struct);
        usleep(100000);
    }
}

bool is_black() {
    /// Is sensor value in the black domain?
    float sensor = get_contrast();
    return sensor > (brain.set_point - 100) && sensor < high_reflection;
}

bool color_is_black() {
/// Is color sensor value in the black domain?
    float sensor = color_struct.reflected_red;
//    float blue_sensor = color_struct.reflected_blue;
//    float green_sensor = color_struct.reflected_green;
    return sensor > (color_set_point - 100) && sensor < red_high_reflection;
//    return (red_sensor < color_set_point[0] && red_sensor < red_high_reflection);
//           (blue_sensor < color_set_point[1] && blue_sensor < blue_high_reflection) &&
//           (green_sensor < color_set_point[2] && green_sensor < green_high_reflection);
}


bool intersection() {
    return is_black() && color_is_black();
}

vector<int> translate_coordinates(int x, int y) {
    /// Translates coordinate system coordinates to nested vector coordinates
    return {x, int(brain.grid.size() - y - 1)};
}

int scan_surroundings() {
    /// Returns the information in the surrounding tiles
    vector<direction> dir_codes = {{UP,    0},
                                   {DOWN,  0},
                                   {RIGHT, 0},
                                   {LEFT,  0}}; // {right, down, up, left}
    vector<vector<int>> c = {{0,  1},
                             {0,  -1},
                             {-1, 0},
                             {-1, 0}};

    for (unsigned int i = 0; i < dir_codes.size(); i++) {
        int x = brain.current_coordinates[0] + c[i][0];
        int y = brain.current_coordinates[1] + translate_coordinates(0, c[i][1])[1];

        if (x < 0 || x > brain.grid[0].size() || y < 0 || y > brain.grid.size()) {
            dir_codes[i].code = -1;
        } else {
            dir_codes[i].code = brain.grid[x][y];
        }
    }

    direction tmp;
    for (unsigned int i = 0; i < dir_codes.size(); i++) {
        tmp = dir_codes[0];
        if (dir_codes[i].code > tmp.code) {
            tmp = dir_codes[i];
        }
        if (tmp.code == 3) {
            cout << "E.V.E. found!" << endl;
            dodge(0, 360, 0);
            brain.exit = true;
        }
    }
    return tmp.dir;
}

void turn_to_destination(int direction) {
    int turn = brain.driving_direction - direction;
    if (turn == -1 or turn == 3) {
        dodge(0, -90, 0);
    } else if (turn == 1 or turn == -3) {
        dodge(0, 90, 0);
    } else if (turn == 2 or turn == -2) {
        dodge(0, 180, 0);
    }
}

vector<int> motor_correction() {
    /// Returns PID calculated motor correction for left and right motor
    float output = calculate_correction();
    float comp = calc_compensation(brain.last_error);
    return {bound(brain.motor_power - comp - output, -100, 100), bound(brain.motor_power - comp + output, -100, 100)};
}

vector<int> get_new_coordinates(int direction, vector<int> current_position) {
    if (direction == UP) return {current_position[0], current_position[1] + 1};
    if (direction == DOWN) return {current_position[0], current_position[1] - 1};
    if (direction == RIGHT) return {current_position[0] + 1, current_position[1]};
    if (direction == LEFT) return {current_position[0] - 1, current_position[1]};
    return {0, 0};
}

void update_virtual_grid() {
    /// Update virtual grid based on position and driving direction
    // Set current position value to 1 for explored.
    brain.grid[brain.current_coordinates[0]][brain.current_coordinates[1]] = 1;

    // Set new position for Wall-E based on brain.driving direction
    brain.last_coordinates = brain.current_coordinates;

    brain.current_coordinates = get_new_coordinates(brain.driving_direction, brain.current_coordinates);
    brain.grid[brain.current_coordinates[0]][brain.current_coordinates[1]] = 4;
}

void drive() {
    /// Threaded function that applies certain drive mode.
    while (!brain.exit) {
        vector<int> correction = motor_correction();
        if (brain.driving_mode == STOP || brain.driving_mode == FREE) {
            usleep(250000);
            continue;
        }
        if (brain.driving_mode == LINE) {
            /// Follows line by sending corrections to motors according to calculated error
            steer_left(correction[0]);
            steer_right(correction[1]);
            usleep(brain.pid_update_frequency_ms);
        }
        if (brain.driving_mode == GRID) {
            /// Drives the grid based on intersections and LINE drive mode

            while (!intersection()) {
                correction = motor_correction();
                steer_left(correction[0]);
                steer_right(correction[1]);
                usleep(brain.pid_update_frequency_ms);
            }
            cout << "Intersection found!" << endl;
            // Intersection has been found
            int direction = scan_surroundings();    // Get desired direction
            cout << "New direction: " << direction << endl;
            turn_to_destination(direction);         // Turn Wall-E to next intersection

            // update GRID parameters
            brain.driving_direction = direction;
            update_virtual_grid();
        }
    }
}

void find_line() {
    brain.driving_mode = FREE;
    while (brain.driving_mode == FREE && !is_black()) {
        usleep(500000);
    }
    stop_driving();
    cout << "Wall-E found the line again!, starting PID controller." << endl;
    dodge(0, -90, 0);
    brain.driving_mode = LINE;
}


void around_object() {
    thread findLine(find_line);
    /// main function to drive around the obstacle. it calls all the functions in the right order
    vector<vector<int>> v_around_object = {{-90, 90, 1},
                                           {90,  1},
                                           {90,  0,  20}};
    while (brain.driving_mode == OBJECT) {
        for (int i = 0; i < v_around_object.size(); i++) {
            dodge(0, v_around_object[i][0], 0);
            if (v_around_object[i][1] == 90 or v_around_object[i][1] == 0) {
                turn_head(v_around_object[i][1]);
            } else if (v_around_object[i][1] == 1) {
                no_object();
            }
            if (brain.driving_mode == LINE) {
                break;
            }
            if (v_around_object[i].size() == 3 and v_around_object[i][2] == 1) {
                dodge(1, 0, 20);
            } else if (v_around_object[i][1] == 20) {
                motor_power(20);
            }
        }
    }
    findLine.join();
}


void object_in_the_way() {
    /// If a object is in the way of the PID it stops the PID.
    sleep(1); //TODO: this is bad practice
    while (!brain.exit) {
        if (sonic_struct.cm < limited_distance) {
            brain.driving_mode = STOP;
            brain.driving_mode = OBJECT;
            around_object();
        }
    }
}

void set_drive_mode() {
    string mode = "STOP";
    cout << "Enter drive mode (STOP, LINE, GRID, FREE): ";
    cin >> mode;
    brain.driving_mode = "DRIVE_MODE_" + mode;
    cout << endl;
}

void set_grid_parameters() {
    int x = 0, y = 0;
    cout << "Enter desired grid size as integers divided by a space (x y): ";
    cin >> x >> y;

    for (unsigned int i = 0; i <= x; i++) {
        vector<int> tmp;
        for (unsigned int j = 0; j <= y; j++) {
            tmp.push_back(0);
        }
        brain.grid.push_back(tmp);
    }

    bool deciding = true;
    while (deciding) {
        cout << "Enter destination coordinates as integers divided by a space (x y): ";
        cin >> x >> y;

        if (x <= brain.max_x || x >= 0 || y <= brain.max_y || y >= 0) {
            deciding = false;
        }
        cout << endl;
    }
    vector<int> tmp = translate_coordinates(x, y);
    brain.grid[tmp[0]][tmp[1]] = 3;
}

int main() {
    setup();
    calibrate();

    // Set driving mode based on user input
    string drive_mode = "STOP";
    set_drive_mode();

    if (brain.driving_mode == GRID) {
        set_grid_parameters();
    }

    // Start sensor threads
    thread scan_distance(scan_ultrasonic);
    thread stop_object(object_in_the_way);
    thread scan_color_values(find_color_values);  //TODO: change this

    // Start driving thread
    thread init_drive(drive);

    while (!brain.exit) {
        // Just a infinite loop to keep the threads running
        usleep(500000);
    }
    stop();
}