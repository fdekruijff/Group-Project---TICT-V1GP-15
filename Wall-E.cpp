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

int16_t red_high_reflection = 0;                // much Red
int16_t red_low_reflection = 0;                    // little Red
int16_t blue_high_reflection = 0;                // much blue
int16_t blue_low_reflection = 0;                // little blue
int16_t green_high_reflection = 0;                // much green
int16_t green_low_reflection = 0;                // little green

vector<int> color_set_point = {0, 0, 0};

/// Driving modes variable declaration
const string LINE = "DRIVE_MODE_LINE";
const string STOP = "DRIVE_MODE_STOP";
const string GRID = "DRIVE_MODE_GRID";
const string FREE = "DRIVE_MODE_FREE";
const string OBJECT = "DRIVE_MODE_OBJECT";
const string UP = "DIRECTION_UP";
const string DOWNN = "DIRECTION_DOWN";
const string LEFT = "DIRECTION_LEFT";
const string RIGHT = "DIRECTION_RIGHT";

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
    float p_gain = 0.275;                       // Domain: [0.275, 0.325]
    float compensation_multiplier = 950.0;      // Domain: [750, 1200]
    int motor_power = 35;                     // Domain: [10, 80]
    int pid_update_frequency_ms = 15000;      // Domain: [10000, 175000]
    int max_x = 5;                            // GRID, max x value
    int max_y = 3;                            // GRID, max y value
    bool exit = false;                         // Exit boolean to stop Wall-E
    string driving_mode = STOP;                  // Default driving mode
    string driving_direction = RIGHT;            // Driving direction on GRID as seen from below the coordinate system
    vector<int> current_coordinates = {0, 0};  // Current position of Wall-E on the GRID.
    vector<int> last_coordinates = {0, 0};     // Previous position of Wall-E on the GRID.
    vector<vector<int>> grid;                  // 0 = unexplored, 1 = obstruction, 2 = explored, 3 = destination, 4 = Wall-E
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

void dodge(int turn_drive, int degrees, int distance) {
    //Makes Wall-E turn and drive straight
    int power = 40;
    degrees /= 5.625; //360 conversion to 64

    //turn
    if (turn_drive == 0) {
		BP.set_motor_limits(m_left,35,1200);
		BP.set_motor_limits(m_right,35,1200);
		
		BP.set_motor_position_relative(m_left,degrees*5.95);
		BP.set_motor_position_relative(m_right,degrees*5.85*-1);

        //drive
    } else if (turn_drive == 1) {
        if (distance < 0) {
            distance *= -1;
            power *= -1;
        }
        BP.set_motor_power(m_left, power);
        BP.set_motor_power(m_right, power);
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
        usleep(12500);
    }
    high_reflection = max_vector(tmp);
    low_reflection = min_vector(tmp);
}

void measure_color_contrast() {
    /// Thread function that reads color sensor values and calculates maximum and minimum from local vector.
    vector<int16_t> red_tmp;
    vector<int16_t> blue_tmp;
    vector<int16_t> green_tmp;

    while (calibrating) {
        BP.get_sensor(s_color, color_struct);
        red_tmp.push_back(color_struct.reflected_red);
        blue_tmp.push_back(color_struct.reflected_blue);
        green_tmp.push_back(color_struct.reflected_green);
        usleep(25000);
    }
    red_high_reflection = max_vector(red_tmp);
    red_low_reflection = min_vector(red_tmp);
    blue_high_reflection = max_vector(blue_tmp);
    blue_low_reflection = min_vector(blue_tmp);
    green_high_reflection = max_vector(green_tmp);
    green_low_reflection = min_vector(green_tmp);
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

    motor_power_limit(40);
    for (int i = 0; i < power_profile.size(); i++) {
        BP.set_motor_position_relative(m_right, power_profile[i][0]);
        BP.set_motor_position_relative(m_left, power_profile[i][1]);
        usleep(650000);
    }
    calibrating = false;
    stop_driving();
    motor_power_limit(100);
    measure.join();
    color_measure.join();
    brain.set_point = (high_reflection + low_reflection) / 2;
    color_set_point[0] = (red_high_reflection + red_low_reflection) / 2;
    color_set_point[1] = (blue_high_reflection + blue_low_reflection) / 2;
    color_set_point[2] = (green_high_reflection + green_low_reflection) / 2;
    cout << "Calibration finished." << endl <<
         "high:" << int(high_reflection) << " low:" << int(low_reflection) << " set:" << brain.set_point << endl;
    cout << "Red_high:   " << red_high_reflection << " Red_low:   " << red_low_reflection << endl <<
         "Blue_high:  " << blue_high_reflection << " Blue_low:  " << blue_low_reflection << endl <<
         "Green_high: " << green_high_reflection << " Green_low: " << green_low_reflection << endl;
}

int turn_head(int degree) {
    BP.set_motor_position(m_head, degree);
}

int no_object(int mode) {
    ///keeps on driving till there is no object.
    bool to_object = false;
    bool object = false;
    bool end_of_object = false;
    while (!to_object) {
        if (to_object == false and object == false and end_of_object == false) {
            dodge(1, 0, 1);
            if (sonic_struct.cm < 20) {
                object = true;
            }
        }
        if (to_object == false and object == true and end_of_object == false) {
            dodge(1, 0, 1);
            if (sonic_struct.cm > 30) {
                end_of_object = true;
            }
        }
        if (to_object == false and object == true and end_of_object == true) {
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

void turn_head_body(int degrees) {
    ///turns head and body at the same time in threads.
    dodge(0, degrees, 0);
    turn_head(degrees);
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

void find_color_values()
// uses the color sensor to return the color values to use them for calibation.
{
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

bool is_white() {
    /// Is sensor value in the white domain?
    float sensor = get_contrast();
    return sensor < high_reflection && sensor > brain.set_point;
}

bool color_is_black() {
/// Is color sensor value in the black domain?
    float red_sensor = color_struct.reflected_red;
    float blue_sensor = color_struct.reflected_blue;
    float green_sensor = color_struct.reflected_green;
    return (red_sensor < color_set_point[0] && red_sensor < red_high_reflection) &&
           (blue_sensor < color_set_point[1] && blue_sensor < blue_high_reflection) &&
           (green_sensor < color_set_point[2] && green_sensor < green_high_reflection);
}

bool intersection() {
    return is_black() && color_is_black();
}

vector<int> translate_coordinates(int x, int y) {
    /// Translates coordinate system coordinates to nested vector coordinates
    return{x, int(brain.grid.size() - y)};
}

string scan_surroundings() {
    /// Returns the best direction to move to
    vector<int> dir_codes; // {up, down, left, right}

    for (int i = 0; i < 4; i++) { // Check 4 directions

    }
}

vector<int> motor_correction() {
    float output = calculate_correction();
    float comp = calc_compensation(brain.last_error);
    return {bound(brain.motor_power - comp - output, -100, 100), bound(brain.motor_power - comp + output, -100, 100)};
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
            // RIGHT == x+1     UP    == y+1
            // LEFT  == x-1     DOWN  == y-1

            while (!intersection()) {

            }
            // Intersection has been found


        }
    }
}

void find_line() {
    brain.driving_mode = FREE;
    motor_power(20);
    while (brain.driving_mode == FREE && !is_black()) {
        usleep(500000);
    }
    stop_driving();
    cout << "Wall-E found the line again!, starting PID controller." << endl;
    dodge(0, -90, 0);
    brain.driving_mode = LINE;
}


void around_object() {
    thread findLine (find_line);
    /// main function to drive around the obstacle. it calls all the functions in the right order
    vector<vector<int>> v_around_object = {{-90,90,1},{90,1},{90,0,20}};
    while (brain.driving_mode == OBJECT){
        for (int i = 0; i < v_around_object.size(); i++){
                dodge(0,v_around_object[i][0],0);
                if (v_around_object[i][1] == 90 or v_around_object[i][1] == 0){
                    turn_head(v_around_object[i][1]);
                }else if (v_around_object[i][1] == 1){
                     no_object(1);
                }
                if (brain.driving_mode == LINE){
                    break;
                }
                if ( v_around_object[i].size() == 3 and v_around_object[i][2] == 1){
                    dodge(1, 0, 20);
                }else if (v_around_object[i][1] == 20){
                    motor_power(20);
            }
        }
    }  
}


void object_in_the_way() {
    /// If a object is in the way of the PID it stops the PID.
    sleep(1); //TODO: this is bad practice
    while (!brain.exit) {
        if (sonic_struct.cm < limited_distance) {
            brain.driving_mode = STOP;
	    brian.driving_mode = OBJECT;
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
    cout << "Enter desired grid size as integers divided by a spece (x y): ";
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

        if (x <= brain.max_x || x >= 0 || y <= brain.max_y || y >= 0) {
            deciding = false;
        }
        cout << endl;
    }
    brain.grid[x][y] = 2;
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
    thread scan_color_values(find_color_values);

    // Start driving thread
    thread init_drive(drive);

    while (!brain.exit) {
        // Just a infinite loop to keep the threads running
        usleep(500000);
    }
    stop();
}
