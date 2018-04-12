#include "Wall-E.h"
#include "lib/sensor_func.cpp"
#include "lib/motor_func.cpp"
#include "lib/grid_func.cpp"
#include "lib/pid_func.cpp"


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
    BP.set_sensor_type(s_color, SENSOR_TYPE_NXT_COLOR_RED);
    BP.set_sensor_type(s_ultrasonic, SENSOR_TYPE_NXT_ULTRASONIC);
    // TODO: calibrate head position
}

int bound(float value, int begin, int end) {
    /// Cap value between begin and end range. Used to keep PID motor values in boundaries.
    if (value < begin) return begin;
    if (value > end) return end;
    return int(value);
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
            // Intersection has been found
            cout << "Intersection found" << endl;
            dodge(1, 0, 10);
            int direction = scan_surroundings();    // Get desired direction
            turn_to_destination(direction);         // Turn Wall-E to next intersection

            // update GRID parameters
            brain.driving_direction = direction;
            update_virtual_grid();
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
//    thread scan_distance(scan_ultrasonic);
//    thread stop_object(object_in_the_way);

    // Start driving thread
    thread init_drive(drive);

    while (!brain.exit) {
        // Just a infinite loop to keep the threads running
        usleep(500000);
    }
    stop();
}