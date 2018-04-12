#include "../Wall-E.h"

void find_line() {
<<<<<<< HEAD
	/// Finds line when driving straight.
=======
    sleep(5);
>>>>>>> e0ee5c7abf597534c2363f758b78ae5c8020ab1f
    brain.driving_mode = FREE;
    while (brain.driving_mode == FREE && !is_black()) {
        usleep(500000);
    }
    stop_driving();
    cout << "Wall-E found the line again!, starting PID controller." << endl;
    if (x.joinable()) x.join();
    dodge(0, -90, 0);
    turn_head(-90);
    brain.driving_mode = LINE;
}

int bound(float value, int begin, int end) {
    /// Cap value between begin and end range. Used to keep PID motor values in boundaries.
    if (value < begin) return begin;
    if (value > end) return end;
    return int(value);
}

float calc_compensation(float x) {
	/// Calculates compensation for motor correction. 
    return float((1.0 / brain.compensation_multiplier) * (x * x));
}

float calculate_correction() {
    /// PID calculations are performed here. Magic for the most part.
    float sensor = line_val();
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

vector<int> motor_correction() {
    /// Returns PID calculated motor correction for left and right motor.
    float output = calculate_correction();
    float comp = calc_compensation(brain.last_error);
    return {bound(brain.motor_power - comp - output, -100, 100), bound(brain.motor_power - comp + output, -100, 100)};
}
