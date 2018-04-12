#include "../Wall-E.h"

vector<int16_t> get_contrast() {
    /// Returns the reflected black / white contrast.
    BP.get_sensor(s_contrast, contrast_struct);
    BP.get_sensor(s_color, color_struct);
    return {contrast_struct.reflected, color_struct.reflected_red};
}

int16_t val_vector(vector<int16_t> const vec, bool max_min) {
    /// Returns the highest or lowest integer value of a vector.
    // TODO: fix  duplicate code here
    if(!max_min){
		int16_t tmp = vec[0];
		for (unsigned int i = 0; i < vec.size(); i++) {
			if (vec[i] > tmp) {
				tmp = vec[i];
			}
		}
	}else if(max_min){
		int16_t tmp = 10000;
		for (unsigned int i = 0; i < vec.size(); i++) {
			if (vec[i] < tmp && vec[i] != 0) {
				tmp = vec[i];
			}
		}
	}
	return tmp;
}

void measure_contrast() {
    /// Thread function that reads sensor values and calculates maximum and minimum from local vector.
    vector<int16_t> _tmp;   // Sensor 1
    vector<int16_t> __tmp;  // Sensor 2
    while (calibrating) {
        vector<int16_t> c = get_contrast();
        _tmp.push_back(c[0]);
        __tmp.push_back(c[1]);
        usleep(12500);
    }
    high_reflection = val_vector(_tmp, false);
    low_reflection = val_vector(_tmp, true);
    red_high_reflection = val_vector(__tmp, true);
    red_low_reflection = val_vector(__tmp, false);
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
    brain.set_point = (high_reflection + low_reflection) / 2;
    brain.color_set_point = (red_high_reflection + red_low_reflection) / 2;
    cout << "Calibration finished." << endl <<
         "high:" << int(high_reflection) << " low:" << int(low_reflection) << " set:" << brain.set_point << endl;
    cout << "r_high:" << int(red_high_reflection) << " r_low:" << int(red_low_reflection) << " r_set:"
         << brain.color_set_point << endl;
}

bool is_black() {
    /// Is sensor value in the black domain?
    float sensor = get_contrast()[0];
    return sensor > (brain.set_point - 100);
}

bool color_is_black() {
    /// Is color sensor value in the black domain?
    float sensor = get_contrast()[1];
    return sensor < (brain.color_set_point - 50);
}

bool intersection() {
	/// Detects intersections.
    return is_black() && color_is_black();
}

int16_t line_val() {
    /// Returns the reflected black / white contrast.
    BP.get_sensor(s_contrast, contrast_struct);
    return contrast_struct.reflected;
}

int no_object() {
    /// Keeps on driving till there is no object.
    // TODO: make sure the boolean logic works here
    bool to_object = false;
    bool object = false;
    bool end_of_object = false;
    while (!to_object) {
        if (!object and !end_of_object) {
            dodge(true, 0, 1);
            if (scan_ultrasonic() < 20) {
                object = true;
            }
        }
        if (object and !end_of_object) {
            dodge(true, 0, 1);
            if (scan_ultrasonic() > 30) {
                end_of_object = true;
            }
        }
        if (object and end_of_object) {
            dodge(true, 0, 20);
            to_object = true;
        }
    }
}

int scan_ultrasonic() {
    /// Returns ultrasonic value.
	BP.get_sensor(s_ultrasonic, sonic_struct);
    return sonic_struct.cm;
}

void object_in_the_way() {
    /// If a object is in the way of the PID it stops the PID.
    sleep(1); //TODO: this is bad practice
<<<<<<< HEAD
    while (!brain.exit) {
        if (sonic_struct.cm < brain.limited_distance) {
=======
    while (!brain.exit and brain.driving_mode == LINE) {
        if (scan_ultrasonic() < brain.limited_distance) {
>>>>>>> eb65f3eabb399ac261ccae525f60a059cd762b65
            brain.driving_mode = STOP;
            thread move_around (around_object);
            find_line();
            if (move_around.joinable()) move_around.join();
        }
    }
}
