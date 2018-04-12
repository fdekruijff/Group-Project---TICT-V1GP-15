#include "../Wall-E.h"

void stop_driving() {
    /// Stops driving Wall-E.
    BP.set_motor_power(m_right, 0);
    BP.set_motor_power(m_left, 0);
}

void motor_power(int power) {
    /// Set motor power to specific power level simultaneously.
    BP.set_motor_power(m_left, uint8_t(power));
    BP.set_motor_power(m_right, uint8_t(power));
}

void motor_power_limit(int power, int dps) {
    /// Set motor power to specific power limit simultaneously.
    BP.set_motor_limits(m_left, uint8_t(power), uint8_t(dps));
    BP.set_motor_limits(m_right, uint8_t(power), uint8_t(dps));
}

int turn_head(int degree) {
	// Turns head to disered position.
    // TODO: this doesn't work
    BP.set_motor_position(m_head, degree);
}

void dodge(int turn_drive, int degrees, int distance) {
    // TODO: fix static motor calls and static values
	// TODO: fix bool
<<<<<<< HEAD
    ///Makes Wall-E turn and drive straight.
=======
    //Makes Wall-E turn and drive straight
>>>>>>> e0ee5c7abf597534c2363f758b78ae5c8020ab1f
    int power = 40;

    //turn
    if (turn_drive == 0) {
		motor_power_limit(35, 1200)

        BP.set_motor_position_relative(m_left, int32_t(degrees * 5.95));
        BP.set_motor_position_relative(m_right, int32_t(degrees * 5.85 * -1));

	//drive
    } else if (turn_drive == 1) {
        if (distance < 0) {
            distance *= -1;
            power *= -1;
        }
		motor_power(power)
        usleep(76927 * distance);
        stop_driving();
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

<<<<<<< HEAD
void around_object() {
    thread findLine(find_line);
    /// Main function to drive around the obstacle. it calls all the functions in the right order.
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
=======
int around_object() {
    /// main function to drive around the obstacle. it calls all the functions in the right order
    dodge(0, -90, 0);
    turn_head(90);
    dodge(1, 0, 20);
    dodge(0, 180, 0);
    no_object(1);
    dodge(0,180, 0);
    motor_power(20);
    sleep(99999999);
}
>>>>>>> e0ee5c7abf597534c2363f758b78ae5c8020ab1f
