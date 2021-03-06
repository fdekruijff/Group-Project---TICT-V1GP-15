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

void dodge(bool turn_drive, int degrees, int distance) {
    // TODO: fix static motor calls and static values
    //Makes Wall-E turn and drive straight
    int power = 40;

    //turn
    if (!turn_drive) {
        motor_power_limit(35, 1200);

        BP.set_motor_position_relative(m_left, int32_t(degrees * 5.95));
        BP.set_motor_position_relative(m_right, int32_t(degrees * 5.85 * -1));

        usleep(abs(degrees*15000));
    } else if (turn_drive) {
        if (distance < 0) {
            distance *= -1;
            power *= -1;
        }
        motor_power(power);
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

void around_object() {
    /// main function to drive around the obstacle. it calls all the functions in the right order
    dodge(false, -90, 0);
    turn_head(90);
    dodge(true, 0, 20);
    dodge(false, 180, 0);
    no_object();
    dodge(false,180, 0);
    motor_power(20);
    sleep(99999999);
}