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
<<<<<<< HEAD
    BP.set_motor_limits(m_left, uint8_t(power), dps);
    BP.set_motor_limits(m_right, uint8_t(power), dps);
=======
    BP.set_motor_limits(m_left, uint8_t(power), uint8_t(dps));
    BP.set_motor_limits(m_right, uint8_t(power), uint8_t(dps));
>>>>>>> eb65f3eabb399ac261ccae525f60a059cd762b65
}

int turn_head(int degree) {
        BP.set_motor_dps(m_head, degree);
        sleep(1);
        BP.set_motor_dps(m_head, 0);
}

void dodge(bool turn_drive, int degrees, int distance) {
    // TODO: fix static motor calls and static values
<<<<<<< HEAD
    // TODO: fix bool
=======
	// TODO: fix bool
>>>>>>> eb65f3eabb399ac261ccae525f60a059cd762b65
    //Makes Wall-E turn and drive straight
    int power = 40;

    //turn
    if (!turn_drive) {
<<<<<<< HEAD
        motor_power_limit(35, 1200);
=======
		motor_power_limit(35, 1200);
>>>>>>> eb65f3eabb399ac261ccae525f60a059cd762b65

        BP.set_motor_position_relative(m_left, int32_t(degrees * 5.95));
        BP.set_motor_position_relative(m_right, int32_t(degrees * 5.85 * -1));

<<<<<<< HEAD
        usleep(abs(degrees*20000));

        //drive
=======
	usleep(-1*degrees*15000);    
	    
	//drive
>>>>>>> eb65f3eabb399ac261ccae525f60a059cd762b65
    } else if (turn_drive) {
        if (distance < 0) {
            distance *= -1;
            power *= -1;
        }
<<<<<<< HEAD
        motor_power(power);
=======
		motor_power(power);
>>>>>>> eb65f3eabb399ac261ccae525f60a059cd762b65
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
<<<<<<< HEAD
    dodge(1, 0, 20);
    dodge(0, 180, 0);
    no_object();
    dodge(0,180, 0);
=======
    dodge(true, 0, 20);
    dodge(false, 180, 0);
    no_object(1);
    dodge(false,180, 0);
>>>>>>> eb65f3eabb399ac261ccae525f60a059cd762b65
    motor_power(20);
    sleep(99999999);
}
