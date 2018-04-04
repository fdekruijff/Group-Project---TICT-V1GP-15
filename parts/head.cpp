#include <cmath>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include "./piprograms/BrickPi3.cpp"
#include <thread>
#include <vector>


using namespace std;


BrickPi3 BP;
uint8_t s_contrast = PORT_2;            // Light sensor
uint8_t m_head = PORT_A;                // Head motor
uint8_t m_left = PORT_B;                // Left motor
uint8_t m_right = PORT_C;               // Right motor
int afstand = 0; 

//Exit signal


//void exit_signal_handler(int signo);

void brick_py_setup() {
	//signal(SIGINT, exit_signal_handler);
	BP.detect();
	BP.set_sensor_type(s_contrast, SENSOR_TYPE_NXT_LIGHT_ON);
	BP.set_sensor_type(PORT_3, SENSOR_TYPE_NXT_ULTRASONIC); //Ultrasonic sensor setup
}

void stop_head(void){
	//Useless function
	BP.set_motor_power(m_right, -128);
}


int turn_head(int degree){
	BP.set_motor_position(PORT_A, degree);
	return 1;
}


void scan_ultrasonic(){
	sensor_ultrasonic_t Ultrasonic2;
	while (true){
		BP.get_sensor(PORT_3, Ultrasonic2); //Check 1000 times Ultrasonic sensor to get data
        if (Ultrasonic2.cm != 0){
            afstand = Ultrasonic2.cm;
        }
        sleep(0.01);
    }
} 


int main(){
	brick_py_setup();
    thread scan (scan_ultrasonic);
    sleep(5);
    for (int x = 0; x < 100;x++){
    sleep(2);
    cout << afstand<< "rechtdoor" <<endl;
    }
	stop_head();
}



