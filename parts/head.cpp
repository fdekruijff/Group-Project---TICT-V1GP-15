#include <cmath>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include "./piprograms/BrickPi3.cpp"
#include <thread>
#include <vector>

using namespace std;


BrickPi3 BP;
uint8_t s_contrast = PORT_2;            // Light sensor
uint8_t m_head = PORT_A;                // Head motor
uint8_t m_left = PORT_B;                // Left motor
uint8_t m_right = PORT_C;               // Right motor

//Exit signal


//void exit_signal_handler(int signo);

void brick_py_setup() {
	//signal(SIGINT, exit_signal_handler);
	BP.detect();
	BP.set_sensor_type(s_contrast, SENSOR_TYPE_NXT_LIGHT_ON);
	BP.set_sensor_type(PORT_3, SENSOR_TYPE_NXT_ULTRASONIC); //Ultrasonic sensor setup
}

void stopHead(void){
	//Useless function
	BP.set_motor_power(m_right, -128);
}


int turnHead(int degree){
	BP.set_motor_position(PORT_A, degree);
	return 1;
}

int head(void){
	sensor_ultrasonic_t Ultrasonic2;
	
	for (int i =0; i<1000 ;i++){
		BP.get_sensor(PORT_3, Ultrasonic2); //Check 1000 times Ultrasonic sensor to get data
	}
	return Ultrasonic2.cm;
}

int turnHeadScan(int degree){
	turnHead(degree);
	sleep(0.5);
	int dist = head();
	sleep(0.5);
	turnHead(degree*-1);
	return dist;
}


int main(){
	brick_py_setup();
	cout << turnHeadScan(50)<< endl;
	stopHead();
}



