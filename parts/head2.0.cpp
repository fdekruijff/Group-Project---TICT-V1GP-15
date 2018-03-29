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
void exit_signal_handler(int signo);

void exit_signal_handler(int signo){
	if(signo == SIGINT){
		BP.reset_all(); 
		exit(-2);
	}
}


void brick_py_setup() {
	BP.detect();
	BP.set_sensor_type(s_contrast, SENSOR_TYPE_NXT_LIGHT_ON);
	signal(SIGINT, exit_signal_handler);
	BP.set_sensor_type(PORT_3, SENSOR_TYPE_NXT_ULTRASONIC); //Ultrasonic sensor setup
}

void stop_head(void){
	//Useless function
	BP.set_motor_power(m_right, 0);
}


int turn_head(int degree){
	BP.set_motor_position(PORT_A, degree);
	return 1;
}

int head(void){
	sensor_ultrasonic_t Ultrasonic2;
	
	for (int i =0; i<1000 ;i++){
		BP.get_sensor(PORT_3, Ultrasonic2) //Check 1000 times Ultrasonic sensor to get data
	}
	return Utrasonic2.cm;
}


int main(){
	brick_py_setup();
	
	int degree = 50;
	
	turn_head(degree);
	cout << head()<< endl;
	turn_head(degree*-1);
}



