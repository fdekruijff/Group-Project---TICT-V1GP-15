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


float scan(void){
	sensor_ultrasonic_t Ultrasonic2;
	vector<int> sensordata = {};
	for (int i =0; i<1000 ;i++){
		BP.get_sensor(PORT_3, Ultrasonic2); //Check 1000 times Ultrasonic sensor to get data
        if (Ultrasonic2.cm != 0){
            sensordata.push_back(Ultrasonic2.cm);
        }
        sleep(0.01);
	}
    float average = 0;
    for (int i = 0; i < sensordata.size();i++){
        average += sensordata[i];
    }
	return average / sensordata.size();
}

float turn_head_scan(int degree){
	turn_head(degree);
	sleep(1);
	float dist = scan();
	sleep(1);
	turn_head(degree*-1);
	return dist;
}


int main(){
	brick_py_setup();
    for (int x = 0; x < 25;x++){
	//cout << turn_head_scan(50)<< "opzij"<<endl;
    sleep(0.2);
    cout << scan()<< "rechtdoor" <<endl;
    }
	stop_head();
}



