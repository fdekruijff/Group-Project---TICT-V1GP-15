#include <cmath>
#include <iostream>
#include <thread>
#include "./piprograms/BrickPi3.cpp"
#include <vector>
#include <unistd.h>
#include <algorithm>
#include <signal.h>     

using namespace std;

BrickPi3 BP;

void exit_signal_handler(int signo);

void stop(void){
	BP.set_motor_power(PORT_A, 0);
	//BP.set_motor_power(PORT_B, 0);
	//BP.set_motor_power(PORT_C, 0);
}

void brick_py_setup() {
	signal(SIGINT, exit_signal_handler); // register the exit function for Ctrl+C
	BP.detect(); // Make sure that the BrickPi3 is communicating and that the firmware is compatible with the drivers.
	BP.set_motor_limits(PORT_A, 40, 0);
}


int turnHead(int degree){
    BP.set_motor_position(PORT_A, degree);
    return 1;
}

int head(int degrees){
    turnHead(degrees);
  signal(SIGINT, exit_signal_handler); // register the exit function for Ctrl+C
  BP.detect(); // Make sure that the BrickPi3 is communicating and that the firmware is compatible with the drivers.
  int error;
  BP.set_sensor_type(PORT_3, SENSOR_TYPE_NXT_ULTRASONIC);
  sensor_ultrasonic_t Ultrasonic2;
for (int i =0; i < 1000;i++){
    if(BP.get_sensor(PORT_3, Ultrasonic2) == 0){
		//cout << "Ultrasonic sensor (S2): "   << Ultrasonic2.cm << "cm" << endl;
		//usleep(100);
 	}
}
turnHead(degrees*-1);
cout << Ultrasonic2.cm<< endl;
}


int main(){
	brick_py_setup();
    head(50);
}


// Signal handler that will be called when Ctrl+C is pressed to stop the program
void exit_signal_handler(int signo){
  if(signo == SIGINT){
    BP.reset_all();    // Reset everything so there are no run-away motors
    exit(-2);
  }
}
