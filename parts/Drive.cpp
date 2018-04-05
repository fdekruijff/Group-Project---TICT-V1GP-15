#include "BrickPi3.h"   // BrickPi3
#include <iostream>     // cout
#include <unistd.h>     // sleep
#include <signal.h>     // catching exit signals

using namespace std;

BrickPi3 BP;
uint8_t s_contrast = PORT_2;            // Light sensor
uint8_t m_head = PORT_A;                // Head motor
uint8_t m_left = PORT_B;                // Left motor
uint8_t m_right = PORT_C;               // Right motor

void exit_signal_handler(int signo);

//Stop
void stop(void){
	
	 BP.set_motor_power(PORT_A, 0);
	 BP.set_motor_power(PORT_B, 0);
	 BP.set_motor_power(PORT_C, 0);
}

//Move Forward
void fwd(void){
	for(int i=0; i < 8; i++){
		BP.set_motor_power(PORT_B, (i*10));
		BP.set_motor_power(PORT_C, (i*10));
		usleep(100000);
	}
	//leep(0.5);
	stop();
}

//Drive
void drive(vector<int> distance){
	if(distance[0]== 0){
		//turn
		BP.set_motor_dps(m_left, 180);
		BP.set_motor_dps(m_right, 180*-1);
		sleep(1);
		stop();
	}else if(distance[0]==1){
		//drive
		int degree =38.5 * distance[2];
		for(int i=0; i<(degree/360); i++){
			if(i==0){
				BP.set_motor_position_relative(m_left, degree%360);
				BP.set_motor_position_relative(m_right, degree%360);
				usleep(500000);
			}else{
				BP.set_motor_dps(m_left, 180);
				BP.set_motor_dps(m_right, 180);
				sleep(2);
				stop();
			}
		}
	}
}


//Move Left
void left(void){
	BP.set_motor_power(PORT_B, 20);
	BP.set_motor_power(PORT_C, -20);
}


//Move Right
void right(void){
	 BP.set_motor_dps(PORT_B, -360);
	 BP.set_motor_dps(PORT_C, 360);
	 sleep(1);
	 stop();
}


//Move backward
void back(void){
	 BP.set_motor_dps(PORT_B, -80);
	 BP.set_motor_dps(PORT_C, -80);
	 sleep(3);
	 stop();
}


int main(){
	signal(SIGINT, exit_signal_handler); // register the exit function for Ctrl+C
  	BP.detect(); // Make sure that the BrickPi3 is communicating and that the firmware is compatible with the drivers.
  	BP.set_motor_limits(PORT_A, 40, 0);
	BP.set_motor_limits(PORT_B, 60, 0);
	BP.set_motor_limits(PORT_C, 60, 0);
  	char inp;

	while(true){
		cout << "Press f (forward), b (backward), l(left), r (right), s (stop), d (drive): " << endl;
		cin >> inp;	//Take input from the terminal
		//Move the bot
		if(inp=='f') {
			  	fwd();
			}
		else if (inp=='b') {
			  	back();
			}
		else if (inp=='l'){
				left();
			}
		else if (inp=='r'){
				right();
			}
		else if (inp=='d'){
				vector<int> s = {1, 0, 10} //drive or steer, degrees(rigth positive, left negative), distance
				drive(s);
			}
		else if (inp=='s'){
				stop();
			}
	}

	return 0;
}

// Signal handler that will be called when Ctrl+C is pressed to stop the program
void exit_signal_handler(int signo){
  if(signo == SIGINT){
    BP.reset_all();    // Reset everything so there are no run-away motors
    exit(-2);
  }
}