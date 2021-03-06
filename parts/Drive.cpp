#include <cmath>
#include <iostream>
#include <unistd.h>
#include "./piprograms/BrickPi3.cpp"
#include <thread>
#include <signal.h>
#include <vector>

using namespace std;

BrickPi3 BP;
uint8_t s_contrast = PORT_2;            // Light sensor
uint8_t m_head = PORT_A;                // Head motor
uint8_t m_left = PORT_B;                // Left motor
uint8_t m_right = PORT_C;               // Right motor

void exit_signal_handler(int signo);

//Stop
void stop(void){
	 BP.set_motor_power(m_right, 0);
	 BP.set_motor_power(m_left, 0);
}

//Drive
void dodge(int turn_drive, int degrees, int distance){
	//Makes Wall-E turn and drive straight
	int power =40;
	
	if(turn_drive== 0){
		BP.set_motor_limits(m_left,35,1200);
		BP.set_motor_limits(m_right,35,1200);
		
		BP.set_motor_position_relative(m_left,degrees*5.95);
		BP.set_motor_position_relative(m_right,degrees*5.85*-1);
	}else if(turn_drive==1){
		if(distance < 0){
			distance *= -1;
			power *= -1;
		}
		BP.set_motor_power(m_left, power);
		BP.set_motor_power(m_right, power);
		usleep(76927*distance);
		stop();
	}
	
}



int main(){
	signal(SIGINT, exit_signal_handler); // register the exit function for Ctrl+C
  	BP.detect(); // Make sure that the BrickPi3 is communicating and that the firmware is compatible with the drivers.
	BP.set_motor_limits(PORT_B, 60, 0);
	BP.set_motor_limits(PORT_C, 60, 0);
  	char inp;

	while(true){
		cout << "Press s (stop), d (drive): " << endl;
		cin >> inp;	//Take input from the terminal
		if (inp=='d'){
				//drive or steer, degrees, (rigth positive, left negative), distance
				int turn_drive = 0;
				int degrees = 90;
				int distance =-10;
				dodge(turn_drive, degrees, distance);
				
		}else if (inp=='s'){
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