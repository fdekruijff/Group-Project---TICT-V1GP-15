//Groupproject V1C 5 Esmee Blom, Stijn Fenijn, Bryan Campagne, Floris de Kruijff en Quin Ligteringen
//

#include "BrickPi3.h"   // BrickPi3
#include <iostream>     // cout
#include <unistd.h>     // sleep
#include <signal.h>     // catching exit signals

using namespace std;

BrickPi3 BP;

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

//Move own length
void length(void){
	BP.set_motor_dps(PORT_B, 360);
	BP.set_motor_dps(PORT_C, 360);
	 sleep(3);
	 stop();
}

void doorgaan(void){
	char doorgaan;
	 BP.set_motor_power(PORT_B, 20);
	 BP.set_motor_power(PORT_C, 20);
	 usleep(500000);
	 BP.set_motor_power(PORT_B, 40);
	 BP.set_motor_power(PORT_C, 40);
	 usleep(500000);
	 BP.set_motor_power(PORT_B, 60);
	 BP.set_motor_power(PORT_C, 60);
	 usleep(500000);
	 BP.set_motor_power(PORT_B, 80);
	 BP.set_motor_power(PORT_C, 80);
	 sleep(2);
	 stop();
}


//Move Left
void left(void){
	 BP.set_motor_position_relative(PORT_B, 270);
	 BP.set_motor_position_relative(PORT_C, -270);
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

//Move head
void head(int degree){
	 BP.set_motor_position_relative(PORT_A, degree); //110
	 sleep(1);
	 BP.set_motor_position_relative(PORT_A, (degree*-1));
	 sleep(1);
	 stop();
}

int main(){
	signal(SIGINT, exit_signal_handler); // register the exit function for Ctrl+C
  	BP.detect(); // Make sure that the BrickPi3 is communicating and that the firmware is compatible with the drivers.
  	BP.set_motor_limits(PORT_A, 40, 0);
	BP.set_motor_limits(PORT_B, 80, 0);
	BP.set_motor_limits(PORT_C, 80, 0);
  	char inp;

	while(true){
		cout << "Press f (forward), b (backward), l(left), r (right), h (head), s (stop), l (lengt): " << endl;
		cin >> inp;	//Take input from the terminal
		//Move the bot
		if(inp=='f') {
			  	fwd();
			}
		else if (inp=='d'){
				doorgaan();
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
		else if (inp=='h'){
				cout << "Degrees:" << endl;
				int degree;
				cin >> degree;
				head(degree);
			}
		else if (inp=='s'){
				stop();
			}
		else if (inp=='l'){
				length();
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