#include "BrickPi3.h"   // for BrickPi3
#include <iostream>     // for printf
#include <unistd.h>     // for usleep
#include <signal.h>     // for catching exit signals
#include <vector>       // for dynamic array

using namespace std;

BrickPi3 BP;

void exit_signal_handler(int signo);

//Stop
void stop(void)
	 BP.set_motor_power(PORT_A, 0);
	 BP.set_motor_power(PORT_B, 0);
	 BP.set_motor_power(PORT_C, 0);