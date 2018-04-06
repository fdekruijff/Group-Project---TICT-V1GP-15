#include "BrickPi3.h"
#include <cmath>
#include <iostream>
#include <unistd.h>
#include "./piprograms/BrickPi3.cpp"
#include <thread>
#include <vector>

using namespace std;

BrickPi3 BP;
uint8_t s_color = PORT_1;				// Colour sensor
uint8_t s_contrast = PORT_2;            // Light sensor
uint8_t m_head = PORT_A;                // Head motor
uint8_t m_left = PORT_B;                // Left motor
uint8_t m_right = PORT_C;               // Right motor

bool Calibrate = false;
// set colour reflection values for when the robot wil see an intersection
int i_red = 300;
int i_green = 300;
int i_blue = 300;

	int16_t Color_Value = 0;
	sensor_color_t      Color1;

void Setup()
{
	BP.detect();
	BP.set_sensor_type(s_color, SENSOR_TYPE_NXT_COLOR_FULL);
}

void Find_Red()
{
	while(true)
	{
		error = 0;
		if(BP.get_sensor(s_color, Color1) == 0)
		{
			cout << "Color sensor (S1): detected  " << (int) Color1.color;
			cout << " red" << setw(4) << Color1.reflected_red;
			cout << " green" << setw(4) << Color1.reflected_green;
			cout << " blue" << setw(4) << Color1.reflected_blue;
			cout << " ambient" << setw(4) << Color1.ambient << endl;
		}
	}
}

int main()
{
	Setup();
//	signal(SIGINT, exit_signal_handler); // register the exit function for Ctrl+C
	Find_Red();
}