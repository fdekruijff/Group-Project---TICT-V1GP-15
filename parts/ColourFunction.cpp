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

int16_t Color_Value = 0;

void Setup()
{
	BP.detect();
	BP.set_sensor_type(PORT_1, SENSOR_TYPE_NXT_COLOR_FULL);
	sensor_color_t      Color1;
}

bool Find_Red(void)
{
	cout << BP.get_sensor(PORT_1, Color1);
}

int main()
{
	signal(SIGINT, exit_signal_handler); // register the exit function for Ctrl+C
	
}