#include <cmath>
#include <iostream>
#include <thread>
#include "../piprograms/BrickPi3.cpp"
#include <vector>
#include <unistd.h>
#include <algorithm>


using namespace std;

void brick_py_setup() {
    signal(SIGINT, exit_signal_handler);
    BP.detect();
    BP.set_sensor_type(PORT_2, SENSOR_TYPE_NXT_ULTRASONIC);
    sensor_ultrasonic_t Ultrasonic2;
}


int head(int degree){
    if (degree == 0){
         BP.get_sensor(PORT_3, Ultrasonic2);
         return Ultrasonic2.cm;
    }else{
        if (turnHead(degree) == 1){
            BP.get_sensor(PORT_3, Ultrasonic2);
            if (turnHead(degree *-1) == 1){
                return Ultrasonic2.cm;
            }
        }
    }
}


int turnHead(degree){
    BP.set_motor_position(PORT_A, degree);
    retun 1;
}
\

int main(){ 
    int degrees = 0 ;
    cout << "give degrees";
    cin >> degrees;
    cout << head(degrees);
}
