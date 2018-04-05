#include <cmath>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include "./piprograms/BrickPi3.cpp"
#include <thread>
#include <vector>


//if a object is in the way of the PID it stops the PID.
void object_in_the_way(){
	int limited_distance = 20;
	if (afstand < limited_distance && afstand != 0){
		brain.driving_mode = STOP;
	}
}


//turns head and body at the same time in threads. 
void turn_head_body(int degrees){
	thread turn (left(degrees));
	thread head_turn (turn_head(degrees));
}


//keeps on driving till there is no object.
int no_object(){
     while (afstand != 0){
        //drive 1 cm 
    } 
}


// main function to drive around the obstacle. it calls all the funcions in the right order
int around_object(){
    turn_head_body(90);
    no_object();
    //drive 10 cm 
    turn_head_body(90*-1);
    //drive 10 cm
    turn_head(90);
    no_object();
    turn_head_body(90*-1);
    //drive size_object;
    findline();   
} 

