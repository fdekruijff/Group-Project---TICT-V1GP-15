#include <cmath>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include "./piprograms/BrickPi3.cpp"
#include <thread>
#include <vector>

int size_object = 0;

int no_object(){
     while (afstand != 0){
        //drive 1 cm 
        size_object++;
    } 
}


int around_object(0){
    turn_head_body(90);
    no_object
    //drive 10 cm 
    turn_head_body(90*-1);
    //drive 10 cm
    turn_head(90);
    no_object();
    turn_head_body(90*-1);
    //drive size_object;
    findline();   
} 

