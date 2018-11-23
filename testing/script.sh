#!/bin/bash


deviation=0;


for ((test_var=0;test_var<=0;test_var++)); 
do
    for ((var=0;var<=0;var++)); 
    do 
    

        for ((i=25;i<=60;i+=2));
        do 
            roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=$i deviation:=$deviation seed:=0
        done
    done
done


