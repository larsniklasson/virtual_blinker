#!/bin/bash


deviation=5;


for ((test_var=2;test_var<=2;test_var++)); 
do
    for ((var=0;var<=0;var++)); 
    do 
    

        for ((i=85;i<=85;i++));
        do 
            roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=$i deviation:=$deviation seed:=0
        done
    done
done


