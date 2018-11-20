#!/bin/bash

for ((seed=0;seed<=2;seed++)); 
do 

for ((test_var=0;test_var<=3;test_var++)); 
do

for ((deviation=0;deviation<=6;deviation++)); 
do

    for ((var=0;var<=2;var++)); 
    do 
    

        for ((i=0;i<=79;i++));
        do 
            roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=$i deviation:=$deviation seed:=$seed
        done


        



    done
done
done
done

