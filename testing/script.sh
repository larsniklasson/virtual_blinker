#!/bin/bash


seed=0

for ((deviation=0;deviation<9;deviation++)); 

do

    for ((danger=0;danger<3;danger++)); 

    do

        for ((test_var=0;test_var<4;test_var++)); 
        do

            for ((var=0;var<3;var++)); 
            do
                roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=0 deviation:=$deviation seed:=$seed danger:=$danger
                roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=3 deviation:=$deviation seed:=$seed danger:=$danger
                roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=6 deviation:=$deviation seed:=$seed danger:=$danger
                roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=9 deviation:=$deviation seed:=$seed danger:=$danger
            done
        done
    done
done
