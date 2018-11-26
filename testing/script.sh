#!/bin/bash


seed=0

for ((deviation=8;deviation<9;deviation++)); 

do

    for ((danger=0;danger<1;danger++)); 

    do

        for ((test_var=3;test_var<4;test_var++)); 
        do

            for ((seed=0;seed<4;seed++));
            do

            for ((var=0;var<3;var++)); 
            do
#               roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=0 deviation:=$deviation seed:=$seed danger:=$danger
                roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=4 deviation:=$deviation seed:=$seed danger:=$danger
                roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=5 deviation:=$deviation seed:=$seed danger:=$danger
                roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=6 deviation:=$deviation seed:=$seed danger:=$danger
#               roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=9 deviation:=$deviation seed:=$seed danger:=$danger

                #for ((i=0;i<10;i++));
                #do
                #    sleep .1
                #    roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=$i deviation:=$deviation seed:=$seed danger:=$danger
                #done
            done
            done
        done
    done
done
