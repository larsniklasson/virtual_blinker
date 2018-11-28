#!/bin/bash


for ((seed=0;seed<10;seed++));
do
    for ((test_var=0;test_var<4;test_var++)); 
    do
        for ((var=0;var<3;var++)); 
        do
            for ((deviation=0;deviation<9;deviation++)); 
            do
                for ((danger=0;danger<3;danger++)); 
                do
                    for ((i=0;i<10;i++));
                    do
                        roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=$i deviation:=$deviation seed:=$seed danger:=$danger
                    done
                done
            done

        done
    done
done
