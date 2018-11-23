#!/bin/bash


deviation=5;


for ((danger=0;danger<=2;danger++)); 

do

for ((test_var=2;test_var<=3;test_var++)); 
do

for ((var=0;var<=2;var++)); 
do

for ((seed=0;seed<=0;seed++)); 
do
    
        
            for ((i=0;i<10;i+=4));
            do 
                roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=$i deviation:=$deviation seed:=$seed danger:=$danger
            done



done
done
done
done