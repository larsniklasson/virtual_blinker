#!/bin/bash
seed=0


test_var=0

for ((var=0;var<3;var++))
do

for ((deviation=0;deviation<9;deviation++))
do



danger=0
starting_dist=0
roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=$starting_dist deviation:=$deviation seed:=$seed danger:=$danger
starting_dist=9
roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=$starting_dist deviation:=$deviation seed:=$seed danger:=$danger

danger=1
starting_dist=4
roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=$starting_dist deviation:=$deviation seed:=$seed danger:=$danger
starting_dist=5
roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=$starting_dist deviation:=$deviation seed:=$seed danger:=$danger

danger=2
starting_dist=4
roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=$starting_dist deviation:=$deviation seed:=$seed danger:=$danger
starting_dist=5
roslaunch virtual_blinker sim.launch test_var:=$test_var variation:=$var starting_dist:=$starting_dist deviation:=$deviation seed:=$seed danger:=$danger





#for ((danger=0;danger<3;danger++))

#do
#for ((starting_dist=0;starting_dist<10;starting_dist++))
#do



#done
#done
done
done