from utils.intersection import Intersection
import numpy as np
import time
import random
import numpy as np

class Car:
    def __init__(self, travelling_direction, turn, 
                 starting_distance, is_good_behaving, speed_deviation):
        
        #the (original) direction the vehicle is (was) travelling
        self.travelling_direction = travelling_direction

        self.turn = turn
        self.starting_distance = starting_distance

        # follows expectation, uses emergency break and sends blinker
        self.is_good_behaving = is_good_behaving
        
        # constant deviation from speed profile.
        self.speed_deviation = speed_deviation


#Non-random
CAR_DICT = {
    0 : Car("south", "straight", 90, True, 0),
    1 : Car("south", "straight", 70, True, 0),
    2 : Car("south", "straight", 50, True, 0),
    3 : Car("south", "straight", 30, True, 0),
    4 : Car("south", "straight", 10, True, 0),
    5 : Car("south", "straight", 80, True, 0),
    6 : Car("south", "straight", 20, True, 0),
    7 : Car("east", "straight", 90, True, 0),
    8 : Car("east", "straight", 70, True, 0),
    9 : Car("east", "straight", 50, True, 0),
    10 : Car("west", "straight", 50, True, 0),
    11 : Car("east", "straight", 50, True, 0),
    12 : Car("west", "straight", 0, True, 0),
    13 : Car("east", "straight", 0, True, 0),
    14 : Car("south", "straight", 0, True, 0),
    15 : Car("north", "straight", 0, True, 0),
    16 : Car("west", "straight", 30, True, 0),
    17 : Car("east", "straight", 30, True, 0),
    18 : Car("south", "straight", 30, True, 0),
    19 : Car("north", "straight", 30, True, 0)
}

#generate a semi-random car dict using the seed given in launch file
#travelling direction and starting distance fixed
def generateRandom():

    CARS_DICT_RANDOM = {
        0 : Car("west", None, 70, None, None),
        1 : Car("east", None, 70, True, None),
        2 : Car("north", None, 70, True, None),
        3 : Car("south", None, 70, True, None),
        4 : Car("west", None, 50, True, None),
        5 : Car("east", None, 50, True, None),
        6 : Car("north", None, 50, True, None),
        7 : Car("south", None, 50, True, None),
        8 : Car("west", None, 30, True, None),
        9 : Car("east", None, 30, True, None)
    }

    turns = np.random.choice(["left", "right", "straight"], size=10, p = [0.4, 0.4, 0.2])
    speed_deviations = (np.random.random(10) -0.5) * 6
    is_good_behaving_bools = np.random.random(10) <= 0.8
    print is_good_behaving_bools

    for id, car in CARS_DICT_RANDOM.iteritems():

        #avoid deadlocks
        if car.travelling_direction in ["south", "west"] and turns[id] == "left":
            turns[id] = np.random.choice(["right", "straight"])

        car.turn = turns[id]
        car.is_good_behaving = is_good_behaving_bools[id]
        car.speed_deviation = speed_deviations[id]

    #each car will use the same seed so this dict will be the same for every car
    return CARS_DICT_RANDOM

def getCarDict(is_random):
    if is_random:
        return generateRandom()
    else:
        return CAR_DICT
    

intersection = Intersection()

x_deviation = 0.2
y_deviation = 0.2
theta_deviation = 0.04
speed_deviation = 0.1

slowdown = 1.0
rate = 15 #iterations per second for simulation => rate = msgs sent per second
discard_measurement_time = 0.15

pid = 0.4, 0.0, 0.05
lookahead = 5
carlength = 4

Es_threshold = 0.8   #determines if well behaved vehicles go or stop
risk_threshold = 0.3 #Break if higher
grant_threshold = 0.9 #grant if P(Es=go) is greater than threshold

save_id = 1 #for debugging

gap_lower_limit = -1.3
gap_upper_limit = 3

risk_gap_lower_limit = -1#-0.5
risk_gap_upper_limit = 2

error_weights = [125, 125, 125, 1]