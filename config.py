from utils.Intersection import Intersection
import numpy as np
import time
import random
import numpy as np

GEN_CONFIG = {
    "intersection": Intersection(),
    #plot,save,nr_cars done in sim.launch
}

def car(td, turn, sd, re, spd):
    return {"travelling_direction": td, 
            "turn": turn, 
            "starting_distance": sd, 
            "use_riskestimation": re,
            "speedDev": spd}


CARS = {
    0 : car("north", "left", 70, True, 0),
    1 : car("west", "right", 70, True, 0),
    2 : car("north", "right", 70, True, 0),
    3 : car("south", "left", 70, True, 0),
    4 : car("west", "right", 50, True, 0),
    5 : car("east", "right", 50, True, 0),
    6 : car("north", "straight", 50, True, 0),
    7 : car("south", "right", 50, True, 0),
    8 : car("west", "left", 30, True, 0),
    9 : car("east", "straight", 30, True, 0)
}

CARS_RANDOM = {
    0 : car("west", "straight", 70, True, 0),
    1 : car("east", "left", 70, True, 0),
    2 : car("north", "right", 70, True, 0),
    3 : car("south", "left", 70, True, 0),
    4 : car("west", "right", 50, True, 0),
    5 : car("east", "right", 50, True, 0),
    6 : car("north", "straight", 50, True, 0),
    7 : car("south", "right", 50, True, 0),
    8 : car("west", "left", 30, True, 0),
    9 : car("east", "straight", 30, True, 0)
}

def generateRandom():

    t = np.random.choice(["left", "right", "straight"], size=10, p = [0.4, 0.4, 0.2])
    #r = (np.random.random(10) - 0.5) * 20
    s = (np.random.random(10) -0.5) * 6
    b = np.random.random(10) <= 0.8
    print b

    for k,v in CARS_RANDOM.iteritems():
        if v["travelling_direction"] in ["south", "west"] and t[k] == "left":
            t[k] = np.random.choice(["right", "straight"])
        v["turn"] = t[k]
        #v["starting_distance"] += r[k]
        v["use_riskestimation"] = b[k]
        v["speedDev"] = s[k]

def getCarDict(random):
    if random > 0:
        generateRandom()
        return CARS_RANDOM
    else:
        return CARS
    

SIM_CONFIG = {
    "x_deviation" : 0.2,
    "y_deviation" : 0.2,
    "theta_deviation" : 0.04,
    "speed_deviation": 0.1,
    "slowdown": 1.0,
    "rate": 15,
    "pid" : (0.4, 0.0, 0.05),
    "lookahead": 5,
    "carlength": 4,
    "discard_measurement_time": 0.15, #seconds
    "Es_threshold": 0.8,
    "risk_threshold": 0.3,
    "save_id" : 1

}

RISK_CONFIG = {
    "grant_threshold": 0.9
}
