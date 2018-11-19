from utils.intersection import Intersection
import numpy as np
import time
import random
import numpy as np

class Car:
    def __init__(self, travelling_direction, turn, 
                 starting_distance, use_eb, use_mn, lose_com, speed_dev, noisy):
        
        #the (original) direction the vehicle is (was) travelling
        self.travelling_direction = travelling_direction

        self.turn = turn
        self.starting_distance = starting_distance

        self.use_eb = use_eb
        self.use_mn = use_mn

        self.lose_com = lose_com
        self.speed_dev = speed_dev

        self.noisy = noisy



def getCarDict(test_var, variation, starting_distance, deviation):
    td1 = "north"
    turn1 = "straight"

    if variation == 0:
        td2 = "south"
        turn2 = "left"
        
    if variation == 1:
        td2 = "west"
        turn2 = "straight"
    if variation == 2:
        td2 = "west"
        turn2 = "right"


    nr_dist = 70.0

    if variation == 0:

        if deviation == 3:
            sd1 = 40
            sd2 = 15+70 * starting_distance/nr_dist

        elif deviation == 4:
            sd1 = 10
            sd2 = 20+70 * starting_distance/nr_dist
            
        else:
            sd1 = 20
            sd2 = 15 + 70 * starting_distance/nr_dist
    

    if variation == 1:
        if deviation == 3:
            sd1 = 55
            sd2 = 10 + 60 * starting_distance / nr_dist
        elif deviation == 4:
            sd1 = 35
            sd2 = 15+62 * starting_distance / nr_dist
        else:
            sd1 = 30
            sd2 = 5+55 * starting_distance/nr_dist

    if variation == 2:

        if deviation == 3:
            sd1 = 55
            sd2 = 15+60* starting_distance/nr_dist
        elif deviation == 4:
            sd1 = 35
            sd2 = 30+60 *starting_distance/nr_dist
        else:
            sd1 = 30
            sd2 = 15 + 62 * starting_distance/nr_dist


    #normal
    if deviation == 0:

        c1 = Car(td1, turn1, sd1, True, True, False, 0, 0)
        c2 = Car(td2, turn2, sd2, True, True, False, 0, 0)
    
    #malicious
    if deviation == 1:
        
        c1 = Car(td1, turn1, sd1, True, True, False, 0, 0)
        c2 = Car(td2, turn2, sd2, False, False, False, 0, 0)

    #both fast
    if deviation == 2:
        
        c1 = Car(td1, turn1, sd1, True, True, False, 1, 0)
        c2 = Car(td2, turn2, sd2, True, True, False, 1, 0)

    #green fast, blue slow
    if deviation == 3:
        
        c1 = Car(td1, turn1, sd1, True, True, False, 2, 0)
        c2 = Car(td2, turn2, sd2, True, True, False, 1, 0)

    #green slow, blue fast
    if deviation == 4:
        
        c1 = Car(td1, turn1, sd1, True, True, False, 1, 0)
        c2 = Car(td2, turn2, sd2, True, True, False, 2, 0)

    #lose com
    if deviation == 5:
        
        c1 = Car(td1, turn1, sd1, True, True, True, 0, 0)
        c2 = Car(td2, turn2, sd2, True, True, True, 0, 0)

    #noisy
    if deviation == 6:
        
        c1 = Car(td1, turn1, sd1, True, True, False, 0, 1)
        c2 = Car(td2, turn2, sd2, True, True, False, 0, 1)


    #test risk assessment
    if test_var == 0:
        c1.use_eb = False
        c1.use_mn = False
        c2.use_eb = False
        c2.use_mn = False

    if test_var == 1:
        c1.use_mn = False
        c2.use_mn = False

    if test_var == 2:
        c1.use_eb = False
        c2.use_eb = False


    return {0:c1, 1:c2}




intersection = Intersection()

deviations_high = (1, 1, 0.2, 0.5)
deviations_low = (0.2, 0.2, 0.04, 0.1)



slowdown = 1.0
rate = 30 #iterations per second for simulation => rate = msgs sent per second
discard_measurement_time = 0.15

pid = 0.4, 0.0, 0.01
lookahead = 5
carlength = 2.8

risk_threshold = 0.3 #Break if higher
grant_threshold = 0.9 #grant if P(Es=go) is greater than threshold

gap_lower_limit = -1.5
gap_upper_limit = 3

risk_gap_lower_limit = -1#-0.5
risk_gap_upper_limit = 2


max_transmission_delay = 0.2