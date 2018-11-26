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




def getCarDict(test_var, variation, starting_distance, deviation, danger):
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



    #danger = collision, semi-dangerous, non-dangerous. (0,1,2)

    sd1 = -20
    #sd2 = starting_distance-20

    
    if variation == 0:

        if deviation == 2:

            if danger == 0:
                a = 10
                b = 19

            if danger == 1:
                a = -10
                b = 0
                a2 = 30
                b2 = 45

            if danger == 2:
                a = -20
                b = -11
                a2 = 46
                b2 = 65


        elif deviation == 3:

            if danger == 0:
                a = -9
                b = 7

            if danger == 1:
                a = -25
                b = -18
                a2 = 17
                b2 = 30

            if danger == 2:
                a = -35
                b = -26
                a2 = 31
                b2 = 50

        elif deviation == 4:
            if danger == 0:
                a = 57
                b = 67

            if danger == 1:
                a = 37
                b = 44
                a2 = 77
                b2 = 92

            if danger == 2:
                a = 27
                b = 36
                a2 = 93
                b2 = 110
            
        else:
            if danger == 0:
                a = 17
                b = 33

            if danger == 1:
                a = 0
                b = 10
                a2 = 45
                b2 = 57

            if danger == 2:
                a = -10
                b = -1
                a2 = 58
                b2 = 74
        

    if variation == 1:
        if deviation == 2:

            if danger == 0:
                a = -8
                b = -2

            if danger == 1:
                a = -30
                b = -20
                a2 = 8
                b2 = 22

            if danger == 2:
                a = -40
                b = -31
                a2 = 23
                b2 = 36

        elif deviation == 3:

            if danger == 0:
                a = -25
                b = -19

            if danger == 1:
                a = -46
                b = -37
                a2 = -10
                b2 = 2

            if danger == 2:
                a = -55
                b = -47
                a2 = 3
                b2 = 17

        elif deviation == 4:
            if danger == 0:
                a = 7
                b = 17

            if danger == 1:
                a = -13
                b = -3
                a2 = 25
                b2 = 36

            if danger == 2:
                a = -25
                b = -14
                a2 = 37
                b2 = 49
            
        else:
            if danger == 0:
                a = -12
                b = 0

            if danger == 1:
                a = -18
                b = 27
                a2 = 10
                b2 = 20

            if danger == 2:
                a = -38
                b = -28
                a2 = 21
                b2 = 35

    if variation == 2:

        if deviation == 2:

            if danger == 0:
                a = -4
                b = 4

            if danger == 1:
                a = -23
                b = -13
                a2 = 14
                b2 = 26

            if danger == 2:
                a = -34
                b = -24
                a2 = 27
                b2 = 41

        elif deviation == 3:

            if danger == 0:
                a = -25
                b = -14
                

            if danger == 1:
                a = -46
                b = -34
                a2 = -2
                b2 = 8

            if danger == 2:
                a = -57
                b = -47
                a2 = 9
                b2 = 22

        elif deviation == 4:
            if danger == 0:
                a = 29
                b = 40

            if danger == 1:
                a = 13
                b = 22
                a2 = 61
                b2 = 67

            if danger == 2:
                a = 2
                b = 12
                a2 = 68
                b2 = 79
            
        else:
            if danger == 0:
                a = 1
                b = 10

            if danger == 1:
                a = -20
                b = -10
                a2 = 20
                b2 = 34

            if danger == 2:
                a = -30
                b = -21
                a2 = 35
                b2 = 50
    
    if danger == 0:
        sd2 = a + starting_distance * (b-a)/9.0
    else:
        if starting_distance >= 5:
            starting_distance -= 5
            sd2 = a2 + starting_distance * (b2-a2)/4.0
        else:
            sd2 = a + starting_distance * (b-a)/4.0

    
    sd2 -= 20

    

    

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

    #lose com 1
    if deviation == 5:
        
        c1 = Car(td1, turn1, sd1, True, True, 1, 0, 0)
        c2 = Car(td2, turn2, sd2, True, True, 1, 0, 0)
    
    #lose com 2
    if deviation == 6:
        
        c1 = Car(td1, turn1, sd1, True, True, 2, 0, 0)
        c2 = Car(td2, turn2, sd2, True, True, 2, 0, 0)

    #lose com 3
    if deviation == 7:
        
        c1 = Car(td1, turn1, sd1, True, True, 3, 0, 0)
        c2 = Car(td2, turn2, sd2, True, True, 3, 0, 0)

    #noisy
    if deviation == 8:
        
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

deviations_high = (0.2*5, 0.2*5, 0.04*5, 0.1*5)
deviations_low = (0.2, 0.2, 0.04, 0.1)

speedup = 4.0

slowdown = 1 / speedup
rate = 60.0 * speedup #iterations per second for simulation => rate = msgs sent per second

pid = 0.4, 0.0, 0.01
lookahead = 5
carlength = 2.8

risk_thresholds = [0.3, 0.55, 0.55]
#risk_threshold = 0.4 #Break if higher
grant_threshold = 0.8 #grant if P(Es=go) is greater than threshold

gap_lower_limit = -1.5
gap_upper_limit = 2.5

risk_gap_lower_limit = -1.0#-0.5
risk_gap_upper_limit = 1.5


max_transmission_delay = 0.2 / speedup

error_weights = [125, 125, 125, 1]