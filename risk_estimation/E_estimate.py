import gap_models

import math
import numpy as np 
import utils
import random

def Es_estimate(carid, ego_Ic, ego_PS, travelling_directions, intersection, most_likely_states, ttc_hli, grantlist, flag=False):
    

    ego_travelling_direction = travelling_directions[carid]
    ego_course = intersection.courses[ego_travelling_direction, ego_Ic]


    if not travelling_directions[carid] or ego_course.hasLeftIntersection(*ego_PS[:3]) or carid in grantlist:
        return "go",1.0

    #combine directions with most likely states
    # filter out ego-vehicle
    blob = zip(travelling_directions, most_likely_states, ttc_hli, range(len(most_likely_states)))
    blob = blob[:carid] + blob[carid+1:]

    ego_ttc = ego_course.getTimeToCrossing(*ego_PS, Is="go")

    min_go_vehicles = 1 #P(Es = go). Choose lowest probability out of other vehicles
    for td, st, th, i in blob:
        
        if not td: continue # has no direction => started inside or past intersection
        
            
        if flag and carid == 1:
            print i, ego_travelling_direction, ego_Ic, td

        # ego-vehicle has right of way over this vehicle
        if intersection.hasRightOfWay(ego_travelling_direction, ego_Ic, td) and not (i in grantlist):
            continue 
        
        go_turns = {} #P(Es=go) for the different turns
        for turn in intersection.turns:


            hasLeft, ttc = th[turn]
            #course of other vehicle
            c = intersection.courses[td, turn]
            if hasLeft: 
                go_turns[turn] = 1.0
                continue
            
            gap = ttc - ego_ttc
            
            #gap model return P(Es=go) If lower than lowest so far, update min variable
            go = gap_models.gap_model(gap)
            go_turns[turn] = go


        es_go = 0 #P(Es=go) weighted over all turns
        Ic_density = st[0]
        for turn, v in go_turns.iteritems():
            es_go += v * Ic_density[turn]


        if es_go < min_go_vehicles:
            min_go_vehicles = es_go

    g = min_go_vehicles
    if random.random() <= g:
        r = "go"
    else:
        r = "stop"

    return r, g
    