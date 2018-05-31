import gap_models

import math
import numpy as np
import Intersection 
import utils

def Es_estimate(carid, ego_Ic, ego_P, ego_S, travelling_directions, intersection, most_likely_states):
    

    ego_travelling_direction = travelling_directions[carid]
    ego_course = intersection.courses[ego_travelling_direction, ego_Ic]

    if not travelling_directions[carid] or ego_course.hasLeftIntersection(*ego_P):
        return "go",1

    #combine directions with most likely states
    # filter out ego-vehicle
    blob = zip(travelling_directions, most_likely_states)
    blob = blob[:carid] + blob[carid+1:]

    ego_ttc = ego_course.getTimeToCrossing(ego_P[0], ego_P[1], ego_P[2], ego_S, "go")

    min_go_vehicles = 1 #P(Es = go). Choose lowest probability out of other vehicles
    for td, st in blob:
        
        if not td: continue # has no direction => started inside or past intersection
        
        # ego-vehicle has right of way over this vehicle
        if intersection.hasRightOfWay(ego_travelling_direction, ego_Ic, td): continue 
        
        go_turns = {} #P(Es=go) for the different turns
        for turn in intersection.turns:
            #course of other vehicle
            c = intersection.courses[td, turn]
            if c.hasLeftIntersection(*st[1]): 
                go_turns[turn] = 1.0
                continue
            
            ttc = c.getTimeToCrossing(*st[1], speed = st[2], Is="go")
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
    return np.random.choice(["stop", "go"], p=(1-g, g)), g
    