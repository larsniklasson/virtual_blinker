import gap_models

import math
import numpy as np
import Intersection 
import utils

def Es_estimate(carid, particle, travelling_directions, intersection, most_likely_states):
    
    
    ego_Ic = particle.Ic
    ego_P = particle.P
    ego_S = particle.S
    ego_travelling_direction = travelling_directions[carid]
    ego_course = intersection.courses[ego_travelling_direction, ego_Ic]

    if not travelling_directions[carid] or ego_course.hasLeftIntersection(*ego_P):
        return "go"

    #combine directions with most likely states
    # filter out ego-vehicle
    blob = zip(travelling_directions, most_likely_states)
    blob = blob[:carid] + blob[carid+1:]

    ego_ttc = ego_course.getTimeToCrossing(ego_P[0], ego_P[1], ego_P[2], ego_S, "go")

    min_go = 1 #P(Es = go). Choose lowest probability out of other vehicles
    for td, st in blob:
        
        if not td: continue # has no direction => started inside or past intersection
        
        # ego-vehicle has right of way over this vehicle
        if intersection.hasRightOfWay(ego_travelling_direction, ego_Ic, td): continue 
        
        #course of other vehicle
        c = intersection.courses[td, st.Ic]
        if c.hasLeftIntersection(*st.P): continue
        
        ttc = c.getTimeToCrossing(*st.P, speed = st.S, Is = st.Is)
        gap = ttc - ego_ttc
        
        #gap model return P(Es=go) If lower than lowest so far, update min variable
        go = gap_models.gap_model(gap)
        if go < min_go:
            min_go = go

    return np.random.choice(["stop", "go"], p=(1-min_go, min_go))
    