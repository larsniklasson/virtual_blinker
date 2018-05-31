
import numpy as np
import random
def Is_estimate(Is_tminus1, Es_estimate):
    P_comply = 0.75
    

    if (Is_tminus1 == "go" and Es_estimate == "go"):
        p_go = P_comply
    elif (Is_tminus1 == "go" and Es_estimate == "stop"):
        p_go = 0.5
    elif (Is_tminus1 =="stop" and Es_estimate == "go"):
        p_go = 0.5
    elif (Is_tminus1 == "stop" and Es_estimate =="stop"):
        p_go = 1 - P_comply
    
    if random.random() <= p_go:
        return "go"
    else:
        return "stop"



def Ic_estimate(Ic, turns):
    P_same = 0.5
    nr_turns = len(turns)

    density = []
    for t in turns:
        if t == Ic:
            density.append(P_same)
        else:
            density.append((1 - P_same) / (nr_turns-1))

    return choice(turns, density)


def choice(turns, density):
    s = sorted(zip(turns, density), key = lambda (_,d):-d)
    r = random.random()
    dsum = 0
    for t,d in s:
        if r <= dsum + d:
            return t
        else:
            dsum += d