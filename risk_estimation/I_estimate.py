
import numpy as np
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
    
    return np.random.choice(["stop", "go"], p=(1-p_go, p_go))




def Ic_estimate(Ic, turns):
    P_same = 0.5
    nr_turns = len(turns)

    density = []
    for t in turns:
        if t == Ic:
            density.append(P_same)
        else:
            density.append((1 - P_same) / (nr_turns-1))

    return np.random.choice(turns, p=density)