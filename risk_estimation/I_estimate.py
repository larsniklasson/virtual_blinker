
import sys
sys.path.append("..")
from config import *

import numpy as np
import random

P_comply = RISK_CONFIG["Is_comply"]
P_same = RISK_CONFIG["Ic_same"]

def Is_estimate(Is_tminus1, Es_estimate):

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
    nr_turns = len(turns)

    density = []
    for t in turns:
        if t == Ic:
            density.append(P_same)
        else:
            density.append((1 - P_same) / (nr_turns-1))

    return choice(turns, density)


def foo((_,b)):
    return -b

def choice(turns, density, n=1):
    rr = []
    for _ in range(n):
        s = sorted(zip(turns, density), key = foo)
        r = random.random()
        dsum = 0
        for t,d in s:
            if r <= dsum + d:
                rr.append(t)
                break
            else:
                dsum += d

    if len(rr) == 1:
        return rr[0]
    else:
        return rr
                