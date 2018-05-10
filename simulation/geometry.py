from math import *

class Point:
    def __init__(self, x , y):
        self.x = x
        self.y = y
        
    def __repr__(self):
        return "(" + str(self.x) + " " + str(self.y) + ")"



def getDirection((x1,y1), (x2,y2)):
    return atan2(y2-y1, x2-x1)

def getLookaheadPoint((x,y), direction, lookahead):
    return (x + lookahead * cos(direction), y + lookahead * sin(direction))


def getDistanceFromLine(p, (l1, l2)):
    return abs((l2.x - l1.x) * (l1.y-p.y) - (l1.x-p.x) * (l2.y-l1.y)) / (sqrt((l2.x-l1.x) * (l2.x-l1.x) + (l2.y-l1.y) * (l2.y-l1.y)))


def is_left_of_line(p, (l1, l2)):
    return ((l2.x - l1.x) * (p.y - l1.y) - (l2.y - l1.y) * (p.x - l1.x)) <= 0


def getDistanceBetweenPoints((x1,y1),(x2,y2)):
    dx = x2-x1
    dy = y2-y1
    return sqrt(dx**2 + dy**2)

def hasPassedLine(p, (l1, l2)):
    
    if l1.x - l2.x !=0 and l1.y - l2.y !=0:
        slope = float(l1.y - l2.y) / float(l1.x - l2.x)
        prependularSlope = (-1)/slope
        prependularM = l2.y - l2.x*prependularSlope
        
        if l1.y < l2.y:
            #up
            return (p.x*prependularSlope + prependularM - p.y) < 0
        else:
            #down
            return (p.x*prependularSlope + prependularM - p.y) > 0
    
    elif l1.x - l2.x:
        #straight in x direction
        if l1.x < l2.x:
            #right
            return p.x > l2.x
        else:
            #left
            return p.x < l2.x
    
    else:
        #straight in y direction
        if l1.y < l2.y:
            #up
            return p.y > l2.y
        else:
            #down
            return p.y < l2.y
