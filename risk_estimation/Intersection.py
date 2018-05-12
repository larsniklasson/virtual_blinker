# a python class to contain intersection information
import math
import sys
sys.path.append("..")
from utils.course import *

class Intersection:

    def __init__(self):
        
        self.turns = ("left", "straight", "right")
        self.travelling_directions = ("north", "east", "south", "west")

        self.courses = {}
        for d in self.travelling_directions:
            for t in self.turns:
                self.courses[(d,t)] = Course(d,t)

        self.prioTable = {"left": 
                            {"opposing": False, 
                            "rightof":True, 
                            "leftof": True},
                         "straight": 
                            {"opposing": True, 
                            "rightof":True, 
                            "leftof": True},
                         "right": 
                            {"opposing": True, 
                            "rightof":True, 
                            "leftof": True}}

        self.nonPrioTable = {"left": 
                                {"opposing": False, 
                                "rightof":False, 
                                "leftof": False},
                            "straight": 
                                {"opposing": True, 
                                "rightof":False, 
                                "leftof": False},
                            "right": 
                                {"opposing": True, 
                                "rightof":True, 
                                "leftof": False}}


    
    def isOnPrioLane(self, travelling_direction):
        return travelling_direction == "south" or travelling_direction == "north"

    def getRelativePosition(self, td1, td2):
        id1 = self.travelling_directions.index(td1)
        id2 = self.travelling_directions.index(td2)

        diff = id2 - id1

        if diff == 3:
            diff = -1
        elif diff == -3:
            diff = 1

        if abs(diff) == 2:
            return "opposing"
        elif diff == -1:
            return "leftof"
        elif diff == 1:
            return "rightof"

    def hasRightOfWay(self, travelling_direction, turn, other_vehicle_td):
        if travelling_direction == other_vehicle_td:
            return True

        rel_pos_opposing = self.getRelativePosition(travelling_direction, other_vehicle_td)
        if self.isOnPrioLane(travelling_direction):
            return self.prioTable[turn][rel_pos_opposing]

        else:
            return self.nonPrioTable[turn][rel_pos_opposing]


    def getTravellingDirection(self, x, y, theta):
        if theta > pi:
            theta -= 2*pi
        if theta < -pi:
            theta += 2*pi

        if x > 7.5 and (theta >= 3*math.pi/4 or theta <= -3*math.pi/4):
            return "west"
        
        if x < -7.5 and theta <= math.pi/4 and theta >= -math.pi/4:
            return "east"

        if y > 7.5 and theta <= -math.pi/4 and theta >= -3*math.pi/4:
            return "south"
        
        if y < -7.5 and theta >= math.pi/4 and theta <= 3*math.pi/4:
            return "north"
        else:
            return None

