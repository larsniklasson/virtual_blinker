# a python class to contain intersection information
import math
import sys
sys.path.append("..")
from utils.course import *

from threading import Thread, Lock

class Intersection:

    def __init__(self):
        
        self.turns = ("left", "straight", "right")
        self.travelling_directions = ("north", "east", "south", "west")
        self.laneWidth = 7.5

        #mutex used when changing priorities of the intersection
        self.mutex = Lock()

        #dict of course-instances to lookup
        self.courses = {}
        for d in self.travelling_directions:
            for t in self.turns:
                self.courses[(d,t)] = Course(d,t)

        #to determine who has right of way TODO make this better looking
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
        
        #vehicles coming in these two directions are priority by default in this intersection.
        #unless changed by a maneuver negotiation protocol
        self.priority_directions = ["south","north"]

        self.relativePositions = {}
        for td1 in self.travelling_directions:
            for td2 in self.travelling_directions:
                if td1 != td2:
                    rp = self.getRelativePosition(td1, td2)
                    self.relativePositions[td1, td2] = rp



    #south-north is prio lane TODO store this some other place
    # TODO this will change when priority changes. so it should not be fixed
    def isOnPrioLane(self, travelling_direction):
        return travelling_direction in self.priority_directions

    # uses directions list above to determine whether or not other vehicle (second argument) is 
    # to the left of, to the right of or opposite the ego-vehicle (the first argument)
    def getRelativePosition(self, td1, td2):
        id1 = self.travelling_directions.index(td1)
        id2 = self.travelling_directions.index(td2)

        diff = id2 - id1

        #wrap around
        if diff == 3:
            diff = -1
        elif diff == -3:
            diff = 1

        if abs(diff) == 2:
            return "opposing"
        elif diff == 1:
            return "leftof"
        elif diff == -1:
            return "rightof"

    def hasRightOfWay(self, travelling_direction, turn, other_vehicle_td):
        #this case is unintresting for now
        if travelling_direction == other_vehicle_td:
            return True 

        #calculate relative position and lookup if ego-vehicle has priority
        #rel_pos_opposing = self.getRelativePosition(travelling_direction, other_vehicle_td)
        rel_pos_opposing = self.relativePositions[travelling_direction, other_vehicle_td]
        if self.isOnPrioLane(travelling_direction):
            return self.prioTable[turn][rel_pos_opposing]

        else:
            return self.nonPrioTable[turn][rel_pos_opposing]

    
    def getTravellingDirection(self, x, y, theta):
        if theta > math.pi:
            theta -= 2*math.pi
        if theta < -math.pi:
            theta += 2*math.pi

        if x > self.laneWidth and (theta >= 3*math.pi/4 or theta <= -3*math.pi/4):
            return "west"
        
        if x < -self.laneWidth and theta <= math.pi/4 and theta >= -math.pi/4:
            return "east"

        if y > self.laneWidth and theta <= -math.pi/4 and theta >= -3*math.pi/4:
            return "south"
        
        if y < -self.laneWidth and theta >= math.pi/4 and theta <= 3*math.pi/4:
            return "north"
        else:
            return None

    ## Return agents that the selected agent hasn't got right-of-way to 
    def getUnsafeAgents(self, agent_pose, agent_poses):

        unsafe_agents = {}
        agent_direction = self.getTravellingDirection(*agent_pose)

        # Check if any of the other agents have priority over the selected agents for the 3 possible turns
        for turn in self.turns:
            ids = []
            for id, pose in agent_poses:
                if not self.hasRightOfWay(agent_direction, turn, self.getTravellingDirection(*pose)):
                    ids.append(id)
            unsafe_agents[turn] = ids

        return unsafe_agents


