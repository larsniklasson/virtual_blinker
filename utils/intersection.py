# a python class to contain intersection information
import math
import sys
sys.path.append("..")
from utils.course import *

class Intersection:

    def __init__(self):

        #how long you should project into the intersection to intersect with
        # other courses. The left turn can intersect with other courses on (roughly) three different
        # places
        self.extra_distances = [3.25, 11.75, pi*11.75/4.0, pi*11.75/2.0 - 3.25]
        
        #refer to indices in array above
        self.turn_extras = {"left": [0, 2, 3], 
                            "right": [0], 
                            "straight": [0, 1]}

        
        self.turns = ("left", "straight", "right")
        self.travelling_directions = ("north", "east", "south", "west")
        self.laneWidth = 7.5

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

    def hasRightOfWay(self, ego_td, ego_turn, other_vehicle_td):
        #this case is uninteresting for now
        if ego_td == other_vehicle_td:
            return True 

        #calculate relative position and lookup if ego-vehicle has priority
        rel_pos = self.getRelativePosition(ego_td, other_vehicle_td)
        
        if self.isOnPrioLane(ego_td):
            return self.prioTable[ego_turn][rel_pos]
        else:
            return self.nonPrioTable[ego_turn][rel_pos]

    # for 2 courses, returns distance from edge of intersection to the intersection point
    # distance is approximate to not have too many cases
    def getExtras(self, ego_td, ego_turn, other_td, other_turn):
        relPos = self.getRelativePosition(ego_td, other_td)
        ret = None
        flip = False

        if relPos in ["leftof", "rightof"]:

            # cases are symmetrical
            if relPos == "rightof":
                tmp = ego_turn
                ego_turn = other_turn
                other_turn = tmp
                flip = True

            if ego_turn == "left":
                if other_turn == "left":
                    ret = 2, 2
                elif other_turn == "straight":
                    ret = 0, 1
            elif ego_turn == "straight":
                if other_turn == "left":
                    ret = 1, 3
                elif other_turn == "straight":
                    ret = 0, 1
            elif ego_turn == "right":
                if other_turn == "straight":
                    ret = 0, 1
        
        if relPos == "opposing":
            if ego_turn == "left":
                if other_turn in ["right", "straight"]:
                    ret = 3, 0
                elif other_turn == "left":
                    ret = 2, 2
            elif ego_turn == "right":
                if other_turn == "left":
                    ret = 0, 3
            elif ego_turn == "straight":
                if other_turn == "left":
                    ret = 0, 3

        if flip:
            ret = (ret[1], ret[0])
        
        return ret
    
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

    #given two courses, do they intersect?
    def doesCoursesIntersect(self, td1, turn1, td2, turn2):
        if td1 == td2:
            return False

        relPos = self.getRelativePosition(td1, td2)
        if relPos in ["leftof", "rightof"]:
            
            if relPos == "rightof":
                tmp = turn1
                turn1 = turn2
                turn2 = tmp

            return turn2 == "straight" or turn2 == "left" and turn1 != "right"

        if relPos == "opposing":
            return turn1 == "left" or turn2 == "left"

                   
    def doesCoursesOverlap(self, td1, turn1, td2, turn2):
        if td1 == td2:
            return True
        relPos = self.getRelativePosition(td1, td2)
        if relPos in ["leftof", "rightof"]:

            if relPos == "rightof":
                tmp = turn1
                turn1 = turn2
                turn2 = tmp

            return turn1 == "straight" and turn2 == "left" or \
               turn1 == "right" and turn2 == "straight"

        elif relPos == "opposing":
            return turn1 == "right" and turn2 == "left" or \
                turn2 == "right" and turn1 == "left"

    """
    ## Return agents that the selected agent hasn't got right-of-way to 
    def getUnsafeAgents(self, agent_pose, agent_poses):

        unsafe_agents = {}
        agent_direction = self.getTravellingDirection(agent_pose[0], agent_pose[1], agent_pose[2])

        # Check if any of the other agents have priority over the selected agents for the 3 possible turns
        for turn in self.turns:
            ids = []
            for id, pose in agent_poses:
                if not self.hasRightOfWay(agent_direction, turn, self.getTravellingDirection(pose[0], pose[1], pose[2])):
                    ids.append(id)
            unsafe_agents[turn] = ids

        return 
    """
        