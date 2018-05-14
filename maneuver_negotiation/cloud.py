## The Cloud
## Gets data from all vehicles and broadcast their membership sets? Or upon request?

import rospy
import custom_msgs.msg as cm
import zookeeper
import numpy as np


priorityMatrix = np.array([[1,0,0],[0,0,0],[0,0,0]])
nonPriorityMatrix = np.array([[1,1,1],[0,1,1],[0,0,1]])

TM = 1 #SM update period, TODO Take simulation speed into account (SLOWDOWN)
TD = 1 #TODO get the common values
TMan = 1

## Doesn't have to be reliable? 
## It's now reliable due to zookeepers use of TCP, but could be made unreliable if the cloud started to listen to ros topics instead
class MembershipCloud:

    nbr_agents = 4 #TODO Get from launch file: rospy.get_param('nr_cars')

    agent_registry = {}

    def __init__(self):

        rospy.init_node('cloud',anonymous=True)

        #Use zookeeper storage instead
        self.handle = zookeeper.init('127.0.0.1:2181')#(maneuver_negotiator_config.GENERAL_OPTIONS['zookeeper-server'])
        
        #Initiate all agents in the local version of the AR set
        self.getARset()


    
    ## Store AR locally TODO Take segments into account
    ## We could also ask for the children instead of looping through 1 to nmr_agents
    def getARset(self):
        for i in range(1, self.nbr_agents + 1):
            (data, stat) = zookeeper.get(self.handle, "/root/segment/" + str(i), True)
            self.agent_registry[i] = data


    ## Return Agents that are within communication distance from the Agent with id aID until time t_end
    def getReachableAgents(self, aID, t_end):
        R = []
        for agent in self.agent_registry.keys():
            if self.isReachable(aID, agent):
                R.append(agent)
        return R

    ## Return True if agent a is within communication distance of b TODO To be implemented
    def isReachable(self, aID, bID):
        #ARa = self.agent_registry[aID]
        #ARb = self.agent_registry[bID]
        return True
    
    ## Return Agents that may cross the path of the Agent with id aID any time from now until t_end
    ## Based on a right-of-way matrix
    def getUnsafeAgents(self, aID, t_end):
        return []

    ## Update the Safety Membership (SM) of Agent with id aID
    ## The agent will have Maneuvre Oppertunity (MO) if all unsafe agents are reachable
    def updateAgentSM(self, aID):
        t_stamp = rospy.get_rostime()
        U = self.getUnsafeAgents(aID, t_stamp + TM + 2*TD + TMan)
        R = self.getReachableAgents(aID, t_stamp + TM + 2*TD + TMan)
        MO = True
        SM = U

        for agent in U:
            if not (agent in R):
                MO = False
                SM = []
        MR = [t_stamp, MO, SM]
        zookeeper.set(self.handle, "/root/mr/" + str(aID), str(MR))
        

    ## Periodically calculate the Safety Membership (SM) for every agent in the Agent Registry
    def calcSM(self):

        rate = rospy.Rate(1/TM)
    
        while not rospy.is_shutdown():
            rate.sleep()

            #Read ARset and store locally
            self.getARset()

            for agent_id in self.agent_registry.keys():
                self.updateAgentSM(agent_id)

    

if __name__ == '__main__':
    mc = MembershipCloud()
    mc.calcSM()


#Deprecated

# Subscribe to all car topics
    #for i in range(1, self.nbr_agents + 1):
    #    rospy.Subscriber("/car_state" + str(i), cm.CarState, self.stateCallback, queue_size=10)

## Update Agent Registry
#def stateCallback(self, msg):
#    pass