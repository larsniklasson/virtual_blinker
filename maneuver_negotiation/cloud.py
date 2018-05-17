#!/usr/bin/env python 
## The Cloud
## Gets data from all vehicles and broadcast their membership sets? Or upon request?

import rospy
#import custom_msgs.msg as cm
import zookeeper
import numpy as np
import communication_config
import sys
sys.path.append("..")
import maneuver_negotiation.maneuver_negotiator_config as config
import risk_estimation.Intersection
import virtual_blinker.msg as cm



#priorityMatrix = np.array([[1,0,0],[0,0,0],[0,0,0]])
#nonPriorityMatrix = np.array([[1,1,1],[0,1,1],[0,0,1]])

TM = communication_config.GENERAL_OPTIONS['TM'] #SM's update period
TD = communication_config.GENERAL_OPTIONS['TD'] 
TMan = communication_config.GENERAL_OPTIONS['TMan']

r_com = 200 #200 m

## Doesn't have to be reliable? 
## It's now reliable due to zookeepers use of TCP, but could be made unreliable if the cloud started to listen to ros topics instead
class MembershipCloud:

    def __init__(self, intersection):

        rospy.init_node('cloud',anonymous=True)

        self.nbr_agents = 4 #TODO Get from launch file: rospy.get_param('nr_cars')

        self.agent_registry = {}

        #Use zookeeper storage instead
        self.handle = zookeeper.init(communication_config.NETWORK_OPTIONS['zookeeper-server'])
        
        #Initiate all agents in the local version of the AR set
        self.getARset()

        self.intersection = intersection

        self.ros_measurements = None
        #subscribe to simulation nodes to get the time:
        self.car_state_subscriber_handle = rospy.Subscriber(config.ROS_COMMUNICATION_OPTIONS['car-state-topic'][1],cm.CarState,self.update_time)
        #self.car_state_subscriber_handle = rospy.Subscriber(maneuver_negotiation.maneuver_negotiator_config.ROS_COMMUNICATION_OPTIONS  

    def get_time(self):
        if (self.ros_measurements is None):
            return 0
        else:
            return self.ros_measurements.t

    def update_time(self,data):
        #print("updating")
        self.ros_measurements = data
    
    ## Store AR locally TODO Take segments into account
    ## If multiple threads used this has to be changed
    def getARset(self):

        children = zookeeper.get_children(self.handle, "/root/segment", True)
        #for i in range(1, self.nbr_agents + 1):
        for child in children:
            (data, stat) = zookeeper.get(self.handle, "/root/segment/" + str(child), True)
            self.agent_registry[child] = data


    ## Return Agents that are within communication distance from the Agent with id aID until time t_end
    def getReachableAgents(self, aID, t_end):
        R = []
        for agent in self.agent_registry.keys():
            if self.isReachable(aID, agent):
                R.append(agent)
        return R

    ## Return True if agent a is within communication distance of b
    def isReachable(self, aID, bID):
        a_p = eval(self.agent_registry[aID][1])
        b_p = eval(self.agent_registry[bID][1])

        # If within communication distance
        if (a_p[0] - b_p[0])**2 + (a_p[1] - b_p[1])**2 < r_com**2:
            return True
        return False

        
    
    ## Return Agents that may cross the path of the Agent with id aID any time from now until t_end
    ## Based on a right-of-way matrix
    def getUnsafeAgents(self, aID, t_end):
        # TODO Use Intersection to get direction and facing inwards, It should use the prio matrix
        # Threading issue? Compute here instead?
        other_agent_poses = []
        for agent in self.agent_registry.keys():
            other_agent_poses.append([agent, self.agent_registry[agent][1]])
        return self.intersection.unsafeAgents(self.agent_registry[aID][1], other_agent_poses) #Implemented in springClean

    ## Update the Safety Membershtimeip (SM) of Agent with id aID
    ## The agent will have Maneuvre Oppertunity (MO) if all unsafe agents are reachable
    def updateAgentSM(self, aID):
        #t_stamp = rospy.get_rostime()
        t_stamp = self.get_time()
        #print("tstamp sec = {0}, nsec = {1}, ".format(t_stamp.secs,t_stamp.nsecs))
        #print("tstamp = {0}".format(t_stamp))
        #return
        U = self.getUnsafeAgents(aID, t_stamp + TM + 2*TD + TMan)
        R = self.getReachableAgents(aID, t_stamp + TM + 2*TD + TMan)
        
        MR = {}

        # Determine the MO and SM for every turn
        for turn in U.keys():
            MO = True
            SM = U[turn]
            for agent in U[turn]:
                if not (agent in R):
                    MO = False
                    SM = []
                    break
            MR[turn] = [t_stamp, MO, SM]

        #Update MR for agent in zookeeper    
        zookeeper.set(self.handle, "/root/mr/" + str(aID), str(MR))
        

    ## Periodically calculate the Safety Membership (SM) for every agent in the Agent Registry
    def calcSM(self):
        global TM
        #print("tm is " + str(TM))

        rate = rospy.Rate(TM)
    
        while not rospy.is_shutdown():
            rate.sleep()

            #Read ARset and store locally
            self.getARset()

            for agent_id in self.agent_registry.keys():
                self.updateAgentSM(agent_id)

    

if __name__ == '__main__':
    #mc = MembershipCloud(Intersection.Intersection((0.,0.), 7.5, Intersection.IntersectionType.GIVE_WAY_4, 'east-west')) #Center, lane width, type, alignment
    mc = MembershipCloud(risk_estimation.Intersection.Intersection())
    mc.calcSM()


#Deprecated

# Subscribe to all car topics
    #for i in range(1, self.nbr_agents + 1):
    #    rospy.Subscriber("/car_state" + str(i), cm.CarState, self.stateCallback, queue_size=10)

## Update Agent Registry
#def stateCallback(self, msg):
#    pass