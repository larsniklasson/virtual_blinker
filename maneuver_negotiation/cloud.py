#!/usr/bin/env python 
## The Cloud
## Gets data from all vehicles and broadcast their membership sets? Or upon request?

import rospy
import os
#import custom_msgs.msg as cm
import zookeeper
import numpy as np
import maneuver_negotiator_config
import sys
sys.path.append("..")
import maneuver_negotiation.maneuver_negotiator_config as config
import utils.Intersection
import virtual_blinker.msg as cm
import config as sim_config




#priorityMatrix = np.array([[1,0,0],[0,0,0],[0,0,0]])
#nonPriorityMatrix = np.array([[1,1,1],[0,1,1],[0,0,1]])

TM = config.GENERAL_OPTIONS['TM'] #period of membership update protocol,  #SM's update period
TD = config.GENERAL_OPTIONS['TD'] #upperbound on transmission delay
TMan = config.GENERAL_OPTIONS['TMan'] #upperbound on maneuver execution time
commRadius = config.GENERAL_OPTIONS['CommRadius'] #communication radius which v2x are possible by a car


## Doesn't have to be reliable? 
## It's now reliable due to zookeepers use of TCP, but could be made unreliable if the cloud started to listen to ros topics instead
class MembershipCloud:

    def __init__(self, intersection):

        rospy.init_node('cloud',anonymous=True)

        self.nbr_agents = 2 #TODO Get from launch file: rospy.get_param('nr_cars')

        self.agent_registry = {}
        devnull = open(os.devnull,"w")
        zookeeper.set_log_stream(devnull)

        #Use zookeeper storage instead
        self.handle = zookeeper.init(maneuver_negotiator_config.GENERAL_OPTIONS['zookeeper-server'])
        
        #Initiate all agents in the local version of the AR set
        self.getARset()

        self.intersection = intersection

        self.ros_measurements = None
        #subscribe to simulation nodes to get the time:
        self.car_state_subscriber_handle = rospy.Subscriber(maneuver_negotiator_config.ROS_COMMUNICATION_OPTIONS['car-state-topic'][1],cm.CarState,self.update_time)
        #self.car_state_subscriber_handle = rospy.Subscriber(maneuver_negotiation.maneuver_negotiator_config.ROS_COMMUNICATION_OPTIONS  


    def get_time(self):
        if (self.ros_measurements is None):
            return 0
        else:
            #convert to actual simulation time
            return self.ros_measurements.t/(sim_config.SIM_CONFIG["rate"]*sim_config.SIM_CONFIG["slowdown"])

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

            #zookeeper keeps things as string, convert to a list
            self.agent_registry[child] = eval(data)
        #print("agent_registry = " + str(self.agent_registry))


    ## Return Agents that are within communication distance from the Agent with id aID until time t_end
    def getReachableAgents(self, aID, t_end):
        R = []
        for agent in self.agent_registry.keys():
            if self.isReachable(aID, agent):
                R.append(agent)
        return R

    ## Return True if agent a is within communication distance of b
    def isReachable(self, aID, bID):
        a_p = self.agent_registry[aID][1]
        b_p = self.agent_registry[bID][1]

        # If within communication distance
        if (a_p[0] - b_p[0])**2 + (a_p[1] - b_p[1])**2 < commRadius**2:
            return True
        return False

        
    
    ## Return Agents that may cross the path of the Agent with id aID any time from now until t_end
    ## Based on a right-of-way matrix
    def getUnsafeAgents(self, aID, t_end):
        # TODO Use Intersection to get direction and facing inwards, It should use the prio matrix
        # Threading issue? Compute here instead?
        other_agent_poses = []
        for agent in self.agent_registry.keys():
            if self.agent_registry[agent] == []:
                continue
            other_agent_poses.append([agent, self.agent_registry[agent][1]])
        if other_agent_poses == []:
            return []

        return self.intersection.getUnsafeAgents(aID, self.agent_registry[aID][1], other_agent_poses) #Implemented in springClean

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
        zookeeper.set(self.handle, "/root/mr/0","[]")
        zookeeper.set(self.handle, "/root/mr/1","[]")
        #zookeeper.set(self.handle, "/root/mr/2","[]")
        #zookeeper.set(self.handle, "/root/segment/2","[]")
        zookeeper.set(self.handle, "/root/segment/1","[]")
        zookeeper.set(self.handle, "/root/segment/0","[]")


        global TM
        #print("tm is " + str(TM))

        rate = rospy.Rate(TM)
    
        while not rospy.is_shutdown():
            rate.sleep()

            #Read ARset and store locally
            self.getARset()
            if self.agent_registry['0'] == [] or self.agent_registry['1'] == []:# or self.agent_registry['2'] == []:
                pass
            else:
                for agent_id in self.agent_registry.keys():
                    self.updateAgentSM(agent_id)

    

if __name__ == '__main__':
    #mc = MembershipCloud(Intersection.Intersection((0.,0.), 7.5, Intersection.IntersectionType.GIVE_WAY_4, 'east-west')) #Center, lane width, type, alignment
    mc = MembershipCloud(utils.Intersection.Intersection())
    mc.calcSM()


#Deprecated

# Subscribe to all car topics
    #for i in range(1, self.nbr_agents + 1):
    #    rospy.Subscriber("/car_state" + str(i), cm.CarState, self.stateCallback, queue_size=10)

## Update Agent Registry
#def stateCallback(self, msg):
#    pass