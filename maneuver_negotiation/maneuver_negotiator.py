#!/usr/bin/env python
#import zmq
import calendar
from datetime import datetime
import time
from threading import Timer
import random
import zookeeper
import socket
import argparse
import rospy
import virtual_blinker.msg as cm
import std_msgs

import threading

import maneuver_negotiator_config

import os
import sys

sys.path.append("..")
from config import *
RATE = SIM_CONFIG['rate']
SLOWDOWN = SIM_CONFIG['slowdown']


class ManeuverNegotiator:

  ## DEFINE THE STATES
  NORMAL = 1    # Initial state, no requests sent and no grant given
  GET = 2       # Waiting for replies from agents in the Safety Membership set
  GRANTGET = 3  # Grant given to other agent but this agent also wants to ask for permission to do a maneuvre
  GRANT = 4     # Grant given to other agent and this agent doesn't want to do a maneuvre
  EXECUTE = 5   # All permissions recieved
  TRYGET = 6    # Some agents in the Safety Membership set aren't reachable or have answered DENY to our request


  def __init__(self,sim, agent_id,intersection, communication_details,risk_estimator,initial_direction=None):
    self.sim = sim
    #Make all the necessary variables global
    #all global variables in romi code is now instance variabes:
    self.status = self.NORMAL
    #self.TMan = 10
    self.T_RETRY = 3
    self.T_GRANT = 10
    self.aID = agent_id
    self.tRetry = []
    self.t_grant = []
    self.handle  =[ ] # Zookeeper handle, init this please
    self.grantID = 0  # Id of agent with active grant (The one the vehicle have given a grant to)
    self.tag = [0,0]  # Associated with a message, on the form: [timestamp, agent id]
    self.many = 0     # 1 if more than one agent in in Membership Registry, 0 otherwise
    self.agents_to_ask = [] # Agents in this vehicles Membership Registry

    #these were initialized later in romis code
    #self.T_UPDATE = 5
    self.timeTaken = 0
    #self.time1 = self.clock() # this variable is not used in romi's code
    self.TM = maneuver_negotiator_config.GENERAL_OPTIONS['TM']   # Period of membership protocol
    self.TMan = maneuver_negotiator_config.GENERAL_OPTIONS['TMan']  # Maneuvre time
    self.TA = maneuver_negotiator_config.GENERAL_OPTIONS['TA']     # Period of agent registry update (how often the agents x, v and a is sampled )
    self.TD = maneuver_negotiator_config.GENERAL_OPTIONS['TD']    # Upper bound on transmission delay

    self.M = set()
    self.D = set()
    self.R = set()

    #self.time_delay = 100

    #intersection at which the cars are setup
    #intersection = None

    #all measurements are updated to this variable
    #at the same frequency as simulation publishes them.
    #everytime romi's code calls clock() position()
    #velocity() acceleration(), measurements are taken from this variable
    #make dummy testMsg
    cs1 = cm.CarState()
    cs1.x = 1
    cs1.y = 1
    cs1.theta = 1
    cs1.speed = 1
    #cs1.acceleration = 1
    cs1.id = 1
    self.ros_measurements = cs1

    self.agent_state = [self.clock(), 1,1,1] #dummy vars for now
    self.agent = [agent_id,self.agent_state]

    self.intersection = intersection
    #self.initialize(zookeeper_ip)

    self.communication_details = communication_details

    #used in no_conflict to check risk
    self.risk_estimator = risk_estimator

    self.maneuver_requested=None #maneuver requested in trymaneuver

    self.my_travelling_direction = initial_direction


  def clock(self):
    if self.ros_measurements is None:
      return 1
    else:
      return float(self.ros_measurements.t/(RATE*SLOWDOWN))

  ## Get current position
  def position(self):
    return (float(self.ros_measurements.x), float(self.ros_measurements.y),float(self.ros_measurements.theta))


  ## Get current velocity
  def velocity(self):
    return int(self.ros_measurements.speed)

  ## Get current acceleration
  def acceleration(self):
    return 1


  ## Retry timer expired, tell the other agents to stop their lease of grant to you. Change status to TRYGET since all agents in R can't be reached
  def t_retry(self,intended_course):
    status = self.status
    agents_to_ask = self.agents_to_ask
    print("reached retry trymaneuver")

    if(status == self.GET): # If timer expired for this vehicles current request, ask other agents in D to release their grants
      self.status = self.TRYGET
      message = "RELEASE," + str(self.agent[0]) + "," + str(self.agent[1][0]) + "," + str(self.agent[1][1]) + "," + str(self.agent[1][2]) + "," + str(self.agent[1][3]) + "," + str(self.tag[0]) + "," + str(self.tag[1])

      if(self.many):
        for agents in self.agents_to_ask:
          print(message)
          self.send_udp_message(message,int(agents))
      else:
        self.send_udp_message(message,int(agents_to_ask))
    self.tryManeuver(intended_course)


  def get_MR(self,id1,intended_course):
    #EME: Suggested change (Requires zookeeper to be set up properly)
    (data, stat) = zookeeper.get(self.handle, "/root/mr/" + str(id1), True) 
    data = eval(data)
    membership = data[intended_course]
    membership[2]  = ",".join(membership[2])
    print "membership is ", membership
    return membership

    (data, stat) = zookeeper.get(self.handle, "/root/segment/" + str(id1), True)
    children = zookeeper.get_children(self.handle, "/root/segment", True)
    ask_these = ""
    for child in children:
      if(child != str(id1)):
        ask_these = ask_these + str(child) + ","
    ask_these = ask_these[0:len(ask_these)-1]
    print "ask_these = " + str(ask_these)
    print "ask_these of type" + str(type(ask_these))
    return [stat["mtime"], 1, str(ask_these)]


  def tryManeuver(self,intended_course=None):
    self.maneuver_requested = intended_course

    #socket.identity = identity.encode('ascii')
    if (self.status == self.NORMAL or self.status == self.GRANT):
      self.tag = [self.clock(), self.aID]
    if (self.status == self.NORMAL or self.status == self.TRYGET):
      self.status = self.TRYGET

      self.agent_state = [self.clock(), self.position(), self.velocity(), self.acceleration()]

      self.agent = [self.aID,self.agent_state]
      MR = self.get_MR(self.agent[0],intended_course)

      if(( self.agent_state[0]/(RATE*SLOWDOWN) < MR[0] + 2*self.TMan) and MR[1] == 1):
        if(',' in MR[2]):
          self.agents_to_ask = MR[2].split(',')
          self.many = 1
        else:
          self.agents_to_ask = MR[2]
          self.many = 0
        if(self.many):
          for car in self.agents_to_ask:
            self.D.add(str(car))
            self.R.add(str(car))
        else:
          self.D.add(str(self.agents_to_ask))
          self.R.add(str(self.agents_to_ask))  
        message = "GET," + str(self.agent[0]) + "," + str(self.agent[1][0]) + "," + str(self.agent[1][1]) + "," + str(self.agent[1][2]) + "," + str(self.agent[1][3]) + "," + str(self.tag[0]) + "," + str(self.tag[1])
        if (intended_course is not None): #EME Why would this happen?
          message = "GET," + str(self.agent[0]) + "," + str(self.agent[1][0]) + "," + str(self.agent[1][1]) + "," + str(self.agent[1][2]) + "," + str(self.agent[1][3]) + "," + str(self.tag[0]) + "," + str(self.tag[1]) + "," + str(intended_course)
        print(message)
        if(self.last()):
          self.status = self.EXECUTE
          self.doManeuver(self.TMan)
          self.status = self.NORMAL

        elif(self.many):
          self.status = self.GET
          for agents in self.agents_to_ask:
            print(message)
            self.send_udp_message(message,int(agents))
            print("sent")
          self.tRetry = Timer(2*self.TD, self.t_retry, args=(intended_course,))
          print("starting retry")
          self.tRetry.start()
        else:
          self.status = self.GET
          print("agents_to_ask = " + str(self.agents_to_ask))
          self.send_udp_message(message,int(self.agents_to_ask))
          print("starting t_retry by car {0} first".format(self.aID))
          self.tRetry = Timer(2*self.TD, self.t_retry, args=(intended_course,)) # Retry later if messages delayed more than the upperbound of transmission delays
          self.tRetry.start()
          # print("set timer first time")

      else:
        #print( self.agent_state[0] < MR[0] + 2*self.TMan) and MR[1] == 1)
        print "self.agent_state[0] is ", self.agent_state[0]
        print "MR[0] is ", MR[0]
        print "2*self.TMan is", 2*self.TMan

        print("starting t_retry by car {0}, second case".format(self.aID))
        self.tRetry = Timer(self.TA, self.t_retry, args=(intended_course,)) # Wait until a fresh membership is available
        self.tRetry.start()
    elif (self.status == self.GRANT):
      self.status = self.GRANTGET

  def watch_maneuver_requester(self,requester_id,requester_course,requester_maneuver):
    while True:
      print("watchinggg")
    pass

  ## Estimate the expectation of the car with register mAR. No conflict if weighted average over a certain threshold
  def no_conflict(self, mAR, time_tick):
    global RATE
    global SLOWDOWN
    current_time = time_tick/(RATE*SLOWDOWN)
    print("in no conflict")
    print "current time is ", time_tick/(RATE*SLOWDOWN)

    #mAR = [m_dict["Sender"], m_dict["Time"], m_dict["Position"], m_dict["Velocity"], m_dict["Acc"]]
    sender = mAR[0]
    sent_time = float(mAR[1])
    sender_pose =map(float,mAR[2])
    sender_speed = float(mAR[3])
    sender_acceleration = float(mAR[4])
    if(len(mAR)>5):
      sender_maneuver = mAR[5]
    
    """
    No conflict checks time sender enters and leaves intersection overlaps with 
    the recieving vehicle.

    Current scenario is sender is a left turning vehicle, receiver is a straight
    passing vehicle.

    We first estimate earliest and last possible time left turning vehicle can enter 
    the intersection. We estimate the earliest and last possible time left turning
    vehicle can leave. we take earliest enter and last leave duration 
    """

    #calculate earliest possible time sender can enter:
    #this earliest time is when sender receives grant almost immediately , like within 5 ms
    #we project where the car will be after like 5ms using stop profile
    #we then project when the car will arrive at 
    #
    #
    #first, get the course the sender is taking:
    #this can be done from sender_position and sender_course
    stop_interval = 0.005
    sender_travelling_direction = self.intersection.getTravellingDirection(*sender_pose)

    if (sender_travelling_direction is None):
      #this happens while i am at the intersection
      print("sender travel direction is none")
      print "sender state is ", sender_pose
      return False
    sender_course = self.intersection.courses[(sender_travelling_direction,sender_maneuver)]

    stop_pose_state = sender_course.predictNextState(sender_pose[0],sender_pose[1],sender_pose[2],sender_speed,stop_interval,"stop")
    #fromthen onwards it will take profile of go:
    sender_earliest_entering_time = current_time + stop_interval + \
                                    sender_course.getTimeToCrossing(stop_pose_state[0],stop_pose_state[1],stop_pose_state[2],stop_pose_state[3],"go")

    stop_pose_state = sender_course.predictNextState(sender_pose[0],sender_pose[1],sender_pose[2],sender_speed,2*self.TD,"stop")
    #last_entering_time = current_time + (2*self.TD) + sender_course.getTimeToCrossing(*stop_pose_state,"go")
    sender_last_entering_time = current_time + (2*self.TD) + \
                                    sender_course.getTimeToCrossing(stop_pose_state[0],stop_pose_state[1],stop_pose_state[2],stop_pose_state[3],"go")

    sender_last_leaving_time = sender_last_entering_time + self.TMan

    my_state = self.position()
    my_speed = self.velocity()
    my_travelling_direction = self.my_travelling_direction

    if (my_travelling_direction is None):
      #this happens while i am at the intersection
      print("my travelling direction is none")
      print "my state is ", my_state
      return False

    my_course = self.intersection.courses[(my_travelling_direction,"straight")]
    my_entering_time = current_time + my_course.getTimeToCrossing(my_state[0],my_state[1],my_state[2],my_speed,"go")
    my_leaving_time = my_entering_time + self.TMan

    print "sender earliest entering time is: ", sender_earliest_entering_time
    print "sender last entering time is: ", sender_last_entering_time
    print "sender last leaving time is: ", sender_last_leaving_time
    print "my entering time is: ", my_entering_time
    print "my leaving time is: ", my_leaving_time

    not_conflicted = False
    #not conflicted because either party has left the intersection
    if (sender_course.hasLeftIntersection(*sender_pose) or my_course.hasLeftIntersection(*my_state)):
      not_conflicted = True

    safe_gap = my_entering_time - sender_last_leaving_time
    if (safe_gap > 0): #means if I enter intersection later than sender leaves, safe.
      print("not conflicted because i enter before sender")
      not_conflicted = True
    else: #means if sender leaves intersection later than i enter.
      #then we have to compare my leaving time vs his entering time:
      if sender_earliest_entering_time > my_leaving_time: # if sender enters intersection later than I leaves
        print("not conflicted because leaves before i enter")
        #in this case, our estimation says the sender leaves before i enter, but
        #due to unforseen circumstances, sender may slowdown and stop inside
        #the intersection, in this case we have to detect if the vehicle requested is able to finish
        #maneuver in time. if we detect ahead of time that the vehicle will not be able to finish the 
        #maneuver in time, we have to slow our car down, this happens in this separate thread which
        #detects this

        #watch the requester and detect if the vehicle will leave the intersection in time.
        #start a separate thread on it:
        #def watch_maneuver_requester(self,requester_id,requester_course,requester_maneuver):
        self.watch_request_thread = threading.Thread(target=self.watch_maneuver_requester,args=(sender,sender_course,sender_maneuver))
        self.watch_request_thread.start()
        not_conflicted = True
      else:
        print("sender entering while i am just about to leave")
        #possibility for sender entering when i have not left, just about to leave, this should be safe too..
        pass
    
    return not_conflicted
    


    #calculate least    possible time sender can enter:
    #offset these two by TMan to get earliest and last possible exit times. 
    #take the last possible time 
    #return self.risk_estimator.isManeuverOk(int(sender),sender_course)

  ## Dummy function, the agent got permissions from everyone in the SM and executes the manoeuvre in t time units
  def doManeuver(self,t):
    #self.risk_estimator.add_car_to_grantlist(self.aID,self.TMan,self.maneuver_requested)
    print("Doing maneuver")
    self.sim.granted = True
    time.sleep(t)
    print("done")

  ## Returns true when GRANT/DENY received from the last agent
  ## in agents_to_ask, that is agents in the SM
  def last(self):
    print(len(self.R))
    if(len(self.R) == 0):
      return 1
    return 0

  ## Timer for given grant runs out, go back to NORMAL
  def tgrant(self):
    if self.status == self.GRANT:
      self.status = self.NORMAL
      self.grantID = 0
    if self.status == self.GRANTGET: #If you've been waiting to ask for permission, try again
      self.status = self.NORMAL
      self.grantID = 0
      self.tryManeuver()

  ## Update the agent state in the register for this car in the storage server
  def update(self):
    # setvalue = str(self.aID) + "," + str(self.agent_state[1]) + "," + str(self.agent_state[2]) + "," + str(self.agent_state[3])
    #IBR: following is not in romi's code, but i think it is needed:
    # print("updating... car {0}".format(self.aID))
    self.agent_state = [self.clock(), self.position(),self.velocity(),self.acceleration()]
    self.agent[1] = self.agent_state
    zookeeper.set(self.handle, "/root/segment/" + str(self.aID), str(self.agent_state)) #Update values stored in zookeeper

    t_update = threading.Timer(self.TA, self.update) #Set timer to TA = period of agentstate update
    t_update.start()

  ## Return true if received tag precedes this agent's permission request tag
  def precedes(self,tagTs):
    return (tagTs < self.tag[0])

  ## Return true if the agent ID in the tag belongs to the same agent that is already granted
  def grantedID(self,tagID):
    return (tagID == self.grantID)

  ## Called upon delivery of message to process it's content
  def message_processing(self, message, t):

    #when messages are recived, udp sends data as is without any formatting,
    #so normal string splitting in here works.
    #but if messeges were sent via ros, the message received has a data field, and to get
    #the actual data, invoke message.data to get them. so depending on communication method,
    #things will change to accomodate this:

    curtime = self.clock()
    print(curtime-t <= self.TD) 
    if (curtime - t <= self.TD): #Only process message if it arrived within the transmission delay boundary
      if (self.communication_details == 1): #if using ros:
        #received string is 'data: "GET,1,1,1,1,1,1,1"' without single quotes,
        # first split extracts the "GET,1,1,1,1,1,1,1" and the second split separates them like usual
        message_split = message.split('"')[1::2][0].split(',')
      else:
        message_split = message.split(',')
        
        mtype = message_split[0]
        if mtype == "GET":
          message_split[3] = message_split[3][1:]
          message_split[5] = message_split[5][:-1]

          m_dict = {"Type":message_split[0], "Sender":message_split[1], "Time":message_split[2],
                "Position":message_split[3:6], "Velocity":message_split[6], "Acc":message_split[7],
                "TagTime":message_split[8], "TagID":message_split[9], "IntendedCourse":message_split[10]}
          

        if mtype == "DENY":
          m_dict = {"Type":message_split[0], "Sender":message_split[1], "Time":message_split[2],
                "Position":message_split[3:6], "TagTime":message_split[6], "TagID":message_split[7]}

              
        if mtype == "RELEASE":

          m_dict = {"Type":message_split[0], "Sender":message_split[1], "Time":message_split[2],
                "Position":message_split[3:6], "TagTime":message_split[6], "TagID":message_split[7]}


        if mtype == "GRANT":


          m_dict = {"Type":message_split[0], "Sender":message_split[1], "Time":message_split[2],
                "Position":message_split[3:6], "TagTime":message_split[6], "TagID":message_split[7]}

        
  
      #Decode the message
      

      print ("received " + str(message) + " by car " + str(self.aID) + "sent from  car" + m_dict["Sender"])
      # print("status: " + str(self.status))

      if (mtype == "GRANT" or mtype == "DENY") and self.status == self.GET:
        #when a reply is received, membership needs to be queried and
        #vehicles we should get a reply from ie. self.R should be intersection
        #of existing self.R and new membership.
        new_membership = self.get_MR(self.agent[0],self.maneuver_requested)
        print "in request processing, exisiting membership = " ,self.R
        print "in request processing, new membership = " , new_membership

        #check if the packet received as membership is valid:
        if(( self.agent_state[0]/(RATE*SLOWDOWN) < new_membership[0] + 2*self.TMan) and new_membership[1] == 1):
          new_R =set(self.R)& set(new_membership[2])
          print "in request processing, new R = " , new_R


      #If this is an answer to our own request for a manoeuvre
      if((m_dict["Type"] == "GRANT" or m_dict["Type"] == "DENY") and self.status == self.GET):
        # print("Received a grant or deny and status == get")
        self.M.add(m_dict["Type"]) #Add message to set of received messages
        self.R.discard(m_dict["Sender"]) #Remove this agent from agents we're waiting for
        
        if(self.last()): #If all expected replies received
          print("last")
          self.tRetry.cancel() #Stop the retry timer
          print("stopped retry timer")

          #Check if any agent denied our request
          granted = 1
          for m in self.M:
            if (m == "DENY"):
              print("IM DENYING YOU")
              granted = 0
          self.M.clear()

          #Execute if all agents gave us permission to do the manoeuvre
          if(granted):
            self.status = self.EXECUTE
            self.doManeuver(self.TMan)
            self.status = self.NORMAL
          else: #Else ask the others to release the grant they gave to us
            self.status = self.TRYGET
            for agents in self.D:
              s_message = "RELEASE," + str(self.agent[0]) + "," + str(self.agent[1][0]) + "," + str(self.agent[1][1]) + "," + str(self.agent[1][2]) + "," + str(self.agent[1][3]) + "," + str(self.tag[0]) + "," + str(self.tag[1])
              print(s_message)

              self.send_udp_message(s_message,int(agents) )

            # Try to send a request again after the agent states have been updated
            self.tRetry = Timer(self.TA, self.t_retry, args=(self.maneuver_requested,))
            self.tRetry.start()
      
      # The message contains a request
      elif (m_dict["Type"] == "GET"):
        mAR = [m_dict["Sender"], m_dict["Time"], m_dict["Position"], m_dict["Velocity"],
              m_dict["Acc"]]
        if (len(message_split) > 8):
          mAR.append(m_dict["IntendedCourse"])
        #in romi code, time is converted into int
        #if (no_conflict(mAR, int(m_dict["Time"]) + 2*time_delay + TMan) and
        nc = self.no_conflict(mAR, float(m_dict["Time"]) + 2*self.TD + self.TMan)
        print "no conflict", nc
        if (nc and #If no conflict = the manoeuvre can be executed without risk in the time 2TD + TMan (The expectation of the vehicle is to go)
            (self.status == self.NORMAL or self.status == self.TRYGET or #and (we are either not asking for permission, or asking for permission but waiting for T_retry to expire since all agents in SM not reachable or some sent us DENY
            (self.status == self.GET and self.precedes(m_dict["TagTime"])) or #, or we're asking for permission but this request has a lower timestamp
            ((self.status == self.GRANT or self.status == self.GRANTGET) and self.grantedID(m_dict["TagID"])))): # or we have already given permission to this agent)
          print("not conlicted")

          if (self.status == self.NORMAL): #Give permission if you're not making a request yourself and haven't given it to anyone at this moment
            self.status = self.GRANT
            self.grantID = m_dict["TagID"]
          elif (self.status == self.GET or self.status == self.TRYGET): #If you're asking for permission but this request precedes yours, then cancel your own request
            self.tRetry.cancel()
            #Make sure the permissions you've already received are released
            if (self.status == self.GET):
            
              for agents in self.D:
                #print(tmp)
                s_message = "RELEASE," + str(self.agent[0]) + "," + str(self.agent[1][0]) + "," + str(self.agent[1][1]) + "," + str(self.agent[1][2]) + "," + str(self.agent[1][3] + "," + str(self.tag[0]) + "," + str(self.tag[1]))
                self.send_udp_message(s_message,int(agents))
            self.status = self.GRANTGET
            self.grantID = m_dict["TagID"]

          #Send a GRANT to the sender agent
          sender = m_dict["Sender"]
          s_message = "GRANT," + str(self.agent[0]) + "," + str(curtime) + "," + str(self.agent[1][1]) + "," + str(self.agent[1][2]) + "," + str(self.agent[1][3]) + "," + str(self.tag[0]) + "," + str(self.tag[1])
          
          #once we grant, we add the car to the list of grants, 
          #we no longeruse this in timeoverlap branch
          # self.risk_estimator.add_car_to_grantlist(int(sender),self.TMan, m_dict["IntendedCourse"])

          print(s_message)
          self.send_udp_message(s_message,int(sender))
          self.t_grant = threading.Timer(self.TD*2+self.TMan, self.tgrant) #EME Should be set to time at agetn state update - current time + 2TD + TMan
          self.t_grant.start()

        #DENY otherwise
        else: 
          sender = m_dict["Sender"]
          s_message = "DENY," + str(self.agent[0]) + "," + str(curtime) + "," + str(self.agent[1][1]) + "," + str(self.agent[1][2]) + "," + str(self.agent[1][3]) + "," + str(self.tag[0]) + "," + str(self.tag[1])
          self.send_udp_message(s_message,int(sender))

      # A release message arrived, check if it's valid and in that case release the current grant
      #EME Have to check that the RELEASE matches the received grant? (self.grantID == m_dict["TagID"] and also comparing the timestamps? Or is this implicitly handled as Antonio said?)
      elif (m_dict["Type"] == "RELEASE" and (self.status == self.GRANT or self.status == self.GRANTGET)):
        #Stop the grant timer
        self.t_grant.cancel()
        self.grantID = 0
        if (self.status == self.GRANT):
          self.status = self.NORMAL

        #remove the granted car from the grant list because we got a release message
        self.risk_estimator.remove_car_from_grantlist(int(m_dict["Sender"]))
        self.risk_estimator.remove_grant_thread.cancel()

        #If you want to send a request, then try to send
        if (self.status == self.GRANTGET):
          self.status = self.TRYGET
          self.tryManeuver()



  #repurpose this: 
  def setup_ros(self):
    #subscribe to the measurement channel and update variables

    #lars have split the measurements to one topic per car instead f a single topic
    #rospy.Subscriber(maneuver_negotiator_config.GENERAL_OPTIONS['measurement-topic'],cm.TestMsg,self.update_agent_state_from_ros)

    #subscribeto measurements of self
    self.car_state_subscriber_handle = rospy.Subscriber(maneuver_negotiator_config.ROS_COMMUNICATION_OPTIONS['car-state-topic'][self.aID],cm.CarState,self.update_agent_state_from_ros)


    #print("setting up maneuver negotiator ros")
    choose_leader_topic = maneuver_negotiator_config.ROS_COMMUNICATION_OPTIONS['maneuver-negotiation-topics']['choose-leader-topic']
    self.choose_leader_subscriber_handle = rospy.Subscriber(choose_leader_topic, std_msgs.msg.Int8,self.choose_leader_processor)

    #if we use the ros to communicate maneuver negotiations between other cars,
    #this car must be publisher to all other cars. as the number of cars can be any number, it is dynamically built.
    #self.other_cars is a dictionary where key is the agent id of the other cars and value is the rospy.pulisher handle
    #so that we can use that to send messages when we want it.
    self.other_cars = {}
    if (self.communication_details == 1):
      self.sub = rospy.Subscriber(maneuver_negotiator_config.ROS_COMMUNICATION_OPTIONS['maneuver-negotiation-topics'][self.aID][0], \
                                 std_msgs.msg.String, self.ros_message_processor)
      #print("subscribing to " + str(maneuver_negotiator_config.ROS_COMMUNICATION_OPTIONS['maneuver-negotiation-topics'][self.aID][0]))

  #repurpose this: 
  def choose_leader_processor(self, data):
    #we receive the message from chooseleader topic.
    #this topic will be the car id that would initiate to do the maneuver negotiation
    #algorithm

    print("received messsage from choose leader")
    if data == std_msgs.msg.Int8(self.aID):
      self.tryManeuver()


#repurpose this:
  def udp_msg_processor(self):
    #this will either:
    # listen on udp socket if communication is udp,
    # subscribe to rostopic to get messages , if ros
    if self.communication_details == 0:
      sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

      #ip to bind
      host_ip = maneuver_negotiator_config.UDP_COMMUNICATION_OPTIONS[self.aID][0]
      port = maneuver_negotiator_config.UDP_COMMUNICATION_OPTIONS[self.aID][1]
      print('udp thread: binding to ' + str((host_ip,port)))
      sock.bind((host_ip, port))
      while 1:
        msg,address = sock.recvfrom(4096)

        #print("Received -->>: " + msg)
        t = self.clock()
        self.message_processing(msg, t)
    else:
      #subscribe to ross
      #in the romis implementation, clock() is caled and then message_processing is called.
      #with the answer returned from clock().
      #in the ros implementation, i call the clock() inside ros_message_processor, and then 
      #pass the gotten string from the message into message_processing along with the clock() value.
      #hope thats not a problem
      # self.sub = rospy.Subscriber(maneuver_negotiator_config.ROS_COMMUNICATION_OPTIONS['maneuver-negotiation-topics'][self.aID][0], \
                                #  std_msgs.msg.String, self.ros_message_processor)
      # above code commented because it is now done by setup_ros()
      pass

  def ros_message_processor(self,data):
    t = self.clock()
    self.message_processing(str(data),t)
  




  def update_agent_state_from_ros(self,data):
    self.ros_measurements = data



  def send_udp_message(self,message,agent_id):
    #port number is derived from car id.
    #car id ranges from 1 to 254
    #port number ranges from port_prefix+carid to uppper limit 
    #since all cars exist in same network, but ip addresses are 
    #different, 
    #car_id = int(agent_port) - communication_config.NETWORK_OPTIONS['port-start']

    if self.communication_details == 0:
      car_id = agent_id
      target_car_ip = maneuver_negotiator_config.UDP_COMMUNICATION_OPTIONS[car_id][0]
      sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
      agent_port = maneuver_negotiator_config.UDP_COMMUNICATION_OPTIONS[car_id][1]
      print("sending {0} to car {1} by car {2}".format(message,agent_id,self.aID))
      #print("agent id = " + str(agent_id))
      #print(str((target_car_ip,agent_port)))
      sock.sendto(message, (target_car_ip, agent_port))
      sock.close()
    else:
      #we use ros:
      #check if we have the target agent_id's publish handle
      if agent_id in self.other_cars:
        publisher = self.other_cars[agent_id]
      else:
        self.other_cars[agent_id] = rospy.Publisher(maneuver_negotiator_config.ROS_COMMUNICATION_OPTIONS['maneuver-negotiation-topics'][agent_id][0],\
                                                    std_msgs.msg.String,queue_size=10)

      print("sending {0} to car {1} by car {2}".format(message,agent_id,self.aID))
      publisher = self.other_cars[agent_id]
      publisher.publish(std_msgs.msg.String(message))

      # if not, we make a new publisher and use that
      # if we have, we use that handle to publish message
      
      
  def initialize(self):
    #self.handle = zookeeper.init(communication_config.NETWORK_OPTIONS['zookeeper-server'])
    devnull = open(os.devnull,"w")
    zookeeper.set_log_stream(devnull)
    self.handle = zookeeper.init(maneuver_negotiator_config.GENERAL_OPTIONS['zookeeper-server'])

    self.t_update = Timer(self.TA, self.update)
    self.t_update.start()

    self.setup_ros()

    #start thread that listens on network for udp packets from other cars
    net_thread = threading.Thread(target= self.udp_msg_processor)
    net_thread.start()

    #rospy spinning to be done by the main method which creates this class.
    #spinning here will make the initialize() in a loop where
    #it cannot return to the main method. 
    #rospy.spin()




