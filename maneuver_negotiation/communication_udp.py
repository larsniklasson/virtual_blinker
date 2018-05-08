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
import communication_config
import rospy
import custom_msgs.msg as cm
import std_msgs

import threading

import os
import sys
#p = os.path.abspath(os.path.dirname(__file__))
#print(p)
#exit()
#print(sys.path)
#lib_path = os.path.abspath(os.path.join(p, '..', '..', 'risk_estimation', 'src', 'scripts')) #TODO folder structure here
lib_path = os.path.abspath(communication_config.GENERAL_OPTIONS['risk-estimator-path']) #TODO folder structure here
sys.path.append(lib_path)
import Intersection



#import rospy
#import custom_msgs as cm
def clock():
  #t = calendar.timegm(time.gmtime())

  #gets time from ros simulation enviornment
  return float(ros_measurements.t)

  #return t

#DEFINE THE STATES
NORMAL = 1
GET = 2
DELAY_GET = 3
GRANT = 4
EXECUTE = 5
TRYGET = 6
#Make all the necessary variables global
#global time
global status
global agent_state
global agent
global TMan
global T_RETRY
global T_GRANT
global aID
global tRetry
global t_grant
global handle
global grantID
global tag
global many
global agents_to_ask

#ibrahim udp adoption:
#global host_ip = '127.0.0.1'
host_ip = '127.0.0.1'
#intersection at which the cars are setup
intersection = None
#port_prefix = '900'

#all measurements are updated to this variable
#at the same frequency as simulation publishes them.
#everytime romi's code calls clock() position()
#velocity() acceleration(), measurements are taken from this variable
#make dummy testMsg
#cs1 = cm.CarState( std_msgs.msg.Float32(1),\
#                   std_msgs.msg.Float32(1),\
#                   std_msgs.msg.Float32(1),\
#                   std_msgs.msg.Float32(1),\
#                   std_msgs.msg.Int32(1) )
#cs2 = cm.CarState( std_msgs.msg.Float32(1),\
#                   std_msgs.msg.Float32(1),\
#                   std_msgs.msg.Float32(1),\
#                   std_msgs.msg.Float32(1),\
#                   std_msgs.msg.Int32(2) )
cs1 = cm.CarState()
cs1.x = 1
cs1.y = 1
cs1.theta = 1
cs1.speed = 1
cs1.acceleration = 1
cs1.id = 1
cs2 = cm.CarState()
cs2.x = 1
cs2.y = 1
cs2.theta = 1
cs2.speed = 1
cs2.acceleration = 1
cs2.id = 2
ros_measurements = cm.TestMsg()
ros_measurements.t = 1
ros_measurements.cs1 = cs1
ros_measurements.cs2 = cs2

#Initiate variables if needed
agents_to_ask = []
many = 0
tag = [0,0]
grantID = 0
aID = 1
T_RETRY = 3
T_GRANT = 10
T_UPDATE = 5
TMan = 10
x = 5
v = 30
a = 0
timeTaken = 0

status = NORMAL

time1 = clock()

#A mock up agent_state, will be updated continously to ZooKeeper
agent_state = [time1, x, v, a]
agent = [aID, agent_state]
TM = 15
M = set()
D = set()
R = set()
time_delay = 100 #Upper bound on transmission delay


def position():
  global x

  #for now we give the x cordinate.

  if aID == 1:
    return int(ros_measurements.cs1.x)
  elif aID == 2:
    return int(ros_measurements.cs2.x)
  
  return x
def velocity():
  global v

  if aID == 1:
    return int(ros_measurements.cs1.speed)
  elif aID == 2:
    return int(ros_measurements.cs2.speed)
  return v

def acceleration():
  global a

  if aID == 1:
    return int(ros_measurements.cs1.acceleration)
  elif aID == 2:
    return int(ros_measurements.cs2.acceleration)


  return a

def t_retry():
  global status
  global agents_to_ask
  global host_ip
  print("reached retry")
  if(status == GET):
    status = TRYGET
    message = "RELEASE," + str(agent[0]) + "," + str(agent[1][0]) + "," + str(agent[1][1]) + "," + str(agent[1][2]) + "," + str(agent[1][3]) + "," + str(tag[0]) + "," + str(tag[1])
    if(many):
      for agents in agents_to_ask:
        print(message)
        tmp_agent = str(agents)
        print(tmp_agent)
        tmp = "tcp://localhost:" + tmp_agent
        print(tmp)
        #context = zmq.Context()
        #socket = context.socket(zmq.DEALER)
        #socket.connect(tmp)
        #socket.send(message)
        #socket.disconnect(tmp)

        #sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        #sock.sendto(message, (host_ip, int(agents)))
        #sock.close()
        send_udp_message(message,int(agents))
        print("sent")
    else:
      #tmp = "tcp://localhost:" + str(agents_to_ask)
      #print(tmp)
      #context = zmq.Context()
      #socket = context.socket(zmq.DEALER)
      #socket.connect(tmp)
      #print("connected")
      #socket.send(message)
      #sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
      #sock.sendto(message, (host_ip, int(agents_to_ask)))
      #sock.close()
      send_udp_message(message,int(agents_to_ask))
  tryManeuver()


def get_MR(id1):
  (data, stat) = zookeeper.get(handle, "/root/segment/" + str(id1), True)
  children = zookeeper.get_children(handle, "/root/segment", True)
  ask_these = ""
  for child in children:
    if(child != str(id1)):
      ask_these = ask_these + str(child) + ","
  ask_these = ask_these[0:len(ask_these)-1]
  return [stat["mtime"], 1, str(ask_these)]


def tryManeuver():
  global status
  global agent_state
  global agent
  global time1
  global TM
  global TMan
  global D
  global R
  global tRetry
  global aID
  global tag
  global many
  global agents_to_ask
  global T_RETRY
  global host_ip
  #context = zmq.Context()
  #socket = context.socket(zmq.DEALER)
  identity = 'me'
  #socket.identity = identity.encode('ascii')
  if (status == NORMAL or status == GRANT):
    tag = [clock(), aID]
  if (status == NORMAL or status == TRYGET):
    status = TRYGET

    agent_state = [clock(), position(), velocity(), acceleration()]


    MR = get_MR(agent[0])

    if((agent_state[0] < MR[0] + 2*TMan) and MR[1] == 1):
      if(',' in MR[2]):
        agents_to_ask = MR[2].split(',')
        many = 1
      else:
        agents_to_ask = MR[2]
        many = 0
      if(many):
        for car in agents_to_ask:
          D.add(str(car))
          R.add(str(car))
      else:
        D.add(str(agents_to_ask))
        R.add(str(agents_to_ask))  
      message = "GET," + str(agent[0]) + "," + str(agent[1][0]) + "," + str(agent[1][1]) + "," + str(agent[1][2]) + "," + str(agent[1][3]) + "," + str(tag[0]) + "," + str(tag[1])
      print(message)
      if(last()):
        status = EXECUTE
        doManeuver(TMan)
        status = NORMAL

      elif(many):
        status = GET
        for agents in agents_to_ask:
          print(message)
          tmp_agent = str(agents)
          print(tmp_agent)
          tmp = "tcp://localhost:" + tmp_agent
          print(tmp)
          ##socket.connect(tmp)
          #print(agents)
          #socket.send_string(message)
          #socket.disconnect(tmp)

          #sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
          #sock.sendto(message, (host_ip, int(tmp_agent)))
          #sock.close()
          send_udp_message(message,int(tmp_agent))
          print("sent")
        tRetry = Timer(T_RETRY, t_retry)
        tRetry.start()
      else:
        status = GET
        #tmp = "tcp://localhost:" + str(agents_to_ask)
        #print(tmp)
        #socket.connect(tmp)
        #print("connected")
        #socket.send_string(message)
        #T_RETRY = 2*TD

        #sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        print("agents_to_ask = " + str(agents_to_ask))
        #sock.sendto(message, (host_ip, int(agents_to_ask)))
        #sock.close()
        send_udp_message(message,int(agents_to_ask))
        tRetry = Timer(T_RETRY, t_retry)
        tRetry.start()
        print("set timer first time")


    else:

      tRetry = Timer(T_RETRY, t_retry)
      tRetry.start()


  elif (status == GRANT):
    status = DELAY_GET

def no_conflict(mAR, time):
  

  #mAR = [m_dict["Sender"], m_dict["Time"], m_dict["Position"], m_dict["Velocity"], m_dict["Acc"]]
  sender = mAR[0]
  sent_time = mAR[1]
  sender_position = mAR[2]
  sender_velocity = mAR[3]
  sender_acceleration = mAR[4]

  print("mAR : {0}".format(mAR))
	#print("GRANT ID RIGHT NOW IS: %i", grantID)
  if(grantID != 0):
		if(grantID == mAR[0]):
		  return 1
		else:
		  return 0
  return 1

def doManeuver(t):
  #dummy code, not needed
	#global timeTaken
	#print("TimeTaken = %f" % timeTaken)
	#print("Clock right now = %f " % time.clock())
	#timeTaken = time.clock() - timeTaken
	#print("The time for this algorithm is %f" % timeTaken)
	print("Doing maneuver")
	time.sleep(t)
	print("done")

def last():
  print(len(R))
  if(len(R) == 0):
    return 1
  return 0

def tgrant():
  global status
  global grantID
  if status == GRANT:
    status = NORMAL
    grantID = 0
  if status == DELAY_GET:
    status = NORMAL
    grantID = 0
    tryManeuver()

def update():
  global aID
  setvalue = str(aID) + "," + str(agent_state[1]) + "," + str(agent_state[2]) + "," + str(agent_state[3])
  zookeeper.set(handle, "/root/segment/" + str(aID), str(agent_state))
  t_update = Timer(T_UPDATE, update)
  t_update.start()

def precedes(tagTs):
  global tag
  return (tagTs < tag[0])

def grantedID(tagID):
  global grantID
  return (tagID == grantID)

def message_processing(message, t):
  global status
  global agent
  global time_delay
  global TMan
  global M
  global R
  global D
  global tRetry
  global t_grant
  global grantID
  global T_RETRY
  print(clock()-t <= time_delay)
  if (clock() - t <= time_delay):
    message_split = message.split(',')
    m_dict = {"Type":message_split[0], "Sender":message_split[1], "Time":message_split[2],
              "Position":message_split[3], "Velocity":message_split[4], "Acc":message_split[5],
              "TagTime":message_split[6], "TagID":message_split[7]}
    print("status: " + str(status))
    if((m_dict["Type"] == "GRANT" or m_dict["Type"] == "DENY") and status == GET):
      print("Received a grant or deny and status == get")
      M.add(m_dict["Type"])
      R.discard(m_dict["Sender"])
      if(last()):
        print("last")
        tRetry.cancel()
        print("stopped timer")
        #STOP TIMER FOR RETRY
        granted = 1
        for m in M:
          if (m == "DENY"):
            print("IM DENYING YOU")
            granted = 0
        M.clear()
        if(granted):
          status = EXECUTE
          doManeuver(TMan)
          status = NORMAL
        else:
          status = TRYGET
          for agents in D:
            tmp = "tcp://localhost:" + agents
            print(tmp)
            #context = zmq.Context()
            #socket = context.socket(zmq.DEALER)
            #identity = 'me'
            #socket.identity = identity.encode('ascii')
            s_message = "RELEASE," + str(agent[0]) + "," + str(agent[1][0]) + "," + str(agent[1][1]) + "," + str(agent[1][2]) + "," + str(agent[1][3]) + "," + str(tag[0]) + "," + str(tag[1])
            print(s_message)
            #socket.connect(tmp)
            #socket.send_string(s_message)

            #sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            #sock.sendto(s_message, ('127.0.0.1', int(agents)))
            #sock.close()
            send_udp_message(s_message,int(agents) )
            # T_RETRY = 2*TD + TMan
          tRetry = Timer(T_RETRY, t_retry)
          tRetry.start()
          print("started timer second time")
            #START TIMER FOR RETRY, 2TD+TMan
    elif (m_dict["Type"] == "GET"):
      mAR = [m_dict["Sender"], m_dict["Time"], m_dict["Position"], m_dict["Velocity"],
            m_dict["Acc"]]
      #in romi code, time is converted into int
      #if (no_conflict(mAR, int(m_dict["Time"]) + 2*time_delay + TMan) and
      if (no_conflict(mAR, float(m_dict["Time"]) + 2*time_delay + TMan) and
          (status == NORMAL or status == TRYGET or
          (status == GET and precedes(m_dict["TagTime"])) or
          ((status == GRANT or status == DELAY_GET) and grantedID(m_dict["TagID"])))):
        if (status == NORMAL):
          status = GRANT
          grantID = m_dict["TagID"]
        elif (status == GET or status == TRYGET):
          tRetry.cancel()
          status = DELAY_GET
          grantID = m_dict["TagID"]
        if (status == GET):
          for agents in D:
            #tmp = "tcp://localhost:" + agents
            print(tmp)
            #context = zmq.Context()
            #socket = context.socket(zmq.DEALER)
            s_message = "RELEASE," + str(agent[0]) + "," + str(agent[1][0]) + "," + str(agent[1][1]) + "," + str(agent[1][2]) + "," + str(agent[1][3] + "," + str(tag[0]) + "," + str(tag[1]))
            print(s_message)
            #socket.connect(tmp)
            #socket.send_string(s_message)
            #sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            #sock.sendto(s_message, (host_ip, int(agents)))
            #sock.close()
            send_udp_message(s_message,int(agents))
        sender = m_dict["Sender"]
        s_message = "GRANT," + str(agent[0]) + "," + str(agent[1][0]) + "," + str(agent[1][1]) + "," + str(agent[1][2]) + "," + str(agent[1][3]) + "," + str(tag[0]) + "," + str(tag[1])
        #tmp = "tcp://localhost:" + sender
        #context = zmq.Context()
        #socket = context.socket(zmq.DEALER)
        print(s_message)
        #socket.connect(tmp)
        #socket.send_string(s_message)

        #sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        #sock.sendto(s_message, (host_ip, int(sender)))
        #sock.close()
        send_udp_message(s_message,int(sender))
        t_grant = Timer(T_GRANT, tgrant)
        t_grant.start()
      else:
        sender = m_dict["Sender"]
        s_message = "DENY," + str(agent[0]) + "," + str(agent[1][0]) + "," + str(agent[1][1]) + "," + str(agent[1][2]) + "," + str(agent[1][3]) + "," + str(tag[0]) + "," + str(tag[1])
        #tmp = "tcp://localhost:" + sender
        #context = zmq.Context()
        #socket = context.socket(zmq.DEALER)
        #socket.connect(tmp)
        print("sending msg: " + s_message)
        #socket.send_string(s_message) 

        #sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        #sock.sendto(s_message, (host_ip, int(sender)))
        #sock.close()
        send_udp_message(s_message,int(sender))

    elif (m_dict["Type"] == "RELEASE" and (status == GRANT or status == DELAY_GET)):
      #STOP TIMER FOR T_GRANT
      t_grant.cancel()
      grantID = 0
      if (status == GRANT):
        status = NORMAL
      if (status == DELAY_GET):
        status = TRYGET
        tryManeuver()


def setup_ros():
  #subscribe to the measurement channel and update variables
  rospy.init_node('maneuver_negotiation',anonymous=True)
  measurement_topic = communication_config.ROS_OPTIONS['measurement-topic']
  rospy.Subscriber(measurement_topic,cm.TestMsg,update_agent_state_from_ros)

  choose_leader_topic = communication_config.ROS_OPTIONS['choose-leader-topic']
  rospy.Subscriber(choose_leader_topic, std_msgs.msg.Int8,choose_leader_processor)

def choose_leader_processor(data):
  #we receive the message from chooseleader topic.
  #this topic will be the car id that would initiate to do the maneuver negotiation
  #algorithm

  print("received messsage from choose leader")
  if data == std_msgs.msg.Int8(aID):
    tryManeuver()


def udp_msg_processor(host_ip,port):
  sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
  print('udp thread: binding to ' + str((host_ip,port)))
  sock.bind((host_ip, port))
  msg,address = sock.recvfrom(4096)

  #print("Received -->>: " + msg)
  t = clock()
  #if(msg == "1"):
  #    global timeTaken
  #    timeTaken = time.clock()
  #    print("TIME BEFORE TRYMANEUVER = %f" % timeTaken)
  #    tryManeuver()
  #else:
  message_processing(msg, t)
  




def update_agent_state_from_ros(data):
## part that communicates with ros to get measurements of cars:
## Update the register of this car in the cloud
# in here, we should get the measrurements from rostopic /measurement
# corresponding to the id of the car
#
    #get correct measurements corresponding to car's id:
    #todo: tell lars to update custom_msgs to include any number of 
    #cars. right now the number of car is fixed to two.
    
    #romis code uses position as a single variable, x.
    #i would use position as a tuple of (x, y)
    #global aID
    #global agent_state
    #if (aID == 1):
    #    measurement = data.cs1

    #elif (aID == 2):
    #    measurement = data.cs1
    #else:
    #    print("incorrect id given in parameter for car's id")
    #get time
    #for the sake of making a prototype, i use only x cordinate
    #todo: fix with x,y
    # our main method should not read data while the funciton is updating
    # position from ros channel. can we use a mutex here? todo
    #agent_state = [data.t,measurement.x,measurement.speed,measurement.acceleration]

    #http://effbot.org/zone/thread-synchronization.htm
    #global variable ros_measurments is updated extremely frequenty
    #(as frequent as ros publishes them)
    #caution must be taken when such a high frequency changing variabe
    #is read by another thread.
    # the following act of replacing variable value is thread safe
    global ros_measurements
    ros_measurements = data



def main():
  global aID
  global handle
  parser = argparse.ArgumentParser("runs Maneuver Negotiation protocol")
  parser.add_argument('id', metavar='I', type=int, help='car id ')

  #accept parameters to indicate the intersection we are dealing with:
  #(self,center_point,lane_width,intersection_type,marker_alignment):
  parser.add_argument('center_x',metavar='cx',type=float, help='center x cordinate of intersection')
  parser.add_argument('center_y',metavar='cy',type=float, help='center y cordinate of intersection')
  parser.add_argument('lane_width',metavar='lw',type=float, help='widh of the lane')
  parser.add_argument('type',metavar='type',type=int, help='intersection type: 0 for give-way, 1 for stop intersection')
  parser.add_argument('marker',metavar='marker',type=int, help='intersection alignment: 0 for north-south, 1 for east-west')

  args = parser.parse_args()

  if args.id > 254:
    print("car id given is too high.")
    exit()

  #store the intersection details in a varaible
  global intersection
  intersection = Intersection.Intersection((args.center_x,args.center_y),args.lane_width, \
                 Intersection.IntersectionType.GIVE_WAY_4 if args.type == 0 else Intersection.IntersectionType.STOP_4,
                 'north-south' if args.marker == 0 else 'east-west')



  handle = zookeeper.init(communication_config.NETWORK_OPTIONS['zookeeper-server'])
  #id1 = int(input())
  id1 = int(args.id)

  #setup network:
  global host_ip
  host_ip = communication_config.NETWORK_OPTIONS['network-prefix'] + str(id1)

  aID = id1
  agent[0] = id1
  port = communication_config.NETWORK_OPTIONS['port-start'] + id1
  print(id1)
  t_update = Timer(T_UPDATE, update)
  t_update.start()
#  if(id1 == 5560):
#    tryManeuver()
  #context = zmq.Context()
  #r_socket = context.socket(zmq.ROUTER)
  #poller = zmq.Poller()
  #poller.register(r_socket, zmq.POLLIN)
  #tmp = "tcp://*:" + str(id1)
  #r_socket.bind(tmp)

  #sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
  #print('binding to ' + str((host_ip,port)))
  #sock.bind((host_ip, port))

  setup_ros()

  #start thread that listens on network for udp packets from other cars
  net_thread = threading.Thread(target= udp_msg_processor,args=(host_ip,port))
  net_thread.start()
  #main thread processes ros messages
  rospy.spin()

  #while 1:
  	#socks = dict(poller.poll())
    #if r_socket in socks:
    #  sendr, msg = r_socket.recv_multipart()

		#msgbuffer = Queue()
		#msgbuffer.put(msg)
    #msg,address = sock.recvfrom(4096)

    #print("Received -->>: " + msg)
    #t = clock()
    #if(msg == "1"):
    #    global timeTaken
    #    timeTaken = time.clock()
    #    print("TIME BEFORE TRYMANEUVER = %f" % timeTaken)
    #    tryManeuver()
    #else:
    #message_processing(msg, t)



#    for i in range(0, 2):
def send_udp_message(message,agent_id):
  #port number is derived from car id.
  #car id ranges from 1 to 254
  #port number ranges from port_prefix+carid to uppper limit 
  #since all cars exist in same network, but ip addresses are 
  #different, 
  #car_id = int(agent_port) - communication_config.NETWORK_OPTIONS['port-start']
  car_id = agent_id
  target_car_ip = communication_config.NETWORK_OPTIONS['network-prefix'] + str(car_id)
  sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
  agent_port = communication_config.NETWORK_OPTIONS['port-start'] + int(car_id)
  print("agent id = " + str(agent_id))
  print(str((target_car_ip,agent_port)))
  sock.sendto(message, (target_car_ip, agent_port))
  sock.close()
    


if __name__ == '__main__':
  main()
