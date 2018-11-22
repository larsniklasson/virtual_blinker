import sys
sys.path.append("..")

import virtual_blinker.msg as msg
import config
import rospy
from threading import Timer
import random
import time

class ManueverNegotiator:
    def __init__(self, risk_estimator, id, turn, nr_cars, sim):
        self.risk_estimator = risk_estimator
        self.granted = False
        self.id = id
        self.turn = turn
        self.publishers = [rospy.Publisher("man_neg" + str(i), msg.ManNeg, queue_size=10) for i in range(nr_cars)]

        self.currentAsk = []
        self.tag = 0
        self.sim = sim


    def tryManeuver(self):
        if self.tag == 0: self.tag = time.time() + self.id / 100000.0
        if rospy.is_shutdown():return
        print self.id, ": entering tryman"
        self.currentAsk = self.risk_estimator.getVehiclesToAsk() # re already knows the turn
        print self.id, ": currentAsk = ", self.currentAsk
        if self.currentAsk == []:
            self.granted = True
            return

        self.grants = []
        message = msg.ManNeg("GET", self.id, self.turn, time.time(), self.tag)   #float(str(time.time())[7:])
        for car in self.currentAsk:
            print self.id, ": sending get to ", car
            #Timer(random.random() * config.max_transmission_delay*1.5, self.publish, [self.publishers[car], message]).start()
            self.publishers[car].publish(message)


        if self.sim.t > 25*config.rate:
            pass
        else:
            self.timer = Timer(2 * config.max_transmission_delay, self.tryManeuver)
            self.timer.start()

    def publish(self, publisher, msg):
        publisher.publish(msg)

    def messageCallback(self, message):
        if self.sim.onehaspassed and not (self.sim.zerohaspassed or self.sim.onehaspassed2):
            print self.id, "ignoring man msg"
            return
        now = time.time()
        if now - message.time < config.max_transmission_delay:
            if message.type == "GET":
                if message.info == "left" and self.turn == "left" and message.id in self.currentAsk:
                    if message.tag < self.tag:
                        nc = True, False
                    else:
                        nc = False
                else:

                    self.risk_estimator.removeVehicleFromGrantList(message.id)
                    nc = self.risk_estimator.no_conflict(message.id, message.info)
                print self.id, "nc: ", nc
                if nc:
                    if not nc[1]:
                        self.risk_estimator.addVehicleToGrantList(message.id)
                    grant_msg = msg.ManNeg("GRANT", self.id, "", time.time(), 0)
                    print self.id, ": sending grant to ", message.id
                    self.publishers[message.id].publish(grant_msg)
                else:
                    print self.id, ":", message.id, "NOT granted"
            
            if message.type == "RELEASE":
                self.risk_estimator.removeVehicleFromGrantList(message.id)
            
            if message.type == "GRANT":
                self.grants.append(message.id)
                if sorted(self.currentAsk) == sorted(self.grants):
                    self.timer.cancel()
                    self.granted = True
                    print self.id, ": granted!!!"
                    self.currentAsk = []
        else:
            print self.id, "msg too old, ignoring it"
              