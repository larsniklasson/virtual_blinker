
import sys
sys.path.append("..")

from collections import OrderedDict
from particle_filter import ParticleFilter
import plotter
import time
import os
import glob
from E_estimate import *
dir_path = os.path.dirname(os.path.realpath(__file__))
plot_folder = os.path.join(dir_path, "plotfolder")
from utils.Intersection import *
from threading import Lock,Timer
from config import *

wipe_dir = GEN_CONFIG["wipe_plot_dir"]
intersection = GEN_CONFIG["intersection"]

class RiskEstimator:

    def __init__(self, n_particles, initial_measurements, init_time, deviations, plot, plotafter=0):

        if not os.path.exists(plot_folder):
            os.makedirs(plot_folder)


        if plot and wipe_dir:
            self.plotafter = plotafter
            files = glob.glob(plot_folder + "/*")
            for f in files:
                os.remove(f)


        self.plot = plot #boolean
        self.last_t = init_time

        # to avoid modifying particle and weights while another thread such as maneuver negotiator's no_conflict is
        # reading
        self.mutex = Lock() 
        

        #get initial directions that the vehicle are travelling towards
        self.travelling_directions = [intersection.getTravellingDirection(x, y, theta) for (x, y, theta, _) in initial_measurements]

        #initialize the particle filters
        self.particle_filters = \
            [ParticleFilter(self.travelling_directions, intersection, n_particles, m, deviations) \
                for m in initial_measurements]

        self.updateMostLikelyStates()

        self.grantList= {}  #key = car id, value = [estimated time of finishing intersction, turn]


    def add_car_to_grantlist(self, id, time_finishing,turn):
        self.grantList[id] = [time_finishing,turn]
        self.remove_grant_thread = Timer(time_finishing,self.remove_car_from_grantlist,args=(id))

    def remove_car_from_grantlist(self,id):
        if id in self.grantList:
            del self.grantList[id]


    #todo this is quite ugly, fix this
    #todo save ego-id in RiskEstimator class
    # sets override intention
    def setKnownIc(self, id, Ic):
        self.mutex.acquire()
        self.particle_filters[id].setKnownIc(Ic)
        self.mutex.release()
    def removeKnownIc(self, id):
        self.mutex.acquire()
        self.particle_filters[id].removeKnownIc()
        self.mutex.release()

    def setKnownIs(self, id, Is):
        self.mutex.acquire()
        self.particle_filters[id].setKnownIs(Is)
        self.mutex.release()
    def removeKnownIs(self, id):
        self.mutex.acquire()
        self.particle_filters[id].removeKnownIs()
        self.mutex.release()

    def precalculateForMostLikelyStates(self):
        return [self.precalculate(s, td) for s, td in zip(self.most_likely_states, self.travelling_directions)]
        

    #precalculate hasLeftIntersection and timeToCrossing for one of the most likely states.
    def precalculate(self, state, td):
        d = {}
        _, PS = state
        for turn in intersection.turns:
            c = intersection.courses[td,turn]
            hli = c.hasLeftIntersection(*PS[:3])
            if hli:
                ttc = 0
            else:
                ttc = c.getTimeToCrossing(*PS, Is="go")
            d[turn] = hli, ttc
        return d

    def updateMostLikelyStates(self):

        self.most_likely_states = [f.get_most_likely_state() for f in self.particle_filters]



    def update_state(self, t, measurements):
        self.mutex.acquire()
        
        precalculation = self.precalculateForMostLikelyStates()

        interval = t - self.last_t

        for i, pfilter in enumerate(self.particle_filters):
            pfilter.step_time(i, measurements[i], self.most_likely_states, interval, precalculation)
        
        self.updateMostLikelyStates()

        self.last_t = t

        if self.plot and t >= self.plotafter:
            plotter.plot_particles(self.particle_filters, measurements, t, plot_folder)
        
        self.mutex.release()

    def isManeuverOk(self, id, turn):
        
        self.mutex.acquire()
        precalculation = self.precalculateForMostLikelyStates()

        pf = self.particle_filters[id]
        
        go_sum = 0
        for p, w in zip(pf.particles, pf.weights):
            _, go = Es_estimate(id, turn, p.PS, pf.travelling_directions, pf.intersection, self.most_likely_states, precalculation)
            
            go_sum += float(go*w)
        
        self.mutex.release()
        return go_sum > 0.5





    # probability(Es = "go")
    def getExpectation(self, id):
        self.mutex.acquire()
        density =  self.particle_filters[id].Es_density["go"]
        self.mutex.release()
        return density

    def get_risk(self):
        risks = []
        self.mutex.acquire()
        for pfilter in self.particle_filters:
            risk = 0
            for i,s in enumerate(pfilter.particles):
                weight = pfilter.weights[i]
                
                if ((s.Es == "stop" and s.Is == "go") or (s.Es == "go" and s.Is == "stop")):
                    risk += weight
            
            risks.append(risk)
        
        self.mutex.release()
        return risks

