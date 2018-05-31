from collections import OrderedDict
from particle_filter import ParticleFilter
import plotter
import time
import os
import glob
from E_estimate import *
dir_path = os.path.dirname(os.path.realpath(__file__))
plot_folder = os.path.join(dir_path, "plotfolder")
import Intersection
from threading import Lock


class RiskEstimator:

    def __init__(self, n_particles, intersection, initial_measurements, pose_covariance, speed_deviation, init_time,mutex,plot=False, wipe_dir=False):

        if not os.path.exists(plot_folder):
            os.makedirs(plot_folder)


        if plot and wipe_dir:
            files = glob.glob(plot_folder + "/*")
            for f in files:
                os.remove(f)


        self.intersection = intersection
        self.plot = plot #boolean
        self.last_t = init_time
        self.mutex = mutex

        #get initial directions that the vehicle are travelling towards
        travelling_directions = [intersection.getTravellingDirection(x, y, theta) for (x, y, theta, _) in initial_measurements]

        #initialize the particle filters
        self.particle_filters = \
            [ParticleFilter(travelling_directions, self.intersection, n_particles, m, pose_covariance, speed_deviation) \
                for m in initial_measurements]


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
        self.mutex.aquire()
        self.particle_filters[id].removeKnownIs()
        self.mutex.release()


    def update_state(self, t, measurements):
        self.mutex.acquire()
        
        most_likely_states = [f.get_most_likely_state() for f in self.particle_filters]

        interval = t - self.last_t

        for i, pfilter in enumerate(self.particle_filters):
            pfilter.step_time(i, measurements[i], most_likely_states, interval)
        
        self.last_t = t

        if self.plot:
            plotter.plot_particles(self.particle_filters, measurements, t, plot_folder)
        
        self.mutex.release()

    def isManeuverOk(self, id, turn):
        self.mutex.acquire()
        most_likely_states = [f.get_most_likely_state() for f in self.particle_filters]
        pf = self.particle_filters[id]
        
        go_sum = 0
        for p, w in zip(pf.particles, pf.weights):
            _, go = Es_estimate(id, turn, p.P, p.S, pf.travelling_directions, pf.intersection, most_likely_states)
            
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

